#include "EdgePathFixer.h"
#include "PathCalculator.h"
#include "../../routing/UnifiedRetryChain.h"
#include "PathCleanup.h"
#include <cmath>
#include "arborvia/common/Logger.h"


namespace arborvia {

EdgePathFixer::EdgePathFixer(IPathFinder& pathFinder)
    : pathFinder_(pathFinder) {
}

EdgePathFixer::~EdgePathFixer() = default;

UnifiedRetryChain& EdgePathFixer::getRetryChain(float gridSize) {
    if (!retryChain_ || lastGridSize_ != gridSize) {
        // Create shared_ptr with null deleter (we don't own pathFinder_)
        auto pathFinderPtr = std::shared_ptr<IPathFinder>(&pathFinder_, [](IPathFinder*){});
        retryChain_ = std::make_unique<UnifiedRetryChain>(pathFinderPtr, gridSize);
        lastGridSize_ = gridSize;
    }
    return *retryChain_;
}

void EdgePathFixer::recalculateBendPoints(
    EdgeLayout& layout,
    const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
    float effectiveGridSize,
    const std::unordered_map<EdgeId, EdgeLayout>* otherEdges,
    const std::unordered_set<NodeId>* movedNodes) {

    PathCalculator calculator(pathFinder_);
    calculator.recalculateBendPoints(layout, nodeLayouts, effectiveGridSize, otherEdges, movedNodes);
}

bool EdgePathFixer::detectAndFixDiagonals(
    EdgeId edgeId,
    std::unordered_map<EdgeId, EdgeLayout>& edgeLayouts,
    const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
    const std::unordered_set<NodeId>& movedNodes,
    float effectiveGridSize,
    std::unordered_map<EdgeId, EdgeLayout>& /* otherEdges */) {

    auto shouldUpdateNode = [&movedNodes](NodeId nid) -> bool {
        return movedNodes.empty() || movedNodes.count(nid) > 0;
    };

    auto it = edgeLayouts.find(edgeId);
    if (it == edgeLayouts.end()) return true;

    EdgeLayout& layout = it->second;
    bool needsRetry = false;

    // Diagonal detection logic
    if (layout.bendPoints.empty()) {
        float dx = std::abs(layout.sourcePoint.x - layout.targetPoint.x);
        float dy = std::abs(layout.sourcePoint.y - layout.targetPoint.y);
        if (dx > 1.0f && dy > 1.0f) {
            needsRetry = true;
        }
    } else {
        float dx_src = std::abs(layout.sourcePoint.x - layout.bendPoints[0].position.x);
        float dy_src = std::abs(layout.sourcePoint.y - layout.bendPoints[0].position.y);
        const auto& lastBend = layout.bendPoints.back();
        float dx_tgt = std::abs(lastBend.position.x - layout.targetPoint.x);
        float dy_tgt = std::abs(lastBend.position.y - layout.targetPoint.y);

        bool sourceDiagonal = (dx_src > 1.0f && dy_src > 1.0f);
        bool targetDiagonal = (dx_tgt > 1.0f && dy_tgt > 1.0f);

        if (sourceDiagonal || targetDiagonal) {
            needsRetry = true;
        }
    }

    if (!needsRetry) return true;

    // Check if both nodes are stationary
    bool srcNodeMoved = shouldUpdateNode(layout.from);
    bool tgtNodeMoved = shouldUpdateNode(layout.to);

    if (!srcNodeMoved && !tgtNodeMoved) {
        return true;  // Skip retry for stationary nodes
    }

    // Save original state for rollback
    EdgeLayout originalLayout = layout;

    // Use UnifiedRetryChain for full retry sequence
    auto& retryChain = getRetryChain(effectiveGridSize);

    UnifiedRetryChain::RetryConfig config;
    config.maxSnapRetries = 9;
    config.maxNodeEdgeCombinations = 16;
    config.enableCooperativeReroute = true;
    config.gridSize = effectiveGridSize;
    config.movedNodes = &movedNodes;

    auto result = retryChain.calculatePath(
        edgeId, layout, edgeLayouts, nodeLayouts, config);

    if (result.success) {
        layout = result.layout;

        // Update other edges if they were rerouted
        for (const auto& reroutedLayout : result.reroutedEdges) {
            edgeLayouts[reroutedLayout.id] = reroutedLayout;
        }

        return true;
    }

    // Retry failed - restore original layout
    layout = originalLayout;

    return false;
}

bool EdgePathFixer::validateAndFixDirectionConstraints(
    EdgeLayout& layout,
    float effectiveGridSize) {

    if (layout.bendPoints.empty()) return false;

    // === INTEGER GRID CALCULATION HELPERS ===
    auto toGrid = [effectiveGridSize](float pixel) -> int {
        return static_cast<int>(std::round(pixel / effectiveGridSize));
    };
    auto toPixel = [effectiveGridSize](int grid) -> float {
        return grid * effectiveGridSize;
    };

    bool sourceVertical = (layout.sourceEdge == NodeEdge::Top || layout.sourceEdge == NodeEdge::Bottom);
    bool targetVertical = (layout.targetEdge == NodeEdge::Top || layout.targetEdge == NodeEdge::Bottom);

    const Point& firstBend = layout.bendPoints.front().position;
    const Point& lastBend = layout.bendPoints.back().position;

    constexpr float DIRECTION_TOLERANCE = 0.5f;
    bool firstSegmentVertical = std::abs(layout.sourcePoint.x - firstBend.x) < DIRECTION_TOLERANCE;
    bool lastSegmentVertical = std::abs(lastBend.x - layout.targetPoint.x) < DIRECTION_TOLERANCE;

    bool orthogonalViolation = (sourceVertical != firstSegmentVertical || targetVertical != lastSegmentVertical);

    // Check source direction
    bool sourceDirectionViolation = false;
    if (!orthogonalViolation && sourceVertical) {
        float dyFirst = firstBend.y - layout.sourcePoint.y;
        bool shouldGoUp = (layout.sourceEdge == NodeEdge::Top);
        bool shouldGoDown = (layout.sourceEdge == NodeEdge::Bottom);
        if ((shouldGoUp && dyFirst > DIRECTION_TOLERANCE) || (shouldGoDown && dyFirst < -DIRECTION_TOLERANCE)) {
            sourceDirectionViolation = true;
        }
    } else if (!orthogonalViolation && !sourceVertical) {
        float dxFirst = firstBend.x - layout.sourcePoint.x;
        bool shouldGoLeft = (layout.sourceEdge == NodeEdge::Left);
        bool shouldGoRight = (layout.sourceEdge == NodeEdge::Right);
        if ((shouldGoLeft && dxFirst > DIRECTION_TOLERANCE) || (shouldGoRight && dxFirst < -DIRECTION_TOLERANCE)) {
            sourceDirectionViolation = true;
        }
    }

    // Check target direction
    bool targetDirectionViolation = false;
    if (!orthogonalViolation && targetVertical) {
        float dyLast = layout.targetPoint.y - lastBend.y;
        bool shouldEnterFromAbove = (layout.targetEdge == NodeEdge::Top);
        bool shouldEnterFromBelow = (layout.targetEdge == NodeEdge::Bottom);
        if ((shouldEnterFromAbove && dyLast < -DIRECTION_TOLERANCE) || (shouldEnterFromBelow && dyLast > DIRECTION_TOLERANCE)) {
            targetDirectionViolation = true;
        }
    } else if (!orthogonalViolation && !targetVertical) {
        float dxLast = layout.targetPoint.x - lastBend.x;
        bool shouldEnterFromLeft = (layout.targetEdge == NodeEdge::Left);
        bool shouldEnterFromRight = (layout.targetEdge == NodeEdge::Right);
        if ((shouldEnterFromLeft && dxLast < -DIRECTION_TOLERANCE) || (shouldEnterFromRight && dxLast > DIRECTION_TOLERANCE)) {
            targetDirectionViolation = true;
        }
    }

    // Fix source direction violation (using integer grid calculation)
    if (sourceDirectionViolation && !layout.bendPoints.empty()) {
        bool srcVertical = (layout.sourceEdge == NodeEdge::Top || layout.sourceEdge == NodeEdge::Bottom);
        const Point& fBend = layout.bendPoints.front().position;

        // Convert to grid coordinates
        int gSrcX = toGrid(layout.sourcePoint.x);
        int gSrcY = toGrid(layout.sourcePoint.y);
        int gFBendX = toGrid(fBend.x);
        int gFBendY = toGrid(fBend.y);
        int gTgtX = toGrid(layout.targetPoint.x);

        if (srcVertical) {
            bool shouldGoUp = (layout.sourceEdge == NodeEdge::Top);
            int gClearanceY = shouldGoUp
                ? std::min(gSrcY - 2, gFBendY - 1)
                : std::max(gSrcY + 2, gFBendY + 1);

            bool sameVerticalLine = (gSrcX == gFBendX);
            Point clearancePoint = {toPixel(gSrcX), toPixel(gClearanceY)};

            if (sameVerticalLine) {
                bool sourceToClearanceGoesUp = gClearanceY < gSrcY;
                bool clearanceToFirstBendGoesUp = gFBendY < gClearanceY;
                bool wouldCreateSpike = (sourceToClearanceGoesUp != clearanceToFirstBendGoesUp);

                if (wouldCreateSpike) {
                    int gOffsetX = 2;
                    if (gTgtX > gSrcX) gOffsetX = -gOffsetX;
                    int gDetourX = gSrcX + gOffsetX;

                    Point detourPoint1 = {toPixel(gDetourX), toPixel(gClearanceY)};
                    Point detourPoint2 = {toPixel(gDetourX), toPixel(gFBendY)};

                    layout.bendPoints.insert(layout.bendPoints.begin(), {detourPoint2});
                    layout.bendPoints.insert(layout.bendPoints.begin(), {detourPoint1});
                    layout.bendPoints.insert(layout.bendPoints.begin(), {clearancePoint});
                } else {
                    layout.bendPoints.insert(layout.bendPoints.begin(), {clearancePoint});
                }
            } else {
                Point connectionPoint = {toPixel(gFBendX), toPixel(gClearanceY)};
                layout.bendPoints.insert(layout.bendPoints.begin(), {connectionPoint});
                layout.bendPoints.insert(layout.bendPoints.begin(), {clearancePoint});
            }
        } else {
            bool shouldGoLeft = (layout.sourceEdge == NodeEdge::Left);
            int gClearanceX = shouldGoLeft
                ? std::min(gSrcX - 2, gFBendX - 1)
                : std::max(gSrcX + 2, gFBendX + 1);

            int gTgtY = toGrid(layout.targetPoint.y);
            bool sameHorizontalLine = (gSrcY == gFBendY);
            Point clearancePoint = {toPixel(gClearanceX), toPixel(gSrcY)};

            if (sameHorizontalLine) {
                bool sourceToClearanceGoesLeft = gClearanceX < gSrcX;
                bool clearanceToFirstBendGoesLeft = gFBendX < gClearanceX;
                bool wouldCreateSpike = (sourceToClearanceGoesLeft != clearanceToFirstBendGoesLeft);

                if (wouldCreateSpike) {
                    int gOffsetY = 2;
                    if (gTgtY > gSrcY) gOffsetY = -gOffsetY;
                    int gDetourY = gSrcY + gOffsetY;

                    Point detourPoint1 = {toPixel(gClearanceX), toPixel(gDetourY)};
                    Point detourPoint2 = {toPixel(gFBendX), toPixel(gDetourY)};

                    layout.bendPoints.insert(layout.bendPoints.begin(), {detourPoint2});
                    layout.bendPoints.insert(layout.bendPoints.begin(), {detourPoint1});
                    layout.bendPoints.insert(layout.bendPoints.begin(), {clearancePoint});
                } else {
                    layout.bendPoints.insert(layout.bendPoints.begin(), {clearancePoint});
                }
            } else {
                Point connectionPoint = {toPixel(gClearanceX), toPixel(gFBendY)};
                layout.bendPoints.insert(layout.bendPoints.begin(), {connectionPoint});
                layout.bendPoints.insert(layout.bendPoints.begin(), {clearancePoint});
            }
        }
    }

    // Fix target direction violation (using integer grid calculation)
    if (targetDirectionViolation && !layout.bendPoints.empty()) {
        bool tgtVertical = (layout.targetEdge == NodeEdge::Top || layout.targetEdge == NodeEdge::Bottom);
        const Point& lBend = layout.bendPoints.back().position;

        // Convert to grid coordinates
        int gTgtX = toGrid(layout.targetPoint.x);
        int gTgtY = toGrid(layout.targetPoint.y);
        int gLBendX = toGrid(lBend.x);
        int gLBendY = toGrid(lBend.y);
        int gSrcX = toGrid(layout.sourcePoint.x);
        int gSrcY = toGrid(layout.sourcePoint.y);

        if (tgtVertical) {
            bool shouldEnterFromAbove = (layout.targetEdge == NodeEdge::Top);
            int gClearanceY = shouldEnterFromAbove
                ? std::min(gTgtY - 2, gLBendY - 1)
                : std::max(gTgtY + 2, gLBendY + 1);

            bool sameVerticalLine = (gLBendX == gTgtX);

            if (sameVerticalLine) {
                bool lastBendToClearanceGoesUp = gClearanceY < gLBendY;
                bool clearanceToTargetGoesUp = gTgtY < gClearanceY;
                bool wouldCreateSpike = (lastBendToClearanceGoesUp != clearanceToTargetGoesUp);

                if (wouldCreateSpike) {
                    int gOffsetX = 2;
                    if (gSrcX > gTgtX) gOffsetX = -gOffsetX;
                    int gDetourX = gLBendX + gOffsetX;

                    Point detourPoint1 = {toPixel(gDetourX), toPixel(gLBendY)};
                    Point detourPoint2 = {toPixel(gDetourX), toPixel(gClearanceY)};
                    Point clearancePoint = {toPixel(gTgtX), toPixel(gClearanceY)};

                    layout.bendPoints.push_back({detourPoint1});
                    layout.bendPoints.push_back({detourPoint2});
                    layout.bendPoints.push_back({clearancePoint});
                } else {
                    Point clearancePoint = {toPixel(gTgtX), toPixel(gClearanceY)};
                    layout.bendPoints.push_back({clearancePoint});
                }
            } else {
                Point connectionPoint = {toPixel(gLBendX), toPixel(gClearanceY)};
                Point clearancePoint = {toPixel(gTgtX), toPixel(gClearanceY)};
                layout.bendPoints.push_back({connectionPoint});
                layout.bendPoints.push_back({clearancePoint});
            }
        } else {
            bool shouldEnterFromLeft = (layout.targetEdge == NodeEdge::Left);
            int gClearanceX = shouldEnterFromLeft
                ? std::min(gTgtX - 2, gLBendX - 1)
                : std::max(gTgtX + 2, gLBendX + 1);

            bool sameHorizontalLine = (gLBendY == gTgtY);

            if (sameHorizontalLine) {
                bool lastBendToClearanceGoesLeft = gClearanceX < gLBendX;
                bool clearanceToTargetGoesLeft = gTgtX < gClearanceX;
                bool wouldCreateSpike = (lastBendToClearanceGoesLeft != clearanceToTargetGoesLeft);

                if (wouldCreateSpike) {
                    int gOffsetY = 2;
                    if (gSrcY > gTgtY) gOffsetY = -gOffsetY;
                    int gDetourY = gLBendY + gOffsetY;

                    Point detourPoint1 = {toPixel(gLBendX), toPixel(gDetourY)};
                    Point detourPoint2 = {toPixel(gClearanceX), toPixel(gDetourY)};
                    Point clearancePoint = {toPixel(gClearanceX), toPixel(gTgtY)};

                    layout.bendPoints.push_back({detourPoint1});
                    layout.bendPoints.push_back({detourPoint2});
                    layout.bendPoints.push_back({clearancePoint});
                } else {
                    Point clearancePoint = {toPixel(gClearanceX), toPixel(gTgtY)};
                    layout.bendPoints.push_back({clearancePoint});
                }
            } else {
                Point connectionPoint = {toPixel(gClearanceX), toPixel(gLBendY)};
                Point clearancePoint = {toPixel(gClearanceX), toPixel(gTgtY)};
                layout.bendPoints.push_back({connectionPoint});
                layout.bendPoints.push_back({clearancePoint});
            }
        }
    }

    // Clean up after direction fix
    if (sourceDirectionViolation || targetDirectionViolation) {
        std::vector<Point> allPoints;
        allPoints.push_back(layout.sourcePoint);
        for (const auto& bp : layout.bendPoints) {
            allPoints.push_back(bp.position);
        }
        allPoints.push_back(layout.targetPoint);

        PathCleanup::removeSpikesAndDuplicates(allPoints);

        layout.bendPoints.clear();
        for (size_t j = 1; j + 1 < allPoints.size(); ++j) {
            layout.bendPoints.push_back({allPoints[j]});
        }
        return true;  // Direction was fixed
    }
    return false;  // No fix needed
}

} // namespace arborvia
