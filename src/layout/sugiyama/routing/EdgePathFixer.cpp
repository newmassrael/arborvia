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

    // Fix source direction violation
    if (sourceDirectionViolation && !layout.bendPoints.empty()) {
        bool srcVertical = (layout.sourceEdge == NodeEdge::Top || layout.sourceEdge == NodeEdge::Bottom);
        const Point& fBend = layout.bendPoints.front().position;

        if (srcVertical) {
            bool shouldGoUp = (layout.sourceEdge == NodeEdge::Top);
            float clearanceY = shouldGoUp
                ? std::min(layout.sourcePoint.y - effectiveGridSize * 2, fBend.y - effectiveGridSize)
                : std::max(layout.sourcePoint.y + effectiveGridSize * 2, fBend.y + effectiveGridSize);
            if (effectiveGridSize > 0) {
                clearanceY = std::round(clearanceY / effectiveGridSize) * effectiveGridSize;
            }

            bool sameVerticalLine = std::abs(layout.sourcePoint.x - fBend.x) < 0.5f;
            Point clearancePoint = {layout.sourcePoint.x, clearanceY};

            if (sameVerticalLine) {
                bool sourceToClearanceGoesUp = clearanceY < layout.sourcePoint.y;
                bool clearanceToFirstBendGoesUp = fBend.y < clearanceY;
                bool wouldCreateSpike = (sourceToClearanceGoesUp != clearanceToFirstBendGoesUp);

                if (wouldCreateSpike) {
                    float offsetX = effectiveGridSize > 0 ? effectiveGridSize * 2 : 40.0f;
                    if (layout.targetPoint.x > layout.sourcePoint.x) offsetX = -offsetX;
                    float detourX = layout.sourcePoint.x + offsetX;
                    if (effectiveGridSize > 0) detourX = std::round(detourX / effectiveGridSize) * effectiveGridSize;

                    Point detourPoint1 = {detourX, clearanceY};
                    Point detourPoint2 = {detourX, fBend.y};

                    layout.bendPoints.insert(layout.bendPoints.begin(), {detourPoint2});
                    layout.bendPoints.insert(layout.bendPoints.begin(), {detourPoint1});
                    layout.bendPoints.insert(layout.bendPoints.begin(), {clearancePoint});
                } else {
                    layout.bendPoints.insert(layout.bendPoints.begin(), {clearancePoint});
                }
            } else {
                Point connectionPoint = {fBend.x, clearanceY};
                if (effectiveGridSize > 0) connectionPoint.x = std::round(connectionPoint.x / effectiveGridSize) * effectiveGridSize;
                layout.bendPoints.insert(layout.bendPoints.begin(), {connectionPoint});
                layout.bendPoints.insert(layout.bendPoints.begin(), {clearancePoint});
            }
        } else {
            bool shouldGoLeft = (layout.sourceEdge == NodeEdge::Left);
            float clearanceX = shouldGoLeft
                ? std::min(layout.sourcePoint.x - effectiveGridSize * 2, fBend.x - effectiveGridSize)
                : std::max(layout.sourcePoint.x + effectiveGridSize * 2, fBend.x + effectiveGridSize);
            if (effectiveGridSize > 0) {
                clearanceX = std::round(clearanceX / effectiveGridSize) * effectiveGridSize;
            }

            bool sameHorizontalLine = std::abs(layout.sourcePoint.y - fBend.y) < 0.5f;
            Point clearancePoint = {clearanceX, layout.sourcePoint.y};

            if (sameHorizontalLine) {
                bool sourceToClearanceGoesLeft = clearanceX < layout.sourcePoint.x;
                bool clearanceToFirstBendGoesLeft = fBend.x < clearanceX;
                bool wouldCreateSpike = (sourceToClearanceGoesLeft != clearanceToFirstBendGoesLeft);

                if (wouldCreateSpike) {
                    float offsetY = effectiveGridSize > 0 ? effectiveGridSize * 2 : 40.0f;
                    if (layout.targetPoint.y > layout.sourcePoint.y) offsetY = -offsetY;
                    float detourY = layout.sourcePoint.y + offsetY;
                    if (effectiveGridSize > 0) detourY = std::round(detourY / effectiveGridSize) * effectiveGridSize;

                    Point detourPoint1 = {clearanceX, detourY};
                    Point detourPoint2 = {fBend.x, detourY};

                    layout.bendPoints.insert(layout.bendPoints.begin(), {detourPoint2});
                    layout.bendPoints.insert(layout.bendPoints.begin(), {detourPoint1});
                    layout.bendPoints.insert(layout.bendPoints.begin(), {clearancePoint});
                } else {
                    layout.bendPoints.insert(layout.bendPoints.begin(), {clearancePoint});
                }
            } else {
                Point connectionPoint = {clearanceX, fBend.y};
                if (effectiveGridSize > 0) connectionPoint.y = std::round(connectionPoint.y / effectiveGridSize) * effectiveGridSize;
                layout.bendPoints.insert(layout.bendPoints.begin(), {connectionPoint});
                layout.bendPoints.insert(layout.bendPoints.begin(), {clearancePoint});
            }
        }
    }

    // Fix target direction violation
    if (targetDirectionViolation && !layout.bendPoints.empty()) {
        bool tgtVertical = (layout.targetEdge == NodeEdge::Top || layout.targetEdge == NodeEdge::Bottom);
        const Point& lBend = layout.bendPoints.back().position;

        if (tgtVertical) {
            bool shouldEnterFromAbove = (layout.targetEdge == NodeEdge::Top);
            float clearanceY = shouldEnterFromAbove
                ? std::min(layout.targetPoint.y - effectiveGridSize * 2, lBend.y - effectiveGridSize)
                : std::max(layout.targetPoint.y + effectiveGridSize * 2, lBend.y + effectiveGridSize);
            if (effectiveGridSize > 0) {
                clearanceY = std::round(clearanceY / effectiveGridSize) * effectiveGridSize;
            }

            bool sameVerticalLine = std::abs(lBend.x - layout.targetPoint.x) < 0.5f;

            if (sameVerticalLine) {
                bool lastBendToClearanceGoesUp = clearanceY < lBend.y;
                bool clearanceToTargetGoesUp = layout.targetPoint.y < clearanceY;
                bool wouldCreateSpike = (lastBendToClearanceGoesUp != clearanceToTargetGoesUp);

                if (wouldCreateSpike) {
                    float offsetX = effectiveGridSize > 0 ? effectiveGridSize * 2 : 40.0f;
                    if (layout.sourcePoint.x > layout.targetPoint.x) offsetX = -offsetX;
                    float detourX = lBend.x + offsetX;
                    if (effectiveGridSize > 0) detourX = std::round(detourX / effectiveGridSize) * effectiveGridSize;

                    Point detourPoint1 = {detourX, lBend.y};
                    Point detourPoint2 = {detourX, clearanceY};
                    Point clearancePoint = {layout.targetPoint.x, clearanceY};

                    layout.bendPoints.push_back({detourPoint1});
                    layout.bendPoints.push_back({detourPoint2});
                    layout.bendPoints.push_back({clearancePoint});
                } else {
                    Point clearancePoint = {layout.targetPoint.x, clearanceY};
                    layout.bendPoints.push_back({clearancePoint});
                }
            } else {
                Point connectionPoint = {lBend.x, clearanceY};
                if (effectiveGridSize > 0) connectionPoint.x = std::round(connectionPoint.x / effectiveGridSize) * effectiveGridSize;
                Point clearancePoint = {layout.targetPoint.x, clearanceY};
                layout.bendPoints.push_back({connectionPoint});
                layout.bendPoints.push_back({clearancePoint});
            }
        } else {
            bool shouldEnterFromLeft = (layout.targetEdge == NodeEdge::Left);
            float clearanceX = shouldEnterFromLeft
                ? std::min(layout.targetPoint.x - effectiveGridSize * 2, lBend.x - effectiveGridSize)
                : std::max(layout.targetPoint.x + effectiveGridSize * 2, lBend.x + effectiveGridSize);
            if (effectiveGridSize > 0) {
                clearanceX = std::round(clearanceX / effectiveGridSize) * effectiveGridSize;
            }

            bool sameHorizontalLine = std::abs(lBend.y - layout.targetPoint.y) < 0.5f;

            if (sameHorizontalLine) {
                bool lastBendToClearanceGoesLeft = clearanceX < lBend.x;
                bool clearanceToTargetGoesLeft = layout.targetPoint.x < clearanceX;
                bool wouldCreateSpike = (lastBendToClearanceGoesLeft != clearanceToTargetGoesLeft);

                if (wouldCreateSpike) {
                    float offsetY = effectiveGridSize > 0 ? effectiveGridSize * 2 : 40.0f;
                    if (layout.sourcePoint.y > layout.targetPoint.y) offsetY = -offsetY;
                    float detourY = lBend.y + offsetY;
                    if (effectiveGridSize > 0) detourY = std::round(detourY / effectiveGridSize) * effectiveGridSize;

                    Point detourPoint1 = {lBend.x, detourY};
                    Point detourPoint2 = {clearanceX, detourY};
                    Point clearancePoint = {clearanceX, layout.targetPoint.y};

                    layout.bendPoints.push_back({detourPoint1});
                    layout.bendPoints.push_back({detourPoint2});
                    layout.bendPoints.push_back({clearancePoint});
                } else {
                    Point clearancePoint = {clearanceX, layout.targetPoint.y};
                    layout.bendPoints.push_back({clearancePoint});
                }
            } else {
                Point connectionPoint = {clearanceX, lBend.y};
                if (effectiveGridSize > 0) connectionPoint.y = std::round(connectionPoint.y / effectiveGridSize) * effectiveGridSize;
                Point clearancePoint = {clearanceX, layout.targetPoint.y};
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
