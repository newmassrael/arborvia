#include "EdgePathFixer.h"
#include "PathCalculator.h"
#include "../../snap/GridSnapCalculator.h"
#include "../../pathfinding/ObstacleMap.h"
#include "PathCleanup.h"
#include <cmath>
#include <array>

#ifndef EDGE_ROUTING_DEBUG
#define EDGE_ROUTING_DEBUG 0
#endif

namespace arborvia {

EdgePathFixer::EdgePathFixer(IPathFinder& pathFinder)
    : pathFinder_(pathFinder) {
}

void EdgePathFixer::recalculateBendPoints(
    EdgeLayout& layout,
    const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
    float effectiveGridSize,
    const std::unordered_map<EdgeId, EdgeLayout>* otherEdges) {

    PathCalculator calculator(pathFinder_);
    calculator.recalculateBendPoints(layout, nodeLayouts, effectiveGridSize, otherEdges);
}

bool EdgePathFixer::detectAndFixDiagonals(
    EdgeId edgeId,
    std::unordered_map<EdgeId, EdgeLayout>& edgeLayouts,
    const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
    const std::unordered_set<NodeId>& movedNodes,
    float effectiveGridSize,
    std::unordered_map<EdgeId, EdgeLayout>& otherEdges) {

    auto shouldUpdateNode = [&movedNodes](NodeId nid) -> bool {
        return movedNodes.empty() || movedNodes.count(nid) > 0;
    };

    auto it = edgeLayouts.find(edgeId);
    if (it == edgeLayouts.end()) return true;

    EdgeLayout& layout = it->second;
    bool needsRetry = false;

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

    // Save original state
    Point originalSourcePoint = layout.sourcePoint;
    Point originalTargetPoint = layout.targetPoint;
    int originalSourceSnapIndex = layout.sourceSnapIndex;
    int originalTargetSnapIndex = layout.targetSnapIndex;
    std::vector<BendPoint> originalBendPoints = layout.bendPoints;
    NodeEdge origSrcEdge = layout.sourceEdge;
    NodeEdge origTgtEdge = layout.targetEdge;

    bool swapSucceeded = false;

    // Try source-side swaps
    std::vector<EdgeId> srcSwapCandidates;
    for (const auto& [otherEdgeId, otherLayout] : edgeLayouts) {
        if (otherEdgeId == edgeId) continue;
        if (otherLayout.from == layout.from && otherLayout.sourceEdge == layout.sourceEdge) {
            // Only swap with edges whose source node moved (if movedNodes is specified)
            if (!movedNodes.empty() && movedNodes.count(otherLayout.from) == 0) {
                continue;  // Skip - other edge's source node didn't move
            }
            srcSwapCandidates.push_back(otherEdgeId);
        }
    }

    for (EdgeId swapWithId : srcSwapCandidates) {
        if (swapSucceeded) break;
        auto swapIt = edgeLayouts.find(swapWithId);
        if (swapIt == edgeLayouts.end()) continue;

        Point swapOriginalSourcePoint = swapIt->second.sourcePoint;
        int swapOriginalSnapIndex = swapIt->second.sourceSnapIndex;
        std::vector<BendPoint> swapOriginalBendPoints = swapIt->second.bendPoints;
        // Also save target points - recalculateBendPoints might modify them
        Point layoutOriginalTargetPoint = layout.targetPoint;
        int layoutOriginalTargetSnapIndex = layout.targetSnapIndex;
        Point swapOriginalTargetPoint = swapIt->second.targetPoint;
        int swapOriginalTargetSnapIndex = swapIt->second.targetSnapIndex;

        std::swap(layout.sourcePoint, swapIt->second.sourcePoint);
        std::swap(layout.sourceSnapIndex, swapIt->second.sourceSnapIndex);

        recalculateBendPoints(layout, nodeLayouts, effectiveGridSize, nullptr);
        recalculateBendPoints(swapIt->second, nodeLayouts, effectiveGridSize, nullptr);

        // Restore target points for unmoved target nodes (recalculateBendPoints may have modified them)
        if (!movedNodes.empty()) {
            if (movedNodes.count(layout.to) == 0) {
                layout.targetPoint = layoutOriginalTargetPoint;
                layout.targetSnapIndex = layoutOriginalTargetSnapIndex;
            }
            if (movedNodes.count(swapIt->second.to) == 0) {
                swapIt->second.targetPoint = swapOriginalTargetPoint;
                swapIt->second.targetSnapIndex = swapOriginalTargetSnapIndex;
            }
        }

        bool stillDiagonal = false;
        if (layout.bendPoints.empty()) {
            float dx = std::abs(layout.sourcePoint.x - layout.targetPoint.x);
            float dy = std::abs(layout.sourcePoint.y - layout.targetPoint.y);
            stillDiagonal = (dx > 1.0f && dy > 1.0f);
        }

        bool swapBecameDiagonal = false;
        if (swapIt->second.bendPoints.empty()) {
            float dx = std::abs(swapIt->second.sourcePoint.x - swapIt->second.targetPoint.x);
            float dy = std::abs(swapIt->second.sourcePoint.y - swapIt->second.targetPoint.y);
            swapBecameDiagonal = (dx > 1.0f && dy > 1.0f);
        }

        if (!stillDiagonal && !swapBecameDiagonal) {
            swapSucceeded = true;
        } else {
            layout.sourcePoint = originalSourcePoint;
            layout.sourceSnapIndex = originalSourceSnapIndex;
            layout.bendPoints = originalBendPoints;
            swapIt->second.sourcePoint = swapOriginalSourcePoint;
            swapIt->second.sourceSnapIndex = swapOriginalSnapIndex;
            swapIt->second.bendPoints = swapOriginalBendPoints;
        }
    }

    // Try target-side swaps if source swaps failed
    if (!swapSucceeded) {
        std::vector<EdgeId> tgtSwapCandidates;
        for (const auto& [otherEdgeId, otherLayout] : edgeLayouts) {
            if (otherEdgeId == edgeId) continue;
            if (otherLayout.to == layout.to && otherLayout.targetEdge == layout.targetEdge) {
                // Only swap with edges whose target node moved (if movedNodes is specified)
                if (!movedNodes.empty() && movedNodes.count(otherLayout.to) == 0) {
                    continue;  // Skip - other edge's target node didn't move
                }
                tgtSwapCandidates.push_back(otherEdgeId);
            }
        }

        for (EdgeId swapWithId : tgtSwapCandidates) {
            if (swapSucceeded) break;
            auto swapIt = edgeLayouts.find(swapWithId);
            if (swapIt == edgeLayouts.end()) continue;

            Point swapOriginalTargetPoint = swapIt->second.targetPoint;
            int swapOriginalSnapIndex = swapIt->second.targetSnapIndex;
            std::vector<BendPoint> swapOriginalBendPoints = swapIt->second.bendPoints;
            // Also save source points - recalculateBendPoints might modify them
            Point layoutOriginalSourcePoint = layout.sourcePoint;
            int layoutOriginalSourceSnapIndex = layout.sourceSnapIndex;
            Point swapOriginalSourcePoint = swapIt->second.sourcePoint;
            int swapOriginalSourceSnapIndex = swapIt->second.sourceSnapIndex;

            std::swap(layout.targetPoint, swapIt->second.targetPoint);
            std::swap(layout.targetSnapIndex, swapIt->second.targetSnapIndex);

            recalculateBendPoints(layout, nodeLayouts, effectiveGridSize, nullptr);
            recalculateBendPoints(swapIt->second, nodeLayouts, effectiveGridSize, nullptr);

            // Restore source points for unmoved source nodes (recalculateBendPoints may have modified them)
            if (!movedNodes.empty()) {
                if (movedNodes.count(layout.from) == 0) {
                    layout.sourcePoint = layoutOriginalSourcePoint;
                    layout.sourceSnapIndex = layoutOriginalSourceSnapIndex;
                }
                if (movedNodes.count(swapIt->second.from) == 0) {
                    swapIt->second.sourcePoint = swapOriginalSourcePoint;
                    swapIt->second.sourceSnapIndex = swapOriginalSourceSnapIndex;
                }
            }

            bool stillDiagonal = false;
            if (layout.bendPoints.empty()) {
                float dx = std::abs(layout.sourcePoint.x - layout.targetPoint.x);
                float dy = std::abs(layout.sourcePoint.y - layout.targetPoint.y);
                stillDiagonal = (dx > 1.0f && dy > 1.0f);
            }

            bool swapBecameDiagonal = false;
            if (swapIt->second.bendPoints.empty()) {
                float dx = std::abs(swapIt->second.sourcePoint.x - swapIt->second.targetPoint.x);
                float dy = std::abs(swapIt->second.sourcePoint.y - swapIt->second.targetPoint.y);
                swapBecameDiagonal = (dx > 1.0f && dy > 1.0f);
            }

            if (!stillDiagonal && !swapBecameDiagonal) {
                swapSucceeded = true;
            } else {
                layout.targetPoint = originalTargetPoint;
                layout.targetSnapIndex = originalTargetSnapIndex;
                layout.bendPoints = originalBendPoints;
                swapIt->second.targetPoint = swapOriginalTargetPoint;
                swapIt->second.targetSnapIndex = swapOriginalSnapIndex;
                swapIt->second.bendPoints = swapOriginalBendPoints;
            }
        }
    }

    // Exhaustive NodeEdge search if swaps failed
    if (!swapSucceeded) {
        auto srcNodeIt = nodeLayouts.find(layout.from);
        auto tgtNodeIt = nodeLayouts.find(layout.to);

        if (srcNodeIt != nodeLayouts.end() && tgtNodeIt != nodeLayouts.end()) {
            const auto& srcNode = srcNodeIt->second;
            const auto& tgtNode = tgtNodeIt->second;

            constexpr std::array<NodeEdge, 4> allEdges = {
                NodeEdge::Top, NodeEdge::Bottom, NodeEdge::Left, NodeEdge::Right
            };

            bool exhaustiveSuccess = false;

            for (NodeEdge srcEdge : allEdges) {
                if (exhaustiveSuccess) break;
                for (NodeEdge tgtEdge : allEdges) {
                    if (exhaustiveSuccess) break;

                    int srcCandidateCount = GridSnapCalculator::getCandidateCount(srcNode, srcEdge, effectiveGridSize);
                    int tgtCandidateCount = GridSnapCalculator::getCandidateCount(tgtNode, tgtEdge, effectiveGridSize);

                    for (int srcConnIdx = 0; srcConnIdx < std::max(1, srcCandidateCount) && !exhaustiveSuccess; ++srcConnIdx) {
                        for (int tgtConnIdx = 0; tgtConnIdx < std::max(1, tgtCandidateCount) && !exhaustiveSuccess; ++tgtConnIdx) {
                            int srcCandidateIdx = 0, tgtCandidateIdx = 0;
                            Point newSrc = GridSnapCalculator::calculateSnapPosition(srcNode, srcEdge, srcConnIdx, std::max(1, srcCandidateCount), effectiveGridSize, &srcCandidateIdx);
                            Point newTgt = GridSnapCalculator::calculateSnapPosition(tgtNode, tgtEdge, tgtConnIdx, std::max(1, tgtCandidateCount), effectiveGridSize, &tgtCandidateIdx);

                            if (srcEdge == origSrcEdge && tgtEdge == origTgtEdge &&
                                srcCandidateIdx == originalSourceSnapIndex && tgtCandidateIdx == originalTargetSnapIndex) {
                                continue;
                            }

                            ObstacleMap obstacles;
                            obstacles.buildFromNodes(nodeLayouts, effectiveGridSize, 0);

                            GridPoint startGrid = obstacles.pixelToGrid(newSrc);
                            GridPoint goalGrid = obstacles.pixelToGrid(newTgt);

                            PathResult pathResult = pathFinder_.findPath(
                                startGrid, goalGrid, obstacles,
                                layout.from, layout.to,
                                srcEdge, tgtEdge,
                                {}, {});

                            if (pathResult.found && pathResult.path.size() >= 2) {
                                layout.sourceEdge = srcEdge;
                                layout.targetEdge = tgtEdge;
                                layout.sourcePoint = newSrc;
                                layout.targetPoint = newTgt;
                                layout.sourceSnapIndex = srcCandidateIdx;
                                layout.targetSnapIndex = tgtCandidateIdx;
                                layout.bendPoints.clear();

                                for (size_t i = 1; i + 1 < pathResult.path.size(); ++i) {
                                    const auto& gp = pathResult.path[i];
                                    Point pixelPoint = obstacles.gridToPixel(gp.x, gp.y);
                                    layout.bendPoints.push_back(BendPoint{pixelPoint, false});
                                }
                                exhaustiveSuccess = true;
                            }
                        }
                    }
                }
            }

            if (!exhaustiveSuccess) {
                layout.sourcePoint = originalSourcePoint;
                layout.targetPoint = originalTargetPoint;
                layout.sourceSnapIndex = originalSourceSnapIndex;
                layout.targetSnapIndex = originalTargetSnapIndex;
                layout.sourceEdge = origSrcEdge;
                layout.targetEdge = origTgtEdge;
                layout.bendPoints = originalBendPoints;
                return false;
            }
        }
    }

    return true;
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
