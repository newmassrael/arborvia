#include "AStarRetrySystem.h"
#include "../../sugiyama/routing/EdgeRoutingUtils.h"
#include "../../snap/GridSnapCalculator.h"
#include "../../snap/SnapIndexManager.h"
#include "../../pathfinding/ObstacleMap.h"
#include "arborvia/layout/api/IPathFinder.h"
#include "arborvia/core/GeometryUtils.h"

#include <array>
#include <iostream>

// Debug flag for retry system
#ifndef EDGE_ROUTING_DEBUG
#define EDGE_ROUTING_DEBUG 0
#endif

namespace arborvia {

// =============================================================================
// Constructor
// =============================================================================

AStarRetrySystem::AStarRetrySystem(IPathFinder& pathFinder, float gridSize)
    : pathFinder_(pathFinder)
    , gridSize_(gridSize) {}

// =============================================================================
// Main Entry Point
// =============================================================================

SnapRetryResult AStarRetrySystem::tryRetry(
    EdgeLayout& layout,
    std::unordered_map<EdgeId, EdgeLayout>& edgeLayouts,
    const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
    const std::unordered_map<EdgeId, EdgeLayout>* otherEdges,
    const SnapRetryConfig& config) {

#if EDGE_ROUTING_DEBUG
    std::cout << "[A* RETRY] Edge " << layout.id << " starting retry sequence" << std::endl;
#endif

    // Layer 1: Snap Index Retry (concentric expansion on same NodeEdge)
    auto result = trySnapIndexRetry(layout, edgeLayouts, nodeLayouts, otherEdges, config);
    if (result.success) {
#if EDGE_ROUTING_DEBUG
        std::cout << "[A* RETRY] Edge " << layout.id << " Layer 1 SUCCESS: snap index changed to "
                  << result.newSourceSnapIndex << std::endl;
#endif
        return result;
    }

    // Layer 2: Neighbor Adjustment (adjust neighbor transition coordinates)
    result = tryNeighborAdjustment(layout, edgeLayouts, nodeLayouts, otherEdges, config);
    if (result.success) {
#if EDGE_ROUTING_DEBUG
        std::cout << "[A* RETRY] Edge " << layout.id << " Layer 2 SUCCESS: neighbor "
                  << result.adjustedNeighborId << " adjusted" << std::endl;
#endif
        return result;
    }

#if EDGE_ROUTING_DEBUG
    std::cout << "[A* RETRY] Edge " << layout.id << " Layer 2 COMPLETED (failed), moving to Layer 3" << std::endl;
#endif

    // Layer 3: NodeEdge Switch (16 combinations, only if preserveDirections=false)
    if (config.enableNodeEdgeSwitch && !config.preserveDirections) {
        result = tryNodeEdgeSwitch(layout, nodeLayouts, otherEdges, config);
        if (result.success) {
#if EDGE_ROUTING_DEBUG
            std::cout << "[A* RETRY] Edge " << layout.id << " Layer 3 SUCCESS: NodeEdge switched to src="
                      << static_cast<int>(result.newSourceEdge) << " tgt="
                      << static_cast<int>(result.newTargetEdge) << std::endl;
#endif
            return result;
        }
    }

#if EDGE_ROUTING_DEBUG
    std::cout << "[A* RETRY] Edge " << layout.id << " ALL LAYERS FAILED: snapRetries="
              << result.snapRetryCount << " neighborAdjusts=" << result.neighborAdjustCount
              << " edgeCombinations=" << result.edgeCombinationCount << std::endl;
#endif

    return result;  // All retries failed
}

// =============================================================================
// Layer 1: Snap Index Retry
// =============================================================================

SnapRetryResult AStarRetrySystem::trySnapIndexRetry(
    EdgeLayout& layout,
    const std::unordered_map<EdgeId, EdgeLayout>& edgeLayouts,
    const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
    const std::unordered_map<EdgeId, EdgeLayout>* otherEdges,
    const SnapRetryConfig& config) {

    SnapRetryResult result;

    auto srcNodeIt = nodeLayouts.find(layout.from);
    if (srcNodeIt == nodeLayouts.end()) {
        return result;
    }

    const NodeLayout& srcNode = srcNodeIt->second;
    float effectiveGridSize = getEffectiveGridSize();

    // Get candidate count for this node-edge (grid-based, not connection count)
    int candidateCount = GridSnapCalculator::getCandidateCount(srcNode, layout.sourceEdge, effectiveGridSize);

#if EDGE_ROUTING_DEBUG
    auto srcConnections = SnapIndexManager::getConnections(edgeLayouts, layout.from, layout.sourceEdge);
    std::cout << "[A* RETRY] Edge " << layout.id << " trySnapIndexRetry: connectionCount="
              << srcConnections.totalCount() << " candidateCount=" << candidateCount
              << " currentIdx=" << layout.sourceSnapIndex
              << " sourceEdge=" << static_cast<int>(layout.sourceEdge) << std::endl;
#endif

    int currentIdx = layout.sourceSnapIndex;

    // Concentric expansion: try nearby candidate indices (+/-1, +/-2, etc.)
    for (int offset = 1; offset <= config.maxSnapRetries; ++offset) {
        for (int sign : {1, -1}) {
            int tryIdx = currentIdx + (offset * sign);
            // Validate against fixed candidate count
            if (tryIdx < 0 || tryIdx >= candidateCount) continue;

            // Check if this index is already used by another edge
            bool indexOccupied = false;
            for (const auto& [edgeId, otherLayout] : edgeLayouts) {
                if (edgeId == layout.id) continue;
                if (otherLayout.from == layout.from &&
                    otherLayout.sourceEdge == layout.sourceEdge &&
                    otherLayout.sourceSnapIndex == tryIdx) {
                    indexOccupied = true;
                    break;
                }
            }
            if (indexOccupied) continue;

            result.snapRetryCount++;

            // Get snap position from candidate index
            Point newSrcPoint = GridSnapCalculator::getPositionFromStoredIndex(
                srcNode, layout.sourceEdge, tryIdx, effectiveGridSize);

            // Create obstacle map and try A*
            ObstacleMap obstacles;
            obstacles.buildFromNodes(nodeLayouts, effectiveGridSize);
            GridPoint startGrid = obstacles.pixelToGrid(newSrcPoint);
            GridPoint goalGrid = obstacles.pixelToGrid(layout.targetPoint);

            // Don't add other edge segments as obstacles during retry
            PathResult pathResult = pathFinder_.findPath(
                startGrid, goalGrid, obstacles,
                layout.from, layout.to,
                layout.sourceEdge, layout.targetEdge,
                {}, {});

            if (pathResult.found && pathResult.path.size() >= 2) {
                result.success = true;
                result.snapIndexChanged = true;
                result.newSourceSnapIndex = tryIdx;
                result.newSourcePoint = newSrcPoint;
                result.needsFullReroute = true;
                result.bendPoints.clear();

                std::cout << "[DEBUG RETRY SUCCESS] Edge " << layout.id
                          << " candidateCount=" << candidateCount
                          << " snapIdx " << currentIdx << " -> " << tryIdx
                          << " newSrcPoint=(" << newSrcPoint.x << "," << newSrcPoint.y << ")"
                          << " needsFullReroute=true"
                          << std::endl;
                return result;
            }
        }
    }

#if EDGE_ROUTING_DEBUG
    std::cout << "[A* RETRY] Edge " << layout.id << " snap index retry FAILED after "
              << result.snapRetryCount << " attempts" << std::endl;
#endif

    return result;
}

// =============================================================================
// Layer 2: Neighbor Adjustment
// =============================================================================

SnapRetryResult AStarRetrySystem::tryNeighborAdjustment(
    EdgeLayout& layout,
    std::unordered_map<EdgeId, EdgeLayout>& edgeLayouts,
    const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
    const std::unordered_map<EdgeId, EdgeLayout>* otherEdges,
    const SnapRetryConfig& config) {

    SnapRetryResult result;
    float effectiveGridSize = getEffectiveGridSize();

    // Collect ALL neighbor edges (from same source OR same target node)
    std::vector<EdgeId> neighborEdges;
    for (const auto& [edgeId, edgeLayout] : edgeLayouts) {
        if (edgeId != layout.id) {
            if (edgeLayout.from == layout.from || edgeLayout.to == layout.to ||
                edgeLayout.from == layout.to || edgeLayout.to == layout.from) {
                neighborEdges.push_back(edgeId);
            }
        }
    }

    if (neighborEdges.empty()) {
#if EDGE_ROUTING_DEBUG
        std::cout << "[A* RETRY] Edge " << layout.id << " no neighbor edges to adjust" << std::endl;
#endif
        return result;
    }

#if EDGE_ROUTING_DEBUG
    std::cout << "[A* RETRY] Edge " << layout.id << " trying " << neighborEdges.size()
              << " potential blocking neighbors:";
    for (EdgeId nid : neighborEdges) {
        std::cout << " " << nid;
    }
    std::cout << std::endl;
#endif

    constexpr std::array<NodeEdge, 4> allNodeEdges = {
        NodeEdge::Top, NodeEdge::Bottom, NodeEdge::Left, NodeEdge::Right
    };

    // For each neighbor, try rerouting it to different NodeEdges
    for (EdgeId neighborId : neighborEdges) {
        auto neighborIt = edgeLayouts.find(neighborId);
        if (neighborIt == edgeLayouts.end()) continue;

        EdgeLayout& neighbor = neighborIt->second;

        // Save original state
        NodeEdge origSourceEdge = neighbor.sourceEdge;
        NodeEdge origTargetEdge = neighbor.targetEdge;
        Point origSourcePoint = neighbor.sourcePoint;
        Point origTargetPoint = neighbor.targetPoint;
        int origSourceSnapIdx = neighbor.sourceSnapIndex;
        int origTargetSnapIdx = neighbor.targetSnapIndex;
        auto origBends = neighbor.bendPoints;

        // Try different NodeEdge combinations for the neighbor
        for (NodeEdge newSrcEdge : allNodeEdges) {
            for (NodeEdge newTgtEdge : allNodeEdges) {
                if (newSrcEdge == origSourceEdge && newTgtEdge == origTargetEdge) {
                    continue;
                }

                result.neighborAdjustCount++;

                auto srcNodeIt = nodeLayouts.find(neighbor.from);
                auto tgtNodeIt = nodeLayouts.find(neighbor.to);
                if (srcNodeIt == nodeLayouts.end() || tgtNodeIt == nodeLayouts.end()) continue;

                const NodeLayout& srcNode = srcNodeIt->second;
                const NodeLayout& tgtNode = tgtNodeIt->second;

                // Count existing edges on the new source NodeEdge
                int srcEdgeCount = 0;
                int tgtEdgeCount = 0;
                for (const auto& [otherId, otherLayout] : edgeLayouts) {
                    if (otherId != neighborId) {
                        if (otherLayout.from == neighbor.from && otherLayout.sourceEdge == newSrcEdge)
                            srcEdgeCount++;
                        if (otherLayout.to == neighbor.to && otherLayout.targetEdge == newTgtEdge)
                            tgtEdgeCount++;
                    }
                }

                int newSrcConnectionIdx = srcEdgeCount;
                int newTgtConnectionIdx = tgtEdgeCount;
                int newSrcTotal = srcEdgeCount + 1;
                int newTgtTotal = tgtEdgeCount + 1;

                int newSrcCandidateIdx = 0;
                int newTgtCandidateIdx = 0;
                Point newSrcPoint = GridSnapCalculator::calculateSnapPosition(
                    srcNode, newSrcEdge, newSrcConnectionIdx, newSrcTotal, effectiveGridSize, &newSrcCandidateIdx);
                Point newTgtPoint = GridSnapCalculator::calculateSnapPosition(
                    tgtNode, newTgtEdge, newTgtConnectionIdx, newTgtTotal, effectiveGridSize, &newTgtCandidateIdx);

                // Try rerouting neighbor with new NodeEdges
                ObstacleMap neighborObstacles;
                neighborObstacles.buildFromNodes(nodeLayouts, effectiveGridSize);

                // Add all other edges (except neighbor, X, and edges with stale bendPoints)
                for (const auto& [otherId, otherLayout] : edgeLayouts) {
                    if (otherId != neighborId && otherId != layout.id) {
                        if (!hasFreshBendPoints(otherLayout, effectiveGridSize)) {
                            continue;
                        }
                        neighborObstacles.addEdgeSegments({{otherId, otherLayout}}, neighborId);
                    }
                }

                GridPoint neighborStart = neighborObstacles.pixelToGrid(newSrcPoint);
                GridPoint neighborGoal = neighborObstacles.pixelToGrid(newTgtPoint);

                PathResult neighborPath = pathFinder_.findPath(
                    neighborStart, neighborGoal, neighborObstacles,
                    neighbor.from, neighbor.to,
                    newSrcEdge, newTgtEdge,
                    {}, {});

                if (!neighborPath.found || neighborPath.path.size() < 2) {
                    continue;
                }

                // Build neighbor's new bendPoints
                std::vector<BendPoint> newNeighborBends;
                for (size_t i = 1; i + 1 < neighborPath.path.size(); ++i) {
                    Point pixelPoint = neighborObstacles.gridToPixel(neighborPath.path[i].x, neighborPath.path[i].y);
                    newNeighborBends.push_back({pixelPoint});
                }

                // Check neighbor's path is valid (no diagonal)
                bool neighborValid = true;
                if (!newNeighborBends.empty()) {
                    float dx = std::abs(newSrcPoint.x - newNeighborBends[0].position.x);
                    float dy = std::abs(newSrcPoint.y - newNeighborBends[0].position.y);
                    if (dx > 1.0f && dy > 1.0f) {
                        neighborValid = false;
                    }
                }
                if (!neighborValid) continue;

                // Temporarily apply neighbor's new route
                neighbor.sourceEdge = newSrcEdge;
                neighbor.targetEdge = newTgtEdge;
                neighbor.sourcePoint = newSrcPoint;
                neighbor.targetPoint = newTgtPoint;
                neighbor.sourceSnapIndex = newSrcCandidateIdx;
                neighbor.targetSnapIndex = newTgtCandidateIdx;
                neighbor.bendPoints = newNeighborBends;

                // Now try A* for X with neighbor's NEW path as obstacle
                auto xSrcNodeIt = nodeLayouts.find(layout.from);
                if (xSrcNodeIt == nodeLayouts.end()) continue;
                const NodeLayout& xSrcNode = xSrcNodeIt->second;

                int candidateCount = GridSnapCalculator::getCandidateCount(xSrcNode, layout.sourceEdge, effectiveGridSize);

                bool foundXPath = false;
                Point successSrcPoint = layout.sourcePoint;
                int successSnapIdx = layout.sourceSnapIndex;
                PathResult xPath;

                std::vector<int> tryIndices = {layout.sourceSnapIndex};
                for (int offset = 1; offset <= 2; ++offset) {
                    tryIndices.push_back(layout.sourceSnapIndex + offset);
                    tryIndices.push_back(layout.sourceSnapIndex - offset);
                }

                for (int xTryIdx : tryIndices) {
                    if (xTryIdx < 0 || xTryIdx >= candidateCount) continue;

                    Point xSrcPoint = (xTryIdx == layout.sourceSnapIndex)
                        ? layout.sourcePoint
                        : GridSnapCalculator::getPositionFromStoredIndex(xSrcNode, layout.sourceEdge, xTryIdx, effectiveGridSize);

                    ObstacleMap testObstacles;
                    testObstacles.buildFromNodes(nodeLayouts, effectiveGridSize);

                    for (const auto& [otherId, otherLayout] : edgeLayouts) {
                        if (otherId != layout.id) {
                            testObstacles.addEdgeSegments({{otherId, otherLayout}}, layout.id);
                        }
                    }

                    GridPoint startGrid = testObstacles.pixelToGrid(xSrcPoint);
                    GridPoint goalGrid = testObstacles.pixelToGrid(layout.targetPoint);

                    xPath = pathFinder_.findPath(
                        startGrid, goalGrid, testObstacles,
                        layout.from, layout.to,
                        layout.sourceEdge, layout.targetEdge,
                        {}, {});

                    if (xPath.found && xPath.path.size() >= 2) {
                        foundXPath = true;
                        successSrcPoint = xSrcPoint;
                        successSnapIdx = xTryIdx;
                        break;
                    }
                }

                if (!foundXPath) {
                    // Rollback neighbor
                    neighbor.sourceEdge = origSourceEdge;
                    neighbor.targetEdge = origTargetEdge;
                    neighbor.sourcePoint = origSourcePoint;
                    neighbor.targetPoint = origTargetPoint;
                    neighbor.sourceSnapIndex = origSourceSnapIdx;
                    neighbor.targetSnapIndex = origTargetSnapIdx;
                    neighbor.bendPoints = origBends;
                    continue;
                }

                // Build X's bendPoints
                ObstacleMap finalObstacles;
                finalObstacles.buildFromNodes(nodeLayouts, effectiveGridSize);
                for (const auto& [otherId, otherLayout] : edgeLayouts) {
                    if (otherId != layout.id) {
                        finalObstacles.addEdgeSegments({{otherId, otherLayout}}, layout.id);
                    }
                }

                std::vector<BendPoint> xBends;
                for (size_t i = 1; i + 1 < xPath.path.size(); ++i) {
                    Point pixelPoint = finalObstacles.gridToPixel(xPath.path[i].x, xPath.path[i].y);
                    xBends.push_back({pixelPoint});
                }

                // Check X's path is valid (no diagonal)
                bool xValid = true;
                if (!xBends.empty()) {
                    float dx = std::abs(successSrcPoint.x - xBends[0].position.x);
                    float dy = std::abs(successSrcPoint.y - xBends[0].position.y);
                    if (dx > 1.0f && dy > 1.0f) {
                        xValid = false;
                    }
                }

                if (xValid) {
                    result.success = true;
                    result.neighborAdjusted = true;
                    result.adjustedNeighborId = neighborId;
                    result.bendPoints = xBends;

                    if (successSnapIdx != layout.sourceSnapIndex) {
                        result.snapIndexChanged = true;
                        result.newSourceSnapIndex = successSnapIdx;
                    }
                    return result;
                }

                // Rollback neighbor
                neighbor.sourceEdge = origSourceEdge;
                neighbor.targetEdge = origTargetEdge;
                neighbor.sourcePoint = origSourcePoint;
                neighbor.targetPoint = origTargetPoint;
                neighbor.sourceSnapIndex = origSourceSnapIdx;
                neighbor.targetSnapIndex = origTargetSnapIdx;
                neighbor.bendPoints = origBends;
            }
        }
    }

#if EDGE_ROUTING_DEBUG
    std::cout << "[A* RETRY] Edge " << layout.id << " neighbor adjustment FAILED after "
              << result.neighborAdjustCount << " attempts" << std::endl;
#endif

    return result;
}

// =============================================================================
// Layer 3: NodeEdge Switch
// =============================================================================

SnapRetryResult AStarRetrySystem::tryNodeEdgeSwitch(
    EdgeLayout& layout,
    const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
    const std::unordered_map<EdgeId, EdgeLayout>* otherEdges,
    const SnapRetryConfig& config) {

    SnapRetryResult result;

    auto srcNodeIt = nodeLayouts.find(layout.from);
    auto tgtNodeIt = nodeLayouts.find(layout.to);
    if (srcNodeIt == nodeLayouts.end() || tgtNodeIt == nodeLayouts.end()) {
        return result;
    }

    const NodeLayout& srcNode = srcNodeIt->second;
    const NodeLayout& tgtNode = tgtNodeIt->second;
    float effectiveGridSize = getEffectiveGridSize();

#if EDGE_ROUTING_DEBUG
    std::cout << "[A* RETRY] Edge " << layout.id << " ENTERING tryNodeEdgeSwitch (Layer 3)" << std::endl;
#endif

    constexpr std::array<NodeEdge, 4> allEdges = {
        NodeEdge::Top, NodeEdge::Bottom, NodeEdge::Left, NodeEdge::Right
    };

    NodeEdge origSrcEdge = layout.sourceEdge;
    NodeEdge origTgtEdge = layout.targetEdge;

    for (NodeEdge srcEdge : allEdges) {
        for (NodeEdge tgtEdge : allEdges) {
            if (srcEdge == origSrcEdge && tgtEdge == origTgtEdge) {
                continue;
            }

            result.edgeCombinationCount++;

            int srcEdgeCount = 0;
            int tgtEdgeCount = 0;
            if (otherEdges) {
                srcEdgeCount = EdgeRoutingUtils::countEdgesOnNodeEdge(*otherEdges, layout.from, srcEdge, layout.id);
                tgtEdgeCount = EdgeRoutingUtils::countEdgesOnNodeEdge(*otherEdges, layout.to, tgtEdge, layout.id);
            }

            int srcCandidateCount = GridSnapCalculator::getCandidateCount(srcNode, srcEdge, effectiveGridSize);
            int tgtCandidateCount = GridSnapCalculator::getCandidateCount(tgtNode, tgtEdge, effectiveGridSize);

            int minPositions = 3;
            int srcEffectiveTotalCount = std::max(srcEdgeCount + 1, minPositions);
            int tgtEffectiveTotalCount = std::max(tgtEdgeCount + 1, minPositions);

            bool foundPath = false;
            Point successSrcPoint, successTgtPoint;
            int successSrcCandidateIdx = 0, successTgtCandidateIdx = 0;
            PathResult pathResult;

            for (int srcConnIdx = 0; srcConnIdx < srcEffectiveTotalCount && !foundPath; ++srcConnIdx) {
                for (int tgtConnIdx = 0; tgtConnIdx < tgtEffectiveTotalCount && !foundPath; ++tgtConnIdx) {
                    int srcCandidateIdx = 0, tgtCandidateIdx = 0;
                    Point srcPoint = GridSnapCalculator::calculateSnapPosition(srcNode, srcEdge, srcConnIdx, srcEffectiveTotalCount, effectiveGridSize, &srcCandidateIdx);
                    Point tgtPoint = GridSnapCalculator::calculateSnapPosition(tgtNode, tgtEdge, tgtConnIdx, tgtEffectiveTotalCount, effectiveGridSize, &tgtCandidateIdx);

                    ObstacleMap obstacles;
                    obstacles.buildFromNodes(nodeLayouts, effectiveGridSize);
                    GridPoint startGrid = obstacles.pixelToGrid(srcPoint);
                    GridPoint goalGrid = obstacles.pixelToGrid(tgtPoint);

                    if (otherEdges) {
                        std::unordered_map<EdgeId, EdgeLayout> freshEdges;
                        for (const auto& [edgeId, edgeLayout] : *otherEdges) {
                            if (hasFreshBendPoints(edgeLayout, effectiveGridSize)) {
                                freshEdges[edgeId] = edgeLayout;
                            }
                        }
                        obstacles.addEdgeSegments(freshEdges, layout.id);
                    }

                    pathResult = pathFinder_.findPath(
                        startGrid, goalGrid, obstacles,
                        layout.from, layout.to,
                        srcEdge, tgtEdge,
                        {}, {});

                    if (pathResult.found && pathResult.path.size() >= 2) {
                        foundPath = true;
                        successSrcPoint = srcPoint;
                        successTgtPoint = tgtPoint;
                        successSrcCandidateIdx = srcCandidateIdx;
                        successTgtCandidateIdx = tgtCandidateIdx;
                    }
                }
            }

            if (!foundPath) {
                continue;
            }

            result.success = true;
            result.nodeEdgeChanged = true;
            result.newSourceEdge = srcEdge;
            result.newTargetEdge = tgtEdge;
            result.newSourceSnapIndex = successSrcCandidateIdx;
            result.newTargetSnapIndex = successTgtCandidateIdx;
            result.snapIndexChanged = true;

            ObstacleMap coordObstacles;
            coordObstacles.buildFromNodes(nodeLayouts, effectiveGridSize);

            result.bendPoints.clear();
            for (size_t i = 1; i + 1 < pathResult.path.size(); ++i) {
                Point pixelPoint = coordObstacles.gridToPixel(pathResult.path[i].x, pathResult.path[i].y);
                result.bendPoints.push_back({pixelPoint});
            }

#if EDGE_ROUTING_DEBUG
            std::cout << "[A* RETRY] Edge " << layout.id << " NodeEdge switch: src="
                      << static_cast<int>(origSrcEdge) << "->" << static_cast<int>(srcEdge)
                      << " tgt=" << static_cast<int>(origTgtEdge) << "->" << static_cast<int>(tgtEdge)
                      << " candidateIdx=(" << successSrcCandidateIdx << "," << successTgtCandidateIdx << ")" << std::endl;
#endif
            return result;
        }
    }

#if EDGE_ROUTING_DEBUG
    std::cout << "[A* RETRY] Edge " << layout.id << " NodeEdge switch FAILED after "
              << result.edgeCombinationCount << " combinations" << std::endl;
#endif

    return result;
}

// =============================================================================
// Helper Methods
// =============================================================================

bool AStarRetrySystem::hasFreshBendPoints(const EdgeLayout& layout, float gridSize) {
    return EdgeRoutingUtils::hasFreshBendPoints(layout, gridSize);
}

float AStarRetrySystem::getEffectiveGridSize() const {
    return gridSize_ > 0.0f ? gridSize_ : constants::PATHFINDING_GRID_SIZE;
}

}  // namespace arborvia
