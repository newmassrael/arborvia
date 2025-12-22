#include "arborvia/layout/api/EdgeLayoutResolver.h"
#include "arborvia/layout/api/IPathFinder.h"
#include "arborvia/layout/util/LayoutUtils.h"
#include "../snap/GridSnapCalculator.h"
#include "../pathfinding/ObstacleMap.h"
#include "../routing/UnifiedRetryChain.h"
#include "arborvia/common/Logger.h"

namespace arborvia {

EdgeLayoutResolver::Result EdgeLayoutResolver::resolve(
    EdgeLayout& layout,
    const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
    float gridSize,
    std::shared_ptr<IPathFinder> pathFinder) {

    // Step 1: Resolve snap points (always succeeds if nodes exist)
    auto snapResult = resolveSnapPointsOnly(layout, nodeLayouts, gridSize);
    if (!snapResult.success) {
        return snapResult;
    }

    // Step 2: Compute bendPoints
    if (pathFinder) {
        // Use A* pathfinding for bend points
        if (!computeBendPoints(layout, nodeLayouts, pathFinder)) {
            // A* failed - clear bendPoints to avoid orphaned data
            layout.bendPoints.clear();
            LOG_WARN("[EdgeLayoutResolver] A* failed for edge {}->{}, bendPoints cleared",
                     layout.from, layout.to);
            // Still return success - edge is valid, just has no bends
        }
    } else {
        // No pathfinder - clear bendPoints (straight line)
        layout.bendPoints.clear();
    }

    // Step 3: Compute label position
    computeLabelPosition(layout);

    return Result::ok();
}

EdgeLayoutResolver::Result EdgeLayoutResolver::resolveSnapPointsOnly(
    EdgeLayout& layout,
    const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
    float gridSize,
    const std::unordered_set<NodeId>* movedNodes) {

    // Find source node
    auto srcIt = nodeLayouts.find(layout.from);
    if (srcIt == nodeLayouts.end()) {
        return Result::fail("Source node not found: " + std::to_string(layout.from));
    }
    const NodeLayout& srcNode = srcIt->second;

    // Find target node
    auto tgtIt = nodeLayouts.find(layout.to);
    if (tgtIt == nodeLayouts.end()) {
        return Result::fail("Target node not found: " + std::to_string(layout.to));
    }
    const NodeLayout& tgtNode = tgtIt->second;

    // Determine which sides should be updated based on movedNodes
    bool srcMoved = isNodeMoved(movedNodes, layout.from);
    bool tgtMoved = isNodeMoved(movedNodes, layout.to);

    // Compute sourcePoint from sourceSnapIndex (only if source node moved)
    if (srcMoved) {
        if (layout.sourceSnapIndex >= 0) {
            layout.sourcePoint = GridSnapCalculator::getPositionFromStoredIndex(
                srcNode, layout.sourceEdge, layout.sourceSnapIndex, gridSize);
        } else {
            // Fallback: use center of node edge
            layout.sourcePoint = GridSnapCalculator::computeSnapPointFromRatio(
                srcNode, layout.sourceEdge, 0.5f, gridSize);
        }
    }

    // Compute targetPoint from targetSnapIndex (only if target node moved)
    if (tgtMoved) {
        if (layout.targetSnapIndex >= 0) {
            layout.targetPoint = GridSnapCalculator::getPositionFromStoredIndex(
                tgtNode, layout.targetEdge, layout.targetSnapIndex, gridSize);
        } else {
            // Fallback: use center of node edge
            layout.targetPoint = GridSnapCalculator::computeSnapPointFromRatio(
                tgtNode, layout.targetEdge, 0.5f, gridSize);
        }
    }

    // Store gridSize for future operations
    layout.usedGridSize = gridSize;

    return Result::ok();
}

std::vector<EdgeId> EdgeLayoutResolver::resolveAll(
    std::unordered_map<EdgeId, EdgeLayout>& layouts,
    const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
    float gridSize,
    std::shared_ptr<IPathFinder> pathFinder) {

    std::vector<EdgeId> failed;
    for (auto& [edgeId, layout] : layouts) {
        auto result = resolve(layout, nodeLayouts, gridSize, pathFinder);
        if (!result.success) {
            LOG_ERROR("[EdgeLayoutResolver] Failed to resolve edge {}: {}",
                      edgeId, result.error);
            failed.push_back(edgeId);
        }
    }
    return failed;
}

std::vector<EdgeId> EdgeLayoutResolver::resolveEdges(
    std::unordered_map<EdgeId, EdgeLayout>& layouts,
    const std::vector<EdgeId>& edgeIds,
    const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
    float gridSize,
    std::shared_ptr<IPathFinder> pathFinder) {

    std::vector<EdgeId> failed;
    for (EdgeId edgeId : edgeIds) {
        auto it = layouts.find(edgeId);
        if (it == layouts.end()) {
            LOG_WARN("[EdgeLayoutResolver] Edge {} not found in layouts", edgeId);
            continue;
        }

        auto result = resolve(it->second, nodeLayouts, gridSize, pathFinder);
        if (!result.success) {
            LOG_ERROR("[EdgeLayoutResolver] Failed to resolve edge {}: {}",
                      edgeId, result.error);
            failed.push_back(edgeId);
        }
    }
    return failed;
}

bool EdgeLayoutResolver::validate(
    const EdgeLayout& layout,
    const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
    float gridSize,
    float tolerance) {

    // Create a temporary copy to compute expected values
    EdgeLayout expected = layout;
    auto result = resolveSnapPointsOnly(expected, nodeLayouts, gridSize);
    if (!result.success) {
        LOG_WARN("[EdgeLayoutResolver::validate] Cannot validate edge {}: {}",
                 layout.id, result.error);
        return false;
    }

    // Compare sourcePoint
    float srcDist = layout.sourcePoint.distanceTo(expected.sourcePoint);
    if (srcDist > tolerance) {
        LOG_WARN("[EdgeLayoutResolver::validate] Edge {} sourcePoint mismatch: "
                 "actual=({},{}) expected=({},{})",
                 layout.id,
                 layout.sourcePoint.x, layout.sourcePoint.y,
                 expected.sourcePoint.x, expected.sourcePoint.y);
        return false;
    }

    // Compare targetPoint
    float tgtDist = layout.targetPoint.distanceTo(expected.targetPoint);
    if (tgtDist > tolerance) {
        LOG_WARN("[EdgeLayoutResolver::validate] Edge {} targetPoint mismatch: "
                 "actual=({},{}) expected=({},{})",
                 layout.id,
                 layout.targetPoint.x, layout.targetPoint.y,
                 expected.targetPoint.x, expected.targetPoint.y);
        return false;
    }

    return true;
}

bool EdgeLayoutResolver::computeBendPoints(
    EdgeLayout& layout,
    const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
    std::shared_ptr<IPathFinder> pathFinder) {

    // Self-loops need special handling - skip A* for now
    if (layout.from == layout.to) {
        layout.bendPoints.clear();
        return true;
    }

    float gridSize = layout.usedGridSize > 0 ? layout.usedGridSize : 20.0f;

    // Build obstacle map
    ObstacleMap obstacles;
    obstacles.buildFromNodes(nodeLayouts, gridSize, 0);

    // Convert to grid coordinates
    GridPoint startGrid = GridPoint::fromPixel(layout.sourcePoint, gridSize);
    GridPoint goalGrid = GridPoint::fromPixel(layout.targetPoint, gridSize);

    // Find path using A*
    auto pathResult = pathFinder->findPath(
        startGrid, goalGrid, obstacles,
        layout.from, layout.to,
        layout.sourceEdge, layout.targetEdge);

    if (!pathResult.found) {
        LOG_DEBUG("[EdgeLayoutResolver] A* path not found for edge {}->{}",
                  layout.from, layout.to);
        return false;
    }

    if (pathResult.path.size() < 2) {
        LOG_DEBUG("[EdgeLayoutResolver] A* path too short for edge {}->{}",
                  layout.from, layout.to);
        return false;
    }

    // Convert grid path to pixel bend points
    // Skip first (source) and last (target) points
    layout.bendPoints.clear();
    for (size_t i = 1; i + 1 < pathResult.path.size(); ++i) {
        Point pixelPos = pathResult.path[i].toPixel(gridSize);
        layout.bendPoints.push_back(BendPoint{pixelPos});
    }

    return true;
}

void EdgeLayoutResolver::computeLabelPosition(EdgeLayout& layout) {
    layout.labelPosition = LayoutUtils::calculateEdgeLabelPosition(layout);
}

EdgeLayoutResolver::Result EdgeLayoutResolver::resolveWithRetry(
    EdgeId edgeId,
    EdgeLayout& layout,
    std::unordered_map<EdgeId, EdgeLayout>& otherEdges,
    const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
    float gridSize,
    std::shared_ptr<IPathFinder> pathFinder,
    const std::unordered_set<NodeId>* movedNodes) {

    if (!pathFinder) {
        return Result::fail("PathFinder is required for resolveWithRetry");
    }

    // Step 1: Compute snapPoints from snapIndex (respecting movedNodes constraint)
    auto snapResult = resolveSnapPointsOnly(layout, nodeLayouts, gridSize, movedNodes);
    if (!snapResult.success) {
        return snapResult;
    }

    // Step 2: Compute bendPoints using UnifiedRetryChain
    UnifiedRetryChain retryChain(pathFinder, gridSize);
    UnifiedRetryChain::RetryConfig config;
    config.gridSize = gridSize;
    config.movedNodes = movedNodes;

    auto retryResult = retryChain.calculatePath(edgeId, layout, otherEdges, nodeLayouts, config);

    if (retryResult.success) {
        // Save snapPoints computed in Step 1 (respecting movedNodes)
        Point savedSourcePoint = layout.sourcePoint;
        Point savedTargetPoint = layout.targetPoint;
        int savedSourceSnapIndex = layout.sourceSnapIndex;
        int savedTargetSnapIndex = layout.targetSnapIndex;

        // Copy entire layout from retry result (includes bendPoints)
        layout = retryResult.layout;

        // Restore snapPoints for unmoved nodes
        bool srcMoved = isNodeMoved(movedNodes, layout.from);
        bool tgtMoved = isNodeMoved(movedNodes, layout.to);

        if (!srcMoved) {
            layout.sourcePoint = savedSourcePoint;
            layout.sourceSnapIndex = savedSourceSnapIndex;
        }
        if (!tgtMoved) {
            layout.targetPoint = savedTargetPoint;
            layout.targetSnapIndex = savedTargetSnapIndex;
        }

        // Ensure orthogonality after snapPoint adjustments
        // bendPoints may not align with restored snapPoints
        retryChain.ensureEndpointOrthogonality(layout);

        LOG_DEBUG("[EdgeLayoutResolver] resolveWithRetry succeeded for edge {} "
                  "(astar={}, coop={}, bends={})",
                  edgeId, retryResult.astarAttempts, retryResult.cooperativeAttempts,
                  layout.bendPoints.size());
    } else {
        // A* failed even with retries - ensure orthogonality as fallback
        LOG_WARN("[EdgeLayoutResolver] resolveWithRetry failed for edge {}: {}, "
                 "applying orthogonality fix",
                 edgeId, retryResult.failureReason);
        retryChain.ensureEndpointOrthogonality(layout);
    }

    // Step 3: Compute label position
    computeLabelPosition(layout);

    return Result::ok();
}

std::vector<EdgeId> EdgeLayoutResolver::resolveEdgesWithRetry(
    std::unordered_map<EdgeId, EdgeLayout>& layouts,
    const std::vector<EdgeId>& edgeIds,
    const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
    float gridSize,
    std::shared_ptr<IPathFinder> pathFinder,
    const std::unordered_set<NodeId>* movedNodes) {

    std::vector<EdgeId> failed;

    // Build otherEdges map (all edges not in edgeIds)
    std::unordered_set<EdgeId> targetEdgeSet(edgeIds.begin(), edgeIds.end());
    std::unordered_map<EdgeId, EdgeLayout> otherEdges;
    for (const auto& [eid, elayout] : layouts) {
        if (targetEdgeSet.find(eid) == targetEdgeSet.end()) {
            otherEdges[eid] = elayout;
        }
    }

    // Resolve each edge
    for (EdgeId edgeId : edgeIds) {
        auto it = layouts.find(edgeId);
        if (it == layouts.end()) {
            LOG_WARN("[EdgeLayoutResolver] Edge {} not found in layouts", edgeId);
            continue;
        }

        auto result = resolveWithRetry(
            edgeId, it->second, otherEdges, nodeLayouts,
            gridSize, pathFinder, movedNodes);

        if (!result.success) {
            LOG_ERROR("[EdgeLayoutResolver] Failed to resolve edge {}: {}",
                      edgeId, result.error);
            failed.push_back(edgeId);
        } else {
            // Update otherEdges with the resolved layout for subsequent edges
            otherEdges[edgeId] = it->second;
        }
    }

    return failed;
}

}  // namespace arborvia
