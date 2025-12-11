#include "layout/constraints/EdgePathValidityConstraint.h"
#include "pathfinding/ObstacleMap.h"
#include "pathfinding/AStarPathFinder.h"

#include "arborvia/common/Logger.h"

#ifndef EDGE_ROUTING_DEBUG
#define EDGE_ROUTING_DEBUG 0
#endif

namespace arborvia {

EdgePathValidityConstraint::EdgePathValidityConstraint(float gridSize)
    : gridSize_(gridSize) {
}

Point EdgePathValidityConstraint::calculateSnapPointOnEdge(
    const NodeLayout& node,
    NodeEdge edge,
    int snapIndex) const {

    float x = node.position.x;
    float y = node.position.y;
    float w = node.size.width;
    float h = node.size.height;

    // Calculate position along the edge based on snap index
    // Snap index 0 is centered, negative is left/up, positive is right/down
    float offset = static_cast<float>(snapIndex) * gridSize_;

    switch (edge) {
        case NodeEdge::Top:
            return {x + w * 0.5f + offset, y};
        case NodeEdge::Bottom:
            return {x + w * 0.5f + offset, y + h};
        case NodeEdge::Left:
            return {x, y + h * 0.5f + offset};
        case NodeEdge::Right:
            return {x + w, y + h * 0.5f + offset};
    }
    return {x + w * 0.5f, y + h * 0.5f};
}

ConstraintResult EdgePathValidityConstraint::check(const ConstraintContext& ctx) const {
    auto nodeIt = ctx.nodeLayouts.find(ctx.nodeId);
    if (nodeIt == ctx.nodeLayouts.end()) {
        return ConstraintResult::ok();
    }

    // Create temporary node layouts with the proposed position
    auto tempNodeLayouts = ctx.nodeLayouts;
    NodeLayout tempLayout;
    tempLayout.id = ctx.nodeId;
    tempLayout.position = ctx.newPosition;
    tempLayout.size = nodeIt->second.size;
    tempNodeLayouts[ctx.nodeId] = tempLayout;

    // Build obstacle map with nodes
    // Include edge layouts in bounds calculation to prevent out-of-bounds segments
    ObstacleMap obstacles;
    obstacles.buildFromNodes(tempNodeLayouts, ctx.gridSize, 0, &ctx.edgeLayouts);

    // Collect edges connected to this node for validation
    std::vector<EdgeId> connectedEdges;
    std::vector<EdgeId> invalidEdges;
    
    for (const auto& [edgeId, edgeLayout] : ctx.edgeLayouts) {
        if (edgeLayout.from == ctx.nodeId || edgeLayout.to == ctx.nodeId) {
            connectedEdges.push_back(edgeId);
        }
    }

    // Check all edges connected to this node
    AStarPathFinder pathFinder;

    for (EdgeId edgeId : connectedEdges) {
        auto it = ctx.edgeLayouts.find(edgeId);
        if (it == ctx.edgeLayouts.end()) continue;
        const EdgeLayout& edgeLayout = it->second;
        
        // Build obstacle map including other edges (but not the current edge being checked)
        // Include edge layouts in bounds calculation to prevent out-of-bounds segments
        ObstacleMap edgeObstacles;
        edgeObstacles.buildFromNodes(tempNodeLayouts, ctx.gridSize, 0, &ctx.edgeLayouts);

        // Add all edges as obstacles (addEdgeSegments will exclude the current edge)
        edgeObstacles.addEdgeSegments(ctx.edgeLayouts, edgeId);

        // Calculate source and target points based on node position
        Point srcPoint, tgtPoint;
        NodeId srcNodeId, tgtNodeId;
        NodeEdge srcEdge, tgtEdge;

        if (edgeLayout.from == ctx.nodeId) {
            // This node is the source
            srcNodeId = ctx.nodeId;
            tgtNodeId = edgeLayout.to;
            srcEdge = edgeLayout.sourceEdge;
            tgtEdge = edgeLayout.targetEdge;
            
            // Recalculate source point based on new position
            srcPoint = calculateSnapPointOnEdge(tempLayout, srcEdge, edgeLayout.sourceSnapIndex);
            tgtPoint = edgeLayout.targetPoint;
        } else {
            // This node is the target
            srcNodeId = edgeLayout.from;
            tgtNodeId = ctx.nodeId;
            srcEdge = edgeLayout.sourceEdge;
            tgtEdge = edgeLayout.targetEdge;
            
            srcPoint = edgeLayout.sourcePoint;
            // Recalculate target point based on new position
            tgtPoint = calculateSnapPointOnEdge(tempLayout, tgtEdge, edgeLayout.targetSnapIndex);
        }

        // Convert to grid coordinates
        GridPoint startGrid = edgeObstacles.pixelToGrid(srcPoint);
        GridPoint goalGrid = edgeObstacles.pixelToGrid(tgtPoint);

        // Try A* pathfinding with edge obstacles
        LOG_DEBUG("[CALLER:EdgePathValidityConstraint.cpp] A* findPath called");
        auto result = pathFinder.findPath(
            startGrid, goalGrid, edgeObstacles,
            srcNodeId, tgtNodeId,
            srcEdge, tgtEdge,
            {}, {});

        if (!result.found || result.path.size() < 2) {
            invalidEdges.push_back(edgeId);
#if EDGE_ROUTING_DEBUG
            LOG_DEBUG("[EdgePathValidityConstraint] Edge {} INVALID: src=({},{}) tgt=({},{}) startGrid=({},{}) goalGrid=({},{}) found={} pathSize={}",
                      edgeId, srcPoint.x, srcPoint.y, tgtPoint.x, tgtPoint.y,
                      startGrid.x, startGrid.y, goalGrid.x, goalGrid.y,
                      result.found, result.path.size());
            // Check if start/goal are blocked
            std::unordered_set<NodeId> excludes = {srcNodeId, tgtNodeId};
            bool startBlocked = edgeObstacles.isBlocked(startGrid.x, startGrid.y, excludes);
            bool goalBlocked = edgeObstacles.isBlocked(goalGrid.x, goalGrid.y, excludes);
            LOG_DEBUG("[EdgePathValidityConstraint]   startBlocked={} goalBlocked={}", startBlocked, goalBlocked);
#endif
        }
    }

    if (!invalidEdges.empty()) {
#if EDGE_ROUTING_DEBUG
        LOG_DEBUG("[EdgePathValidityConstraint] Node {} at ({},{}) REJECTED: {} edge(s) have no valid path",
                  ctx.nodeId, ctx.newPosition.x, ctx.newPosition.y, invalidEdges.size());
#endif
        return ConstraintResult::failWithEdges(
            "No valid A* path for " + std::to_string(invalidEdges.size()) + " edge(s)",
            std::move(invalidEdges));
    }

    return ConstraintResult::ok();
}

}  // namespace arborvia
