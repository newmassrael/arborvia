#include "PathCalculator.h"
#include "ObstacleMap.h"
#include "arborvia/layout/LayoutOptions.h"
#include <cmath>
#include <iostream>

#ifndef EDGE_ROUTING_DEBUG
#define EDGE_ROUTING_DEBUG 0
#endif

namespace arborvia {

namespace constants {
    constexpr float PATHFINDING_GRID_SIZE = 10.0f;
}

PathCalculator::PathCalculator(IPathFinder& pathFinder)
    : pathFinder_(pathFinder) {
}

bool PathCalculator::hasFreshBendPoints(const EdgeLayout& layout, float gridSize) {
    if (layout.bendPoints.empty()) {
        return true;  // No bendPoints = direct connection, considered fresh
    }

    // Check source side: sourcePoint -> firstBend
    const Point& src = layout.sourcePoint;
    const Point& firstBend = layout.bendPoints[0].position;
    float dx_src = std::abs(src.x - firstBend.x);
    float dy_src = std::abs(src.y - firstBend.y);
    bool sourceDiagonal = (dx_src > gridSize && dy_src > gridSize);

    // Check target side: lastBend -> targetPoint
    const Point& lastBend = layout.bendPoints.back().position;
    const Point& tgt = layout.targetPoint;
    float dx_tgt = std::abs(lastBend.x - tgt.x);
    float dy_tgt = std::abs(lastBend.y - tgt.y);
    bool targetDiagonal = (dx_tgt > gridSize && dy_tgt > gridSize);

    // If either side has diagonal, bendPoints are stale
    return !(sourceDiagonal || targetDiagonal);
}

void PathCalculator::handleSelfLoop(
    EdgeLayout& layout,
    const NodeLayout& srcNode,
    float effectiveGridSize) {
    
    constexpr float BASE_OFFSET = 30.0f;
    
    // Generate orthogonal path for self-loop
    Point srcExt, tgtExt;
    
    // Calculate source extension point (perpendicular to source edge)
    switch (layout.sourceEdge) {
        case NodeEdge::Top:
            srcExt = {layout.sourcePoint.x, layout.sourcePoint.y - BASE_OFFSET};
            break;
        case NodeEdge::Bottom:
            srcExt = {layout.sourcePoint.x, layout.sourcePoint.y + BASE_OFFSET};
            break;
        case NodeEdge::Left:
            srcExt = {layout.sourcePoint.x - BASE_OFFSET, layout.sourcePoint.y};
            break;
        case NodeEdge::Right:
            srcExt = {layout.sourcePoint.x + BASE_OFFSET, layout.sourcePoint.y};
            break;
    }
    
    // Calculate target extension point (perpendicular to target edge)
    switch (layout.targetEdge) {
        case NodeEdge::Top:
            tgtExt = {layout.targetPoint.x, layout.targetPoint.y - BASE_OFFSET};
            break;
        case NodeEdge::Bottom:
            tgtExt = {layout.targetPoint.x, layout.targetPoint.y + BASE_OFFSET};
            break;
        case NodeEdge::Left:
            tgtExt = {layout.targetPoint.x - BASE_OFFSET, layout.targetPoint.y};
            break;
        case NodeEdge::Right:
            tgtExt = {layout.targetPoint.x + BASE_OFFSET, layout.targetPoint.y};
            break;
    }
    
    // Snap to grid
    srcExt.x = std::round(srcExt.x / effectiveGridSize) * effectiveGridSize;
    srcExt.y = std::round(srcExt.y / effectiveGridSize) * effectiveGridSize;
    tgtExt.x = std::round(tgtExt.x / effectiveGridSize) * effectiveGridSize;
    tgtExt.y = std::round(tgtExt.y / effectiveGridSize) * effectiveGridSize;
    
    layout.bendPoints.clear();
    
    // Check if we need a corner point
    constexpr float EPSILON = 1.0f;
    if (std::abs(srcExt.x - tgtExt.x) > EPSILON && std::abs(srcExt.y - tgtExt.y) > EPSILON) {
        // Need corner - pick the one outside the node
        Point cornerA = {srcExt.x, tgtExt.y};
        Point cornerB = {tgtExt.x, srcExt.y};
        
        float nodeLeft = srcNode.position.x;
        float nodeRight = srcNode.position.x + srcNode.size.width;
        float nodeTop = srcNode.position.y;
        float nodeBottom = srcNode.position.y + srcNode.size.height;
        
        bool cornerAInside = (cornerA.x >= nodeLeft && cornerA.x <= nodeRight &&
                              cornerA.y >= nodeTop && cornerA.y <= nodeBottom);
        
        Point corner = cornerAInside ? cornerB : cornerA;
        
        layout.bendPoints.push_back({srcExt});
        layout.bendPoints.push_back({corner});
        layout.bendPoints.push_back({tgtExt});
    } else {
        // srcExt and tgtExt are aligned
        layout.bendPoints.push_back({srcExt});
        layout.bendPoints.push_back({tgtExt});
    }
    
#if EDGE_ROUTING_DEBUG
    std::cout << "[PathCalculator] Edge " << layout.id << " SELF-LOOP: "
              << "src=(" << layout.sourcePoint.x << "," << layout.sourcePoint.y << ") "
              << "tgt=(" << layout.targetPoint.x << "," << layout.targetPoint.y << ") "
              << "bends=" << layout.bendPoints.size() << std::endl;
#endif
}

bool PathCalculator::tryAStarPathfinding(
    EdgeLayout& layout,
    const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
    float effectiveGridSize,
    const std::unordered_map<EdgeId, EdgeLayout>* otherEdges) {
    
    // Get source node
    auto srcNodeIt = nodeLayouts.find(layout.from);
    if (srcNodeIt == nodeLayouts.end()) {
        return false;
    }
    const NodeLayout& srcNode = srcNodeIt->second;
    
    // Calculate edge length for snap positions
    float edgeLength = 0.0f;
    if (layout.sourceEdge == NodeEdge::Top || layout.sourceEdge == NodeEdge::Bottom) {
        edgeLength = srcNode.size.width;
    } else {
        edgeLength = srcNode.size.height;
    }
    
    // Generate snap positions to try
    std::vector<float> snapPositions;
    
    // Calculate current position ratio
    float currentRatio = 0.5f;
    if (layout.sourceEdge == NodeEdge::Top || layout.sourceEdge == NodeEdge::Bottom) {
        currentRatio = (layout.sourcePoint.x - srcNode.position.x) / edgeLength;
    } else {
        currentRatio = (layout.sourcePoint.y - srcNode.position.y) / edgeLength;
    }
    
    // Add current position first
    snapPositions.push_back(currentRatio);
    
    // Add alternative positions
    std::vector<float> alternatives = {0.2f, 0.4f, 0.5f, 0.6f, 0.8f, 0.1f, 0.9f, 0.3f, 0.7f};
    for (float alt : alternatives) {
        if (std::abs(alt - currentRatio) > 0.05f) {
            snapPositions.push_back(alt);
        }
    }
    
    Point originalSourcePoint = layout.sourcePoint;
    
    for (size_t attempt = 0; attempt < snapPositions.size(); ++attempt) {
        float ratio = snapPositions[attempt];
        
        // Calculate source point for this ratio
        Point trySourcePoint;
        if (layout.sourceEdge == NodeEdge::Top) {
            trySourcePoint = {srcNode.position.x + edgeLength * ratio, srcNode.position.y};
        } else if (layout.sourceEdge == NodeEdge::Bottom) {
            trySourcePoint = {srcNode.position.x + edgeLength * ratio, srcNode.position.y + srcNode.size.height};
        } else if (layout.sourceEdge == NodeEdge::Left) {
            trySourcePoint = {srcNode.position.x, srcNode.position.y + edgeLength * ratio};
        } else {
            trySourcePoint = {srcNode.position.x + srcNode.size.width, srcNode.position.y + edgeLength * ratio};
        }
        
        // Quantize to grid
        trySourcePoint.x = std::round(trySourcePoint.x / effectiveGridSize) * effectiveGridSize;
        trySourcePoint.y = std::round(trySourcePoint.y / effectiveGridSize) * effectiveGridSize;
        
        // Build obstacle map
        ObstacleMap obstacles;
        obstacles.buildFromNodes(nodeLayouts, effectiveGridSize, 0);
        
        GridPoint startGrid = obstacles.pixelToGrid(trySourcePoint);
        GridPoint goalGrid = obstacles.pixelToGrid(layout.targetPoint);
        
        // Find additional nodes to exclude
        std::unordered_set<NodeId> extraStartExcludes;
        std::unordered_set<NodeId> extraGoalExcludes;
        
        for (const auto& [nodeId, node] : nodeLayouts) {
            if (nodeId == layout.from || nodeId == layout.to) continue;
            
            if (trySourcePoint.x >= node.position.x &&
                trySourcePoint.x <= node.position.x + node.size.width &&
                trySourcePoint.y >= node.position.y &&
                trySourcePoint.y <= node.position.y + node.size.height) {
                extraStartExcludes.insert(nodeId);
            }
            
            if (layout.targetPoint.x >= node.position.x &&
                layout.targetPoint.x <= node.position.x + node.size.width &&
                layout.targetPoint.y >= node.position.y &&
                layout.targetPoint.y <= node.position.y + node.size.height) {
                extraGoalExcludes.insert(nodeId);
            }
        }
        
        // Add other edges as obstacles
        if (otherEdges) {
            std::unordered_map<EdgeId, EdgeLayout> freshEdges;
            for (const auto& [edgeId, edgeLayout] : *otherEdges) {
                if (hasFreshBendPoints(edgeLayout, effectiveGridSize)) {
                    freshEdges[edgeId] = edgeLayout;
                }
            }
            obstacles.addEdgeSegments(freshEdges, layout.id);
        }
        
        // Try A* pathfinding
        PathResult pathResult = pathFinder_.findPath(
            startGrid, goalGrid, obstacles,
            layout.from, layout.to,
            layout.sourceEdge, layout.targetEdge,
            extraStartExcludes, extraGoalExcludes);
        
        if (pathResult.found && pathResult.path.size() >= 2) {
            layout.sourcePoint = trySourcePoint;
            layout.bendPoints.clear();
            for (size_t i = 1; i + 1 < pathResult.path.size(); ++i) {
                Point pixelPoint = obstacles.gridToPixel(pathResult.path[i].x, pathResult.path[i].y);
                layout.bendPoints.push_back({pixelPoint});
            }
            
#if EDGE_ROUTING_DEBUG
            std::cout << "[PathCalculator] Edge " << layout.id << " SUCCESS"
                      << (attempt > 0 ? " (after retry)" : "")
                      << ": path.size=" << pathResult.path.size()
                      << " bends=" << layout.bendPoints.size() << std::endl;
#endif
            return true;
        }
    }
    
    // All attempts failed
    layout.sourcePoint = originalSourcePoint;
    layout.bendPoints.clear();
    
#if EDGE_ROUTING_DEBUG
    std::cout << "[PathCalculator] Edge " << layout.id << " ALL RETRIES FAILED!" << std::endl;
#endif
    return false;
}

void PathCalculator::recalculateBendPoints(
    EdgeLayout& layout,
    const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
    float gridSize,
    const std::unordered_map<EdgeId, EdgeLayout>* otherEdges) {
    
    float effectiveGridSize = gridSize > 0.0f ? gridSize : constants::PATHFINDING_GRID_SIZE;
    
    auto srcNodeIt = nodeLayouts.find(layout.from);
    if (srcNodeIt == nodeLayouts.end()) {
        return;
    }
    const NodeLayout& srcNode = srcNodeIt->second;
    
    // Handle self-loops specially
    if (layout.from == layout.to) {
        handleSelfLoop(layout, srcNode, effectiveGridSize);
        return;
    }
    
    // Regular edge - use A* pathfinding
    tryAStarPathfinding(layout, nodeLayouts, effectiveGridSize, otherEdges);
}

} // namespace arborvia
