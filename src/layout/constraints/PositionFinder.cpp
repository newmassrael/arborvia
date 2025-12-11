#include "arborvia/layout/constraints/PositionFinder.h"
#include "arborvia/common/Logger.h"
#include "pathfinding/ObstacleMap.h"
#include "pathfinding/AStarPathFinder.h"

#include <cmath>
#include <queue>
#include <set>
#include <sstream>

namespace arborvia {

PositionFinder::PositionFinder(const Config& config)
    : config_(config) {}

ConstraintPlacementResult PositionFinder::placeNode(
    NodeId nodeId,
    Point desiredPosition,
    Size nodeSize,
    std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
    const std::unordered_map<EdgeId, EdgeLayout>& edgeLayouts,
    const Graph& graph) {

    // Step 1: Check if desired position is valid
    if (checkNoOverlap(nodeId, desiredPosition, nodeSize, nodeLayouts) &&
        validateAllEdgePaths(nodeId, desiredPosition, nodeSize, nodeLayouts, edgeLayouts, graph)) {
        
        // Desired position is valid - place directly
        NodeLayout layout;
        layout.id = nodeId;
        layout.position = desiredPosition;
        layout.size = nodeSize;
        nodeLayouts[nodeId] = layout;
        
        return ConstraintPlacementResult::ok(desiredPosition);
    }

    // Step 2: Desired position invalid - search for nearest valid position
    auto validPos = findNearestValidPosition(
        nodeId, desiredPosition, nodeSize, nodeLayouts, edgeLayouts, graph);

    if (!validPos.has_value()) {
        return ConstraintPlacementResult::fail(
            desiredPosition, 
            "Could not find valid position within search distance");
    }

    // Step 3: Place at the adjusted position
    NodeLayout layout;
    layout.id = nodeId;
    layout.position = *validPos;
    layout.size = nodeSize;
    nodeLayouts[nodeId] = layout;

    float distance = std::sqrt(
        std::pow(validPos->x - desiredPosition.x, 2) +
        std::pow(validPos->y - desiredPosition.y, 2));

    return ConstraintPlacementResult::adjusted(
        desiredPosition, *validPos,
        "Position adjusted by " + std::to_string(static_cast<int>(distance)) + " pixels to satisfy constraints");
}

bool PositionFinder::validateAllEdgePaths(
    NodeId nodeId,
    Point nodePosition,
    Size nodeSize,
    const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
    const std::unordered_map<EdgeId, EdgeLayout>& edgeLayouts,
    [[maybe_unused]] const Graph& graph) const {

    // Create temporary node layouts with the proposed position
    auto tempNodeLayouts = nodeLayouts;
    NodeLayout tempLayout;
    tempLayout.id = nodeId;
    tempLayout.position = nodePosition;
    tempLayout.size = nodeSize;
    tempNodeLayouts[nodeId] = tempLayout;

    // Build obstacle map with nodes
    // Include edge layouts in bounds calculation to prevent out-of-bounds segments
    ObstacleMap obstacles;
    obstacles.buildFromNodes(tempNodeLayouts, config_.gridSize, 0, &edgeLayouts);

    // Collect edges connected to this node for validation
    std::vector<EdgeId> connectedEdges;
    for (const auto& [edgeId, edgeLayout] : edgeLayouts) {
        if (edgeLayout.from == nodeId || edgeLayout.to == nodeId) {
            connectedEdges.push_back(edgeId);
        }
    }

    // Check all edges connected to this node
    AStarPathFinder pathFinder;

    for (EdgeId edgeId : connectedEdges) {
        auto it = edgeLayouts.find(edgeId);
        if (it == edgeLayouts.end()) continue;
        const EdgeLayout& edgeLayout = it->second;
        
        // Build obstacle map including other edges (but not the current edge being checked)
        // Include edge layouts in bounds calculation to prevent out-of-bounds segments
        ObstacleMap edgeObstacles;
        edgeObstacles.buildFromNodes(tempNodeLayouts, config_.gridSize, 0, &edgeLayouts);

        // Add all edges as obstacles (addEdgeSegments will exclude the current edge)
        edgeObstacles.addEdgeSegments(edgeLayouts, edgeId);

        // Calculate source and target points based on node position
        Point srcPoint, tgtPoint;
        NodeId srcNodeId, tgtNodeId;
        NodeEdge srcEdge, tgtEdge;

        if (edgeLayout.from == nodeId) {
            // This node is the source
            srcNodeId = nodeId;
            tgtNodeId = edgeLayout.to;
            srcEdge = edgeLayout.sourceEdge;
            tgtEdge = edgeLayout.targetEdge;
            
            // Recalculate source point based on new position
            srcPoint = calculateSnapPointOnEdge(tempLayout, srcEdge, edgeLayout.sourceSnapIndex);
            tgtPoint = edgeLayout.targetPoint;
        } else {
            // This node is the target
            srcNodeId = edgeLayout.from;
            tgtNodeId = nodeId;
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
        LOG_DEBUG("[CALLER:PositionFinder.cpp] A* findPath called");
        auto result = pathFinder.findPath(
            startGrid, goalGrid, edgeObstacles,
            srcNodeId, tgtNodeId,
            srcEdge, tgtEdge,
            {}, {});

        if (!result.found || result.path.size() < 2) {
            return false;  // No valid path for this edge
        }
    }

    return true;  // All edges have valid paths
}

bool PositionFinder::checkNoOverlap(
    NodeId nodeId,
    Point position,
    Size size,
    const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
    float margin) const {

    float x1 = position.x - margin;
    float y1 = position.y - margin;
    float x2 = position.x + size.width + margin;
    float y2 = position.y + size.height + margin;

    for (const auto& [otherId, otherNode] : nodeLayouts) {
        if (otherId == nodeId) continue;

        float ox1 = otherNode.position.x;
        float oy1 = otherNode.position.y;
        float ox2 = ox1 + otherNode.size.width;
        float oy2 = oy1 + otherNode.size.height;

        // AABB intersection check
        if (x1 < ox2 && x2 > ox1 && y1 < oy2 && y2 > oy1) {
            return false;  // Overlap detected
        }
    }

    return true;
}

std::optional<Point> PositionFinder::findNearestValidPosition(
    NodeId nodeId,
    Point center,
    Size nodeSize,
    const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
    const std::unordered_map<EdgeId, EdgeLayout>& edgeLayouts,
    const Graph& graph) const {

    // BFS-like search: try positions in increasing distance from center
    // Using a priority queue ordered by distance

    struct Candidate {
        Point position;
        float distance;
        bool operator>(const Candidate& other) const {
            return distance > other.distance;
        }
    };

    std::priority_queue<Candidate, std::vector<Candidate>, std::greater<Candidate>> queue;
    std::set<std::pair<int, int>> visited;

    // Snap center to grid
    float step = config_.searchStepSize;
    int centerGridX = static_cast<int>(std::round(center.x / step));
    int centerGridY = static_cast<int>(std::round(center.y / step));

    queue.push({center, 0.0f});
    visited.insert({centerGridX, centerGridY});

    int iterations = 0;
    float maxDist = config_.maxSearchDistance;

    while (!queue.empty() && iterations < config_.maxIterations) {
        auto current = queue.top();
        queue.pop();
        iterations++;

        // Check if this position is valid
        if (checkNoOverlap(nodeId, current.position, nodeSize, nodeLayouts) &&
            validateAllEdgePaths(nodeId, current.position, nodeSize, nodeLayouts, edgeLayouts, graph)) {
            return current.position;
        }

        // Add neighbors (4-directional)
        const int dx[] = {1, -1, 0, 0};
        const int dy[] = {0, 0, 1, -1};

        int currGridX = static_cast<int>(std::round(current.position.x / step));
        int currGridY = static_cast<int>(std::round(current.position.y / step));

        for (int i = 0; i < 4; i++) {
            int nx = currGridX + dx[i];
            int ny = currGridY + dy[i];

            if (visited.count({nx, ny})) continue;
            visited.insert({nx, ny});

            Point nextPos = {nx * step, ny * step};
            float dist = std::sqrt(
                std::pow(nextPos.x - center.x, 2) +
                std::pow(nextPos.y - center.y, 2));

            if (dist <= maxDist) {
                queue.push({nextPos, dist});
            }
        }
    }

    return std::nullopt;  // No valid position found
}

// Helper function to calculate snap point position on a node edge
Point PositionFinder::calculateSnapPointOnEdge(
    const NodeLayout& node,
    NodeEdge edge,
    int snapIndex) const {

    // Calculate position based on snap index
    // This is a simplified version - should match GridSnapCalculator logic
    float edgeLength;
    Point basePoint;

    switch (edge) {
        case NodeEdge::Top:
            edgeLength = node.size.width;
            basePoint = node.position;
            break;
        case NodeEdge::Bottom:
            edgeLength = node.size.width;
            basePoint = {node.position.x, node.position.y + node.size.height};
            break;
        case NodeEdge::Left:
            edgeLength = node.size.height;
            basePoint = node.position;
            break;
        case NodeEdge::Right:
            edgeLength = node.size.height;
            basePoint = {node.position.x + node.size.width, node.position.y};
            break;
    }

    // Calculate candidate count (excluding corners)
    int candidateCount = static_cast<int>(edgeLength / config_.gridSize) - 1;
    if (candidateCount < 1) candidateCount = 1;

    // Clamp snap index
    if (snapIndex < 0) snapIndex = 0;
    if (snapIndex >= candidateCount) snapIndex = candidateCount - 1;

    // Calculate position
    float offset = config_.gridSize * (snapIndex + 1);

    switch (edge) {
        case NodeEdge::Top:
        case NodeEdge::Bottom:
            return {basePoint.x + offset, basePoint.y};
        case NodeEdge::Left:
        case NodeEdge::Right:
            return {basePoint.x, basePoint.y + offset};
    }

    return basePoint;
}

}  // namespace arborvia
