#include "SelfLoopRouter.h"
#include "arborvia/layout/LayoutUtils.h"

namespace arborvia {

SelfLoopRouter::AdjacentEdges SelfLoopRouter::getAdjacentEdges(NodeEdge sourceEdge) {
    switch (sourceEdge) {
        case NodeEdge::Right:
            return {NodeEdge::Top, NodeEdge::Bottom};
        case NodeEdge::Left:
            return {NodeEdge::Bottom, NodeEdge::Top};
        case NodeEdge::Top:
            return {NodeEdge::Right, NodeEdge::Left};
        case NodeEdge::Bottom:
            return {NodeEdge::Left, NodeEdge::Right};
        default:
            return {NodeEdge::Top, NodeEdge::Bottom};
    }
}

bool SelfLoopRouter::isValidSelfLoopCombination(NodeEdge source, NodeEdge target) {
    // Same edge is forbidden
    if (source == target) {
        return false;
    }

    // Opposite edges are forbidden
    if ((source == NodeEdge::Left && target == NodeEdge::Right) ||
        (source == NodeEdge::Right && target == NodeEdge::Left) ||
        (source == NodeEdge::Top && target == NodeEdge::Bottom) ||
        (source == NodeEdge::Bottom && target == NodeEdge::Top)) {
        return false;
    }

    // Only adjacent edges are valid
    return true;
}

bool SelfLoopRouter::shouldBeAtLastIndex(NodeEdge thisEdge, NodeEdge otherEdge) {
    // Self-loops are placed at corners. This function determines if an endpoint
    // should be at the last index (corner position) or first index.
    //
    // Snap index ordering:
    // - Vertical edges (Left/Right): index 0 = top, max = bottom
    // - Horizontal edges (Top/Bottom): index 0 = left, max = right
    //
    // Corner placement rules:
    // - Right→Top: source at top of right (First), target at right of top (Last)
    // - Right→Bottom: source at bottom of right (Last), target at right of bottom (Last)
    // - Left→Top: source at top of left (First), target at left of top (First)
    // - Left→Bottom: source at bottom of left (Last), target at left of bottom (First)
    // - Top→Right: source at right of top (Last), target at top of right (First)
    // - Top→Left: source at left of top (First), target at top of left (First)
    // - Bottom→Right: source at right of bottom (Last), target at bottom of right (Last)
    // - Bottom→Left: source at left of bottom (First), target at bottom of left (Last)

    switch (thisEdge) {
        case NodeEdge::Right:
        case NodeEdge::Left:
            // Vertical edges: Last if other edge is Bottom
            return (otherEdge == NodeEdge::Bottom);

        case NodeEdge::Top:
        case NodeEdge::Bottom:
            // Horizontal edges: Last if other edge is Right
            return (otherEdge == NodeEdge::Right);

        default:
            return false;
    }
}

void SelfLoopRouter::applySelfLoopCornerPositioning(
    std::vector<std::pair<EdgeId, bool>>& connections,
    const std::unordered_map<EdgeId, EdgeLayout>& edgeLayouts,
    NodeEdge nodeEdge) {

    if (connections.size() <= 1) {
        return;
    }

    // Separate self-loop endpoints from other connections
    // Note: Multiple self-loops at the same position preserve their input order
    std::vector<std::pair<EdgeId, bool>> firstPositionEndpoints;  // Should be at index 0
    std::vector<std::pair<EdgeId, bool>> lastPositionEndpoints;   // Should be at last index
    std::vector<std::pair<EdgeId, bool>> otherConnections;

    // Reserve capacity to avoid reallocations
    const size_t maxSize = connections.size();
    firstPositionEndpoints.reserve(maxSize);
    lastPositionEndpoints.reserve(maxSize);
    otherConnections.reserve(maxSize);

    for (const auto& conn : connections) {
        auto [edgeId, isSource] = conn;
        auto it = edgeLayouts.find(edgeId);
        if (it == edgeLayouts.end()) {
            otherConnections.push_back(conn);
            continue;
        }

        const EdgeLayout& layout = it->second;
        if (layout.from == layout.to) {
            NodeEdge otherEdge = isSource ? layout.targetEdge : layout.sourceEdge;
            if (shouldBeAtLastIndex(nodeEdge, otherEdge)) {
                lastPositionEndpoints.push_back(conn);
            } else {
                firstPositionEndpoints.push_back(conn);
            }
        } else {
            otherConnections.push_back(conn);
        }
    }

    // Rebuild connections: first position endpoints + others + last position endpoints
    connections.clear();
    connections.insert(connections.end(), firstPositionEndpoints.begin(), firstPositionEndpoints.end());
    connections.insert(connections.end(), otherConnections.begin(), otherConnections.end());
    connections.insert(connections.end(), lastPositionEndpoints.begin(), lastPositionEndpoints.end());
}

float SelfLoopRouter::calculateSpacing(float gridSize) {
    return gridSize > 0.0f ? gridSize : DEFAULT_SPACING;
}

EdgeLayout SelfLoopRouter::route(
    EdgeId edgeId,
    NodeId from,
    const NodeLayout& nodeLayout,
    int loopIndex,
    const LayoutOptions& options) {

    EdgeLayout layout;
    layout.id = edgeId;
    layout.from = from;
    layout.to = from;  // Self-loop: source == target

    const auto& config = options.channelRouting.selfLoop;
    float offset = config.loopOffset + static_cast<float>(loopIndex) * config.stackSpacing;

    // Determine direction based on config or auto-detect
    SelfLoopDirection dir = config.preferredDirection;

    // For auto, default to Right (most common for state diagrams)
    if (dir == SelfLoopDirection::Auto) {
        dir = SelfLoopDirection::Right;
    }

    float gridSize = options.gridConfig.cellSize;
    float spacing = calculateSpacing(gridSize);

    // Node boundaries
    float nodeLeft = nodeLayout.position.x;
    float nodeRight = nodeLayout.position.x + nodeLayout.size.width;
    float nodeTop = nodeLayout.position.y;
    float nodeBottom = nodeLayout.position.y + nodeLayout.size.height;

    // CONSTRAINT: Self-loops must use adjacent edges (not same or opposite)
    // Each direction routes from one edge to an adjacent edge at a corner
    switch (dir) {
        case SelfLoopDirection::Right:
            // Right → Top (top-right corner)
            layout.sourceEdge = NodeEdge::Right;
            layout.targetEdge = NodeEdge::Top;

            // Source: Right edge, near top (corner position)
            layout.sourcePoint = {nodeRight, nodeTop + spacing};
            // Target: Top edge, near right (corner position)
            layout.targetPoint = {nodeRight - spacing, nodeTop};

            // Bend points: route around top-right corner
            layout.bendPoints.push_back({{nodeRight + offset, layout.sourcePoint.y}});
            layout.bendPoints.push_back({{nodeRight + offset, nodeTop - offset}});
            layout.bendPoints.push_back({{layout.targetPoint.x, nodeTop - offset}});
            break;

        case SelfLoopDirection::Left:
            // Left → Bottom (bottom-left corner)
            layout.sourceEdge = NodeEdge::Left;
            layout.targetEdge = NodeEdge::Bottom;

            // Source: Left edge, near bottom (corner position)
            layout.sourcePoint = {nodeLeft, nodeBottom - spacing};
            // Target: Bottom edge, near left (corner position)
            layout.targetPoint = {nodeLeft + spacing, nodeBottom};

            // Bend points: route around bottom-left corner
            layout.bendPoints.push_back({{nodeLeft - offset, layout.sourcePoint.y}});
            layout.bendPoints.push_back({{nodeLeft - offset, nodeBottom + offset}});
            layout.bendPoints.push_back({{layout.targetPoint.x, nodeBottom + offset}});
            break;

        case SelfLoopDirection::Top:
            // Top → Right (top-right corner)
            layout.sourceEdge = NodeEdge::Top;
            layout.targetEdge = NodeEdge::Right;

            // Source: Top edge, near right (corner position)
            layout.sourcePoint = {nodeRight - spacing, nodeTop};
            // Target: Right edge, near top (corner position)
            layout.targetPoint = {nodeRight, nodeTop + spacing};

            // Bend points: route around top-right corner
            layout.bendPoints.push_back({{layout.sourcePoint.x, nodeTop - offset}});
            layout.bendPoints.push_back({{nodeRight + offset, nodeTop - offset}});
            layout.bendPoints.push_back({{nodeRight + offset, layout.targetPoint.y}});
            break;

        case SelfLoopDirection::Bottom:
            // Bottom → Left (bottom-left corner)
            layout.sourceEdge = NodeEdge::Bottom;
            layout.targetEdge = NodeEdge::Left;

            // Source: Bottom edge, near left (corner position)
            layout.sourcePoint = {nodeLeft + spacing, nodeBottom};
            // Target: Left edge, near bottom (corner position)
            layout.targetPoint = {nodeLeft, nodeBottom - spacing};

            // Bend points: route around bottom-left corner
            layout.bendPoints.push_back({{layout.sourcePoint.x, nodeBottom + offset}});
            layout.bendPoints.push_back({{nodeLeft - offset, nodeBottom + offset}});
            layout.bendPoints.push_back({{nodeLeft - offset, layout.targetPoint.y}});
            break;

        case SelfLoopDirection::Auto:
            // Auto is converted to Right above, this case should never be reached
            break;
    }

    // Calculate label position
    layout.labelPosition = LayoutUtils::calculateEdgeLabelPosition(layout);

    return layout;
}

}  // namespace arborvia
