#include "SelfLoopRouter.h"
#include "arborvia/layout/util/LayoutUtils.h"
#include "../../snap/GridSnapCalculator.h"
#include "arborvia/common/Logger.h"

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

    // Determine direction based on config or auto-detect
    SelfLoopDirection dir = config.preferredDirection;
    if (dir == SelfLoopDirection::Auto) {
        dir = SelfLoopDirection::Right;
    }

    float gridSize = options.gridConfig.cellSize;
    if (gridSize <= 0.0f) {
        gridSize = DEFAULT_SPACING;  // Fallback
    }

    // === ALL CALCULATIONS IN GRID UNITS (integers) ===
    // Convert pixel to grid: round(pixel / gridSize)
    // Convert grid to pixel: grid * gridSize
    auto toGrid = [gridSize](float pixel) -> int {
        return static_cast<int>(std::round(pixel / gridSize));
    };
    auto toPixel = [gridSize](int grid) -> float {
        return grid * gridSize;
    };

    // Node boundaries in grid units
    int gNodeLeft = toGrid(nodeLayout.position.x);
    int gNodeRight = toGrid(nodeLayout.position.x + nodeLayout.size.width);
    int gNodeTop = toGrid(nodeLayout.position.y);
    int gNodeBottom = toGrid(nodeLayout.position.y + nodeLayout.size.height);

    // Spacing in grid units (minimum 1 cell)
    int gSpacing = std::max(1, toGrid(calculateSpacing(gridSize)));

    // Loop offset in grid units (already stored as grid units)
    int gLoopOffset = std::max(1, config.loopOffsetGrids);
    int gStackSpacing = std::max(0, config.stackSpacingGrids);
    int gSnapOffset = loopIndex * gStackSpacing;

    // Limit snap offset to 40% of smaller dimension
    int maxSnapOffset = std::min(gNodeRight - gNodeLeft, gNodeBottom - gNodeTop) * 4 / 10;
    gSnapOffset = std::min(gSnapOffset, maxSnapOffset);

    // Total offset from node edge for bend points
    int gOffset = gLoopOffset + gSnapOffset;

    // CONSTRAINT: Self-loops must use adjacent edges (not same or opposite)
    // Each direction routes from one edge to an adjacent edge at a corner
    // Reserve space for 3 bend points (all cases use exactly 3)
    layout.bendPoints.reserve(3);

    // Grid coordinates for source, target, and bend points
    int gSrcX = 0, gSrcY = 0, gTgtX = 0, gTgtY = 0;
    int gBend1X = 0, gBend1Y = 0, gBend2X = 0, gBend2Y = 0, gBend3X = 0, gBend3Y = 0;

    switch (dir) {
        case SelfLoopDirection::Right:
            // Right → Top (top-right corner)
            layout.sourceEdge = NodeEdge::Right;
            layout.targetEdge = NodeEdge::Top;

            // Source: Right edge, shifted down from top
            gSrcX = gNodeRight;
            gSrcY = gNodeTop + gSpacing + gSnapOffset;
            // Target: Top edge, shifted left from right
            gTgtX = gNodeRight - gSpacing - gSnapOffset;
            gTgtY = gNodeTop;

            // Bend points: route around top-right corner
            gBend1X = gNodeRight + gOffset;  gBend1Y = gSrcY;
            gBend2X = gNodeRight + gOffset;  gBend2Y = gNodeTop - gOffset;
            gBend3X = gTgtX;                 gBend3Y = gNodeTop - gOffset;
            break;

        case SelfLoopDirection::Left:
            // Left → Bottom (bottom-left corner)
            layout.sourceEdge = NodeEdge::Left;
            layout.targetEdge = NodeEdge::Bottom;

            // Source: Left edge, shifted up from bottom
            gSrcX = gNodeLeft;
            gSrcY = gNodeBottom - gSpacing - gSnapOffset;
            // Target: Bottom edge, shifted right from left
            gTgtX = gNodeLeft + gSpacing + gSnapOffset;
            gTgtY = gNodeBottom;

            // Bend points: route around bottom-left corner
            gBend1X = gNodeLeft - gOffset;   gBend1Y = gSrcY;
            gBend2X = gNodeLeft - gOffset;   gBend2Y = gNodeBottom + gOffset;
            gBend3X = gTgtX;                 gBend3Y = gNodeBottom + gOffset;
            break;

        case SelfLoopDirection::Top:
            // Top → Right (top-right corner)
            layout.sourceEdge = NodeEdge::Top;
            layout.targetEdge = NodeEdge::Right;

            // Source: Top edge, shifted left from right
            gSrcX = gNodeRight - gSpacing - gSnapOffset;
            gSrcY = gNodeTop;
            // Target: Right edge, shifted down from top
            gTgtX = gNodeRight;
            gTgtY = gNodeTop + gSpacing + gSnapOffset;

            // Bend points: route around top-right corner
            gBend1X = gSrcX;                 gBend1Y = gNodeTop - gOffset;
            gBend2X = gNodeRight + gOffset;  gBend2Y = gNodeTop - gOffset;
            gBend3X = gNodeRight + gOffset;  gBend3Y = gTgtY;
            break;

        case SelfLoopDirection::Bottom:
            // Bottom → Left (bottom-left corner)
            layout.sourceEdge = NodeEdge::Bottom;
            layout.targetEdge = NodeEdge::Left;

            // Source: Bottom edge, shifted right from left
            gSrcX = gNodeLeft + gSpacing + gSnapOffset;
            gSrcY = gNodeBottom;
            // Target: Left edge, shifted up from bottom
            gTgtX = gNodeLeft;
            gTgtY = gNodeBottom - gSpacing - gSnapOffset;

            // Bend points: route around bottom-left corner
            gBend1X = gSrcX;                 gBend1Y = gNodeBottom + gOffset;
            gBend2X = gNodeLeft - gOffset;   gBend2Y = gNodeBottom + gOffset;
            gBend3X = gNodeLeft - gOffset;   gBend3Y = gTgtY;
            break;

        case SelfLoopDirection::Auto:
            // Auto is converted to Right above, this case should never be reached
            // Initialize to avoid uninitialized variable warnings
            gSrcX = gSrcY = gTgtX = gTgtY = 0;
            gBend1X = gBend1Y = gBend2X = gBend2Y = gBend3X = gBend3Y = 0;
            break;
    }

    // === CONVERT GRID COORDINATES TO PIXELS ===
    layout.sourcePoint = {toPixel(gSrcX), toPixel(gSrcY)};
    layout.targetPoint = {toPixel(gTgtX), toPixel(gTgtY)};
    
    // NOTE: snapIndex is no longer stored - computed from position as needed
    // using GridSnapCalculator::getCandidateIndexFromPosition(node, edge, point, gridSize)
    
    LOG_DEBUG("[SNAP-TRACE] SelfLoopRouter::route edge={} SOURCE pos=({},{}) TARGET pos=({},{})",
              edgeId, layout.sourcePoint.x, layout.sourcePoint.y,
              layout.targetPoint.x, layout.targetPoint.y);
    
    layout.bendPoints.push_back({{toPixel(gBend1X), toPixel(gBend1Y)}});
    layout.bendPoints.push_back({{toPixel(gBend2X), toPixel(gBend2Y)}});
    layout.bendPoints.push_back({{toPixel(gBend3X), toPixel(gBend3Y)}});

    // Calculate label position
    layout.labelPosition = LayoutUtils::calculateEdgeLabelPosition(layout);

    return layout;
}

int SelfLoopRouter::calculateLoopIndex(
    EdgeId edgeId,
    NodeId nodeId,
    const std::unordered_map<EdgeId, EdgeLayout>& layouts) {
    // Count self-loops on this node with SMALLER edgeIds
    // This ensures consistent ordering regardless of parallel execution order:
    // - edgeId=1 counts 0 smaller self-loops → loopIndex=0
    // - edgeId=2 counts 1 smaller self-loop (edgeId=1) → loopIndex=1
    int loopIndex = 0;
    for (const auto& [otherId, layout] : layouts) {
        if (otherId < edgeId &&
            layout.from == layout.to &&
            layout.from == nodeId) {
            ++loopIndex;
        }
    }
    return loopIndex;
}

}  // namespace arborvia
