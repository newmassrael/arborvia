#pragma once

#include "../core/Types.h"
#include "LayoutResult.h"

#include <unordered_map>
#include <vector>

namespace arborvia {

/// Forbidden zone representing a region where a node cannot be placed
struct ForbiddenZone {
    Rect bounds;       ///< Bounding rectangle of the forbidden region
    NodeId blockedBy;  ///< Node ID that creates this forbidden zone
};

/// Calculator for valid/invalid drag regions
///
/// Formula: margin(side) = base + max(0, snapPoints - threshold)
/// - base = MIN_NODE_GRID_DISTANCE (5 grid units)
/// - threshold = MIN_NODE_GRID_DISTANCE - 1 (4 snap points)
/// - snapPoints = max edges on that direction between dragged and other node
class ValidRegionCalculator {
public:
    /// Calculate forbidden zones for a dragged node
    /// @param draggedNode The node being dragged
    /// @param nodeLayouts Current node positions and sizes
    /// @param edgeLayouts Current edge layouts (for direction info)
    /// @param gridSize Size of one grid unit in pixels
    /// @return Vector of forbidden zones the dragged node cannot enter
    static std::vector<ForbiddenZone> calculate(
        NodeId draggedNode,
        const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
        const std::unordered_map<EdgeId, EdgeLayout>& edgeLayouts,
        float gridSize);

    /// Fast check if a position is valid (not in any forbidden zone)
    /// @param position Proposed node position (top-left corner)
    /// @param nodeSize Size of the node being placed
    /// @param zones Pre-calculated forbidden zones
    /// @return true if the position is valid
    static bool isValid(
        const Point& position,
        const Size& nodeSize,
        const std::vector<ForbiddenZone>& zones);
};

}  // namespace arborvia
