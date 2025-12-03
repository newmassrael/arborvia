#pragma once

#include "arborvia/core/Types.h"
#include "arborvia/layout/LayoutResult.h"

#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <limits>

namespace arborvia {
namespace algorithms {

/// Grid-based obstacle map for pathfinding
/// Tracks which grid cells are blocked by nodes
class ObstacleMap {
public:
    /// Information about a single grid cell
    /// Supports multiple blocking nodes (for overlapping nodes scenario)
    struct GridCell {
        bool blocked = false;
        std::vector<NodeId> blockingNodes;  ///< All nodes that block this cell
    };

    /// Safe zone boundaries (grid coordinates outside all nodes)
    struct SafeZones {
        int yAbove = 0;   // Grid Y coordinate above all nodes
        int yBelow = 0;   // Grid Y coordinate below all nodes
        int xLeft = 0;    // Grid X coordinate left of all nodes
        int xRight = 0;   // Grid X coordinate right of all nodes
    };

    ObstacleMap() = default;

    /// Build obstacle map from node layouts
    /// @param nodeLayouts All node layouts
    /// @param gridSize Grid cell size in pixels
    /// @param margin Additional margin around nodes (grid units)
    void buildFromNodes(
        const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
        float gridSize,
        int margin = 1);

    /// Check if a grid coordinate is blocked
    /// @param gridX Grid X coordinate
    /// @param gridY Grid Y coordinate
    /// @return True if blocked by any node
    bool isBlocked(int gridX, int gridY) const;

    /// Check if a grid coordinate is blocked, excluding specific nodes
    /// @param gridX Grid X coordinate
    /// @param gridY Grid Y coordinate
    /// @param exclude Nodes to exclude from blocking check
    /// @return True if blocked by a node not in exclude set
    bool isBlocked(int gridX, int gridY, 
                   const std::unordered_set<NodeId>& exclude) const;

    /// Check if an orthogonal segment is blocked
    /// Segment must be axis-aligned (horizontal or vertical)
    /// @param x1, y1 Start grid coordinates
    /// @param x2, y2 End grid coordinates
    /// @param exclude Nodes to exclude from blocking check
    /// @return True if any cell along segment is blocked
    bool segmentBlocked(int x1, int y1, int x2, int y2,
                        const std::unordered_set<NodeId>& exclude) const;

    /// Get the node blocking a specific cell (if any)
    /// @param gridX Grid X coordinate
    /// @param gridY Grid Y coordinate
    /// @return NodeId of blocking node, or INVALID_NODE if not blocked
    NodeId getBlockingNode(int gridX, int gridY) const;

    /// Get safe zone boundaries
    const SafeZones& safeZones() const { return safeZones_; }

    /// Get grid dimensions
    int width() const { return width_; }
    int height() const { return height_; }

    /// Get grid offset (minimum grid coordinates)
    int offsetX() const { return offsetX_; }
    int offsetY() const { return offsetY_; }

    /// Get grid size in pixels
    float gridSize() const { return gridSize_; }

    /// Convert pixel coordinate to grid coordinate
    GridPoint pixelToGrid(const Point& p) const;

    /// Convert grid coordinate to pixel coordinate (cell center)
    Point gridToPixel(int gridX, int gridY) const;

    /// Check if grid coordinates are within bounds
    bool inBounds(int gridX, int gridY) const;

private:
    /// Convert grid coordinates to internal array index
    /// @return -1 if out of bounds
    int toIndex(int gridX, int gridY) const;

    std::vector<GridCell> grid_;
    int width_ = 0;
    int height_ = 0;
    int offsetX_ = 0;
    int offsetY_ = 0;
    float gridSize_ = 1.0f;
    SafeZones safeZones_;
};

}  // namespace algorithms
}  // namespace arborvia
