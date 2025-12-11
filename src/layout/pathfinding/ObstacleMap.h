#pragma once

#include "arborvia/layout/api/IObstacleProvider.h"
#include "arborvia/layout/config/MoveDirection.h"
#include "arborvia/layout/config/LayoutResult.h"

#include <array>
#include <unordered_map>
#include <vector>
#include <limits>

namespace arborvia {


/// Grid-based obstacle map for pathfinding
/// Tracks which grid cells are blocked by nodes
/// 
/// Implements IObstacleProvider interface for use with pathfinding algorithms.
class ObstacleMap : public IObstacleProvider {
public:
    /// Information about a single grid cell
    /// Supports multiple blocking nodes (for overlapping nodes scenario)
    struct GridCell {
        bool blocked = false;
        std::vector<NodeId> blockingNodes;  ///< All nodes that block this cell

        // Direction-aware edge segment blocking:
        // These flags indicate that existing edge segments occupy this cell
        // with specific orientations. Paths can cross at right angles but
        // cannot overlap parallel segments.
        bool horizontalSegment = false;  ///< Horizontal edge segment exists
        bool verticalSegment = false;    ///< Vertical edge segment exists
    };

    /// Visualization info for a cell (for GUI rendering)
    struct CellVisInfo {
        bool isNodeBlocked = false;
        bool hasHorizontalSegment = false;
        bool hasVerticalSegment = false;
        std::vector<NodeId> blockingNodes;
    };

    ObstacleMap() = default;

    /// Build obstacle map from node layouts
    /// @param nodeLayouts All node layouts
    /// @param gridSize Grid cell size in pixels
    /// @param margin Additional margin around nodes (grid units)
    /// @param edgeLayouts Optional edge layouts to include in bounds calculation
    ///        When provided, grid bounds are expanded to include edge segment extents
    ///        This prevents edge segments from being out-of-bounds and unregistered
    void buildFromNodes(
        const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
        float gridSize,
        int margin = 1,
        const std::unordered_map<EdgeId, EdgeLayout>* edgeLayouts = nullptr);

    /// Add edge segments as obstacles
    /// This prevents A* from routing through existing edge paths
    /// @param edgeLayouts Edge layouts to add as obstacles
    /// @param excludeEdgeId Edge ID to exclude (the edge being routed)
    void addEdgeSegments(
        const std::unordered_map<EdgeId, EdgeLayout>& edgeLayouts,
        EdgeId excludeEdgeId = INVALID_EDGE);

    /// Add a single edge's segments as obstacles with direction-aware blocking
    /// Use this when you need to add an edge's segments after initial obstacle setup
    /// @param layout The edge layout to add as obstacles
    void addSingleEdgeSegments(const EdgeLayout& layout);

    /// Clear edge segment obstacles (keeps node obstacles)
    void clearEdgeSegments();

    // === Per-Edge Path Tracking (for Rip-up and Reroute) ===

    /// Mark an edge's path on the grid with cost overlay
    /// This increases the cost for cells along the path (but doesn't block them)
    /// @param edgeId The edge ID
    /// @param path Grid coordinates of the path
    void markEdgePath(EdgeId edgeId, const std::vector<GridPoint>& path);

    /// Clear a specific edge's path from the cost overlay
    /// @param edgeId The edge ID to clear
    void clearEdgePath(EdgeId edgeId);

    /// Clear all edge paths from the cost overlay
    void clearAllEdgePaths();

    /// Get all registered edge paths
    /// @return Map of edge ID to grid path
    const std::unordered_map<EdgeId, std::vector<GridPoint>>& getEdgePaths() const {
        return edgePaths_;
    }

    /// Check if an edge path is registered
    bool hasEdgePath(EdgeId edgeId) const {
        return edgePaths_.find(edgeId) != edgePaths_.end();
    }

    /// Check if a grid coordinate is blocked
    /// @param gridX Grid X coordinate
    /// @param gridY Grid Y coordinate
    /// @return True if blocked by any node
    bool isBlocked(int gridX, int gridY) const override;

    /// Check if a grid coordinate is blocked, excluding specific nodes
    /// @param gridX Grid X coordinate
    /// @param gridY Grid Y coordinate
    /// @param exclude Nodes to exclude from blocking check
    /// @return True if blocked by a node not in exclude set
    bool isBlocked(int gridX, int gridY, 
                   const std::unordered_set<NodeId>& exclude) const override;

    /// Check if a grid coordinate is blocked for a specific movement direction
    /// Node obstacles block all directions. Edge segment obstacles only block
    /// parallel movement (horizontal segments block horizontal movement, etc.)
    /// @param gridX Grid X coordinate
    /// @param gridY Grid Y coordinate
    /// @param moveDir The direction of movement to check
    /// @param exclude Nodes to exclude from blocking check
    /// @return True if movement in the specified direction is blocked
    bool isBlockedForDirection(int gridX, int gridY,
                               MoveDirection moveDir,
                               const std::unordered_set<NodeId>& exclude) const override;

    /// Check if an orthogonal segment is blocked
    /// Segment must be axis-aligned (horizontal or vertical)
    /// @param x1, y1 Start grid coordinates
    /// @param x2, y2 End grid coordinates
    /// @param exclude Nodes to exclude from blocking check
    /// @return True if any cell along segment is blocked
    bool segmentBlocked(int x1, int y1, int x2, int y2,
                        const std::unordered_set<NodeId>& exclude) const override;

    /// Get the node blocking a specific cell (if any)
    /// @param gridX Grid X coordinate
    /// @param gridY Grid Y coordinate
    /// @return NodeId of blocking node, or INVALID_NODE if not blocked
    NodeId getBlockingNode(int gridX, int gridY) const;

    /// Get safe zone boundaries
    const SafeZones& safeZones() const override { return safeZones_; }

    /// Get grid dimensions
    int width() const override { return width_; }
    int height() const override { return height_; }

    /// Get grid offset (minimum grid coordinates)
    int offsetX() const { return offsetX_; }
    int offsetY() const { return offsetY_; }

    /// Get grid offset as Point (pixel coordinates)
    Point getOffset() const { return { offsetX_ * gridSize_, offsetY_ * gridSize_ }; }

    /// Get grid size in pixels
    float gridSize() const { return gridSize_; }

    /// Get cell visualization info (for GUI rendering)
    /// @param gridX Grid X coordinate
    /// @param gridY Grid Y coordinate
    /// @return CellVisInfo with blocking details
    CellVisInfo getCellVisInfo(int gridX, int gridY) const;

    /// Convert pixel coordinate to grid coordinate
    GridPoint pixelToGrid(const Point& p) const override;

    /// Convert grid coordinate to pixel coordinate (cell center)
    Point gridToPixel(int gridX, int gridY) const override;

    /// Check if grid coordinates are within bounds
    bool inBounds(int gridX, int gridY) const override;

    // === Cost-based Pathfinding Support ===

    /// Get movement cost for a cell (overrides IObstacleProvider default)
    /// Returns accumulated cost from node blocking and edge paths
    int getCost(int gridX, int gridY) const override;

    /// Get movement cost for a specific direction
    /// Allows direction-aware costing (parallel to existing edge = higher cost)
    int getCostForDirection(int gridX, int gridY, MoveDirection moveDir) const override;

    /// Get movement cost with exclusions and proximity penalty
    /// Includes gradient penalty for cells near unrelated nodes (not in exclude set)
    /// @param gridX Grid X coordinate
    /// @param gridY Grid Y coordinate
    /// @param moveDir The direction of movement
    /// @param exclude Nodes to exclude from proximity penalty (typically source/target)
    /// @return Cost value including proximity penalty for unrelated nodes
    int getCostForDirectionWithExcludes(int gridX, int gridY,
                                         MoveDirection moveDir,
                                         const std::unordered_set<NodeId>& exclude) const override;

private:
    /// Convert grid coordinates to internal array index
    /// @return -1 if out of bounds
    int toIndex(int gridX, int gridY) const;

    /// Mark a line segment on the grid as blocked
    void markSegmentBlocked(const Point& p1, const Point& p2, bool isEdgeSegment);
    
    /// Mark a line segment as blocked, with options to skip start/end cells
    /// Used for edge segments to prevent FIRST MOVE BLOCKED at snap points
    /// @param p1 Start point of segment
    /// @param p2 End point of segment
    /// @param skipStartCell If true, don't block the cell containing p1
    /// @param skipEndCell If true, don't block the cell containing p2
    void markSegmentBlockedWithSkip(const Point& p1, const Point& p2, bool skipStartCell, bool skipEndCell);
    
    /// Mark a line segment as blocked, skipping first and last cells
    /// This allows multiple edges to share entry/exit points near nodes
    void markSegmentBlockedSkipEndpoints(const Point& p1, const Point& p2);

    std::vector<GridCell> grid_;
    std::vector<bool> edgeSegmentCells_;  ///< Track cells blocked by edge segments
    int width_ = 0;
    int height_ = 0;
    int offsetX_ = 0;
    int offsetY_ = 0;
    float gridSize_ = 1.0f;
    SafeZones safeZones_;

    // === Per-Edge Path Tracking ===
    std::unordered_map<EdgeId, std::vector<GridPoint>> edgePaths_;  ///< Edge ID -> grid path
    std::unordered_map<int, int> edgeCostOverlay_;  ///< Cell index -> accumulated edge cost
    std::unordered_map<int, std::vector<EdgeId>> cellToEdges_;  ///< Cell index -> edges using this cell

    // === Pre-calculated Proximity Map ===
    static constexpr int PROXIMITY_RADIUS = 3;  ///< Grid cells beyond node boundary to apply proximity penalty
    static constexpr int MAX_PROXIMITY_ENTRIES = 3;  ///< Max nodes to track per cell for proximity

    /// Entry for a node within proximity range of a cell
    struct ProximityEntry {
        NodeId nodeId = INVALID_NODE;
        int distance = PROXIMITY_RADIUS + 1;  ///< Distance to node boundary (0 = adjacent)
    };

    /// Pre-calculated proximity info for each cell
    /// Stores up to MAX_PROXIMITY_ENTRIES closest nodes
    std::vector<std::array<ProximityEntry, MAX_PROXIMITY_ENTRIES>> proximityMap_;

    /// Build proximity map during buildFromNodes
    void buildProximityMap();
};


}  // namespace arborvia
