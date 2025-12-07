#pragma once

#include "../core/Types.h"
#include "MoveDirection.h"

#include <unordered_set>

namespace arborvia {

/// Safe zone boundaries (grid coordinates outside all obstacles)
struct SafeZones {
    int yAbove = 0;   ///< Grid Y coordinate above all obstacles
    int yBelow = 0;   ///< Grid Y coordinate below all obstacles
    int xLeft = 0;    ///< Grid X coordinate left of all obstacles
    int xRight = 0;   ///< Grid X coordinate right of all obstacles
};

/// Abstract interface for obstacle detection in pathfinding
/// 
/// This interface provides a read-only view of obstacles for pathfinding
/// algorithms. Implementations can provide different obstacle sources:
/// - ObstacleMap: Grid-based obstacle detection from node layouts
/// - Custom implementations: Dynamic obstacles, cached obstacles, etc.
/// 
/// Using this interface allows pathfinding algorithms to be decoupled
/// from the specific obstacle representation.
class IObstacleProvider {
public:
    virtual ~IObstacleProvider() = default;

    // === Blocking Queries ===

    /// Check if a grid coordinate is blocked
    /// @param gridX Grid X coordinate
    /// @param gridY Grid Y coordinate
    /// @return True if blocked by any obstacle
    virtual bool isBlocked(int gridX, int gridY) const = 0;

    /// Check if a grid coordinate is blocked, excluding specific nodes
    /// @param gridX Grid X coordinate
    /// @param gridY Grid Y coordinate
    /// @param exclude Nodes to exclude from blocking check
    /// @return True if blocked by an obstacle not in exclude set
    virtual bool isBlocked(int gridX, int gridY, 
                          const std::unordered_set<NodeId>& exclude) const = 0;

    /// Check if a grid coordinate is blocked for a specific movement direction
    /// This enables direction-aware obstacle detection:
    /// - Node obstacles block all movement directions
    /// - Edge segment obstacles only block parallel movement (not perpendicular)
    /// @param gridX Grid X coordinate  
    /// @param gridY Grid Y coordinate
    /// @param moveDir The direction of movement to check
    /// @param exclude Nodes to exclude from blocking check
    /// @return True if movement in the specified direction is blocked
    virtual bool isBlockedForDirection(int gridX, int gridY,
                                       MoveDirection moveDir,
                                       const std::unordered_set<NodeId>& exclude) const = 0;

    /// Check if an orthogonal segment is blocked
    /// Segment must be axis-aligned (horizontal or vertical)
    /// @param x1, y1 Start grid coordinates
    /// @param x2, y2 End grid coordinates
    /// @param exclude Nodes to exclude from blocking check
    /// @return True if any cell along segment is blocked
    virtual bool segmentBlocked(int x1, int y1, int x2, int y2,
                               const std::unordered_set<NodeId>& exclude) const = 0;

    // === Grid Information ===

    /// Get safe zone boundaries (areas guaranteed to be unblocked)
    virtual const SafeZones& safeZones() const = 0;

    /// Get grid dimensions
    virtual int width() const = 0;
    virtual int height() const = 0;

    /// Check if grid coordinates are within bounds
    virtual bool inBounds(int gridX, int gridY) const = 0;

    // === Cost Queries (for dynamic cost-based pathfinding) ===

    /// Cost constants for pathfinding
    static constexpr int COST_FREE = 1;           ///< Empty cell (base cost)
    static constexpr int COST_EDGE_PATH = 50;     ///< Existing edge path
    static constexpr int COST_PROXIMITY_MAX = 30; ///< Maximum proximity penalty (closest to unrelated node)
    static constexpr int COST_BLOCKED = 999999;   ///< Completely blocked (node interior)

    /// Get movement cost for a cell
    /// Returns COST_BLOCKED for node-blocked cells, or accumulated cost from edge paths
    /// @param gridX Grid X coordinate
    /// @param gridY Grid Y coordinate
    /// @return Cost value (lower is better, COST_BLOCKED means impassable)
    virtual int getCost(int gridX, int gridY) const {
        return isBlocked(gridX, gridY) ? COST_BLOCKED : COST_FREE;
    }

    /// Get movement cost for a specific direction
    /// Allows direction-aware costing (e.g., parallel to existing edge = higher cost)
    /// @param gridX Grid X coordinate
    /// @param gridY Grid Y coordinate
    /// @param moveDir The direction of movement
    /// @return Cost value for moving in that direction
    virtual int getCostForDirection(int gridX, int gridY, MoveDirection moveDir) const {
        std::unordered_set<NodeId> empty;
        return isBlockedForDirection(gridX, gridY, moveDir, empty) ? COST_BLOCKED : COST_FREE;
    }

    /// Get movement cost for a specific direction with exclusions
    /// Includes proximity penalty for cells near unrelated nodes (not in exclude set)
    /// This encourages paths to stay away from nodes they're not connected to
    /// @param gridX Grid X coordinate
    /// @param gridY Grid Y coordinate
    /// @param moveDir The direction of movement
    /// @param exclude Nodes to exclude from proximity penalty (typically source/target)
    /// @return Cost value including proximity penalty for unrelated nodes
    virtual int getCostForDirectionWithExcludes(int gridX, int gridY, 
                                                 MoveDirection moveDir,
                                                 const std::unordered_set<NodeId>& exclude) const {
        // Default implementation: just check blocking, no proximity
        return isBlockedForDirection(gridX, gridY, moveDir, exclude) ? COST_BLOCKED : COST_FREE;
    }

    // === Coordinate Conversion ===

    /// Convert pixel coordinate to grid coordinate
    virtual GridPoint pixelToGrid(const Point& p) const = 0;

    /// Convert grid coordinate to pixel coordinate (cell center)
    virtual Point gridToPixel(int gridX, int gridY) const = 0;
};

}  // namespace arborvia
