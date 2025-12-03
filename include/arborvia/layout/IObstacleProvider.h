#pragma once

#include "../core/Types.h"

#include <unordered_set>

namespace arborvia {
namespace algorithms {

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

    // === Coordinate Conversion ===

    /// Convert pixel coordinate to grid coordinate
    virtual GridPoint pixelToGrid(const Point& p) const = 0;

    /// Convert grid coordinate to pixel coordinate (cell center)
    virtual Point gridToPixel(int gridX, int gridY) const = 0;
};

}  // namespace algorithms
}  // namespace arborvia
