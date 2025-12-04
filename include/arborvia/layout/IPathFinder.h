#pragma once

#include "IObstacleProvider.h"
#include "../core/Types.h"
#include "LayoutEnums.h"

#include <vector>
#include <unordered_set>
#include <memory>

namespace arborvia {
namespace algorithms {

/// Direction of movement in the grid
enum class MoveDirection {
    None,
    Up,
    Down,
    Left,
    Right
};

/// Result of pathfinding operation
struct PathResult {
    bool found = false;
    std::vector<GridPoint> path;  ///< Path in grid coordinates
    int bendCount = 0;            ///< Number of direction changes
    int totalCost = 0;            ///< Total path cost
};

/// Abstract interface for pathfinding algorithms
///
/// Implementations can provide different pathfinding strategies:
/// - A* (default, optimal with heuristic)
/// - Dijkstra (optimal without heuristic)
/// - Jump Point Search (faster for uniform cost grids)
/// - BFS (simpler, unweighted)
///
/// Use this interface to swap pathfinding algorithms without
/// modifying EdgeRouting or other dependent code.
class IPathFinder {
public:
    virtual ~IPathFinder() = default;

    /// Find path between two grid points avoiding obstacles
    /// @param start Starting grid point
    /// @param goal Target grid point
    /// @param obstacles Obstacle map for collision detection
    /// @param sourceNode Source node ID (excluded only at start cell)
    /// @param targetNode Target node ID (excluded only at goal cell)
    /// @param sourceEdge Edge of source node (constrains first move direction)
    /// @param targetEdge Edge of target node (constrains last move direction)
    /// @param extraStartExcludes Additional nodes to exclude at start cell
    /// @param extraGoalExcludes Additional nodes to exclude at goal cell
    /// @return PathResult with path in grid coordinates
    virtual PathResult findPath(
        const GridPoint& start,
        const GridPoint& goal,
        const IObstacleProvider& obstacles,
        NodeId sourceNode,
        NodeId targetNode,
        NodeEdge sourceEdge,
        NodeEdge targetEdge,
        const std::unordered_set<NodeId>& extraStartExcludes = {},
        const std::unordered_set<NodeId>& extraGoalExcludes = {}) const = 0;

    /// Find path via safe zone (guaranteed fallback)
    /// Routes outside all nodes for guaranteed success
    virtual PathResult findPathViaSafeZone(
        const GridPoint& start,
        const GridPoint& goal,
        const IObstacleProvider& obstacles,
        NodeId sourceNode,
        NodeId targetNode,
        NodeEdge sourceEdge,
        NodeEdge targetEdge,
        const std::unordered_set<NodeId>& extraStartExcludes = {},
        const std::unordered_set<NodeId>& extraGoalExcludes = {}) const = 0;

    /// Get algorithm name for debugging/logging
    virtual const char* algorithmName() const = 0;
};

/// Factory function type for creating pathfinders
using PathFinderFactory = std::unique_ptr<IPathFinder>(*)();

}  // namespace algorithms
}  // namespace arborvia
