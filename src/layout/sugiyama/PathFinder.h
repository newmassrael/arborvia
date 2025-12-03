#pragma once

#include "ObstacleMap.h"
#include "arborvia/core/Types.h"
#include "arborvia/layout/LayoutEnums.h"

#include <vector>
#include <unordered_set>

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

/// A* based orthogonal pathfinder for edge routing
/// Finds paths that avoid obstacles and minimize bend points
class PathFinder {
public:
    /// Cost constants for pathfinding
    static constexpr int MOVE_COST = 1;
    static constexpr int BEND_COST = 3;  ///< Penalty for changing direction (minimizes bends)
    static constexpr int MAX_ITERATIONS = 50000;  ///< Safety limit for iterations

    PathFinder() = default;

    /// Find path using A* algorithm with direction constraints
    /// @param start Starting grid point
    /// @param goal Target grid point
    /// @param obstacles Obstacle map
    /// @param sourceNode Source node ID (excluded only at start cell)
    /// @param targetNode Target node ID (excluded only at goal cell)
    /// @param sourceEdge Edge of source node (constrains first move direction)
    /// @param targetEdge Edge of target node (constrains last move direction)
    /// @param extraStartExcludes Additional nodes to exclude at start cell (e.g., nodes containing sourcePoint)
    /// @param extraGoalExcludes Additional nodes to exclude at goal cell (e.g., nodes containing targetPoint)
    /// @return PathResult with path in grid coordinates
    PathResult findPath(
        const GridPoint& start,
        const GridPoint& goal,
        const ObstacleMap& obstacles,
        NodeId sourceNode,
        NodeId targetNode,
        NodeEdge sourceEdge,
        NodeEdge targetEdge,
        const std::unordered_set<NodeId>& extraStartExcludes = {},
        const std::unordered_set<NodeId>& extraGoalExcludes = {}) const;

    /// Find path via safe zone (guaranteed fallback)
    /// Routes outside all nodes for guaranteed success
    /// @param start Starting grid point
    /// @param goal Target grid point
    /// @param obstacles Obstacle map
    /// @param sourceNode Source node ID (excluded only at start cell)
    /// @param targetNode Target node ID (excluded only at goal cell)
    /// @param sourceEdge Edge of source node (constrains first move direction)
    /// @param targetEdge Edge of target node (constrains last move direction)
    /// @param extraStartExcludes Additional nodes to exclude at start cell
    /// @param extraGoalExcludes Additional nodes to exclude at goal cell
    /// @return PathResult with path in grid coordinates
    PathResult findPathViaSafeZone(
        const GridPoint& start,
        const GridPoint& goal,
        const ObstacleMap& obstacles,
        NodeId sourceNode,
        NodeId targetNode,
        NodeEdge sourceEdge,
        NodeEdge targetEdge,
        const std::unordered_set<NodeId>& extraStartExcludes = {},
        const std::unordered_set<NodeId>& extraGoalExcludes = {}) const;

    /// Simple direct path if start and goal are aligned
    /// Returns empty path if direct connection not possible
    PathResult tryDirectPath(
        const GridPoint& start,
        const GridPoint& goal,
        const ObstacleMap& obstacles,
        NodeId sourceNode,
        NodeId targetNode,
        const std::unordered_set<NodeId>& extraStartExcludes,
        const std::unordered_set<NodeId>& extraGoalExcludes) const;

    /// Simple L-shaped path (one bend)
    /// Tries both horizontal-first and vertical-first
    PathResult tryLShapedPath(
        const GridPoint& start,
        const GridPoint& goal,
        const ObstacleMap& obstacles,
        NodeId sourceNode,
        NodeId targetNode,
        const std::unordered_set<NodeId>& extraStartExcludes,
        const std::unordered_set<NodeId>& extraGoalExcludes) const;

private:
    /// Internal node for A* search
    struct SearchNode {
        GridPoint pos;
        MoveDirection lastDir = MoveDirection::None;
        int g = 0;  // Cost from start
        int h = 0;  // Heuristic to goal
        int f() const { return g + h; }
        
        // For priority queue comparison
        bool operator>(const SearchNode& other) const {
            return f() > other.f();
        }
    };

    /// Calculate Manhattan distance heuristic
    static int manhattanDistance(const GridPoint& a, const GridPoint& b);

    /// Get orthogonal neighbors of a grid point
    static std::vector<std::pair<GridPoint, MoveDirection>> getNeighbors(const GridPoint& p);

    /// Validate that a path segment doesn't cross obstacles
    /// For intermediate segments, no nodes are excluded
    bool validateSegment(
        const GridPoint& from,
        const GridPoint& to,
        const ObstacleMap& obstacles) const;
    
    /// Validate segment with per-endpoint exclusion logic
    /// @param isFirstSegment If true, exclude sourceNode + extraStartExcludes from 'from' cell
    /// @param isLastSegment If true, exclude targetNode + extraGoalExcludes from 'to' cell
    bool validateSegmentWithEndpoints(
        const GridPoint& from,
        const GridPoint& to,
        const ObstacleMap& obstacles,
        NodeId sourceNode,
        NodeId targetNode,
        bool isFirstSegment,
        bool isLastSegment,
        const std::unordered_set<NodeId>& extraStartExcludes,
        const std::unordered_set<NodeId>& extraGoalExcludes) const;

    /// Count bends in a path
    static int countBends(const std::vector<GridPoint>& path);

    /// Simplify path by removing collinear points
    static std::vector<GridPoint> simplifyPath(const std::vector<GridPoint>& path);

    /// Get required first move direction based on source edge
    /// For sourceEdge: edge determines exit direction (e.g., Bottom -> exit downward -> move Down)
    static MoveDirection getRequiredSourceDirection(NodeEdge sourceEdge);

    /// Get required last move direction based on target edge
    /// For targetEdge: edge determines entry direction (e.g., Top -> enter from above -> move Down)
    static MoveDirection getRequiredTargetArrivalDirection(NodeEdge targetEdge);
};

}  // namespace algorithms
}  // namespace arborvia
