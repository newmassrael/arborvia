#pragma once

#include "arborvia/layout/api/IPathFinder.h"

namespace arborvia {


/// Context for pathfinding operations - bundles all parameters for cleaner code
struct PathContext {
    GridPoint start;
    GridPoint goal;
    const IObstacleProvider* obstacles;
    NodeId sourceNode;
    NodeId targetNode;
    NodeEdge sourceEdge;
    NodeEdge targetEdge;
    std::unordered_set<NodeId> startExcludes;
    std::unordered_set<NodeId> goalExcludes;

    /// Build exclusion sets including source/target nodes
    void buildExcludes(
        const std::unordered_set<NodeId>& extraStartExcludes,
        const std::unordered_set<NodeId>& extraGoalExcludes) {
        startExcludes = extraStartExcludes;
        startExcludes.insert(sourceNode);
        goalExcludes = extraGoalExcludes;
        goalExcludes.insert(targetNode);
    }
};

/// Fast deterministic L-shaped pathfinder for drag-time edge routing
///
/// This pathfinder generates simple L-shaped or U-shaped paths without
/// complex search algorithms. It's designed for smooth interactive drag
/// operations where speed is more important than optimal path quality.
///
/// Algorithm priority:
/// 1. L-shaped path (horizontal first, then vertical)
/// 2. L-shaped path (vertical first, then horizontal)
/// 3. U-shaped path (horizontal detour)
/// 4. U-shaped path (vertical detour)
/// 5. Safe zone fallback (guaranteed success)
class LShapedPathFinder : public IPathFinder {
public:
    LShapedPathFinder() = default;

    /// Get algorithm name
    const char* algorithmName() const override { return "L-Shaped"; }

    /// Find path using simple L-shaped or U-shaped routing
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
    PathResult findPath(
        const GridPoint& start,
        const GridPoint& goal,
        const IObstacleProvider& obstacles,
        NodeId sourceNode,
        NodeId targetNode,
        NodeEdge sourceEdge,
        NodeEdge targetEdge,
        const std::unordered_set<NodeId>& extraStartExcludes = {},
        const std::unordered_set<NodeId>& extraGoalExcludes = {}) const override;

    /// Find path via safe zone (guaranteed fallback)
    /// Routes outside all nodes for guaranteed success
    PathResult findPathViaSafeZone(
        const GridPoint& start,
        const GridPoint& goal,
        const IObstacleProvider& obstacles,
        NodeId sourceNode,
        NodeId targetNode,
        NodeEdge sourceEdge,
        NodeEdge targetEdge,
        const std::unordered_set<NodeId>& extraStartExcludes = {},
        const std::unordered_set<NodeId>& extraGoalExcludes = {}) const override;

private:
    /// Try L-shaped path: horizontal first, then vertical
    /// Returns path if valid, empty path if blocked
    PathResult tryHorizontalFirst(const PathContext& ctx) const;

    /// Try L-shaped path: vertical first, then horizontal
    /// Returns path if valid, empty path if blocked
    PathResult tryVerticalFirst(const PathContext& ctx) const;

    /// Try U-shaped path with horizontal detour
    /// @param detourDirection Which direction to detour (-1 for left/up, +1 for right/down)
    PathResult tryUShapeHorizontal(const PathContext& ctx, int detourDirection) const;

    /// Try U-shaped path with vertical detour
    /// @param detourDirection Which direction to detour (-1 for left/up, +1 for right/down)
    PathResult tryUShapeVertical(const PathContext& ctx, int detourDirection) const;

    /// Check if a segment is passable
    bool isSegmentClear(
        const GridPoint& from,
        const GridPoint& to,
        const IObstacleProvider& obstacles,
        const std::unordered_set<NodeId>& excludeAtFrom,
        const std::unordered_set<NodeId>& excludeAtTo) const;

    /// Check if direction constraint is satisfied
    static bool isDirectionCompatible(NodeEdge edge, MoveDirection dir);

    /// Get the required move direction for exiting from a node edge
    static MoveDirection getExitDirection(NodeEdge edge);

    /// Get the required move direction for entering to a node edge
    static MoveDirection getEntryDirection(NodeEdge edge);

    /// Count bends in a path
    static int countBends(const std::vector<GridPoint>& path);

    /// Minimum detour distance for U-shaped paths (in grid units)
    static constexpr int MIN_DETOUR_DISTANCE = 3;
};


}  // namespace arborvia
