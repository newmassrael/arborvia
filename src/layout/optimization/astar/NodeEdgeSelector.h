#pragma once

#include "EdgeRoutingContext.h"
#include "arborvia/layout/api/IPathFinder.h"
#include "arborvia/layout/config/LayoutTypes.h"
#include "../../sugiyama/routing/EdgeRoutingUtils.h"

#include <memory>
#include <vector>

namespace arborvia {

// Forward declarations
class ObstacleMap;

/// Handles NodeEdge combination evaluation for edge routing
///
/// Evaluates all 16 combinations (4 source × 4 target edges) for each edge
/// using A* pathfinding to find optimal obstacle-avoiding paths.
///
/// This class is designed for dependency injection via EdgeRoutingContext,
/// enabling clean separation from AStarEdgeOptimizer.
class NodeEdgeSelector {
public:
    /// Result of evaluating a single edge combination
    using CombinationResult = EdgeCombinationResult;

    /// Result of parallel edge evaluation
    struct ParallelEdgeResult {
        EdgeId edgeId;
        EdgeCombinationResult best;
        bool isSelfLoop = false;
        bool hasValidResult = false;
    };

    /// Evaluate edge combinations for a single edge
    /// When context.preserveDirections is true, only evaluates the existing combination.
    /// Otherwise, evaluates all 16 combinations (4 source × 4 target edges).
    /// @param context Routing context with all dependencies
    /// @param edgeId Edge being optimized
    /// @param baseLayout Original layout to derive combinations from
    /// @param assignedLayouts Layouts already assigned (for intersection scoring)
    /// @return Vector of valid combination results, sorted by score (best first)
    static std::vector<CombinationResult> evaluateCombinations(
        const EdgeRoutingContext& context,
        EdgeId edgeId,
        const EdgeLayout& baseLayout,
        const std::unordered_map<EdgeId, EdgeLayout>& assignedLayouts);

    /// Thread-safe version with explicit pathfinder
    /// @param pathFinder Thread-local pathfinder instance for parallel execution
    static std::vector<CombinationResult> evaluateCombinations(
        const EdgeRoutingContext& context,
        EdgeId edgeId,
        const EdgeLayout& baseLayout,
        const std::unordered_map<EdgeId, EdgeLayout>& assignedLayouts,
        IPathFinder& pathFinder);

    /// Evaluate single edge independently (thread-safe)
    /// Uses thread-local pathfinder to avoid data races
    /// @param context Routing context with all dependencies
    /// @param edgeId Edge being optimized
    /// @param baseLayout Original layout
    /// @param currentLayouts All current edge layouts (for self-loop loopIndex)
    /// @param pathFinder Thread-local pathfinder instance
    static ParallelEdgeResult evaluateEdgeIndependent(
        const EdgeRoutingContext& context,
        EdgeId edgeId,
        const EdgeLayout& baseLayout,
        const std::unordered_map<EdgeId, EdgeLayout>& currentLayouts,
        IPathFinder& pathFinder);

    /// Evaluate self-loop combinations for a single edge
    /// Tries all 4 directions (Right, Left, Top, Bottom) and returns
    /// sorted results based on penalty scores.
    static std::vector<CombinationResult> evaluateSelfLoopCombinations(
        const EdgeRoutingContext& context,
        EdgeId edgeId,
        const EdgeLayout& baseLayout,
        const std::unordered_map<EdgeId, EdgeLayout>& assignedLayouts);

    /// Create a candidate layout for a specific source/target edge combination
    /// Calculates actual orthogonal path using A* pathfinder
    /// @param context Routing context with pathfinder
    /// @param base Original layout to derive from
    /// @param sourceEdge Source edge to use
    /// @param targetEdge Target edge to use
    /// @param assignedLayouts Other edge layouts (for snap index collision avoidance)
    /// @param obstacles ObstacleMap for pathfinding
    /// @param pathFound Output: true if A* found a valid path
    static EdgeLayout createCandidateLayout(
        const EdgeRoutingContext& context,
        const EdgeLayout& base,
        NodeEdge sourceEdge,
        NodeEdge targetEdge,
        const std::unordered_map<EdgeId, EdgeLayout>& assignedLayouts,
        ObstacleMap& obstacles,
        bool& pathFound);

    /// Thread-safe version with explicit pathfinder
    static EdgeLayout createCandidateLayout(
        const EdgeRoutingContext& context,
        const EdgeLayout& base,
        NodeEdge sourceEdge,
        NodeEdge targetEdge,
        const std::unordered_map<EdgeId, EdgeLayout>& assignedLayouts,
        ObstacleMap& obstacles,
        bool& pathFound,
        IPathFinder& pathFinder);

    /// Calculate snap point position on a node edge (pixel coordinates)
    /// @param node Node layout
    /// @param edge Which edge of the node
    /// @param position Relative position (0.0 to 1.0, default 0.5 = center)
    /// @return Absolute point coordinates
    static Point calculateSnapPosition(
        const NodeLayout& node,
        NodeEdge edge,
        float position = 0.5f);

    /// Calculate snap point position on a node edge (grid coordinates)
    /// All coordinates are exact grid vertices - no rounding errors
    /// @param node Node layout
    /// @param edge Which edge of the node
    /// @param gridSize Grid cell size
    /// @param position Relative position (0.0 to 1.0, default 0.5 = center)
    /// @return Grid coordinates
    static GridPoint calculateGridPosition(
        const NodeLayout& node,
        NodeEdge edge,
        float gridSize,
        float position = 0.5f);
};

}  // namespace arborvia
