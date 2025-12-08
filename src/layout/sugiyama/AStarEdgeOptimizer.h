#pragma once

#include "arborvia/layout/IEdgeOptimizer.h"
#include "arborvia/layout/IPathFinder.h"
#include "arborvia/layout/ValidRegionCalculator.h"
#include "EdgeRoutingUtils.h"

#include <atomic>
#include <future>
#include <memory>
#include <vector>

namespace arborvia {

// Forward declaration
class ObstacleMap;


/// A* based edge routing optimizer
/// Evaluates all 16 combinations (4 source × 4 target edges) for each edge
/// using A* pathfinding to find optimal obstacle-avoiding paths.
///
/// Use this optimizer for post-drag optimization where path quality is
/// more important than speed. For real-time drag feedback, use
/// GeometricEdgeOptimizer instead.
///
/// Processes edges sequentially, considering already-assigned edges
/// when scoring intersections.
class AStarEdgeOptimizer : public IEdgeOptimizer {
public:
    /// Constructor with optional pathfinder
    /// @param pathFinder Pathfinder for calculating orthogonal routes (uses AStarPathFinder if null)
    /// @param gridSize Grid cell size for pathfinding (0 = use default)
    explicit AStarEdgeOptimizer(
        std::shared_ptr<IPathFinder> pathFinder = nullptr,
        float gridSize = 0.0f);

    /// Optimize edge routing by evaluating all 16 combinations per edge
    /// Uses A* pathfinding for each combination to find optimal paths
    /// @param edges Edges to optimize (processed in order)
    /// @param currentLayouts Current edge layouts (used as base for combinations)
    /// @param nodeLayouts Node positions
    /// @return Map of optimized edge layouts
    std::unordered_map<EdgeId, EdgeLayout> optimize(
        const std::vector<EdgeId>& edges,
        const std::unordered_map<EdgeId, EdgeLayout>& currentLayouts,
        const std::unordered_map<NodeId, NodeLayout>& nodeLayouts) override;

    const char* algorithmName() const override { return "AStar"; }

    /// Regenerate bendPoints using A* pathfinding
    /// Preserves existing sourceEdge/targetEdge, only recreates path geometry
    void regenerateBendPoints(
        const std::vector<EdgeId>& edges,
        std::unordered_map<EdgeId, EdgeLayout>& edgeLayouts,
        const std::unordered_map<NodeId, NodeLayout>& nodeLayouts) override;

    /// Set pathfinder
    void setPathFinder(std::shared_ptr<IPathFinder> pathFinder);

    /// Set grid size for pathfinding
    void setGridSize(float gridSize);

    /// Enable/disable parallel evaluation (default: true)
    void setParallelEnabled(bool enabled) { parallelEnabled_ = enabled; }

private:
    std::shared_ptr<IPathFinder> pathFinder_;
    float gridSize_ = 0.0f;
    bool parallelEnabled_ = true;

    /// Result of evaluating a single edge combination (uses shared type)
    using CombinationResult = EdgeCombinationResult;

    /// Result of parallel edge evaluation
    struct ParallelEdgeResult {
        EdgeId edgeId;
        EdgeCombinationResult best;
        bool isSelfLoop = false;
        bool hasValidResult = false;
    };

    /// Evaluate single edge independently (thread-safe)
    /// Uses thread-local pathfinder to avoid data races
    /// @param pathFinder Thread-local pathfinder instance
    /// @param currentLayouts All current edge layouts (for self-loop loopIndex calculation)
    ParallelEdgeResult evaluateEdgeIndependent(
        EdgeId edgeId,
        const EdgeLayout& baseLayout,
        const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
        const std::vector<ForbiddenZone>& forbiddenZones,
        IPathFinder& pathFinder,
        const std::unordered_map<EdgeId, EdgeLayout>& currentLayouts);

    /// Detect overlapping edges in results
    std::vector<std::pair<EdgeId, EdgeId>> detectOverlaps(
        const std::unordered_map<EdgeId, EdgeLayout>& layouts);

    /// Evaluate edge combinations for a single edge
    /// When preserveDirections() is true, only evaluates the existing combination.
    /// Otherwise, evaluates all 16 combinations (4 source × 4 target edges).
    /// @param edgeId Edge being optimized
    /// @param baseLayout Original layout to derive combinations from
    /// @param assignedLayouts Layouts already assigned (for intersection scoring)
    /// @param nodeLayouts Node positions
    /// @param forbiddenZones Pre-calculated forbidden zones for constraint checking
    /// @return Vector of valid combination results, sorted by score (best first)
    std::vector<CombinationResult> evaluateCombinations(
        EdgeId edgeId,
        const EdgeLayout& baseLayout,
        const std::unordered_map<EdgeId, EdgeLayout>& assignedLayouts,
        const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
        const std::vector<ForbiddenZone>& forbiddenZones);

    /// Thread-safe version with explicit pathfinder
    std::vector<CombinationResult> evaluateCombinations(
        EdgeId edgeId,
        const EdgeLayout& baseLayout,
        const std::unordered_map<EdgeId, EdgeLayout>& assignedLayouts,
        const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
        const std::vector<ForbiddenZone>& forbiddenZones,
        IPathFinder& pathFinder);

    /// Create a candidate layout for a specific source/target edge combination
    /// Calculates actual orthogonal path using A* pathfinder
    /// Uses member pathFinder_
    EdgeLayout createCandidateLayout(
        const EdgeLayout& base,
        NodeEdge sourceEdge,
        NodeEdge targetEdge,
        const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
        ObstacleMap& obstacles,
        bool& pathFound);

    /// Thread-safe version with explicit pathfinder
    EdgeLayout createCandidateLayout(
        const EdgeLayout& base,
        NodeEdge sourceEdge,
        NodeEdge targetEdge,
        const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
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

    /// Get effective grid size (uses default if not set)
    float effectiveGridSize() const;

    /// Calculate forbidden zones for all nodes
    /// Uses base margin (MIN_NODE_GRID_DISTANCE * gridSize) around each node
    /// @param nodeLayouts Node positions
    /// @param gridSize Grid cell size
    /// @return Vector of forbidden zones
    std::vector<ForbiddenZone> calculateForbiddenZones(
        const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
        float gridSize) const;

    /// Result of cooperative rerouting for an edge pair
    struct EdgePairResult {
        EdgeLayout layoutA;
        EdgeLayout layoutB;
        int combinedScore;
        bool valid = false;
    };

    /// Resolve overlapping edge pair by cooperative rerouting
    /// Removes both edges from obstacle map and evaluates 16×16 = 256 combinations
    /// to find the best non-overlapping (pathA, pathB) pair.
    /// @param edgeIdA First edge
    /// @param edgeIdB Second edge
    /// @param layoutA Current layout for first edge
    /// @param layoutB Current layout for second edge
    /// @param assignedLayouts All currently assigned layouts
    /// @param nodeLayouts Node positions
    /// @param forbiddenZones Pre-calculated forbidden zones
    /// @return Best non-overlapping pair result (valid=false if none found)
    EdgePairResult resolveOverlappingPair(
        EdgeId edgeIdA, EdgeId edgeIdB,
        const EdgeLayout& layoutA, const EdgeLayout& layoutB,
        const std::unordered_map<EdgeId, EdgeLayout>& assignedLayouts,
        const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
        const std::vector<ForbiddenZone>& forbiddenZones);

    /// Evaluate self-loop combinations for a single edge
    /// Tries all 4 directions (Right, Left, Top, Bottom) and returns
    /// sorted results based on penalty scores.
    /// @param edgeId Edge being optimized
    /// @param baseLayout Original layout
    /// @param assignedLayouts Layouts already assigned (for overlap checking)
    /// @param nodeLayouts Node positions
    /// @param forbiddenZones Pre-calculated forbidden zones
    /// @return Vector of valid combination results, sorted by score (best first)
    std::vector<CombinationResult> evaluateSelfLoopCombinations(
        EdgeId edgeId,
        const EdgeLayout& baseLayout,
        const std::unordered_map<EdgeId, EdgeLayout>& assignedLayouts,
        const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
        const std::vector<ForbiddenZone>& forbiddenZones);
};


}  // namespace arborvia
