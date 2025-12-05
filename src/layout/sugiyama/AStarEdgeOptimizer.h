#pragma once

#include "arborvia/layout/IEdgeOptimizer.h"
#include "arborvia/layout/IPathFinder.h"
#include "EdgeScorer.h"

#include <memory>
#include <vector>

namespace arborvia {


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
    /// Constructor with optional pathfinder and scoring weights
    /// @param pathFinder Pathfinder for calculating orthogonal routes (uses AStarPathFinder if null)
    /// @param weights Scoring weights for optimization
    /// @param gridSize Grid cell size for pathfinding (0 = use default)
    explicit AStarEdgeOptimizer(
        std::shared_ptr<IPathFinder> pathFinder = nullptr,
        const ScoringWeights& weights = {},
        float gridSize = 0.0f);

    /// Legacy constructor for backward compatibility
    explicit AStarEdgeOptimizer(const ScoringWeights& weights);

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

    /// Set scoring weights
    void setWeights(const ScoringWeights& weights);

    /// Get current scoring weights
    const ScoringWeights& weights() const;

    /// Set pathfinder
    void setPathFinder(std::shared_ptr<IPathFinder> pathFinder);

    /// Set grid size for pathfinding
    void setGridSize(float gridSize);

private:
    EdgeScorer scorer_;
    std::shared_ptr<IPathFinder> pathFinder_;
    float gridSize_ = 0.0f;

    /// Result of evaluating a single edge combination
    struct CombinationResult {
        NodeEdge sourceEdge;
        NodeEdge targetEdge;
        int score;
        EdgeLayout layout;
        bool valid = true;  // False if no valid path exists
    };

    /// Evaluate all 16 combinations for a single edge
    /// @param edgeId Edge being optimized
    /// @param baseLayout Original layout to derive combinations from
    /// @param assignedLayouts Layouts already assigned (for intersection scoring)
    /// @param nodeLayouts Node positions
    /// @return Vector of valid combination results, sorted by score (best first)
    std::vector<CombinationResult> evaluateAllCombinations(
        EdgeId edgeId,
        const EdgeLayout& baseLayout,
        const std::unordered_map<EdgeId, EdgeLayout>& assignedLayouts,
        const std::unordered_map<NodeId, NodeLayout>& nodeLayouts);

    /// Create a candidate layout for a specific source/target edge combination
    /// Calculates actual orthogonal path using A* pathfinder
    /// @param base Original edge layout
    /// @param sourceEdge Desired source node edge
    /// @param targetEdge Desired target node edge
    /// @param nodeLayouts Node positions for snap point and path calculation
    /// @param[out] pathFound Set to true if valid path was found
    /// @return New EdgeLayout with actual bend points (or empty if no path)
    EdgeLayout createCandidateLayout(
        const EdgeLayout& base,
        NodeEdge sourceEdge,
        NodeEdge targetEdge,
        const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
        bool& pathFound);

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
};


}  // namespace arborvia
