#pragma once

#include "arborvia/layout/api/IEdgeOptimizer.h"
#include "arborvia/layout/api/IPathFinder.h"
#include "arborvia/layout/config/LayoutTypes.h"
#include "../../sugiyama/routing/EdgeRoutingUtils.h"

#include <future>
#include <memory>
#include <vector>

namespace arborvia {

// Forward declarations
class UnifiedRetryChain;


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
    explicit AStarEdgeOptimizer(
        std::shared_ptr<IPathFinder> pathFinder = nullptr);

    /// Destructor (defined in .cpp for unique_ptr with forward-declared type)
    ~AStarEdgeOptimizer();

    /// Optimize edge routing by evaluating all 16 combinations per edge
    /// Uses A* pathfinding for each combination to find optimal paths
    /// @param edges Edges to optimize (processed in order)
    /// @param currentLayouts Current edge layouts (used as base for combinations)
    /// @param nodeLayouts Node positions
    /// @param gridSize Grid cell size for pathfinding and snap calculations
    /// @param movedNodes Nodes that were moved (endpoints on other nodes are fixed)
    /// @return Map of optimized edge layouts
    std::unordered_map<EdgeId, EdgeLayout> optimize(
        const std::vector<EdgeId>& edges,
        const std::unordered_map<EdgeId, EdgeLayout>& currentLayouts,
        const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
        float gridSize,
        const std::unordered_set<NodeId>& movedNodes = {}) override;

    const char* algorithmName() const override { return "AStar"; }

    /// Regenerate bendPoints using A* pathfinding
    /// Preserves existing sourceEdge/targetEdge, only recreates path geometry
    void regenerateBendPoints(
        const std::vector<EdgeId>& edges,
        std::unordered_map<EdgeId, EdgeLayout>& edgeLayouts,
        const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
        float gridSize) override;

    /// Set pathfinder
    void setPathFinder(std::shared_ptr<IPathFinder> pathFinder);

    /// Enable/disable parallel evaluation (default: true)
    void setParallelEnabled(bool enabled) { parallelEnabled_ = enabled; }

private:
    std::shared_ptr<IPathFinder> pathFinder_;
    std::unique_ptr<UnifiedRetryChain> retryChain_;
    float gridSize_ = 0.0f;
    float lastRetryChainGridSize_ = 0.0f;
    bool parallelEnabled_ = true;

    /// Initialize or return the unified retry chain
    UnifiedRetryChain& getRetryChain(float gridSize);

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
};


}  // namespace arborvia
