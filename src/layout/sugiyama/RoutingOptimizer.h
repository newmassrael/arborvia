#pragma once

#include "EdgeRouting.h"
#include "arborvia/layout/LayoutOptions.h"
#include "arborvia/layout/IEdgeOptimizer.h"
#include "arborvia/layout/IPathFinder.h"
#include <memory>

namespace arborvia {

/**
 * @brief Handles post-layout edge routing optimization.
 *
 * This class encapsulates the edge optimization workflow:
 * 1. Creates/selects appropriate optimizer based on options
 * 2. Runs optimization on all edges
 * 3. Preserves snap indices during optimization
 * 4. Fixes any indices that need recalculation
 */
class RoutingOptimizer {
public:
    /**
     * @brief Construct a RoutingOptimizer.
     * @param pathFinder Shared pathfinder for A* calculations.
     * @param edgeOptimizer Optional pre-configured edge optimizer.
     */
    RoutingOptimizer(
        std::shared_ptr<IPathFinder> pathFinder,
        IEdgeOptimizer* edgeOptimizer = nullptr);

    /**
     * @brief Optimize edge routing in the result.
     *
     * @param result Layout result containing edge layouts to optimize.
     * @param nodeLayouts All node layouts.
     * @param options Layout options controlling optimization behavior.
     */
    void optimize(
        EdgeRouting::Result& result,
        const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
        const LayoutOptions& options);

private:
    /**
     * @brief Fix snap indices that were invalidated during optimization.
     */
    void fixInvalidSnapIndices(
        EdgeRouting::Result& result,
        const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
        float effectiveGridSize);

    std::shared_ptr<IPathFinder> pathFinder_;
    IEdgeOptimizer* edgeOptimizer_;
};

} // namespace arborvia
