#pragma once

#include "EdgeRouting.h"
#include "arborvia/layout/config/LayoutOptions.h"
#include "arborvia/layout/api/IEdgeOptimizer.h"
#include "arborvia/layout/api/IPathFinder.h"
#include <memory>

namespace arborvia {

/**
 * @brief Handles post-layout edge routing optimization.
 *
 * This class encapsulates the edge optimization workflow:
 * 1. Creates/selects appropriate optimizer based on options
 * 2. Runs optimization on all edges
 * NOTE: Snap indices are no longer stored - computed from positions as needed
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
    std::shared_ptr<IPathFinder> pathFinder_;
    IEdgeOptimizer* edgeOptimizer_;
};

} // namespace arborvia
