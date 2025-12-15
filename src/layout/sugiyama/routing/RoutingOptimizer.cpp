#include "RoutingOptimizer.h"
#include "arborvia/core/GeometryUtils.h"
#include "../../snap/GridSnapCalculator.h"
#include "layout/optimization/OptimizerRegistry.h"
#include "arborvia/layout/config/OptimizerConfig.h"
#include "arborvia/layout/api/EdgePenaltySystem.h"
#include "arborvia/layout/util/LayoutUtils.h"
#include <map>

namespace arborvia {

RoutingOptimizer::RoutingOptimizer(
    std::shared_ptr<IPathFinder> pathFinder,
    IEdgeOptimizer* edgeOptimizer)
    : pathFinder_(std::move(pathFinder))
    , edgeOptimizer_(edgeOptimizer) {
}

void RoutingOptimizer::optimize(
    EdgeRouting::Result& result,
    const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
    const LayoutOptions& options) {

    // Create optimizer based on postDragAlgorithm
    IEdgeOptimizer* optimizer = edgeOptimizer_;
    std::unique_ptr<IEdgeOptimizer> fallbackOptimizer;

    if (!optimizer && options.optimizationOptions.postDragAlgorithm != PostDragAlgorithm::None) {
        switch (options.optimizationOptions.postDragAlgorithm) {
            case PostDragAlgorithm::AStar: {
                OptimizerConfig config = OptimizerConfig::balanced();
                config.pathFinder = pathFinder_;
                config.penaltySystem = EdgePenaltySystem::createDefault();
                fallbackOptimizer = OptimizerRegistry::instance().create("AStar", config);
                break;
            }
            case PostDragAlgorithm::Geometric: {
                OptimizerConfig config = OptimizerConfig::aggressive();
                config.penaltySystem = EdgePenaltySystem::createDefault();
                fallbackOptimizer = OptimizerRegistry::instance().create("Geometric", config);
                break;
            }
            case PostDragAlgorithm::None:
                break;
        }
        optimizer = fallbackOptimizer.get();
    }

    if (!optimizer || result.edgeLayouts.empty()) {
        return;
    }

    // Collect all edge IDs for optimization
    std::vector<EdgeId> edgeIds;
    edgeIds.reserve(result.edgeLayouts.size());
    for (const auto& [edgeId, layout] : result.edgeLayouts) {
        edgeIds.push_back(edgeId);
    }

    // Run optimizer on edges with their final snap positions
    float gridSize = constants::effectiveGridSize(options.gridConfig.cellSize);
    auto optimizedLayouts = optimizer->optimize(edgeIds, result.edgeLayouts, nodeLayouts, gridSize);

    // Merge optimized layouts
    // IMPORTANT: Always recalculate snap indices from final positions
    // The optimizer may change positions while keeping the same NodeEdge,
    // so preserving old indices would cause position/index mismatch
    for (auto& [edgeId, layout] : optimizedLayouts) {
        auto& existing = result.edgeLayouts[edgeId];

        // Self-loops: always trust optimizer's snap indices (position-based from SelfLoopRouter)
        bool isSelfLoop = (layout.from == layout.to);
        if (isSelfLoop) {
            existing = std::move(layout);
            continue;
        }

        // Copy the optimized layout
        // NOTE: snapIndex is no longer stored - computed from position as needed
        existing = std::move(layout);
    }

    // NOTE: fixInvalidSnapIndices removed - snap indices are no longer stored

    // Update label positions after optimization
    for (auto& [edgeId, layout] : result.edgeLayouts) {
        layout.labelPosition = LayoutUtils::calculateEdgeLabelPosition(layout);
    }
}

// NOTE: fixInvalidSnapIndices removed - snap indices are no longer stored
// They are computed from positions as needed using GridSnapCalculator::getCandidateIndexFromPosition

} // namespace arborvia
