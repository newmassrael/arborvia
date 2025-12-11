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
        const float gridSize = options.gridConfig.cellSize;

        switch (options.optimizationOptions.postDragAlgorithm) {
            case PostDragAlgorithm::AStar: {
                OptimizerConfig config = OptimizerConfig::balanced();
                config.gridSize = gridSize;
                config.pathFinder = pathFinder_;
                config.penaltySystem = EdgePenaltySystem::createDefault();
                fallbackOptimizer = OptimizerRegistry::instance().create("AStar", config);
                break;
            }
            case PostDragAlgorithm::Geometric: {
                OptimizerConfig config = OptimizerConfig::aggressive();
                config.gridSize = gridSize;
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
    auto optimizedLayouts = optimizer->optimize(edgeIds, result.edgeLayouts, nodeLayouts);

    // Merge optimized layouts
    // IMPORTANT: Preserve snap indices (optimizer sets them to -1, we keep grid-based values)
    // BUT: Only if NodeEdge hasn't changed (indices are specific to NodeEdge)
    for (auto& [edgeId, layout] : optimizedLayouts) {
        auto& existing = result.edgeLayouts[edgeId];

        // Save original values
        NodeEdge origSourceEdge = existing.sourceEdge;
        NodeEdge origTargetEdge = existing.targetEdge;
        int preservedSourceSnapIndex = existing.sourceSnapIndex;
        int preservedTargetSnapIndex = existing.targetSnapIndex;

        existing = std::move(layout);

        // Only restore snap indices if NodeEdge didn't change
        if (existing.sourceEdge == origSourceEdge) {
            existing.sourceSnapIndex = preservedSourceSnapIndex;
        }
        if (existing.targetEdge == origTargetEdge) {
            existing.targetSnapIndex = preservedTargetSnapIndex;
        }
    }

    // Fix any -1 indices that weren't restored (NodeEdge changed during optimization)
    float gridSizeToUse = constants::effectiveGridSize(options.gridConfig.cellSize);
    fixInvalidSnapIndices(result, nodeLayouts, gridSizeToUse);

    // Update label positions after optimization
    for (auto& [edgeId, layout] : result.edgeLayouts) {
        layout.labelPosition = LayoutUtils::calculateEdgeLabelPosition(layout);
    }
}

void RoutingOptimizer::fixInvalidSnapIndices(
    EdgeRouting::Result& result,
    const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
    float effectiveGridSize) {

    // Group edges by (node, nodeEdge) to properly calculate snap indices
    std::map<std::pair<NodeId, NodeEdge>, std::vector<std::pair<EdgeId, bool>>> connectionsByNodeEdge;
    for (auto& [edgeId, layout] : result.edgeLayouts) {
        connectionsByNodeEdge[{layout.from, layout.sourceEdge}].push_back({edgeId, true});
        connectionsByNodeEdge[{layout.to, layout.targetEdge}].push_back({edgeId, false});
    }

    for (auto& [key, connections] : connectionsByNodeEdge) {
        auto [nodeId, nodeEdge] = key;
        auto nodeIt = nodeLayouts.find(nodeId);
        if (nodeIt == nodeLayouts.end()) continue;
        const NodeLayout& node = nodeIt->second;

        int totalConnections = static_cast<int>(connections.size());
        for (int i = 0; i < totalConnections; ++i) {
            auto [edgeId, isSource] = connections[i];
            EdgeLayout& layout = result.edgeLayouts[edgeId];

            int existingIndex = isSource ? layout.sourceSnapIndex : layout.targetSnapIndex;
            if (existingIndex < 0) {  // -1 means needs fixing
                int candidateIndex = 0;
                Point snapPoint = GridSnapCalculator::calculateSnapPosition(
                    node, nodeEdge, i, totalConnections, effectiveGridSize, &candidateIndex);
                if (isSource) {
                    layout.sourceSnapIndex = candidateIndex;
                    layout.sourcePoint = snapPoint;
                } else {
                    layout.targetSnapIndex = candidateIndex;
                    layout.targetPoint = snapPoint;
                }
            }
        }
    }
}

} // namespace arborvia
