#include "DragOptimizationHandler.h"
#include "../sugiyama/routing/EdgeRoutingUtils.h"
#include "layout/optimization/OptimizerRegistry.h"
#include "arborvia/layout/config/OptimizerConfig.h"
#include "arborvia/layout/api/EdgePenaltySystem.h"
#include <algorithm>
#include <cmath>
#include "arborvia/common/Logger.h"


namespace arborvia {

namespace constants {
    constexpr int SNAP_INDEX_UNASSIGNED = -1;
}

DragOptimizationHandler::DragOptimizationHandler(
    std::shared_ptr<IPathFinder> pathFinder,
    SnapUpdateFunc snapUpdateFunc,
    RecalcBendPointsFunc recalcFunc)
    : pathFinder_(std::move(pathFinder))
    , snapUpdateFunc_(std::move(snapUpdateFunc))
    , recalcFunc_(std::move(recalcFunc)) {
}

bool DragOptimizationHandler::segmentPenetratesNodeInterior(
    const Point& p1, const Point& p2, const NodeLayout& node) {
    return EdgeRoutingUtils::segmentPenetratesNodeInterior(p1, p2, node);
}

std::vector<EdgeId> DragOptimizationHandler::collectPenetratingEdges(
    const std::unordered_map<EdgeId, EdgeLayout>& edgeLayouts,
    const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
    const std::unordered_set<NodeId>& nodesToCheck,
    const std::vector<EdgeId>& excludeEdges) {

    std::vector<EdgeId> penetratingEdges;

    for (const auto& [edgeId, edgeLayout] : edgeLayouts) {
        // Skip excluded edges
        if (std::find(excludeEdges.begin(), excludeEdges.end(), edgeId) != excludeEdges.end()) {
            continue;
        }
        // Skip self-loops
        if (edgeLayout.from == edgeLayout.to) {
            continue;
        }

        bool penetrates = false;
        for (NodeId nodeId : nodesToCheck) {
            if (penetrates) break;

            // Skip if edge connects to this node
            if (edgeLayout.from == nodeId || edgeLayout.to == nodeId) continue;

            auto nodeIt = nodeLayouts.find(nodeId);
            if (nodeIt == nodeLayouts.end()) continue;
            const NodeLayout& node = nodeIt->second;

            // Build path
            std::vector<Point> path;
            path.push_back(edgeLayout.sourcePoint);
            for (const auto& bp : edgeLayout.bendPoints) {
                path.push_back(bp.position);
            }
            path.push_back(edgeLayout.targetPoint);

            // Check each segment
            for (size_t i = 0; i + 1 < path.size(); ++i) {
                if (segmentPenetratesNodeInterior(path[i], path[i + 1], node)) {
                    penetrates = true;
                    break;
                }
            }
        }

        if (penetrates) {
            penetratingEdges.push_back(edgeId);
        }
    }

    return penetratingEdges;
}

std::vector<EdgeId> DragOptimizationHandler::collectAllPenetratingEdges(
    const std::unordered_map<EdgeId, EdgeLayout>& edgeLayouts,
    const std::unordered_map<NodeId, NodeLayout>& nodeLayouts) {

    std::vector<EdgeId> penetratingEdges;

    for (const auto& [edgeId, edgeLayout] : edgeLayouts) {
        // Skip self-loops
        if (edgeLayout.from == edgeLayout.to) continue;

        // Build path
        std::vector<Point> path;
        path.push_back(edgeLayout.sourcePoint);
        for (const auto& bp : edgeLayout.bendPoints) {
            path.push_back(bp.position);
        }
        path.push_back(edgeLayout.targetPoint);

        // Check against all nodes
        bool penetrates = false;
        for (const auto& [nodeId, nodeLayout] : nodeLayouts) {
            if (penetrates) break;
            if (nodeId == edgeLayout.from || nodeId == edgeLayout.to) continue;

            for (size_t i = 0; i + 1 < path.size(); ++i) {
                if (segmentPenetratesNodeInterior(path[i], path[i + 1], nodeLayout)) {
                    penetrates = true;
                    break;
                }
            }
        }

        if (penetrates) {
            penetratingEdges.push_back(edgeId);
        }
    }

    return penetratingEdges;
}

void DragOptimizationHandler::updateEdgeRoutingWithOptimization(
    std::unordered_map<EdgeId, EdgeLayout>& edgeLayouts,
    const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
    const std::vector<EdgeId>& affectedEdges,
    const LayoutOptions& options,
    const std::unordered_set<NodeId>& movedNodes,
    std::shared_ptr<IEdgeOptimizer>& edgeOptimizer) {

    float gridSize = options.gridConfig.cellSize;
    bool usedDragOptimizer = false;

    // Apply drag algorithm if enabled
    LOG_DEBUG("[DragOptimizationHandler] dragAlgorithm={} affectedEdges.size={}", 
              static_cast<int>(options.optimizationOptions.dragAlgorithm), affectedEdges.size());
    if (options.optimizationOptions.dragAlgorithm != DragAlgorithm::None && !affectedEdges.empty()) {
        std::unique_ptr<IEdgeOptimizer> optimizer;

        switch (options.optimizationOptions.dragAlgorithm) {
            case DragAlgorithm::Geometric: {
                LOG_DEBUG("[DragOptimizationHandler] Creating Geometric optimizer");
                OptimizerConfig config = OptimizerConfig::aggressive();
                config.preserveDirections = true;
                config.penaltySystem = EdgePenaltySystem::createDefault();
                optimizer = OptimizerRegistry::instance().create("Geometric", config);
                break;
            }
            case DragAlgorithm::AStar: {
                LOG_DEBUG("[DragOptimizationHandler] Creating AStar optimizer");
                OptimizerConfig config = OptimizerConfig::aggressive();
                config.preserveDirections = false;
                config.pathFinder = pathFinder_;
                config.penaltySystem = EdgePenaltySystem::createDefault();
                optimizer = OptimizerRegistry::instance().create("AStar", config);
                break;
            }
            case DragAlgorithm::None:
                LOG_DEBUG("[DragOptimizationHandler] DragAlgorithm::None - no optimizer created");
                break;
            case DragAlgorithm::HideUntilDrop:
                LOG_DEBUG("[DragOptimizationHandler] DragAlgorithm::HideUntilDrop - early return");
                return;
        }

        if (optimizer) {
            edgeOptimizer = std::move(optimizer);

            std::vector<EdgeId> edgesToOptimize;
            edgesToOptimize.reserve(affectedEdges.size());
            for (EdgeId edgeId : affectedEdges) {
                if (edgeLayouts.find(edgeId) != edgeLayouts.end()) {
                    edgesToOptimize.push_back(edgeId);
                }
            }

            // Pass movedNodes to optimizer - FixedEndpointPenalty will handle constraints
            LOG_DEBUG("[DragOptimizationHandler] Calling optimizer->optimize() for {} edges", edgesToOptimize.size());
            auto optimizedLayouts = edgeOptimizer->optimize(edgesToOptimize, edgeLayouts, nodeLayouts, gridSize, movedNodes);
            LOG_DEBUG("[DragOptimizationHandler] optimizer->optimize() returned {} layouts", optimizedLayouts.size());

            // Copy optimized layouts
            // The optimizer respects constraints, preserving positions and snap indices
            // for endpoints connected to unmoved nodes
            for (auto& [edgeId, layout] : optimizedLayouts) {
                auto it = edgeLayouts.find(edgeId);
                if (it != edgeLayouts.end()) {
                    LOG_DEBUG("[DragOptimizationHandler] Edge {} optimized: srcPt=({},{}) bendPts.size={} tgtPt=({},{})",
                              edgeId, layout.sourcePoint.x, layout.sourcePoint.y,
                              layout.bendPoints.size(),
                              layout.targetPoint.x, layout.targetPoint.y);
                    it->second = layout;
                }
            }

            snapUpdateFunc_(edgeLayouts, nodeLayouts, edgesToOptimize, movedNodes, gridSize, true);
            usedDragOptimizer = true;
        }
    } else {
        snapUpdateFunc_(edgeLayouts, nodeLayouts, affectedEdges, movedNodes, gridSize, false);
    }

    // Skip A* post-processing when drag optimizer was used
    if (usedDragOptimizer) {
        return;
    }

    // Check for edges penetrating moved nodes
    if (!movedNodes.empty()) {
        auto penetratingEdges = collectPenetratingEdges(edgeLayouts, nodeLayouts, movedNodes, affectedEdges);
        if (!penetratingEdges.empty()) {
            snapUpdateFunc_(edgeLayouts, nodeLayouts, penetratingEdges, movedNodes, gridSize, false);
        }
    }

    // Final validation: check ALL edges against ALL nodes
    {
        auto finalPenetratingEdges = collectAllPenetratingEdges(edgeLayouts, nodeLayouts);
        if (!finalPenetratingEdges.empty()) {
            for (EdgeId edgeId : finalPenetratingEdges) {
                auto it = edgeLayouts.find(edgeId);
                if (it != edgeLayouts.end()) {
                    recalcFunc_(it->second, nodeLayouts, gridSize, nullptr, &movedNodes);
                }
            }
        }
    }
}

} // namespace arborvia
