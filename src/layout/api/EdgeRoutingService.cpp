#include "arborvia/layout/api/EdgeRoutingService.h"
#include "arborvia/layout/constraints/ConstraintGateway.h"
#include "arborvia/layout/constraints/IEdgeConstraint.h"
#include "arborvia/layout/util/LayoutUtils.h"
#include "arborvia/core/Graph.h"
#include "arborvia/core/GeometryUtils.h"
#include "arborvia/common/Logger.h"
#include "sugiyama/routing/EdgeRouting.h"

namespace arborvia {

namespace {

// Convert postDragAlgorithm to dragAlgorithm for post-drop processing
LayoutOptions applyPostDragAlgorithm(const LayoutOptions& options) {
    LayoutOptions result = options;
    switch (options.optimizationOptions.postDragAlgorithm) {
        case PostDragAlgorithm::AStar:
            result.optimizationOptions.dragAlgorithm = DragAlgorithm::AStar;
            break;
        case PostDragAlgorithm::Geometric:
        case PostDragAlgorithm::None:
            result.optimizationOptions.dragAlgorithm = DragAlgorithm::Geometric;
            break;
    }
    return result;
}

}  // namespace

EdgeRoutingService::EdgeRoutingService(std::shared_ptr<IPathFinder> pathFinder)
    : pathFinder_(std::move(pathFinder))
    , edgeRouting_(std::make_unique<EdgeRouting>(pathFinder_))
    , constraintGateway_(std::make_unique<ConstraintGateway>()) {
}

EdgeRoutingService::~EdgeRoutingService() = default;

EdgeRoutingService::EdgeRoutingService(EdgeRoutingService&&) noexcept = default;
EdgeRoutingService& EdgeRoutingService::operator=(EdgeRoutingService&&) noexcept = default;

EdgeRoutingService::RoutingResult EdgeRoutingService::routeInitial(
    const Graph& graph,
    const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
    const std::unordered_set<EdgeId>& reversedEdges,
    const LayoutOptions& options) {

    LOG_DEBUG("[EdgeRoutingService] routeInitial: edges={}", graph.edges().size());

    // 1. Create initial layouts WITHOUT optimization (snap positions not final yet)
    auto result = edgeRouting_->route(graph, nodeLayouts, reversedEdges, options, true);

    // 2. Distribute snap points if enabled
    if (options.autoSnapPoints) {
        edgeRouting_->distributeAutoSnapPoints(
            result, nodeLayouts, options.gridConfig.cellSize);
    }

    // 3. Run optimizer
    edgeRouting_->optimizeRouting(result, nodeLayouts, options);

    // 4. Ensure label positions
    ensureLabelPositions(result.edgeLayouts);

    // 5. Validate and wrap
    float gridSize = constants::effectiveGridSize(options.gridConfig.cellSize);
    return validateAndWrap(result.edgeLayouts, nodeLayouts, gridSize);
}

EdgeRoutingService::RoutingResult EdgeRoutingService::updateAfterNodeMove(
    const std::unordered_map<EdgeId, EdgeLayout>& currentLayouts,
    const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
    const std::vector<EdgeId>& affectedEdges,
    const LayoutOptions& options,
    const std::unordered_set<NodeId>& movedNodes) {

    LOG_DEBUG("[EdgeRoutingService] updateAfterNodeMove: affectedEdges={}, movedNodes={}",
              affectedEdges.size(), movedNodes.size());

    // Work on a copy
    std::unordered_map<EdgeId, EdgeLayout> workingLayouts = currentLayouts;
    LayoutOptions postDropOptions = applyPostDragAlgorithm(options);

    // First pass: update directly affected edges
    edgeRouting_->updateEdgeRoutingWithOptimization(
        workingLayouts, nodeLayouts, affectedEdges, postDropOptions, movedNodes);

    // Second pass: find edges passing through moved nodes' forbidden zones
    // Use set for O(1) lookup instead of O(n) std::find
    std::unordered_set<EdgeId> affectedSet(affectedEdges.begin(), affectedEdges.end());
    std::vector<EdgeId> edgesToRecalculate;

    for (const auto& [edgeId, edgeLayout] : workingLayouts) {
        if (affectedSet.count(edgeId)) {
            continue;
        }

        for (const auto& movedNodeId : movedNodes) {
            auto nodeIt = nodeLayouts.find(movedNodeId);
            if (nodeIt != nodeLayouts.end()) {
                if (LayoutUtils::edgePassesThroughNode(edgeLayout, nodeIt->second)) {
                    edgesToRecalculate.push_back(edgeId);
                    LOG_DEBUG("[EdgeRoutingService] Edge {} passes through moved node {}",
                              edgeId, movedNodeId);
                    break;
                }
            }
        }
    }

    // Recalculate pass-through edges
    if (!edgesToRecalculate.empty()) {
        LOG_DEBUG("[EdgeRoutingService] Re-routing {} pass-through edges", edgesToRecalculate.size());
        std::unordered_set<NodeId> emptySet;
        edgeRouting_->updateEdgeRoutingWithOptimization(
            workingLayouts, nodeLayouts, edgesToRecalculate, postDropOptions, emptySet);
    }

    // Ensure label positions
    ensureLabelPositions(workingLayouts);

    // Validate and wrap
    float gridSize = constants::effectiveGridSize(options.gridConfig.cellSize);
    return validateAndWrap(workingLayouts, nodeLayouts, gridSize);
}

EdgeRoutingService::RoutingResult EdgeRoutingService::updateAfterSnapMove(
    const std::unordered_map<EdgeId, EdgeLayout>& currentLayouts,
    const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
    const std::vector<EdgeId>& affectedEdges,
    const LayoutOptions& options) {

    LOG_DEBUG("[EdgeRoutingService] updateAfterSnapMove: affectedEdges={}", affectedEdges.size());

    // Work on a copy
    std::unordered_map<EdgeId, EdgeLayout> workingLayouts = currentLayouts;
    LayoutOptions snapOptions = applyPostDragAlgorithm(options);

    // Update affected edges (no moved nodes for snap move)
    std::unordered_set<NodeId> emptyMovedNodes;
    edgeRouting_->updateEdgeRoutingWithOptimization(
        workingLayouts, nodeLayouts, affectedEdges, snapOptions, emptyMovedNodes);

    // Ensure label positions
    ensureLabelPositions(workingLayouts);

    // Validate and wrap
    float gridSize = constants::effectiveGridSize(options.gridConfig.cellSize);
    return validateAndWrap(workingLayouts, nodeLayouts, gridSize);
}

EdgeRoutingService::RoutingResult EdgeRoutingService::validateAndWrap(
    std::unordered_map<EdgeId, EdgeLayout>& rawLayouts,
    const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
    float gridSize) {

    RoutingResult result;

    EdgeConstraintContext ctx{rawLayouts, nodeLayouts, gridSize, 1.0f};

    for (auto& [edgeId, layout] : rawLayouts) {
        auto validated = constraintGateway_->validateAndWrapRelaxed(layout, ctx);
        if (validated) {
            result.validatedLayouts.emplace(edgeId, std::move(*validated));
        } else {
            LOG_ERROR("[EdgeRoutingService] Edge {} REJECTED - failed hard constraint validation", edgeId);
            result.rejectedEdges.push_back(edgeId);
        }
    }

    LOG_DEBUG("[EdgeRoutingService] Validation complete: valid={}, rejected={}",
              result.validCount(), result.rejectedCount());

    return result;
}

void EdgeRoutingService::ensureLabelPositions(std::unordered_map<EdgeId, EdgeLayout>& layouts) {
    for (auto& [edgeId, layout] : layouts) {
        layout.labelPosition = LayoutUtils::calculateEdgeLabelPosition(layout);
    }
}

}  // namespace arborvia
