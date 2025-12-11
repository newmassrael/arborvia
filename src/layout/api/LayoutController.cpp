#include "arborvia/layout/api/LayoutController.h"
#include "arborvia/layout/util/LayoutUtils.h"
#include "arborvia/layout/constraints/PositionFinder.h"
#include "layout/interactive/ConstraintManager.h"
#include "sugiyama/routing/EdgeRouting.h"

#include <algorithm>
#include <set>

namespace arborvia {

namespace {
    constexpr float DEFAULT_GRID_SIZE = 10.0f;
    constexpr float CONSTRAINT_SEARCH_RADIUS = 400.0f;
    constexpr int CONSTRAINT_MAX_ITERATIONS = 2000;
}

LayoutController::LayoutController(const Graph& graph, const LayoutOptions& options)
    : graph_(graph)
    , options_(options) {

    float gridSize = options_.gridConfig.cellSize > 0 ? options_.gridConfig.cellSize : DEFAULT_GRID_SIZE;
    
    // Initialize constraint manager with default constraints
    auto config = ConstraintConfig::createDefault();
    constraintManager_ = ConstraintFactory::create(config);
    
    positionFinder_ = std::make_unique<PositionFinder>(
        PositionFinderConfig{gridSize, CONSTRAINT_SEARCH_RADIUS, gridSize, CONSTRAINT_MAX_ITERATIONS});
}

LayoutController::~LayoutController() = default;

void LayoutController::initializeFrom(const LayoutResult& result) {
    nodeLayouts_ = result.nodeLayouts();
    edgeLayouts_ = result.edgeLayouts();
}

void LayoutController::initializeFrom(
    const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
    const std::unordered_map<EdgeId, EdgeLayout>& edgeLayouts) {
    nodeLayouts_ = nodeLayouts;
    edgeLayouts_ = edgeLayouts;
}

const NodeLayout* LayoutController::getNode(NodeId id) const {
    auto it = nodeLayouts_.find(id);
    return it != nodeLayouts_.end() ? &it->second : nullptr;
}

const EdgeLayout* LayoutController::getEdge(EdgeId id) const {
    auto it = edgeLayouts_.find(id);
    return it != edgeLayouts_.end() ? &it->second : nullptr;
}

NodeMoveResult LayoutController::moveNode(NodeId nodeId, Point newPosition) {
    // Check if node exists
    auto nodeIt = nodeLayouts_.find(nodeId);
    if (nodeIt == nodeLayouts_.end()) {
        return NodeMoveResult::fail(newPosition, "Node not found");
    }

    // Save original state for rollback
    Point originalPosition = nodeIt->second.position;
    auto originalEdgeLayouts = edgeLayouts_;

    float gridSize = options_.gridConfig.cellSize > 0 ? options_.gridConfig.cellSize : DEFAULT_GRID_SIZE;

    // Step 1: Validate against all constraints using ConstraintManager
    ConstraintContext ctx{nodeId, newPosition, nodeLayouts_, edgeLayouts_, &graph_, gridSize};
    auto validation = constraintManager_->validate(ctx);
    
    if (!validation.valid) {
        return NodeMoveResult::fail(newPosition, 
            validation.failedConstraint + ": " + validation.reason);
    }

    // Step 2: Apply the position change
    nodeIt->second.position = newPosition;

    // Step 3: Update edge routing
    auto affectedEdges = getExpandedAffectedEdges(nodeId);
    updateEdgeRouting(affectedEdges, {nodeId});

    // Step 4: Final validation - check ALL constraints
    auto constraintResult = constraintManager_->validateFinalState(nodeLayouts_, edgeLayouts_);

    if (!constraintResult.satisfied) {
        // Rollback: restore original state
        nodeIt->second.position = originalPosition;
        edgeLayouts_ = originalEdgeLayouts;
        return NodeMoveResult::fail(newPosition, constraintResult.summary());
    }

    // Success
    return NodeMoveResult::ok(newPosition, affectedEdges);
}

EdgeRouteResult LayoutController::rerouteEdges(const std::vector<EdgeId>& edges) {
    EdgeRouteResult result;

    if (edges.empty()) {
        result.success = true;
        return result;
    }

    // Update edge routing
    EdgeRouting routing;
    routing.updateEdgeRoutingWithOptimization(
        edgeLayouts_, nodeLayouts_, edges, options_, {});

    // Validate result
    auto validation = constraintManager_->validateFinalState(nodeLayouts_, edgeLayouts_);

    result.success = validation.satisfied;
    result.failedEdges = {};  // No longer tracked at this level (pre-routing validation handles this)
    result.remainingOverlaps = validation.overlappingEdgePairs;

    if (!result.success) {
        result.reason = validation.summary();
    }

    return result;
}

EdgeRouteResult LayoutController::rerouteAllEdges() {
    std::vector<EdgeId> allEdges;
    allEdges.reserve(edgeLayouts_.size());
    for (const auto& [id, _] : edgeLayouts_) {
        allEdges.push_back(id);
    }
    return rerouteEdges(allEdges);
}

LayoutController::DragPreview LayoutController::getDragPreview(
    NodeId nodeId,
    Point proposedPosition) const {

    DragPreview preview;
    preview.visualPosition = proposedPosition;

    // Check if node exists
    auto nodeIt = nodeLayouts_.find(nodeId);
    if (nodeIt == nodeLayouts_.end()) {
        preview.isValid = false;
        preview.reason = "Node not found";
        return preview;
    }

    float gridSize = options_.gridConfig.cellSize > 0 ? options_.gridConfig.cellSize : DEFAULT_GRID_SIZE;

    // Validate against all constraints using ConstraintManager
    ConstraintContext ctx{nodeId, proposedPosition, nodeLayouts_, edgeLayouts_, &graph_, gridSize};
    auto validation = constraintManager_->validate(ctx);

    preview.isValid = validation.valid;
    if (!validation.valid) {
        preview.reason = validation.failedConstraint + ": " + validation.reason;
    }

    return preview;
}

NodeMoveResult LayoutController::completeDrag(NodeId nodeId, Point finalPosition) {
    return moveNode(nodeId, finalPosition);
}

FinalStateValidationResult LayoutController::validateAll() const {
    return constraintManager_->validateFinalState(nodeLayouts_, edgeLayouts_);
}

bool LayoutController::canMoveNodeTo(NodeId nodeId, Point position) const {
    float gridSize = options_.gridConfig.cellSize > 0 ? options_.gridConfig.cellSize : DEFAULT_GRID_SIZE;
    ConstraintContext ctx{nodeId, position, nodeLayouts_, edgeLayouts_, &graph_, gridSize};
    auto validation = constraintManager_->validate(ctx);
    return validation.valid;
}

void LayoutController::updateEdgeRouting(
    const std::vector<EdgeId>& affectedEdges,
    const std::unordered_set<NodeId>& movedNodes) {

    EdgeRouting routing;
    routing.updateEdgeRoutingWithOptimization(
        edgeLayouts_, nodeLayouts_, affectedEdges, options_, movedNodes);
}

std::vector<EdgeId> LayoutController::getConnectedEdges(NodeId nodeId) const {
    return graph_.getConnectedEdges(nodeId);
}

std::vector<EdgeId> LayoutController::getExpandedAffectedEdges(NodeId nodeId) const {
    // Start with directly connected edges
    auto directEdges = graph_.getConnectedEdges(nodeId);
    std::set<EdgeId> affectedSet(directEdges.begin(), directEdges.end());

    // Find edges passing through the node's area
    auto nodeIt = nodeLayouts_.find(nodeId);
    if (nodeIt == nodeLayouts_.end()) {
        return directEdges;
    }

    const auto& node = nodeIt->second;
    constexpr float MARGIN = 5.0f;

    for (const auto& [edgeId, edge] : edgeLayouts_) {
        if (affectedSet.count(edgeId)) continue;  // Already included

        // Check if any segment passes through node area
        auto points = edge.allPoints();
        for (size_t i = 0; i + 1 < points.size(); ++i) {
            const Point& p1 = points[i];
            const Point& p2 = points[i + 1];

            // Check if segment intersects with node bounds + margin
            float minX = std::min(p1.x, p2.x);
            float maxX = std::max(p1.x, p2.x);
            float minY = std::min(p1.y, p2.y);
            float maxY = std::max(p1.y, p2.y);

            float nodeMinX = node.position.x - MARGIN;
            float nodeMaxX = node.position.x + node.size.width + MARGIN;
            float nodeMinY = node.position.y - MARGIN;
            float nodeMaxY = node.position.y + node.size.height + MARGIN;

            // AABB intersection
            if (maxX >= nodeMinX && minX <= nodeMaxX &&
                maxY >= nodeMinY && minY <= nodeMaxY) {
                affectedSet.insert(edgeId);
                break;
            }
        }
    }

    return std::vector<EdgeId>(affectedSet.begin(), affectedSet.end());
}

}  // namespace arborvia
