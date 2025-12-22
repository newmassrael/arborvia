#include "arborvia/layout/api/LayoutController.h"
#include "arborvia/layout/api/EdgeRoutingService.h"
#include "arborvia/layout/util/LayoutUtils.h"
#include "arborvia/layout/constraints/PositionFinder.h"
#include "arborvia/core/GeometryUtils.h"
#include "layout/interactive/ConstraintManager.h"
#include "sugiyama/routing/EdgeRouting.h"
#include "arborvia/common/Logger.h"

#include <algorithm>
#include <set>

namespace arborvia {

namespace {
    constexpr float DEFAULT_GRID_SIZE = 10.0f;
    constexpr float CONSTRAINT_SEARCH_RADIUS = 400.0f;
    constexpr int CONSTRAINT_MAX_ITERATIONS = 2000;

    // Default size for Point→Regular conversion (in grid units)
    // When a Point node has no saved original size, use this default
    constexpr int DEFAULT_REGULAR_WIDTH_GRID = 6;   // 6 grid units (120px at gridSize=20)
    constexpr int DEFAULT_REGULAR_HEIGHT_GRID = 3;  // 3 grid units (60px at gridSize=20)
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

    edgeRoutingService_ = std::make_unique<EdgeRoutingService>();
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

NodeMoveResult LayoutController::setNodeType(NodeId nodeId, NodeType newType) {
    // Check if node exists
    auto nodeIt = nodeLayouts_.find(nodeId);
    if (nodeIt == nodeLayouts_.end()) {
        return NodeMoveResult::fail({0, 0}, "Node not found");
    }

    NodeLayout& node = nodeIt->second;
    
    // No change needed if already the same type
    if (node.nodeType == newType) {
        return NodeMoveResult::ok(node.position, {});
    }

    // Save original state for rollback
    Size originalSize = node.size;
    NodeType originalType = node.nodeType;
    Point originalPosition = node.position;
    auto originalEdgeLayouts = edgeLayouts_;

    float gridSize = options_.gridConfig.cellSize > 0 ? options_.gridConfig.cellSize : DEFAULT_GRID_SIZE;
    
    Point newPosition = node.position;
    Size newSize = node.size;

    if (newType == NodeType::Point) {
        // Regular → Point conversion
        // 1. Save original size for later restoration
        savedSizesBeforePointConversion_[nodeId] = node.size;
        
        // 2. Position becomes center (grid-based integer calculation)
        //    center_grid = top_left_grid + size_grid / 2
        int topLeftGridX = static_cast<int>(node.position.x / gridSize);
        int topLeftGridY = static_cast<int>(node.position.y / gridSize);
        int sizeGridW = static_cast<int>(node.size.width / gridSize);
        int sizeGridH = static_cast<int>(node.size.height / gridSize);
        
        int centerGridX = topLeftGridX + sizeGridW / 2;
        int centerGridY = topLeftGridY + sizeGridH / 2;
        
        newPosition.x = centerGridX * gridSize;
        newPosition.y = centerGridY * gridSize;

        // 3. Size becomes {0, 0} for Point nodes
        newSize = {0, 0};
    } else {
        // Point → Regular conversion
        // 1. Restore saved size (or use default 12x6 grid units)
        auto savedIt = savedSizesBeforePointConversion_.find(nodeId);
        if (savedIt != savedSizesBeforePointConversion_.end()) {
            newSize = savedIt->second;
            savedSizesBeforePointConversion_.erase(savedIt);
        } else {
            // Use default size constants
            newSize = {DEFAULT_REGULAR_WIDTH_GRID * gridSize, DEFAULT_REGULAR_HEIGHT_GRID * gridSize};
        }
        
        // 2. Position becomes top-left (grid-based integer calculation)
        //    top_left_grid = center_grid - size_grid / 2
        int centerGridX = static_cast<int>(node.position.x / gridSize);
        int centerGridY = static_cast<int>(node.position.y / gridSize);
        int sizeGridW = static_cast<int>(newSize.width / gridSize);
        int sizeGridH = static_cast<int>(newSize.height / gridSize);
        
        int topLeftGridX = centerGridX - sizeGridW / 2;
        int topLeftGridY = centerGridY - sizeGridH / 2;
        
        newPosition.x = topLeftGridX * gridSize;
        newPosition.y = topLeftGridY * gridSize;
    }

    // Apply changes
    node.size = newSize;
    node.nodeType = newType;
    node.position = newPosition;

    // Get connected edges and recalculate routing
    auto affectedEdges = getExpandedAffectedEdges(nodeId);

    // Recalculate edge connection points for the converted node
    for (EdgeId edgeId : affectedEdges) {
        auto edgeIt = edgeLayouts_.find(edgeId);
        if (edgeIt == edgeLayouts_.end()) continue;

        EdgeLayout& edge = edgeIt->second;

        // Find source and target node layouts
        auto srcIt = nodeLayouts_.find(edge.from);
        auto tgtIt = nodeLayouts_.find(edge.to);
        if (srcIt == nodeLayouts_.end() || tgtIt == nodeLayouts_.end()) continue;

        const NodeLayout& srcNode = srcIt->second;
        const NodeLayout& tgtNode = tgtIt->second;

        // Recalculate connection points based on current node types
        // Note: calculateSourceEdgeForPointNode/calculateTargetEdgeForPointNode use center()
        // so they work for both Point and Regular nodes
        if (srcNode.isPointNode()) {
            edge.sourceEdge = LayoutUtils::calculateSourceEdgeForPointNode(srcNode, tgtNode);
            edge.sourceSnapIndex = constants::SNAP_INDEX_POINT_NODE_CENTER;
            edge.sourcePoint = srcNode.center();
        } else if (edge.from == nodeId) {
            // Source was converted to Regular: recalculate sourceEdge
            edge.sourceEdge = LayoutUtils::calculateSourceEdgeForPointNode(srcNode, tgtNode);
            // sourcePoint will be calculated by edge routing
        }

        if (tgtNode.isPointNode()) {
            edge.targetEdge = LayoutUtils::calculateTargetEdgeForPointNode(srcNode, tgtNode);
            edge.targetSnapIndex = constants::SNAP_INDEX_POINT_NODE_CENTER;
            edge.targetPoint = tgtNode.center();
        } else if (edge.to == nodeId) {
            // Target was converted to Regular: recalculate targetEdge
            edge.targetEdge = LayoutUtils::calculateTargetEdgeForPointNode(srcNode, tgtNode);
            // targetPoint will be calculated by edge routing
        }
    }

    // Update edge routing with optimization
    updateEdgeRouting(affectedEdges, {nodeId});

    // Validate final state
    auto constraintResult = constraintManager_->validateFinalState(nodeLayouts_, edgeLayouts_);

    if (!constraintResult.satisfied) {
        // Log detailed overlap information
        LOG_DEBUG("[setNodeType] Validation failed for node {}: newPos=({},{}) newSize=({},{})",
                  nodeId, newPosition.x, newPosition.y, newSize.width, newSize.height);
        
        for (NodeId overlapId : constraintResult.overlappingNodes) {
            auto it = nodeLayouts_.find(overlapId);
            if (it != nodeLayouts_.end()) {
                const auto& ol = it->second;
                LOG_DEBUG("[setNodeType] Overlapping node {}: pos=({},{}) size=({},{})",
                          overlapId, ol.position.x, ol.position.y, ol.size.width, ol.size.height);
            }
        }
        
        for (const auto& [e1, e2] : constraintResult.overlappingEdgePairs) {
            LOG_DEBUG("[setNodeType] Overlapping edges: {} and {}", e1, e2);
        }
        
        // Rollback: restore original state
        node.size = originalSize;
        node.nodeType = originalType;
        node.position = originalPosition;
        edgeLayouts_ = originalEdgeLayouts;
        
        // If we saved size during Regular→Point, remove it since we rolled back
        if (newType == NodeType::Point) {
            savedSizesBeforePointConversion_.erase(nodeId);
        }
        
        return NodeMoveResult::fail(newPosition, constraintResult.summary());
    }

    return NodeMoveResult::ok(newPosition, affectedEdges);
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

    // Use EdgeRoutingService for unified edge routing with constraint validation
    // LayoutController::moveNode()는 항상 "드롭"으로 간주
    // postDragAlgorithm 사용 (dragAlgorithm이 HideUntilDrop이어도 올바르게 동작)
    auto result = edgeRoutingService_->updateAfterNodeMove(
        edgeLayouts_, nodeLayouts_, affectedEdges, options_, movedNodes);

    // Update edge layouts with validated results
    for (auto& [edgeId, validated] : result.validatedLayouts) {
        edgeLayouts_[edgeId] = std::move(validated).extract();
    }

    // Log rejected edges (they will be removed from edgeLayouts)
    for (EdgeId rejected : result.rejectedEdges) {
        LOG_ERROR("[LayoutController] Edge {} REJECTED - failed constraint validation", rejected);
        edgeLayouts_.erase(rejected);
    }
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
