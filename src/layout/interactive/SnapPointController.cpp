#include "arborvia/layout/interactive/SnapPointController.h"
#include "arborvia/layout/config/LayoutResult.h"
#include "../snap/GridSnapCalculator.h"
#include "../pathfinding/ObstacleMap.h"
#include "../pathfinding/AStarPathFinder.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace arborvia {

SnapPointController::SnapPointController()
    : pathFinder_(std::make_shared<AStarPathFinder>()) {
}

SnapPointController::SnapPointController(std::shared_ptr<IPathFinder> pathFinder)
    : pathFinder_(pathFinder ? pathFinder : std::make_shared<AStarPathFinder>()) {
}

SnapPointController::~SnapPointController() = default;

SnapPointController::DragStartResult SnapPointController::startDrag(
    EdgeId edgeId,
    bool isSource,
    const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
    const std::unordered_map<EdgeId, EdgeLayout>& edgeLayouts,
    float gridSize) {

    DragStartResult result;

    // Find the edge
    auto edgeIt = edgeLayouts.find(edgeId);
    if (edgeIt == edgeLayouts.end()) {
        result.reason = "Edge not found";
        return result;
    }

    const EdgeLayout& edge = edgeIt->second;

    // Get the node this snap point is on
    NodeId nodeId = isSource ? edge.from : edge.to;
    auto nodeIt = nodeLayouts.find(nodeId);
    if (nodeIt == nodeLayouts.end()) {
        result.reason = "Node not found";
        return result;
    }

    const NodeLayout& node = nodeIt->second;

    // Store drag state
    isDragging_ = true;
    draggedEdgeId_ = edgeId;
    isDraggingSource_ = isSource;
    dragNodeId_ = nodeId;

    // Store original state
    originalPosition_ = isSource ? edge.sourcePoint : edge.targetPoint;
    originalEdge_ = isSource ? edge.sourceEdge : edge.targetEdge;
    originalSnapIndex_ = isSource ? edge.sourceSnapIndex : edge.targetSnapIndex;
    originalLayout_ = edge;

    // Calculate all snap candidates for all 4 edges
    candidates_.clear();
    float effectiveGridSize = GridSnapCalculator::getEffectiveGridSize(gridSize);

    for (NodeEdge nodeEdge : {NodeEdge::Top, NodeEdge::Bottom, NodeEdge::Left, NodeEdge::Right}) {
        int candidateCount = GridSnapCalculator::getCandidateCount(node, nodeEdge, effectiveGridSize);
        for (int i = 0; i < candidateCount; ++i) {
            Point pos = GridSnapCalculator::getPositionFromCandidateIndex(
                node, nodeEdge, i, effectiveGridSize);
            candidates_.push_back({nodeEdge, i, pos});
        }
    }

    // Build result
    result.success = true;
    result.candidates = candidates_;
    result.originalPosition = originalPosition_;
    result.originalEdge = originalEdge_;
    result.originalSnapIndex = originalSnapIndex_;

    // Initialize preview
    previewLayout_ = edge;
    currentCandidateIndex_ = -1;

    return result;
}

SnapPointController::DragUpdateResult SnapPointController::updateDrag(
    Point mousePosition,
    const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
    float gridSize) {

    DragUpdateResult result;

    if (!isDragging_ || candidates_.empty()) {
        return result;
    }

    // Find nearest candidate
    int nearestIdx = -1;
    float nearestDist = std::numeric_limits<float>::max();

    for (size_t i = 0; i < candidates_.size(); ++i) {
        float dist = mousePosition.distanceTo(candidates_[i].position);
        if (dist < nearestDist) {
            nearestDist = dist;
            nearestIdx = static_cast<int>(i);
        }
    }

    if (nearestIdx < 0) {
        return result;
    }

    result.snappedCandidateIndex = nearestIdx;

    // Only recalculate preview if candidate changed
    if (nearestIdx != currentCandidateIndex_) {
        currentCandidateIndex_ = nearestIdx;
        const auto& candidate = candidates_[nearestIdx];

        // Update preview layout
        previewLayout_ = originalLayout_;
        if (isDraggingSource_) {
            previewLayout_.sourcePoint = candidate.position;
            previewLayout_.sourceEdge = candidate.edge;
        } else {
            previewLayout_.targetPoint = candidate.position;
            previewLayout_.targetEdge = candidate.edge;
        }

        // Calculate A* path
        result.hasValidPreview = calculatePreviewPath(previewLayout_, nodeLayouts, gridSize);
    } else {
        // Same candidate, use cached preview
        // Direct path is valid if close enough (within kMinDirectPathGridCells grid cells)
        result.hasValidPreview = !previewLayout_.bendPoints.empty() ||
            (previewLayout_.sourcePoint.distanceTo(previewLayout_.targetPoint)
             < gridSize * kMinDirectPathGridCells);
    }

    result.previewLayout = previewLayout_;
    return result;
}

SnapPointController::DropResult SnapPointController::completeDrag(
    int snappedCandidateIndex,
    Point mousePosition,
    const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
    std::unordered_map<EdgeId, EdgeLayout>& edgeLayouts,
    const LayoutOptions& options,
    const BendPointRegenerator& regenerator) {

    DropResult result;

    if (!isDragging_) {
        result.reason = "No drag in progress";
        return result;
    }

    auto edgeIt = edgeLayouts.find(draggedEdgeId_);
    if (edgeIt == edgeLayouts.end()) {
        result.reason = "Dragged edge not found";
        cancelDrag();
        return result;
    }

    // Determine target position and snap index
    Point targetPosition;
    NodeEdge targetEdge;
    int targetSnapIndex;

    if (snappedCandidateIndex >= 0 && snappedCandidateIndex < static_cast<int>(candidates_.size())) {
        // Use snapped candidate
        const auto& candidate = candidates_[snappedCandidateIndex];
        targetPosition = candidate.position;
        targetEdge = candidate.edge;
        targetSnapIndex = candidate.candidateIndex;
    } else {
        // Fallback: find closest candidate to mouse position
        int nearestIdx = -1;
        float nearestDist = std::numeric_limits<float>::max();
        for (size_t i = 0; i < candidates_.size(); ++i) {
            float dist = mousePosition.distanceTo(candidates_[i].position);
            if (dist < nearestDist) {
                nearestDist = dist;
                nearestIdx = static_cast<int>(i);
            }
        }

        if (nearestIdx >= 0) {
            const auto& candidate = candidates_[nearestIdx];
            targetPosition = candidate.position;
            targetEdge = candidate.edge;
            targetSnapIndex = candidate.candidateIndex;
        } else {
            result.reason = "No valid candidate found";
            cancelDrag();
            return result;
        }
    }

    // Check for swap partner
    auto occupying = findOccupyingEdge(dragNodeId_, targetEdge, targetSnapIndex,
                                        draggedEdgeId_, edgeLayouts);

    // Update dragged edge
    EdgeLayout& draggedEdge = edgeIt->second;
    if (isDraggingSource_) {
        draggedEdge.sourcePoint = targetPosition;
        draggedEdge.sourceEdge = targetEdge;
        draggedEdge.sourceSnapIndex = targetSnapIndex;
    } else {
        draggedEdge.targetPoint = targetPosition;
        draggedEdge.targetEdge = targetEdge;
        draggedEdge.targetSnapIndex = targetSnapIndex;
    }

    result.affectedEdges.push_back(draggedEdgeId_);

    // Handle swap if needed
    if (occupying.edgeId != INVALID_EDGE) {
        EdgeLayout& swapEdge = edgeLayouts[occupying.edgeId];
        if (occupying.isSource) {
            swapEdge.sourcePoint = originalPosition_;
            swapEdge.sourceEdge = originalEdge_;
            swapEdge.sourceSnapIndex = originalSnapIndex_;
        } else {
            swapEdge.targetPoint = originalPosition_;
            swapEdge.targetEdge = originalEdge_;
            swapEdge.targetSnapIndex = originalSnapIndex_;
        }
        result.swapEdgeId = occupying.edgeId;
        result.swapIsSource = occupying.isSource;
        result.affectedEdges.push_back(occupying.edgeId);
    }

    // Regenerate bend points if callback provided
    if (regenerator) {
        regenerator(edgeLayouts, nodeLayouts, result.affectedEdges, options);
    }

    result.success = true;
    result.actualPosition = targetPosition;
    result.actualEdge = targetEdge;
    result.actualSnapIndex = targetSnapIndex;

    cancelDrag();
    return result;
}

void SnapPointController::cancelDrag() {
    isDragging_ = false;
    draggedEdgeId_ = INVALID_EDGE;
    isDraggingSource_ = true;
    dragNodeId_ = INVALID_NODE;
    candidates_.clear();
    currentCandidateIndex_ = -1;
}

SnapPointController::OccupyingEdge SnapPointController::findOccupyingEdge(
    NodeId nodeId,
    NodeEdge edge,
    int snapIndex,
    EdgeId excludeEdgeId,
    const std::unordered_map<EdgeId, EdgeLayout>& edgeLayouts) const {

    OccupyingEdge result;

    for (const auto& [eid, layout] : edgeLayouts) {
        if (eid == excludeEdgeId) continue;

        // Check source snap point
        if (layout.from == nodeId && layout.sourceEdge == edge &&
            layout.sourceSnapIndex == snapIndex) {
            result.edgeId = eid;
            result.isSource = true;
            return result;
        }

        // Check target snap point
        if (layout.to == nodeId && layout.targetEdge == edge &&
            layout.targetSnapIndex == snapIndex) {
            result.edgeId = eid;
            result.isSource = false;
            return result;
        }
    }

    return result;
}

bool SnapPointController::calculatePreviewPath(
    EdgeLayout& layout,
    const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
    float gridSize) {

    if (!pathFinder_) {
        return false;
    }

    float effectiveGridSize = GridSnapCalculator::getEffectiveGridSize(gridSize);

    // Build obstacle map
    ObstacleMap obstacles;
    obstacles.buildFromNodes(nodeLayouts, effectiveGridSize, 1);

    // Convert to grid coordinates
    GridPoint startGrid = GridPoint::fromPixel(layout.sourcePoint, effectiveGridSize);
    GridPoint goalGrid = GridPoint::fromPixel(layout.targetPoint, effectiveGridSize);

    // Find path
    auto pathResult = pathFinder_->findPath(
        startGrid, goalGrid, obstacles,
        layout.from, layout.to,
        layout.sourceEdge, layout.targetEdge);

    if (!pathResult.found || pathResult.path.size() < 2) {
        return false;
    }

    // Convert grid path to pixel bend points
    layout.bendPoints.clear();
    for (size_t i = 1; i + 1 < pathResult.path.size(); ++i) {
        Point pixelPos = pathResult.path[i].toPixel(effectiveGridSize);
        layout.bendPoints.push_back(BendPoint{pixelPos});
    }

    // Update endpoints from path
    layout.sourcePoint = pathResult.path.front().toPixel(effectiveGridSize);
    layout.targetPoint = pathResult.path.back().toPixel(effectiveGridSize);

    return true;
}

}  // namespace arborvia
