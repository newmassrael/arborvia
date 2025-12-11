#include "arborvia/layout/interactive/SnapPointController.h"
#include "arborvia/layout/config/LayoutResult.h"
#include "arborvia/core/GeometryUtils.h"
#include "../snap/GridSnapCalculator.h"
#include "../pathfinding/ObstacleMap.h"
#include "../pathfinding/AStarPathFinder.h"
#include "../sugiyama/routing/EdgeRouting.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <iostream>

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

    std::cout << "[SnapPointController] startDrag: edge=" << edgeId 
              << " isSource=" << isSource
              << " originalPos=(" << originalPosition_.x << "," << originalPosition_.y << ")"
              << std::endl;

    // Calculate all snap candidates for all 4 edges
    candidates_.clear();
    float gridSizeToUse = constants::effectiveGridSize(gridSize);

    for (NodeEdge nodeEdge : {NodeEdge::Top, NodeEdge::Bottom, NodeEdge::Left, NodeEdge::Right}) {
        int candidateCount = GridSnapCalculator::getCandidateCount(node, nodeEdge, gridSizeToUse);
        for (int i = 0; i < candidateCount; ++i) {
            Point pos = GridSnapCalculator::getPositionFromCandidateIndex(
                node, nodeEdge, i, gridSizeToUse);
            candidates_.push_back({nodeEdge, i, pos, false});  // blocked=false initially
        }
    }

    // Validate each candidate: check if A* can find a path from that position
    // Build obstacle map with nodes and other edge segments
    // Include edge layouts in bounds calculation to prevent out-of-bounds segments
    ObstacleMap obstacles;
    obstacles.buildFromNodes(nodeLayouts, gridSizeToUse, 0, &edgeLayouts);
    obstacles.addEdgeSegments(edgeLayouts, edgeId);  // Add other edges as obstacles

    // Get the other endpoint (target if dragging source, source if dragging target)
    Point otherEndpoint = isSource ? edge.targetPoint : edge.sourcePoint;
    NodeEdge otherEdge = isSource ? edge.targetEdge : edge.sourceEdge;
    NodeId otherNodeId = isSource ? edge.to : edge.from;

    GridPoint goalGrid = obstacles.pixelToGrid(otherEndpoint);

    for (auto& candidate : candidates_) {
        GridPoint startGrid = obstacles.pixelToGrid(candidate.position);
        
        // Try A* with direction constraint
        NodeEdge srcEdge = isSource ? candidate.edge : otherEdge;
        NodeEdge tgtEdge = isSource ? otherEdge : candidate.edge;
        NodeId srcNode = isSource ? nodeId : otherNodeId;
        NodeId tgtNode = isSource ? otherNodeId : nodeId;
        GridPoint pathStart = isSource ? startGrid : goalGrid;
        GridPoint pathGoal = isSource ? goalGrid : startGrid;

        auto pathResult = pathFinder_->findPath(
            pathStart, pathGoal, obstacles,
            srcNode, tgtNode,
            srcEdge, tgtEdge,
            {}, {});

        if (!pathResult.found || pathResult.path.size() < 2) {
            candidate.blocked = true;
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

    // Find nearest NON-BLOCKED candidate
    int nearestIdx = -1;
    float nearestDist = std::numeric_limits<float>::max();

    for (size_t i = 0; i < candidates_.size(); ++i) {
        if (candidates_[i].blocked) continue;  // Skip blocked candidates
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
        if (candidate.blocked) {
            // Blocked candidate - cannot drop here, return to original position
            result.reason = "Candidate is blocked (no valid path)";
            cancelDrag();
            return result;
        }
        targetPosition = candidate.position;
        targetEdge = candidate.edge;
        targetSnapIndex = candidate.candidateIndex;
    } else {
        // Fallback: find closest NON-BLOCKED candidate to mouse position
        int nearestIdx = -1;
        float nearestDist = std::numeric_limits<float>::max();
        for (size_t i = 0; i < candidates_.size(); ++i) {
            if (candidates_[i].blocked) continue;  // Skip blocked candidates
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
    
    std::cout << "[SnapPointController] completeDrag: edge=" << draggedEdgeId_
              << " isSource=" << isDraggingSource_
              << " BEFORE=(" << originalPosition_.x << "," << originalPosition_.y << ")"
              << " AFTER=(" << targetPosition.x << "," << targetPosition.y << ")"
              << std::endl;
    
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

    // Regenerate bend points - MUST run full pathfinding flow:
    // 1. Build ObstacleMap from nodes
    // 2. Add other edge segments as obstacles
    // 3. Run A* pathfinding
    // 4. Update bendPoints with orthogonal path
    if (regenerator) {
        regenerator(edgeLayouts, nodeLayouts, result.affectedEdges, options);
    } else {
        // Default: use EdgeRouting::regenerateBendPointsOnly
        // This ensures ObstacleMap is built and A* pathfinding runs
        EdgeRouting routing;
        routing.regenerateBendPointsOnly(edgeLayouts, nodeLayouts, result.affectedEdges, options);
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

    float gridSizeToUse = constants::effectiveGridSize(gridSize);

    // Build obstacle map
    ObstacleMap obstacles;
    obstacles.buildFromNodes(nodeLayouts, gridSizeToUse, 1);

    // Convert to grid coordinates
    GridPoint startGrid = GridPoint::fromPixel(layout.sourcePoint, gridSizeToUse);
    GridPoint goalGrid = GridPoint::fromPixel(layout.targetPoint, gridSizeToUse);

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
        Point pixelPos = pathResult.path[i].toPixel(gridSizeToUse);
        layout.bendPoints.push_back(BendPoint{pixelPos});
    }

    // Update endpoints from path
    layout.sourcePoint = pathResult.path.front().toPixel(gridSizeToUse);
    layout.targetPoint = pathResult.path.back().toPixel(gridSizeToUse);

    return true;
}

}  // namespace arborvia
