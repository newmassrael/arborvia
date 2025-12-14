#include "arborvia/layout/interactive/SnapPointController.h"
#include "arborvia/layout/config/LayoutResult.h"
#include "arborvia/core/GeometryUtils.h"
#include "../snap/GridSnapCalculator.h"
#include "../pathfinding/ObstacleMap.h"
#include "../pathfinding/AStarPathFinder.h"
#include "../sugiyama/routing/EdgeRouting.h"
#include "../sugiyama/routing/SelfLoopRouter.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include "arborvia/common/Logger.h"

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
    // Compute original snap index from position (position is source of truth)
    float gridSizeForCalc = constants::effectiveGridSize(gridSize);
    originalSnapIndex_ = GridSnapCalculator::getCandidateIndexFromPosition(
        node, originalEdge_, originalPosition_, gridSizeForCalc);
    originalLayout_ = edge;

    LOG_DEBUG("[SnapPointController] startDrag: edge={} isSource={} originalPos=({},{})",
              edgeId, isSource, originalPosition_.x, originalPosition_.y);

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

    // First pass: identify which candidates are occupied by other edges (swap targets)
    // Also block candidates occupied by self-loop endpoints (can't swap with self-loops)
    std::unordered_set<int> swapCandidates;
    std::unordered_set<int> selfLoopOccupied;  // Candidates occupied by self-loop endpoints
    for (size_t i = 0; i < candidates_.size(); ++i) {
        const auto& candidate = candidates_[i];
        for (const auto& [eid, elayout] : edgeLayouts) {
            if (eid == edgeId) continue;
            // Check if this candidate position matches another edge's snap point on this node
            // Compute snap indices from positions (position is source of truth)
            int elayoutSourceSnapIdx = GridSnapCalculator::getCandidateIndexFromPosition(
                node, elayout.sourceEdge, elayout.sourcePoint, gridSizeToUse);
            int elayoutTargetSnapIdx = GridSnapCalculator::getCandidateIndexFromPosition(
                node, elayout.targetEdge, elayout.targetPoint, gridSizeToUse);
            bool matchSource = (elayout.from == nodeId && 
                               elayout.sourceEdge == candidate.edge &&
                               elayoutSourceSnapIdx == candidate.candidateIndex);
            bool matchTarget = (elayout.to == nodeId &&
                               elayout.targetEdge == candidate.edge &&
                               elayoutTargetSnapIdx == candidate.candidateIndex);
            if (matchSource || matchTarget) {
                // Check if occupying edge is a self-loop
                if (elayout.from == elayout.to) {
                    // Self-loop endpoint: cannot swap (moving one endpoint breaks self-loop)
                    selfLoopOccupied.insert(static_cast<int>(i));
                    LOG_DEBUG("[SnapPointController] Candidate {} blocked (occupied by self-loop edge {})",
                              i, eid);
                } else {
                    swapCandidates.insert(static_cast<int>(i));
                    LOG_DEBUG("[SnapPointController] Candidate {} is swap target (occupied by edge {})",
                              i, eid);
                }
                break;
            }
        }
    }

    // Self-loop detection: from == to means both endpoints are on the same node
    bool isSelfLoop = (edge.from == edge.to);

    for (auto& candidate : candidates_) {
        if (isSelfLoop) {
            // Self-loop: validate using SelfLoopRouter constraint
            // Only adjacent edges are valid (not same or opposite edges)
            if (!SelfLoopRouter::isValidSelfLoopCombination(candidate.edge, otherEdge)) {
                candidate.blocked = true;
                LOG_DEBUG("[SnapPointController] Self-loop candidate edge={} blocked (not adjacent to otherEdge={})",
                          static_cast<int>(candidate.edge), static_cast<int>(otherEdge));
            }
        } else {
            // Regular edge: use A* pathfinding validation
            GridPoint startGrid = obstacles.pixelToGrid(candidate.position);

            // Try A* with direction constraint
            NodeEdge srcEdge = isSource ? candidate.edge : otherEdge;
            NodeEdge tgtEdge = isSource ? otherEdge : candidate.edge;
            NodeId srcNode = isSource ? nodeId : otherNodeId;
            NodeId tgtNode = isSource ? otherNodeId : nodeId;
            GridPoint pathStart = isSource ? startGrid : goalGrid;
            GridPoint pathGoal = isSource ? goalGrid : startGrid;

            LOG_DEBUG("[CALLER:SnapPointController.cpp:evaluateCandidate] A* findPath called");
            auto pathResult = pathFinder_->findPath(
                pathStart, pathGoal, obstacles,
                srcNode, tgtNode,
                srcEdge, tgtEdge,
                {}, {});

            if (!pathResult.found || pathResult.path.size() < 2) {
                candidate.blocked = true;
            }
        }
    }
    
    // Mark swap candidates as NOT blocked (they can be selected for swap)
    for (int idx : swapCandidates) {
        if (candidates_[idx].blocked) {
            LOG_DEBUG("[SnapPointController] Unblocking swap candidate {}", idx);
            candidates_[idx].blocked = false;
        }
    }
    
    // Block candidates occupied by self-loop endpoints (cannot swap with self-loops)
    for (int idx : selfLoopOccupied) {
        candidates_[idx].blocked = true;
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
    float gridSizeForCheck = constants::effectiveGridSize(options.gridConfig.cellSize);
    auto occupying = findOccupyingEdge(dragNodeId_, targetEdge, targetSnapIndex,
                                        draggedEdgeId_, edgeLayouts, nodeLayouts, gridSizeForCheck);

    // Update dragged edge
    EdgeLayout& draggedEdge = edgeIt->second;
    
    LOG_DEBUG("[SnapPointController] completeDrag: edge={} isSource={} BEFORE=({},{}) AFTER=({},{})",
              draggedEdgeId_, isDraggingSource_, 
              originalPosition_.x, originalPosition_.y,
              targetPosition.x, targetPosition.y);
    
    if (isDraggingSource_) {
        draggedEdge.sourcePoint = targetPosition;
        draggedEdge.sourceEdge = targetEdge;
        // NOTE: snapIndex is no longer stored - computed from position as needed
    } else {
        draggedEdge.targetPoint = targetPosition;
        draggedEdge.targetEdge = targetEdge;
        // NOTE: snapIndex is no longer stored - computed from position as needed
    }

    result.affectedEdges.push_back(draggedEdgeId_);

    // Handle swap if needed
    if (occupying.edgeId != INVALID_EDGE) {
        EdgeLayout& swapEdge = edgeLayouts[occupying.edgeId];
        if (occupying.isSource) {
            swapEdge.sourcePoint = originalPosition_;
            swapEdge.sourceEdge = originalEdge_;
            // NOTE: snapIndex is no longer stored - computed from position as needed
        } else {
            swapEdge.targetPoint = originalPosition_;
            swapEdge.targetEdge = originalEdge_;
            // NOTE: snapIndex is no longer stored - computed from position as needed
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
    const std::unordered_map<EdgeId, EdgeLayout>& edgeLayouts,
    const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
    float gridSize) const {

    OccupyingEdge result;
    
    // Get the node layout for computing snap indices
    auto nodeIt = nodeLayouts.find(nodeId);
    if (nodeIt == nodeLayouts.end()) {
        return result;  // Node not found
    }
    const NodeLayout& node = nodeIt->second;
    
    for (const auto& [eid, layout] : edgeLayouts) {
        if (eid == excludeEdgeId) continue;

        // Check source snap point - compute index from position
        if (layout.from == nodeId && layout.sourceEdge == edge) {
            int computedSnapIdx = GridSnapCalculator::getCandidateIndexFromPosition(
                node, layout.sourceEdge, layout.sourcePoint, gridSize);
            if (computedSnapIdx == snapIndex) {
                result.edgeId = eid;
                result.isSource = true;
                return result;
            }
        }

        // Check target snap point - compute index from position
        if (layout.to == nodeId && layout.targetEdge == edge) {
            int computedSnapIdx = GridSnapCalculator::getCandidateIndexFromPosition(
                node, layout.targetEdge, layout.targetPoint, gridSize);
            if (computedSnapIdx == snapIndex) {
                result.edgeId = eid;
                result.isSource = false;
                return result;
            }
        }
    }

    return result;
}

bool SnapPointController::calculatePreviewPath(
    EdgeLayout& layout,
    const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
    float gridSize) {

    float gridSizeToUse = constants::effectiveGridSize(gridSize);

    // Self-loop: use geometric path calculation (no A* needed)
    if (layout.from == layout.to) {
        auto nodeIt = nodeLayouts.find(layout.from);
        if (nodeIt == nodeLayouts.end()) {
            return false;
        }

        // Validate: only adjacent edges are valid for self-loops
        if (!SelfLoopRouter::isValidSelfLoopCombination(layout.sourceEdge, layout.targetEdge)) {
            return false;
        }

        // Use geometric path for self-loop preview
        // Calculate extension points perpendicular to each edge
        constexpr float BASE_OFFSET = 30.0f;

        Point srcExt = layout.sourcePoint;
        Point tgtExt = layout.targetPoint;

        // Extend source point outward from node
        switch (layout.sourceEdge) {
            case NodeEdge::Top:    srcExt.y -= BASE_OFFSET; break;
            case NodeEdge::Bottom: srcExt.y += BASE_OFFSET; break;
            case NodeEdge::Left:   srcExt.x -= BASE_OFFSET; break;
            case NodeEdge::Right:  srcExt.x += BASE_OFFSET; break;
        }

        // Extend target point outward from node
        switch (layout.targetEdge) {
            case NodeEdge::Top:    tgtExt.y -= BASE_OFFSET; break;
            case NodeEdge::Bottom: tgtExt.y += BASE_OFFSET; break;
            case NodeEdge::Left:   tgtExt.x -= BASE_OFFSET; break;
            case NodeEdge::Right:  tgtExt.x += BASE_OFFSET; break;
        }

        // Corner point where the two extensions meet
        Point corner;
        if (layout.sourceEdge == NodeEdge::Left || layout.sourceEdge == NodeEdge::Right) {
            corner = {srcExt.x, tgtExt.y};
        } else {
            corner = {tgtExt.x, srcExt.y};
        }

        // Set bend points: src -> srcExt -> corner -> tgtExt -> tgt
        layout.bendPoints.clear();
        layout.bendPoints.push_back(BendPoint{srcExt});
        layout.bendPoints.push_back(BendPoint{corner});
        layout.bendPoints.push_back(BendPoint{tgtExt});

        return true;
    }

    // Regular edge: use A* pathfinding
    if (!pathFinder_) {
        return false;
    }

    // Build obstacle map (margin=0 to match startDrag validation)
    ObstacleMap obstacles;
    obstacles.buildFromNodes(nodeLayouts, gridSizeToUse, 0);

    // Convert to grid coordinates
    GridPoint startGrid = GridPoint::fromPixel(layout.sourcePoint, gridSizeToUse);
    GridPoint goalGrid = GridPoint::fromPixel(layout.targetPoint, gridSizeToUse);

    // Find path
    LOG_DEBUG("[CALLER:SnapPointController.cpp:findPathForEdge] A* findPath called");
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
