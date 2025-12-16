#include "arborvia/layout/interactive/SnapPointController.h"
#include "arborvia/layout/interactive/SnapPointController.h"
#include "arborvia/layout/config/LayoutResult.h"
#include "arborvia/core/GeometryUtils.h"
#include "../snap/GridSnapCalculator.h"
#include "../pathfinding/ObstacleMap.h"
#include "../pathfinding/AStarPathFinder.h"
#include "../pathfinding/CompositePathCalculator.h"
#include "../sugiyama/routing/EdgeRouting.h"
#include "../sugiyama/routing/SelfLoopRouter.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include "arborvia/common/Logger.h"

namespace arborvia {

SnapPointController::SnapPointController()
    : pathFinder_(std::make_shared<AStarPathFinder>())
    , pathCalculator_(CompositePathCalculator::createDefault(pathFinder_)) {
}

SnapPointController::SnapPointController(std::shared_ptr<IPathFinder> pathFinder)
    : pathFinder_(pathFinder ? pathFinder : std::make_shared<AStarPathFinder>())
    , pathCalculator_(CompositePathCalculator::createDefault(pathFinder_)) {
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

    // Point nodes have only one snap point (center) - dragging is meaningless
    if (node.isPointNode()) {
        result.reason = "Point nodes have fixed center connection";
        return result;
    }

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
            // Self-loop validation with auto-adjustment
            if (candidate.edge == otherEdge) {
                // Same edge as other endpoint - blocked (can't have both on same edge)
                candidate.blocked = true;
                LOG_DEBUG("[SnapPointController] Self-loop candidate edge={} blocked (same as otherEdge)",
                          static_cast<int>(candidate.edge));
            } else if (!SelfLoopRouter::isValidSelfLoopCombination(candidate.edge, otherEdge)) {
                // Opposite edges - auto-move other endpoint to create valid corner
                candidate.requiresOtherEndpointMove = true;

                // Find adjacent edge for the other endpoint based on candidate position
                // Pick the corner closest to the candidate position
                bool isHorizontalCandidate = (candidate.edge == NodeEdge::Top || candidate.edge == NodeEdge::Bottom);

                if (isHorizontalCandidate) {
                    // Candidate on Top/Bottom → other endpoint should be on Left or Right
                    // Choose based on candidate X position relative to node center
                    float nodeCenterX = node.position.x + node.size.width / 2.0f;
                    candidate.suggestedOtherEdge = (candidate.position.x < nodeCenterX)
                        ? NodeEdge::Left : NodeEdge::Right;
                } else {
                    // Candidate on Left/Right → other endpoint should be on Top or Bottom
                    // Choose based on candidate Y position relative to node center
                    float nodeCenterY = node.position.y + node.size.height / 2.0f;
                    candidate.suggestedOtherEdge = (candidate.position.y < nodeCenterY)
                        ? NodeEdge::Top : NodeEdge::Bottom;
                }

                // Calculate corner snap index and position for the suggested edge
                // Self-loops should be at corners, so use first or last snap index
                bool useLastIndex = SelfLoopRouter::shouldBeAtLastIndex(
                    candidate.suggestedOtherEdge, candidate.edge);
                int candidateCount = GridSnapCalculator::getCandidateCount(
                    node, candidate.suggestedOtherEdge, gridSizeToUse);
                candidate.suggestedOtherIndex = useLastIndex ? (candidateCount - 1) : 0;
                candidate.suggestedOtherPosition = GridSnapCalculator::getPositionFromCandidateIndex(
                    node, candidate.suggestedOtherEdge, candidate.suggestedOtherIndex, gridSizeToUse);

                LOG_DEBUG("[SnapPointController] Self-loop auto-adjust: candidate edge={} → other edge={} idx={}",
                          static_cast<int>(candidate.edge),
                          static_cast<int>(candidate.suggestedOtherEdge),
                          candidate.suggestedOtherIndex);
            }
            // If adjacent edges, candidate is valid as-is (requiresOtherEndpointMove = false)
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
        previewLayout_.bendPoints.clear();  // Clear old bendPoints before recalculating

        if (isDraggingSource_) {
            previewLayout_.sourcePoint = candidate.position;
            previewLayout_.sourceEdge = candidate.edge;
            // Auto-adjust target for self-loop if needed
            if (candidate.requiresOtherEndpointMove) {
                previewLayout_.targetPoint = candidate.suggestedOtherPosition;
                previewLayout_.targetEdge = candidate.suggestedOtherEdge;
            }
        } else {
            previewLayout_.targetPoint = candidate.position;
            previewLayout_.targetEdge = candidate.edge;
            // Auto-adjust source for self-loop if needed
            if (candidate.requiresOtherEndpointMove) {
                previewLayout_.sourcePoint = candidate.suggestedOtherPosition;
                previewLayout_.sourceEdge = candidate.suggestedOtherEdge;
            }
        }

        // Calculate path for preview
        result.hasValidPreview = calculatePreviewPath(previewLayout_, nodeLayouts, gridSize);

        // If preview calculation failed, create a simple direct path
        // (better than leaving stale bendPoints that create diagonals)
        if (!result.hasValidPreview) {
            previewLayout_.bendPoints.clear();
        }
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
    int selectedCandidateIdx = -1;

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
        selectedCandidateIdx = snappedCandidateIndex;
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
            selectedCandidateIdx = nearestIdx;
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

    // Get the selected candidate for self-loop auto-adjustment check
    const SnapCandidate* selectedCandidate = nullptr;
    if (selectedCandidateIdx >= 0 && selectedCandidateIdx < static_cast<int>(candidates_.size())) {
        selectedCandidate = &candidates_[selectedCandidateIdx];
    }

    LOG_DEBUG("[SnapPointController] completeDrag: edge={} isSource={} BEFORE=({},{}) AFTER=({},{})",
              draggedEdgeId_, isDraggingSource_,
              originalPosition_.x, originalPosition_.y,
              targetPosition.x, targetPosition.y);

    if (isDraggingSource_) {
        draggedEdge.sourcePoint = targetPosition;
        draggedEdge.sourceEdge = targetEdge;
        // Self-loop auto-adjustment: also move target if needed
        if (selectedCandidate && selectedCandidate->requiresOtherEndpointMove) {
            draggedEdge.targetPoint = selectedCandidate->suggestedOtherPosition;
            draggedEdge.targetEdge = selectedCandidate->suggestedOtherEdge;
            LOG_DEBUG("[SnapPointController] Self-loop auto-adjusted target to ({},{}) edge={}",
                      draggedEdge.targetPoint.x, draggedEdge.targetPoint.y,
                      static_cast<int>(draggedEdge.targetEdge));
        }
    } else {
        draggedEdge.targetPoint = targetPosition;
        draggedEdge.targetEdge = targetEdge;
        // Self-loop auto-adjustment: also move source if needed
        if (selectedCandidate && selectedCandidate->requiresOtherEndpointMove) {
            draggedEdge.sourcePoint = selectedCandidate->suggestedOtherPosition;
            draggedEdge.sourceEdge = selectedCandidate->suggestedOtherEdge;
            LOG_DEBUG("[SnapPointController] Self-loop auto-adjusted source to ({},{}) edge={}",
                      draggedEdge.sourcePoint.x, draggedEdge.sourcePoint.y,
                      static_cast<int>(draggedEdge.sourceEdge));
        }
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

    // Use unified path calculator for both self-loops and regular edges
    if (!pathCalculator_) {
        return false;
    }

    float gridSizeToUse = constants::effectiveGridSize(gridSize);

    PathConfig config;
    config.gridSize = gridSizeToUse;
    config.extensionCells = 2;  // Extension points are 2 grid cells from node edge
    config.snapToGrid = true;

    auto result = pathCalculator_->calculatePath(layout, nodeLayouts, config);

    if (!result.success) {
        LOG_DEBUG("[SnapPointController::calculatePreviewPath] Path calculation failed: {}", 
                  result.failureReason);
        return false;
    }

    // Update layout with calculated bend points
    layout.bendPoints = std::move(result.bendPoints);
    return true;
}

}  // namespace arborvia
