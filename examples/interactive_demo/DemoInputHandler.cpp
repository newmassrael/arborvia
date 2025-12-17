#include "DemoInputHandler.h"
#include "DemoColors.h"
#include <arborvia/layout/util/LayoutUtils.h>
#include <arborvia/layout/api/IConstraintValidator.h>
#include <arborvia/layout/api/IDragConstraint.h>
#include <arborvia/common/Logger.h>
#include "../../src/layout/snap/GridSnapCalculator.h"
#include "../../src/layout/routing/OrthogonalRouter.h"
#include <cmath>
#include <algorithm>

namespace arborvia {

DemoInputHandler::DemoInputHandler(std::shared_ptr<ManualLayoutManager> manualManager)
    : manualManager_(std::move(manualManager)) {
}

Point DemoInputHandler::screenToWorld(const DemoState& state, const ImVec2& screen) const {
    return state.view.screenToWorld(screen);
}

InputResult DemoInputHandler::processInput(DemoState& state, const ImGuiIO& io) {
    InputResult result;

    // Handle zoom (mouse wheel)
    if (!io.WantCaptureMouse && io.MouseWheel != 0) {
        handleZoom(state, io);
        result.stateChanged = true;
    }

    // Handle pan (middle mouse drag)
    handlePan(state, io);

    // Skip other graph interaction if ImGui wants mouse or middle-mouse panning
    if (io.WantCaptureMouse || (state.interaction.isPanning && !state.interaction.emptyAreaPanStarted)) {
        return result;
    }

    Point graphMouse = screenToWorld(state, io.MousePos);

    // Update hover states
    updateHoveredNode(state, graphMouse);
    updateHoveredSnapPoint(state, graphMouse);
    updateHoveredBendPoint(state, graphMouse);
    updateHoveredEdge(state, graphMouse);

    // Handle clicks
    handleClicks(state, io, graphMouse);

    // Handle drag threshold
    handleDragThreshold(state, io);

    // Handle empty area pan
    handleEmptyAreaPan(state, io);

    // Handle mouse release
    handleMouseRelease(state, io, graphMouse);

    // Handle ongoing drags
    handleSnapPointDrag(state, io, graphMouse);
    handleBendPointDrag(state, io, graphMouse);
    handleNodeDrag(state, io, graphMouse);

    // Handle keyboard
    handleKeyboard(state, io);

    return result;
}

void DemoInputHandler::handleZoom(DemoState& state, const ImGuiIO& io) {
    // Zoom centered on mouse position
    Point worldBeforeZoom = screenToWorld(state, io.MousePos);

    float zoomDelta = io.MouseWheel * 0.1f;
    float oldZoom = state.view.zoom;
    state.view.zoom = std::clamp(state.view.zoom + zoomDelta, 0.1f, 5.0f);

    // Adjust pan to keep mouse position fixed in world coordinates
    if (state.view.zoom != oldZoom) {
        Point worldAfterZoom = screenToWorld(state, io.MousePos);
        state.view.panOffset.x += worldAfterZoom.x - worldBeforeZoom.x;
        state.view.panOffset.y += worldAfterZoom.y - worldBeforeZoom.y;
    }
}

void DemoInputHandler::handlePan(DemoState& state, const ImGuiIO& io) {
    if (ImGui::IsMouseDragging(ImGuiMouseButton_Middle) && !io.WantCaptureMouse) {
        state.view.panOffset.x += io.MouseDelta.x / state.view.zoom;
        state.view.panOffset.y += io.MouseDelta.y / state.view.zoom;
        state.interaction.isPanning = true;
    } else if (!state.interaction.emptyAreaPanStarted) {
        state.interaction.isPanning = false;
    }
}

void DemoInputHandler::updateHoveredNode(DemoState& state, const Point& graphMouse) {
    state.interaction.hoveredNode = INVALID_NODE;

    for (const auto& [id, layout] : *state.nodeLayouts) {
        bool hit = false;
        if (layout.isPointNode()) {
            // Point nodes: use distance-based hit detection with visual radius
            Point center = layout.center();
            float dx = graphMouse.x - center.x;
            float dy = graphMouse.y - center.y;
            float distSq = dx * dx + dy * dy;
            hit = (distSq <= POINT_NODE_RADIUS * POINT_NODE_RADIUS);
        } else {
            // Normal nodes: use bounds-based hit detection
            Rect bounds = layout.bounds();
            hit = (graphMouse.x >= bounds.x && graphMouse.x <= bounds.right() &&
                   graphMouse.y >= bounds.y && graphMouse.y <= bounds.bottom());
        }
        if (hit) {
            state.interaction.hoveredNode = id;
            break;
        }
    }
}

void DemoInputHandler::updateHoveredSnapPoint(DemoState& state, const Point& graphMouse) {
    state.interaction.hoveredSnapPoint.clear();

    if (!state.renderOptions.showSnapPoints ||
        state.interaction.hoveredNode != INVALID_NODE ||
        state.interaction.draggingSnapPoint.isValid()) {
        return;
    }

    float closestDist = SNAP_HIT_RADIUS * SNAP_HIT_RADIUS;
    for (const auto& [edgeId, layout] : *state.edgeLayouts) {
        // Check source snap point
        float dx = graphMouse.x - layout.sourcePoint.x;
        float dy = graphMouse.y - layout.sourcePoint.y;
        float distSq = dx * dx + dy * dy;
        if (distSq < closestDist) {
            closestDist = distSq;
            state.interaction.hoveredSnapPoint.edgeId = edgeId;
            state.interaction.hoveredSnapPoint.isSource = true;
        }
        // Check target snap point
        dx = graphMouse.x - layout.targetPoint.x;
        dy = graphMouse.y - layout.targetPoint.y;
        distSq = dx * dx + dy * dy;
        if (distSq < closestDist) {
            closestDist = distSq;
            state.interaction.hoveredSnapPoint.edgeId = edgeId;
            state.interaction.hoveredSnapPoint.isSource = false;
        }
    }
}

void DemoInputHandler::updateHoveredBendPoint(DemoState& state, [[maybe_unused]] const Point& graphMouse) {
    state.interaction.hoveredBendPoint.clear();
    state.interaction.bendPointPreview.clear();

    // Bend point editing disabled in current version
    // graphMouse will be used when bend point hover detection is implemented
}

void DemoInputHandler::updateHoveredEdge(DemoState& state, const Point& graphMouse) {
    state.interaction.hoveredEdge = INVALID_EDGE;

    if (state.interaction.hoveredNode != INVALID_NODE ||
        state.interaction.hoveredBendPoint.isValid() ||
        state.interaction.hoveredSnapPoint.isValid()) {
        return;
    }

    constexpr float EDGE_HIT_THRESHOLD = 5.0f;
    for (const auto& [edgeId, layout] : *state.edgeLayouts) {
        auto hitResult = LayoutUtils::hitTestEdge(graphMouse, layout, EDGE_HIT_THRESHOLD);
        if (hitResult.hit) {
            state.interaction.hoveredEdge = edgeId;
            break;
        }
    }
}

void DemoInputHandler::handleClicks(DemoState& state, const ImGuiIO& io, const Point& graphMouse) {
    if (!ImGui::IsMouseClicked(0)) return;

    auto& interaction = state.interaction;

    // Snap point click - start drag
    if (interaction.hoveredSnapPoint.isValid()) {
        interaction.selectedEdge = interaction.hoveredSnapPoint.edgeId;
        interaction.selectedNode = INVALID_NODE;
        interaction.selectedBendPoint.clear();
        interaction.draggingSnapPoint = interaction.hoveredSnapPoint;

        // Get the snap point position
        auto it = state.edgeLayouts->find(interaction.draggingSnapPoint.edgeId);
        if (it != state.edgeLayouts->end()) {
            Point snapPos = interaction.draggingSnapPoint.isSource
                ? it->second.sourcePoint
                : it->second.targetPoint;
            interaction.snapPointDragOffset = {graphMouse.x - snapPos.x, graphMouse.y - snapPos.y};
            interaction.snapPointDragStart = snapPos;

            // Start snap drag using controller
            if (state.snapController) {
                auto dragResult = state.snapController->startDrag(
                    interaction.draggingSnapPoint.edgeId,
                    interaction.draggingSnapPoint.isSource,
                    *state.nodeLayouts,
                    *state.edgeLayouts,
                    state.renderOptions.gridSize
                );
                // If drag start failed, clear the dragging state
                if (!dragResult.success) {
                    interaction.draggingSnapPoint.clear();
                }
            }
        }
        return;
    }

    // Bend point click - start drag
    if (interaction.hoveredBendPoint.isValid()) {
        interaction.selectedNode = INVALID_NODE;
        interaction.selectedEdge = interaction.hoveredBendPoint.edgeId;
        interaction.draggingBendPoint = interaction.hoveredBendPoint;
        const auto& bps = manualManager_->getBendPoints(interaction.draggingBendPoint.edgeId);
        interaction.bendPointDragOffset = {
            graphMouse.x - bps[interaction.draggingBendPoint.bendPointIndex].position.x,
            graphMouse.y - bps[interaction.draggingBendPoint.bendPointIndex].position.y
        };
        return;
    }

    // Bend point preview click - insert bend points
    if (interaction.bendPointPreview.active) {
        // Insert bend points to maintain orthogonal routing
        EdgeId edgeId = interaction.bendPointPreview.edgeId;
        Point clickPos = interaction.bendPointPreview.position;

        auto it = state.edgeLayouts->find(edgeId);
        if (it == state.edgeLayouts->end()) return;
        const auto& edgeLayout = it->second;

        // Capture current edge routing BEFORE adding bend points
        if (!manualManager_->hasManualBendPoints(edgeId)) {
            EdgeRoutingConfig routing;
            routing.sourceEdge = edgeLayout.sourceEdge;
            routing.targetEdge = edgeLayout.targetEdge;

            auto srcNodeIt = state.nodeLayouts->find(edgeLayout.from);
            auto tgtNodeIt = state.nodeLayouts->find(edgeLayout.to);
            float gridSize = state.layoutOptions->gridConfig.cellSize > 0
                ? state.layoutOptions->gridConfig.cellSize : 10.0f;

            if (srcNodeIt != state.nodeLayouts->end()) {
                routing.sourceSnapIndex = GridSnapCalculator::getCandidateIndexFromPosition(
                    srcNodeIt->second, edgeLayout.sourceEdge, edgeLayout.sourcePoint, gridSize);
            }
            if (tgtNodeIt != state.nodeLayouts->end()) {
                routing.targetSnapIndex = GridSnapCalculator::getCandidateIndexFromPosition(
                    tgtNodeIt->second, edgeLayout.targetEdge, edgeLayout.targetPoint, gridSize);
            }
            manualManager_->setEdgeRouting(edgeId, routing);
        }

        const auto& existingBps = manualManager_->getBendPoints(edgeId);
        auto bpResult = OrthogonalRouter::calculateBendPointPair(
            edgeLayout, existingBps, clickPos, interaction.bendPointPreview.insertIndex);

        // Insert both points
        manualManager_->addBendPoint(edgeId, bpResult.insertIndex,
            GridPoint::fromPixel(bpResult.first, state.renderOptions.gridSize),
            state.renderOptions.gridSize);
        manualManager_->addBendPoint(edgeId, bpResult.insertIndex + 1,
            GridPoint::fromPixel(bpResult.second, state.renderOptions.gridSize),
            state.renderOptions.gridSize);

        interaction.selectedBendPoint.edgeId = edgeId;
        interaction.selectedBendPoint.bendPointIndex = static_cast<int>(bpResult.insertIndex) + 1;

        if (doLayoutCallback_) doLayoutCallback_();
        return;
    }

    // Node click - select and prepare for drag
    if (interaction.hoveredNode != INVALID_NODE) {
        interaction.selectedNode = interaction.hoveredNode;
        interaction.selectedEdge = INVALID_EDGE;
        interaction.selectedBendPoint.clear();
        interaction.pendingNodeDrag = true;
        interaction.nodeClickStart = {io.MousePos.x, io.MousePos.y};

        auto nodeIt = state.nodeLayouts->find(interaction.hoveredNode);
        if (nodeIt != state.nodeLayouts->end()) {
            interaction.dragOffset = {
                graphMouse.x - nodeIt->second.position.x,
                graphMouse.y - nodeIt->second.position.y
            };
        }
        return;
    }

    // Edge click - select
    if (interaction.hoveredEdge != INVALID_EDGE) {
        interaction.selectedEdge = interaction.hoveredEdge;
        interaction.selectedNode = INVALID_NODE;
        interaction.selectedBendPoint.clear();
        return;
    }

    // Empty area click - start potential pan
    if (interaction.hoveredNode == INVALID_NODE &&
        interaction.hoveredEdge == INVALID_EDGE &&
        !interaction.hoveredBendPoint.isValid() &&
        !interaction.hoveredSnapPoint.isValid()) {
        interaction.emptyAreaPanStarted = true;
    }
}

void DemoInputHandler::handleDragThreshold(DemoState& state, const ImGuiIO& io) {
    auto& interaction = state.interaction;

    if (!interaction.pendingNodeDrag || !ImGui::IsMouseDragging(ImGuiMouseButton_Left)) {
        return;
    }

    float dx = io.MousePos.x - interaction.nodeClickStart.x;
    float dy = io.MousePos.y - interaction.nodeClickStart.y;
    float distance = std::sqrt(dx * dx + dy * dy);

    if (distance > InteractionState::DRAG_THRESHOLD) {
        interaction.pendingNodeDrag = false;
        interaction.draggedNode = interaction.selectedNode;
        if (startDragCallback_) {
            startDragCallback_(interaction.selectedNode);
        }
    }
}

void DemoInputHandler::handleEmptyAreaPan(DemoState& state, const ImGuiIO& io) {
    auto& interaction = state.interaction;

    if (interaction.emptyAreaPanStarted && ImGui::IsMouseDragging(ImGuiMouseButton_Left)) {
        state.view.panOffset.x += io.MouseDelta.x / state.view.zoom;
        state.view.panOffset.y += io.MouseDelta.y / state.view.zoom;
        interaction.isPanning = true;
    }
}

void DemoInputHandler::handleMouseRelease(DemoState& state, [[maybe_unused]] const ImGuiIO& io, const Point& graphMouse) {
    if (!ImGui::IsMouseReleased(0)) return;

    auto& interaction = state.interaction;

    // Clear pending drag state
    interaction.pendingNodeDrag = false;

    // Handle snap point drag release
    if (interaction.draggingSnapPoint.isValid()) {
        if (state.snapController && state.snapController->isDragging()) {
            Point dropPosition = {
                graphMouse.x - interaction.snapPointDragOffset.x,
                graphMouse.y - interaction.snapPointDragOffset.y
            };

            auto dropResult = state.snapController->completeDrag(
                interaction.snappedCandidateIndex,
                dropPosition,
                *state.nodeLayouts,
                *state.edgeLayouts,
                *state.layoutOptions);

            if (dropResult.success) {
                // DO NOT add to affectedEdges - bend points are already regenerated in completeDrag()
                // Adding them would cause them to be hidden if HideUntilDrop mode is active
                LOG_DEBUG("[SNAP-DROP] completeDrag success: {} edges affected, bendPoints already regenerated",
                          dropResult.affectedEdges.size());
            }
        }
        // Always clear snap drag state when draggingSnapPoint is valid
        // This handles edge cases where controller state got out of sync
        interaction.draggingSnapPoint.clear();
        interaction.snappedCandidateIndex = -1;
        interaction.hasSnapPreview = false;
    }

    // Handle bend point drag release
    if (interaction.draggingBendPoint.isValid()) {
        interaction.draggingBendPoint.clear();
    }

    // Handle node drag release
    if (interaction.draggedNode != INVALID_NODE) {
        if (interaction.isInvalidDragPosition) {
            // Revert to last valid position
            auto nodeIt = state.nodeLayouts->find(interaction.draggedNode);
            if (nodeIt != state.nodeLayouts->end()) {
                nodeIt->second.position = interaction.lastValidPosition;
            }
        }
        
        // Signal drag end - triggers A* optimization via coordinator
        if (endDragCallback_) {
            endDragCallback_(interaction.draggedNode);
        }
        
        interaction.draggedNode = INVALID_NODE;

        // Clear affected edges when not in HideUntilDrop mode
        if (state.layoutOptions->optimizationOptions.dragAlgorithm != DragAlgorithm::HideUntilDrop) {
            interaction.affectedEdges.clear();
        }
        interaction.isInvalidDragPosition = false;
        interaction.lastRoutedPosition = {-9999, -9999};

        // Handle empty area pan release
        if (interaction.emptyAreaPanStarted) {
            if (!interaction.isPanning) {
                // Was just a click - deselect everything
                interaction.selectedNode = INVALID_NODE;
                interaction.selectedEdge = INVALID_EDGE;
                interaction.selectedBendPoint.clear();
            }
            interaction.emptyAreaPanStarted = false;
            interaction.isPanning = false;
        }
    }
}

void DemoInputHandler::handleSnapPointDrag(DemoState& state, const ImGuiIO& io, const Point& graphMouse) {
    auto& interaction = state.interaction;

    if (!interaction.draggingSnapPoint.isValid() || !ImGui::IsMouseDragging(0)) {
        return;
    }

    if (!state.snapController || !state.snapController->isDragging()) {
        return;
    }

    // Skip if mouse hasn't moved (avoid redundant A* calculations)
    if (io.MouseDelta.x == 0.0f && io.MouseDelta.y == 0.0f) {
        return;
    }

    Point dragPosition = {
        graphMouse.x - interaction.snapPointDragOffset.x,
        graphMouse.y - interaction.snapPointDragOffset.y
    };

    auto updateResult = state.snapController->updateDrag(
        dragPosition, *state.nodeLayouts, state.renderOptions.gridSize);

    interaction.snappedCandidateIndex = updateResult.snappedCandidateIndex;
    interaction.hasSnapPreview = updateResult.hasValidPreview;
}

void DemoInputHandler::handleBendPointDrag(DemoState& state, [[maybe_unused]] const ImGuiIO& io, const Point& graphMouse) {
    auto& interaction = state.interaction;

    if (!interaction.draggingBendPoint.isValid() || !ImGui::IsMouseDragging(0)) {
        return;
    }

    EdgeId edgeId = interaction.draggingBendPoint.edgeId;
    int bpIdx = interaction.draggingBendPoint.bendPointIndex;

    const auto& bps = manualManager_->getBendPoints(edgeId);
    if (bpIdx >= static_cast<int>(bps.size())) {
        interaction.draggingBendPoint.clear();
        return;
    }

    auto edgeIt = state.edgeLayouts->find(edgeId);
    if (edgeIt == state.edgeLayouts->end()) return;
    const auto& edgeLayout = edgeIt->second;

    // Determine prev point
    Point prevPoint = (bpIdx == 0)
        ? edgeLayout.sourcePoint
        : bps[bpIdx - 1].position;

    // Determine next point
    Point nextPoint = (bpIdx == static_cast<int>(bps.size()) - 1)
        ? edgeLayout.targetPoint
        : bps[bpIdx + 1].position;

    Point currentPos = bps[bpIdx].position;
    Point dragTarget = {
        graphMouse.x - interaction.bendPointDragOffset.x,
        graphMouse.y - interaction.bendPointDragOffset.y
    };

    bool isLastBend = (bpIdx == static_cast<int>(bps.size()) - 1);
    bool hasNextBend = (bpIdx < static_cast<int>(bps.size()) - 1);

    auto dragResult = OrthogonalRouter::calculateOrthogonalDrag(
        prevPoint, currentPos, nextPoint, dragTarget, hasNextBend, isLastBend);

    // Apply the calculated positions
    if (dragResult.nextAdjusted) {
        manualManager_->moveBendPoint(edgeId, static_cast<size_t>(bpIdx + 1),
            GridPoint::fromPixel(dragResult.adjustedNextPos, state.renderOptions.gridSize),
            state.renderOptions.gridSize);
    }
    manualManager_->moveBendPoint(edgeId, static_cast<size_t>(bpIdx),
        GridPoint::fromPixel(dragResult.newCurrentPos, state.renderOptions.gridSize),
        state.renderOptions.gridSize);

    // Update edge layout for visual feedback
    if (bpIdx < static_cast<int>(edgeIt->second.bendPoints.size())) {
        edgeIt->second.bendPoints[bpIdx].position = dragResult.newCurrentPos;
    }
    if (dragResult.nextAdjusted && bpIdx + 1 < static_cast<int>(edgeIt->second.bendPoints.size())) {
        edgeIt->second.bendPoints[bpIdx + 1].position = dragResult.adjustedNextPos;
    }
}

void DemoInputHandler::handleNodeDrag(DemoState& state, [[maybe_unused]] const ImGuiIO& io, const Point& graphMouse) {
    auto& interaction = state.interaction;

    if (interaction.draggedNode == INVALID_NODE || !ImGui::IsMouseDragging(0)) {
        return;
    }

    auto nodeIt = state.nodeLayouts->find(interaction.draggedNode);
    if (nodeIt == state.nodeLayouts->end()) return;
    auto& layout = nodeIt->second;

    float newX = graphMouse.x - interaction.dragOffset.x;
    float newY = graphMouse.y - interaction.dragOffset.y;

    // Snap to grid
    float gridSize = state.layoutOptions->gridConfig.cellSize;
    if (gridSize > 0.0f) {
        newX = std::round(newX / gridSize) * gridSize;
        newY = std::round(newY / gridSize) * gridSize;
    }

    Point proposedPosition = {newX, newY};

    // Skip if position hasn't changed significantly (avoid redundant computation)
    constexpr float POS_CHANGE_THRESHOLD = 0.1f;
    if (std::abs(proposedPosition.x - interaction.lastValidPosition.x) < POS_CHANGE_THRESHOLD &&
        std::abs(proposedPosition.y - interaction.lastValidPosition.y) < POS_CHANGE_THRESHOLD) {
        return;
    }

    // Always update position during drag (visual feedback)
    layout.position.x = newX;
    layout.position.y = newY;

    // Validate using callback (avoids dangling pointer issue with cached constraintManager)
    bool isValid = true;
    if (validateDragCallback_) {
        isValid = validateDragCallback_(interaction.draggedNode, proposedPosition);
    }

    if (isValid) {
        interaction.isInvalidDragPosition = false;
        interaction.lastValidPosition = proposedPosition;
        manualManager_->setNodePosition(interaction.draggedNode, layout.position);

        // Re-route in non-HideUntilDrop modes
        if (state.layoutOptions->optimizationOptions.dragAlgorithm != DragAlgorithm::HideUntilDrop) {
            if (rerouteEdgesCallback_) rerouteEdgesCallback_();
        }
        interaction.lastRoutedPosition = proposedPosition;
    } else {
        interaction.isInvalidDragPosition = true;
    }
}

void DemoInputHandler::handleKeyboard(DemoState& state, [[maybe_unused]] const ImGuiIO& io) {
    auto& interaction = state.interaction;

    // Delete key for bend point removal
    if (interaction.selectedBendPoint.isValid() && ImGui::IsKeyPressed(ImGuiKey_Delete)) {
        manualManager_->removeBendPoint(
            interaction.selectedBendPoint.edgeId,
            static_cast<size_t>(interaction.selectedBendPoint.bendPointIndex)
        );
        interaction.selectedBendPoint.clear();
        if (doLayoutCallback_) doLayoutCallback_();
    }
}

}  // namespace arborvia
