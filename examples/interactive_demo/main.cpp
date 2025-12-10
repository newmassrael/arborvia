#include <SDL3/SDL.h>
#include <imgui.h>
#include <imgui_impl_sdl3.h>
#include <imgui_impl_sdlrenderer3.h>

#include <arborvia/arborvia.h>
#include <arborvia/layout/interactive/PathRoutingCoordinator.h>
#include <arborvia/layout/api/LayoutController.h>
#include "../../src/layout/interactive/ValidRegionCalculator.h"
#include <arborvia/layout/interactive/SnapPointController.h>
#include "../../src/layout/pathfinding/ObstacleMap.h"
#include "../../src/layout/snap/GridSnapCalculator.h"
#include "../../src/layout/pathfinding/AStarPathFinder.h"
#include "../../src/layout/routing/OrthogonalRouter.h"
#include "../../src/layout/routing/CooperativeRerouter.h"
#include "CommandServer.h"

#include <unordered_map>
#include <unordered_set>
#include <algorithm>
#include <string>
#include <cmath>
#include <fstream>
#include <iostream>
#include <sstream>
#include <mutex>
#include <thread>
#include <chrono>

using namespace arborvia;

// Log capture system for TCP polling
class LogCapture {
public:
    static LogCapture& instance() {
        static LogCapture inst;
        return inst;
    }

    void install() {
        if (!installed_) {
            originalBuf_ = std::cout.rdbuf(&captureBuf_);
            installed_ = true;
        }
    }

    void uninstall() {
        if (installed_) {
            std::cout.rdbuf(originalBuf_);
            installed_ = false;
        }
    }

    std::string getAndClear() {
        std::lock_guard<std::mutex> lock(mutex_);
        std::string result = buffer_.str();
        buffer_.str("");
        buffer_.clear();
        return result;
    }

    std::string get() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return buffer_.str();
    }

    void clear() {
        std::lock_guard<std::mutex> lock(mutex_);
        buffer_.str("");
        buffer_.clear();
    }

    void append(const std::string& s) {
        std::lock_guard<std::mutex> lock(mutex_);
        buffer_ << s;
    }

private:
    LogCapture() : captureBuf_(buffer_, originalBuf_, mutex_) {}
    ~LogCapture() { uninstall(); }

    // Custom streambuf that captures output and also forwards to original
    class CaptureStreamBuf : public std::streambuf {
    public:
        CaptureStreamBuf(std::ostringstream& buffer, std::streambuf*& original, std::mutex& mutex)
            : buffer_(buffer), original_(original), mutex_(mutex) {}

    protected:
        int overflow(int c) override {
            if (c != EOF) {
                std::lock_guard<std::mutex> lock(mutex_);
                buffer_.put(static_cast<char>(c));
                if (original_) {
                    original_->sputc(static_cast<char>(c));
                }
            }
            return c;
        }

        std::streamsize xsputn(const char* s, std::streamsize n) override {
            std::lock_guard<std::mutex> lock(mutex_);
            buffer_.write(s, n);
            if (original_) {
                original_->sputn(s, n);
            }
            return n;
        }

    private:
        std::ostringstream& buffer_;
        std::streambuf*& original_;
        std::mutex& mutex_;
    };

    std::ostringstream buffer_;
    std::streambuf* originalBuf_ = nullptr;
    CaptureStreamBuf captureBuf_;
    mutable std::mutex mutex_;
    bool installed_ = false;
};

// Colors
const ImU32 COLOR_NODE = IM_COL32(200, 200, 200, 255);
const ImU32 COLOR_NODE_HOVER = IM_COL32(150, 200, 255, 255);
const ImU32 COLOR_NODE_DRAG = IM_COL32(100, 180, 255, 255);
const ImU32 COLOR_NODE_BORDER = IM_COL32(80, 80, 80, 255);
const ImU32 COLOR_EDGE = IM_COL32(100, 100, 100, 255);
const ImU32 COLOR_EDGE_AFFECTED = IM_COL32(255, 100, 100, 255);
const ImU32 COLOR_TEXT = IM_COL32(0, 0, 0, 255);
const ImU32 COLOR_SNAP_POINT = IM_COL32(0, 150, 255, 200);
const ImU32 COLOR_SNAP_POINT_HOVER = IM_COL32(255, 200, 0, 255);
const ImU32 COLOR_EDGE_SELECTED = IM_COL32(0, 200, 100, 255);
const ImU32 COLOR_NODE_SELECTED = IM_COL32(100, 255, 100, 255);

// Grid obstacle colors
const ImU32 COLOR_BLOCKED_CELL = IM_COL32(255, 100, 100, 60);  // Red, semi-transparent

// Bend point colors
const ImU32 COLOR_BEND_POINT = IM_COL32(100, 150, 255, 200);
const ImU32 COLOR_BEND_POINT_HOVER = IM_COL32(150, 200, 255, 255);
const ImU32 COLOR_BEND_POINT_SELECTED = IM_COL32(100, 255, 100, 255);
const ImU32 COLOR_BEND_POINT_DRAGGING = IM_COL32(255, 180, 100, 255);
const ImU32 COLOR_BEND_POINT_PREVIEW = IM_COL32(100, 200, 255, 128);

// Bend point interaction state
struct HoveredBendPoint {
    EdgeId edgeId = INVALID_EDGE;
    int bendPointIndex = -1;
    bool isValid() const { return edgeId != INVALID_EDGE && bendPointIndex >= 0; }
    void clear() { edgeId = INVALID_EDGE; bendPointIndex = -1; }
};

struct BendPointPreview {
    EdgeId edgeId = INVALID_EDGE;
    int insertIndex = -1;
    Point position = {0, 0};
    bool active = false;
    void clear() { edgeId = INVALID_EDGE; insertIndex = -1; active = false; }
};

// Snap point interaction state (for source/target snap points)
struct HoveredSnapPoint {
    EdgeId edgeId = INVALID_EDGE;
    bool isSource = true;  // true = source snap, false = target snap
    bool isValid() const { return edgeId != INVALID_EDGE; }
    void clear() { edgeId = INVALID_EDGE; }
};

// Snap point candidate for drag preview
struct SnapPointCandidate {
    NodeEdge edge;
    int candidateIndex;
    Point position;
};

class InteractiveDemo : public IRoutingListener {
public:
    explicit InteractiveDemo(int port = 9999)
        : manualManager_(std::make_shared<ManualLayoutManager>())
        , routingCoordinator_(std::make_shared<PathRoutingCoordinator>())
        , commandServer_(port)
        , port_(port)
    {
        // Enable grid by default
        layoutOptions_.gridConfig.cellSize = gridSize_;

        // Configure routing coordinator
        routingCoordinator_->setDebounceDelay(0);  // Immediate A* after drop (no delay)
        routingCoordinator_->setListener(this);
        routingCoordinator_->setOptimizationCallback(
            [this](const std::vector<EdgeId>& /*affectedEdges*/,
                   const std::unordered_set<NodeId>& movedNodes) {
            // Re-route ALL edges with A* after drop
            // This ensures all transitions are recalculated, including self-loops
            std::vector<EdgeId> allEdges;
            allEdges.reserve(edgeLayouts_.size());
            for (const auto& [edgeId, layout] : edgeLayouts_) {
                allEdges.push_back(edgeId);
            }

            std::cout << "[DEBUG-optimizationCallback] allEdges.size()=" << allEdges.size() 
                      << " movedNodes.size()=" << movedNodes.size() << std::endl;

            LayoutUtils::updateEdgePositions(
                edgeLayouts_, nodeLayouts_, allEdges,
                layoutOptions_, movedNodes);
        });

        setupGraph();
        doLayout();
    }

    // IRoutingListener implementation
    void onOptimizationComplete(const std::vector<EdgeId>& optimizedEdges) override {
        std::cout << "Optimization complete: " << optimizedEdges.size() << " edges re-routed" << std::endl;
        // Clear affected edges now that optimization is complete - edges become visible
        affectedEdges_.clear();
    }

    void saveLayout(const std::string& path) {
        // Create LayoutResult from current state (nodeLayouts_ and edgeLayouts_)
        // This ensures we save the dragged positions, not the original layoutResult_
        LayoutResult currentResult;
        for (const auto& [id, layout] : nodeLayouts_) {
            currentResult.setNodeLayout(id, layout);
        }
        for (const auto& [id, layout] : edgeLayouts_) {
            currentResult.setEdgeLayout(id, layout);
        }
        currentResult.setLayerCount(layoutResult_.layerCount());

        // Get full layout result JSON (includes all coordinates)
        std::string layoutJson = LayoutSerializer::toJson(currentResult);

        // Print formatted JSON to console
        std::cout << "\n=== Full Layout Saved to " << path << " ===" << std::endl;
        std::cout << layoutJson << std::endl;
        std::cout << "==========================================\n" << std::endl;

        // Save full layout result to file
        std::ofstream file(path);
        if (file.is_open()) {
            file << layoutJson;
            file.close();
            std::cout << "Layout saved successfully to: " << path << std::endl;
        } else {
            std::cerr << "Error: Failed to save layout to " << path << std::endl;
        }
    }

    void loadLayout(const std::string& path) {
        if (manualManager_->loadFromFile(path)) {
            doLayout();
        }
    }

    void setupGraph() {
        // Create sample state machine graph
        auto idle = graph_.addNode(Size{200, 100}, "Idle");
        auto running = graph_.addNode(Size{200, 100}, "Running");
        auto paused = graph_.addNode(Size{200, 100}, "Paused");
        auto stopped = graph_.addNode(Size{200, 100}, "Stopped");
        auto error = graph_.addNode(Size{200, 100}, "Error");

        graph_.addEdge(idle, running, "start");
        graph_.addEdge(running, paused, "pause");
        graph_.addEdge(paused, running, "resume");
        graph_.addEdge(running, stopped, "stop");
        graph_.addEdge(paused, stopped, "stop");
        graph_.addEdge(running, error, "fail");
        graph_.addEdge(error, idle, "reset");
        graph_.addEdge(error, error, "retry");  // Self-loop for demo
    }

    void doLayout() {
        SugiyamaLayout layout;
        layout.setOptions(layoutOptions_);
        layout.setManualLayoutManager(manualManager_);
        layoutResult_ = layout.layout(graph_);

        nodeLayouts_.clear();
        edgeLayouts_.clear();

        for (const auto& [id, layout] : layoutResult_.nodeLayouts()) {
            nodeLayouts_[id] = layout;
        }
        for (const auto& [id, layout] : layoutResult_.edgeLayouts()) {
            edgeLayouts_[id] = layout;
        }

        // Sync snap configs from edge layouts (for Auto mode snap point visibility)
        syncSnapConfigsFromEdges();

        // Initialize LayoutController with current state
        layoutController_ = std::make_unique<LayoutController>(graph_, layoutOptions_);
        layoutController_->initializeFrom(nodeLayouts_, edgeLayouts_);

        // Offset for centering
        offset_ = {100.0f, 50.0f};
    }

    // Re-route edges only, preserving current node positions
    void reRouteEdgesOnly() {
        // Save current node positions to manual manager
        for (const auto& [id, layout] : nodeLayouts_) {
            manualManager_->setNodePosition(id, layout.position);
        }

        // Clear edge routings so fresh routing from algorithm is used
        manualManager_->clearAllEdgeRoutings();

        // Do layout with new grid size
        doLayout();

        // Update edge positions to match current node positions
        std::vector<EdgeId> allEdges;
        for (const auto& [edgeId, _] : edgeLayouts_) {
            allEdges.push_back(edgeId);
        }
        LayoutUtils::updateEdgePositions(
            edgeLayouts_, nodeLayouts_, allEdges,
            {}, layoutOptions_.gridConfig.cellSize);
    }

    void syncSnapConfigsFromEdges() {
        // Use library API for snap config synchronization
        manualManager_->syncSnapConfigsFromEdgeLayouts(edgeLayouts_);
    }

    void update() {
        // Update routing coordinator for debounce timing (triggers optimization after delay)
        routingCoordinator_->update(SDL_GetTicks());

        ImGuiIO& io = ImGui::GetIO();

        // Skip graph interaction if ImGui wants mouse
        if (io.WantCaptureMouse) {
            return;
        }

        ImVec2 mousePos = io.MousePos;

        // Convert to graph coordinates
        Point graphMouse = {mousePos.x - offset_.x, mousePos.y - offset_.y};

        // Find hovered node
        hoveredNode_ = INVALID_NODE;
        for (const auto& [id, layout] : nodeLayouts_) {
            Rect bounds = layout.bounds();
            if (graphMouse.x >= bounds.x && graphMouse.x <= bounds.right() &&
                graphMouse.y >= bounds.y && graphMouse.y <= bounds.bottom()) {
                hoveredNode_ = id;
                break;
            }
        }

        // Find hovered snap point (source/target connection points)
        hoveredSnapPoint_.clear();
        if (showSnapPoints_ && hoveredNode_ == INVALID_NODE && !draggingSnapPoint_.isValid()) {
            constexpr float SNAP_HIT_RADIUS = 8.0f;
            float closestDist = SNAP_HIT_RADIUS * SNAP_HIT_RADIUS;
            for (const auto& [edgeId, layout] : edgeLayouts_) {
                // Check source snap point
                float dx = graphMouse.x - layout.sourcePoint.x;
                float dy = graphMouse.y - layout.sourcePoint.y;
                float distSq = dx * dx + dy * dy;
                if (distSq < closestDist) {
                    closestDist = distSq;
                    hoveredSnapPoint_.edgeId = edgeId;
                    hoveredSnapPoint_.isSource = true;
                }
                // Check target snap point
                dx = graphMouse.x - layout.targetPoint.x;
                dy = graphMouse.y - layout.targetPoint.y;
                distSq = dx * dx + dy * dy;
                if (distSq < closestDist) {
                    closestDist = distSq;
                    hoveredSnapPoint_.edgeId = edgeId;
                    hoveredSnapPoint_.isSource = false;
                }
            }
        }

        // Find hovered bend point (manual mode feature - disabled)
        hoveredBendPoint_.clear();
        bendPointPreview_.clear();
        if (false && hoveredNode_ == INVALID_NODE) {
            for (const auto& [edgeId, layout] : edgeLayouts_) {
                const auto& bps = manualManager_->getBendPoints(edgeId);
                for (size_t i = 0; i < bps.size(); ++i) {
                    float dx = graphMouse.x - bps[i].position.x;
                    float dy = graphMouse.y - bps[i].position.y;
                    if (dx * dx + dy * dy < 64.0f) {  // 8 pixel radius
                        hoveredBendPoint_.edgeId = edgeId;
                        hoveredBendPoint_.bendPointIndex = static_cast<int>(i);
                        break;
                    }
                }
                if (hoveredBendPoint_.isValid()) break;
            }
        }

        // Find hovered edge and calculate bend point insertion preview
        hoveredEdge_ = INVALID_EDGE;
        if (hoveredNode_ == INVALID_NODE && !hoveredBendPoint_.isValid()) {
            for (const auto& [id, layout] : edgeLayouts_) {
                auto hitResult = LayoutUtils::hitTestEdge(graphMouse, layout, 8.0f);
                if (hitResult.hit) {
                    hoveredEdge_ = id;
                    // Show insert preview if edge is selected (manual mode - disabled)
                    if (false && id == selectedEdge_) {
                        bendPointPreview_.edgeId = id;
                        bendPointPreview_.insertIndex = hitResult.segmentIndex;
                        bendPointPreview_.position = hitResult.closestPoint;
                        bendPointPreview_.active = true;
                    }
                    break;
                }
            }
        }

        // Handle snap point interaction (start drag)
        if (hoveredSnapPoint_.isValid() && ImGui::IsMouseClicked(0)) {
            draggingSnapPoint_ = hoveredSnapPoint_;
            selectedEdge_ = hoveredSnapPoint_.edgeId;

            // Use SnapPointController to start drag
            auto startResult = snapController_.startDrag(
                hoveredSnapPoint_.edgeId,
                hoveredSnapPoint_.isSource,
                nodeLayouts_,
                edgeLayouts_,
                gridSize_);

            if (startResult.success) {
                snapPointDragStart_ = startResult.originalPosition;
                auto it = edgeLayouts_.find(hoveredSnapPoint_.edgeId);
                if (it != edgeLayouts_.end()) {
                    Point snapPos = hoveredSnapPoint_.isSource ?
                        it->second.sourcePoint : it->second.targetPoint;
                    snapPointDragOffset_ = {graphMouse.x - snapPos.x, graphMouse.y - snapPos.y};
                }
                snappedCandidateIndex_ = -1;
                hasSnapPreview_ = false;
            }
        }

        // Handle bend point interaction
        else if (hoveredBendPoint_.isValid() && ImGui::IsMouseClicked(0)) {
            // Start dragging bend point
            selectedBendPoint_ = hoveredBendPoint_;
            selectedEdge_ = hoveredBendPoint_.edgeId;
            draggingBendPoint_ = hoveredBendPoint_;
            const auto& bps = manualManager_->getBendPoints(draggingBendPoint_.edgeId);
            bendPointDragOffset_ = {
                graphMouse.x - bps[draggingBendPoint_.bendPointIndex].position.x,
                graphMouse.y - bps[draggingBendPoint_.bendPointIndex].position.y
            };
        } else if (bendPointPreview_.active && ImGui::IsMouseClicked(0)) {
            // Insert bend points to maintain orthogonal routing
            EdgeId edgeId = bendPointPreview_.edgeId;
            Point clickPos = bendPointPreview_.position;

            const auto& edgeLayout = edgeLayouts_[edgeId];

            // IMPORTANT: Capture current edge routing BEFORE adding bend points
            // Otherwise addBendPoint() creates a config with default values (Bottom->Top)
            // which would reset the edge routing to wrong values
            if (!manualManager_->hasManualBendPoints(edgeId)) {
                EdgeRoutingConfig routing;
                routing.sourceEdge = edgeLayout.sourceEdge;
                routing.targetEdge = edgeLayout.targetEdge;
                routing.sourceSnapIndex = edgeLayout.sourceSnapIndex;
                routing.targetSnapIndex = edgeLayout.targetSnapIndex;
                manualManager_->setEdgeRouting(edgeId, routing);
            }

            const auto& existingBps = manualManager_->getBendPoints(edgeId);

            // Use library API to calculate bend point pair
            auto bpResult = OrthogonalRouter::calculateBendPointPair(
                edgeLayout, existingBps, clickPos, bendPointPreview_.insertIndex);

            // Insert both points
            manualManager_->addBendPoint(edgeId, bpResult.insertIndex, bpResult.first);
            manualManager_->addBendPoint(edgeId, bpResult.insertIndex + 1, bpResult.second);

            selectedBendPoint_.edgeId = edgeId;
            selectedBendPoint_.bendPointIndex = static_cast<int>(bpResult.insertIndex) + 1;
            doLayout();
        } else if (ImGui::IsMouseClicked(0) && hoveredNode_ != INVALID_NODE) {
            // Handle click for node selection
            selectedNode_ = hoveredNode_;
            selectedEdge_ = INVALID_EDGE;
            selectedBendPoint_.clear();
            draggedNode_ = hoveredNode_;
            auto& layout = nodeLayouts_[draggedNode_];
            dragOffset_ = {graphMouse.x - layout.position.x,
                          graphMouse.y - layout.position.y};
            affectedEdges_ = graph_.getConnectedEdges(draggedNode_);
            // Initialize drag constraint state
            lastValidPosition_ = layout.position;
            isInvalidDragPosition_ = false;
            // Pre-calculate forbidden zones based on edge direction counts
            forbiddenZones_ = ValidRegionCalculator::calculate(
                draggedNode_, nodeLayouts_, edgeLayouts_, gridSize_);
            // Sync LayoutController state before drag
            if (layoutController_) {
                layoutController_->initializeFrom(nodeLayouts_, edgeLayouts_);
            }
            // Notify coordinator that drag started
            routingCoordinator_->onDragStart(affectedEdges_);
        } else if (ImGui::IsMouseClicked(0) && hoveredEdge_ != INVALID_EDGE) {
            selectedEdge_ = hoveredEdge_;
            selectedNode_ = INVALID_NODE;
            selectedBendPoint_.clear();
        } else if (ImGui::IsMouseClicked(0) && hoveredNode_ == INVALID_NODE && hoveredEdge_ == INVALID_EDGE && !hoveredBendPoint_.isValid()) {
            selectedNode_ = INVALID_NODE;
            selectedEdge_ = INVALID_EDGE;
            selectedBendPoint_.clear();
        }

        if (ImGui::IsMouseReleased(0)) {
            // Handle snap point drag release using SnapPointController
            if (draggingSnapPoint_.isValid() && snapController_.isDragging()) {
                Point dropPosition = {graphMouse.x - snapPointDragOffset_.x,
                                      graphMouse.y - snapPointDragOffset_.y};

                auto dropResult = snapController_.completeDrag(
                    snappedCandidateIndex_,
                    dropPosition,
                    nodeLayouts_,
                    edgeLayouts_,
                    layoutOptions_);

                if (!dropResult.success) {
                    std::cout << "[SnapDrag] Failed: " << dropResult.reason << std::endl;
                } else if (dropResult.swapEdgeId != INVALID_EDGE) {
                    std::cout << "[SnapDrag] Swapped with edge " << dropResult.swapEdgeId << std::endl;
                }
            } else if (draggingSnapPoint_.isValid()) {
                snapController_.cancelDrag();
            }
            draggingSnapPoint_.clear();
            snappedCandidateIndex_ = -1;
            hasSnapPreview_ = false;

            // Handle bend point drag release
            if (draggingBendPoint_.isValid()) {
                doLayout();
            }
            draggingBendPoint_.clear();

            if (draggedNode_ != INVALID_NODE) {
                auto& layout = nodeLayouts_[draggedNode_];
                
                // Use LayoutController for final validation with all constraints
                if (layoutController_) {
                    auto result = layoutController_->completeDrag(draggedNode_, layout.position);
                    
                    if (!result.success) {
                        // Constraint validation failed - revert to original position
                        layout.position = lastValidPosition_;
                        manualManager_->setNodePosition(draggedNode_, lastValidPosition_);
                        std::cout << "[Demo] Drag rejected: " << result.reason << std::endl;
                    } else {
                        // Success - sync from controller's validated state
                        for (const auto& [id, nodeLayout] : layoutController_->nodeLayouts()) {
                            nodeLayouts_[id] = nodeLayout;
                        }
                        for (const auto& [id, edgeLayout] : layoutController_->edgeLayouts()) {
                            edgeLayouts_[id] = edgeLayout;
                        }
                        manualManager_->setNodePosition(draggedNode_, result.actualPosition);
                    }
                } else {
                    // Fallback: old behavior if controller not available
                    if (isInvalidDragPosition_) {
                        layout.position = lastValidPosition_;
                        manualManager_->setNodePosition(draggedNode_, lastValidPosition_);
                    }
                    rerouteAffectedEdges();
                }

                // Signal drag end - coordinator will schedule optimization after delay
                routingCoordinator_->onDragEnd({draggedNode_});
            }
            draggedNode_ = INVALID_NODE;
            // Don't clear affectedEdges_ here when HideUntilDrop is active
            // They will be cleared in onOptimizationComplete() after A* finishes
            if (layoutOptions_.optimizationOptions.dragAlgorithm != DragAlgorithm::HideUntilDrop) {
                affectedEdges_.clear();
            }
            isInvalidDragPosition_ = false;
            lastRoutedPosition_ = {-9999, -9999};  // Reset for next drag
        }

        // Handle snap point dragging - use SnapPointController for preview
        if (draggingSnapPoint_.isValid() && ImGui::IsMouseDragging(0) && snapController_.isDragging()) {
            Point dragPosition = {graphMouse.x - snapPointDragOffset_.x,
                                  graphMouse.y - snapPointDragOffset_.y};

            auto updateResult = snapController_.updateDrag(
                dragPosition, nodeLayouts_, gridSize_);

            snappedCandidateIndex_ = updateResult.snappedCandidateIndex;
            hasSnapPreview_ = updateResult.hasValidPreview;
        }
        // Handle bend point dragging with orthogonal constraint
        else if (draggingBendPoint_.isValid() && ImGui::IsMouseDragging(0)) {
            EdgeId edgeId = draggingBendPoint_.edgeId;
            int bpIdx = draggingBendPoint_.bendPointIndex;

            // Get current bend points from manager (source of truth)
            const auto& bps = manualManager_->getBendPoints(edgeId);
            if (bpIdx >= static_cast<int>(bps.size())) {
                draggingBendPoint_.clear();
            } else {
                // Get the edge layout for source/target points
                const auto& edgeLayout = edgeLayouts_[edgeId];

                // Determine prev point (source or previous bend)
                Point prevPoint;
                if (bpIdx == 0) {
                    prevPoint = edgeLayout.sourcePoint;
                } else {
                    prevPoint = bps[bpIdx - 1].position;
                }

                // Determine next point (next bend or target)
                Point nextPoint;
                if (bpIdx == static_cast<int>(bps.size()) - 1) {
                    nextPoint = edgeLayout.targetPoint;
                } else {
                    nextPoint = bps[bpIdx + 1].position;
                }

                Point currentPos = bps[bpIdx].position;
                Point dragTarget = {graphMouse.x - bendPointDragOffset_.x,
                                    graphMouse.y - bendPointDragOffset_.y};

                bool isLastBend = (bpIdx == static_cast<int>(bps.size()) - 1);
                bool hasNextBend = (bpIdx < static_cast<int>(bps.size()) - 1);

                // Use library API for orthogonal drag constraint calculation
                auto dragResult = OrthogonalRouter::calculateOrthogonalDrag(
                    prevPoint, currentPos, nextPoint, dragTarget, hasNextBend, isLastBend);

                // Apply the calculated positions
                if (dragResult.nextAdjusted) {
                    manualManager_->moveBendPoint(edgeId, static_cast<size_t>(bpIdx + 1), dragResult.adjustedNextPos);
                }
                manualManager_->moveBendPoint(edgeId, static_cast<size_t>(bpIdx), dragResult.newCurrentPos);

                // Update edge layout immediately for visual feedback
                auto it = edgeLayouts_.find(edgeId);
                if (it != edgeLayouts_.end()) {
                    if (bpIdx < static_cast<int>(it->second.bendPoints.size())) {
                        it->second.bendPoints[bpIdx].position = dragResult.newCurrentPos;
                    }
                    if (dragResult.nextAdjusted && bpIdx + 1 < static_cast<int>(it->second.bendPoints.size())) {
                        it->second.bendPoints[bpIdx + 1].position = dragResult.adjustedNextPos;
                    }
                }
            }
        } else if (draggedNode_ != INVALID_NODE && ImGui::IsMouseDragging(0)) {
            // Update node position
            auto& layout = nodeLayouts_[draggedNode_];
            float newX = graphMouse.x - dragOffset_.x;
            float newY = graphMouse.y - dragOffset_.y;

            // Snap to grid if enabled
            float gridSize = layoutOptions_.gridConfig.cellSize;
            if (gridSize > 0.0f) {
                newX = std::round(newX / gridSize) * gridSize;
                newY = std::round(newY / gridSize) * gridSize;
            }

            // Full validation using pre-calculated forbidden zones
            // This ensures visualization and validation use the same zones
            Point proposedPosition = {newX, newY};
            auto validation = LayoutUtils::canMoveNodeTo(
                draggedNode_, proposedPosition, nodeLayouts_, forbiddenZones_);

            // Always update position during drag (visual feedback)
            layout.position.x = newX;
            layout.position.y = newY;

            if (validation.valid) {
                // Valid position - save as last valid and re-route edges
                isInvalidDragPosition_ = false;
                lastValidPosition_ = proposedPosition;
                manualManager_->setNodePosition(draggedNode_, layout.position);

                // Re-route connected edges only when position actually changed
                // This prevents flickering when mouse is stationary
                // Skip rerouting in HideUntilDrop mode - edges will be calculated on drop
                constexpr float EPSILON = 0.001f;
                if (layoutOptions_.optimizationOptions.dragAlgorithm != DragAlgorithm::HideUntilDrop &&
                    (std::abs(proposedPosition.x - lastRoutedPosition_.x) > EPSILON ||
                     std::abs(proposedPosition.y - lastRoutedPosition_.y) > EPSILON)) {
                    rerouteAffectedEdges();
                    lastRoutedPosition_ = proposedPosition;
                }
            } else {
                // Invalid position - show red, no edge re-routing
                isInvalidDragPosition_ = true;
            }
        }

        // Handle Delete key for bend point removal
        if (selectedBendPoint_.isValid() && ImGui::IsKeyPressed(ImGuiKey_Delete)) {
            manualManager_->removeBendPoint(
                selectedBendPoint_.edgeId,
                static_cast<size_t>(selectedBendPoint_.bendPointIndex)
            );
            selectedBendPoint_.clear();
            doLayout();
        }
    }

    void rerouteAffectedEdges() {
        // Re-route ALL edges with A* after drop
        // This ensures all transitions are recalculated
        std::vector<EdgeId> allEdges;
        allEdges.reserve(edgeLayouts_.size());
        for (const auto& [edgeId, layout] : edgeLayouts_) {
            allEdges.push_back(edgeId);
        }
        std::cout << "[rerouteAffectedEdges] Processing ALL " << allEdges.size() 
                  << " edges with A* (draggedNode=" << draggedNode_ << ")" << std::endl;
        std::unordered_set<NodeId> movedNodes = {draggedNode_};
        LayoutUtils::updateEdgePositions(
            edgeLayouts_, nodeLayouts_, allEdges,
            layoutOptions_, movedNodes);
    }

    void drawGrid(ImDrawList* drawList, float width, float height) {
        if (gridSize_ <= 0.0f) return;

        ImU32 gridColor = IM_COL32(200, 200, 200, 80);  // Light gray, semi-transparent
        ImU32 gridColorMajor = IM_COL32(180, 180, 180, 120);  // Slightly darker for major lines

        // Calculate grid bounds
        float startX = std::fmod(offset_.x, gridSize_);
        float startY = std::fmod(offset_.y, gridSize_);

        // Draw vertical lines
        int lineCount = 0;
        for (float x = startX; x < width; x += gridSize_) {
            ImU32 color = (lineCount % 5 == 0) ? gridColorMajor : gridColor;
            drawList->AddLine({x, 0}, {x, height}, color, 1.0f);
            lineCount++;
        }

        // Draw horizontal lines
        lineCount = 0;
        for (float y = startY; y < height; y += gridSize_) {
            ImU32 color = (lineCount % 5 == 0) ? gridColorMajor : gridColor;
            drawList->AddLine({0, y}, {width, y}, color, 1.0f);
            lineCount++;
        }
    }

    void drawBlockedCells(ImDrawList* drawList) {
        // Only show during drag, and skip if disabled or no pre-calculated zones
        if (!showBlockedCells_ || gridSize_ <= 0.0f || draggedNode_ == INVALID_NODE) return;

        // Verify dragged node exists
        if (nodeLayouts_.find(draggedNode_) == nodeLayouts_.end()) return;

        // Use pre-calculated forbidden zones (based on edge connectivity)
        for (const auto& zone : forbiddenZones_) {
            // Get the node that created this forbidden zone
            auto nodeIt = nodeLayouts_.find(zone.blockedBy);
            if (nodeIt == nodeLayouts_.end()) continue;
            const auto& blockerLayout = nodeIt->second;

            // Show zone.bounds directly (5-cell margin around blocker)
            // Note: Large dragged nodes may be blocked outside this area too,
            // but showing just the margin is more intuitive for users
            int leftCell = static_cast<int>(std::ceil(zone.bounds.x / gridSize_));
            int topCell = static_cast<int>(std::ceil(zone.bounds.y / gridSize_));
            int rightCell = static_cast<int>(std::ceil((zone.bounds.x + zone.bounds.width) / gridSize_));
            int bottomCell = static_cast<int>(std::ceil((zone.bounds.y + zone.bounds.height) / gridSize_));

            // Node's actual cell range (to exclude from red coloring)
            int nodeLeftCell = static_cast<int>(std::floor(blockerLayout.position.x / gridSize_));
            int nodeTopCell = static_cast<int>(std::floor(blockerLayout.position.y / gridSize_));
            int nodeRightCell = static_cast<int>(std::ceil((blockerLayout.position.x + blockerLayout.size.width) / gridSize_));
            int nodeBottomCell = static_cast<int>(std::ceil((blockerLayout.position.y + blockerLayout.size.height) / gridSize_));

            // Draw forbidden cells (excluding blocker node's own cells)
            for (int gx = leftCell; gx < rightCell; ++gx) {
                for (int gy = topCell; gy < bottomCell; ++gy) {
                    // Skip cells occupied by the blocker node itself
                    if (gx >= nodeLeftCell && gx < nodeRightCell &&
                        gy >= nodeTopCell && gy < nodeBottomCell) {
                        continue;
                    }

                    float x1 = gx * gridSize_ + offset_.x;
                    float y1 = gy * gridSize_ + offset_.y;
                    float x2 = x1 + gridSize_;
                    float y2 = y1 + gridSize_;

                    drawList->AddRectFilled({x1, y1}, {x2, y2}, COLOR_BLOCKED_CELL);
                }
            }
        }
    }

    void drawAStarDebug(ImDrawList* drawList) {
        if (!showAStarDebug_) return;

        // Rebuild obstacle map every frame to reflect current state
        debugObstacles_ = std::make_shared<ObstacleMap>();
        debugObstacles_->buildFromNodes(nodeLayouts_, gridSize_);
        debugObstacles_->addEdgeSegments(edgeLayouts_, debugEdgeId_);

        // Update start/goal from selected or debug edge
        EdgeId targetEdge = (debugEdgeId_ != INVALID_EDGE) ? debugEdgeId_ : selectedEdge_;
        if (targetEdge != INVALID_EDGE) {
            auto it = edgeLayouts_.find(targetEdge);
            if (it != edgeLayouts_.end()) {
                astarStart_ = it->second.sourcePoint;
                astarGoal_ = it->second.targetPoint;
            }
        }

        // Colors for visualization
        ImU32 nodeBlockColor = IM_COL32(255, 200, 0, 80);      // Yellow: node obstacles
        ImU32 hSegmentColor = IM_COL32(100, 100, 255, 100);    // Blue: horizontal edge segments
        ImU32 vSegmentColor = IM_COL32(100, 255, 100, 100);    // Green: vertical edge segments
        ImU32 startColor = IM_COL32(0, 255, 0, 200);           // Bright green: start point
        ImU32 goalColor = IM_COL32(255, 0, 0, 200);            // Red: goal point

        float cellSize = debugObstacles_->gridSize();
        int obsOffsetX = debugObstacles_->offsetX();
        int obsOffsetY = debugObstacles_->offsetY();

        // Draw all grid cells with obstacles
        for (int gy = 0; gy < debugObstacles_->height(); ++gy) {
            for (int gx = 0; gx < debugObstacles_->width(); ++gx) {
                int gridX = gx + obsOffsetX;
                int gridY = gy + obsOffsetY;
                auto info = debugObstacles_->getCellVisInfo(gridX, gridY);

                float x1 = gridX * cellSize + offset_.x;
                float y1 = gridY * cellSize + offset_.y;
                float x2 = x1 + cellSize;
                float y2 = y1 + cellSize;

                // Node obstacles (yellow fill)
                if (info.isNodeBlocked) {
                    drawList->AddRectFilled({x1, y1}, {x2, y2}, nodeBlockColor);
                }
                // Horizontal edge segments (blue horizontal bar)
                if (info.hasHorizontalSegment) {
                    drawList->AddRectFilled({x1, y1 + cellSize*0.35f},
                                           {x2, y2 - cellSize*0.35f}, hSegmentColor);
                }
                // Vertical edge segments (green vertical bar)
                if (info.hasVerticalSegment) {
                    drawList->AddRectFilled({x1 + cellSize*0.35f, y1},
                                           {x2 - cellSize*0.35f, y2}, vSegmentColor);
                }
            }
        }

        // Highlight start/goal points
        if (astarStart_.x >= 0) {
            float x = astarStart_.x + offset_.x;
            float y = astarStart_.y + offset_.y;
            drawList->AddCircleFilled({x, y}, 8, startColor);
            drawList->AddText({x + 10, y - 5}, IM_COL32_WHITE, "START");
        }
        if (astarGoal_.x >= 0) {
            float x = astarGoal_.x + offset_.x;
            float y = astarGoal_.y + offset_.y;
            drawList->AddCircleFilled({x, y}, 8, goalColor);
            drawList->AddText({x + 10, y - 5}, IM_COL32_WHITE, "GOAL");
        }
    }

    void render(ImDrawList* drawList) {
        // Draw grid background first
        ImGuiIO& io = ImGui::GetIO();
        drawGrid(drawList, io.DisplaySize.x, io.DisplaySize.y);

        // Draw blocked cells (node obstacle areas)
        drawBlockedCells(drawList);

        // Draw A* debug visualization
        drawAStarDebug(drawList);

        // Draw edges first
        for (const auto& [id, layout] : edgeLayouts_) {
            bool isAffected = std::find(affectedEdges_.begin(), affectedEdges_.end(), id)
                             != affectedEdges_.end();

            // Hide edges connected to dragged node when in invalid position
            if (isAffected && isInvalidDragPosition_) {
                continue;
            }

            // Hide edges during drag and until A* completes when HideUntilDrop mode is active
            // affectedEdges_ is kept until onOptimizationComplete() clears it
            if (isAffected &&
                layoutOptions_.optimizationOptions.dragAlgorithm == DragAlgorithm::HideUntilDrop) {
                continue;
            }

            // Hide the edge being snap-point-dragged (preview will be shown instead)
            if (draggingSnapPoint_.isValid() && draggingSnapPoint_.edgeId == id && hasSnapPreview_) {
                continue;
            }

            bool isSelected = (id == selectedEdge_);
            bool isHovered = (id == hoveredEdge_);

            ImU32 color = COLOR_EDGE;
            float thickness = 2.0f;
            if (isSelected) {
                color = COLOR_EDGE_SELECTED;
                thickness = 3.0f;
            } else if (isHovered) {
                color = COLOR_NODE_HOVER;
                thickness = 2.5f;
            } else if (isAffected) {
                color = COLOR_EDGE_AFFECTED;
            }

            auto points = layout.allPoints();
            for (size_t i = 1; i < points.size(); ++i) {
                ImVec2 p1 = {points[i-1].x + offset_.x, points[i-1].y + offset_.y};
                ImVec2 p2 = {points[i].x + offset_.x, points[i].y + offset_.y};
                drawList->AddLine(p1, p2, color, thickness);
            }

            // Draw arrowhead
            if (points.size() >= 2) {
                Point last = points.back();
                Point prev = points[points.size() - 2];
                drawArrowhead(drawList, prev, last, color);
            }

            // Draw edge label (using pre-computed labelPosition)
            const EdgeData& edge = graph_.getEdge(id);
            if (!edge.label.empty()) {
                ImVec2 textPos = {layout.labelPosition.x + offset_.x - 20,
                                  layout.labelPosition.y + offset_.y - 15};
                drawList->AddText(textPos, COLOR_TEXT, edge.label.c_str());
            }
        }

        // Draw bend point insertion preview
        if (bendPointPreview_.active) {
            ImVec2 screenPos = {bendPointPreview_.position.x + offset_.x,
                               bendPointPreview_.position.y + offset_.y};
            float size = 5.0f;
            ImVec2 pts[4] = {
                {screenPos.x, screenPos.y - size},
                {screenPos.x + size, screenPos.y},
                {screenPos.x, screenPos.y + size},
                {screenPos.x - size, screenPos.y}
            };
            drawList->AddConvexPolyFilled(pts, 4, COLOR_BEND_POINT_PREVIEW);
            drawList->AddPolyline(pts, 4, IM_COL32(100, 200, 255, 200), ImDrawFlags_Closed, 1.0f);

            // Draw "+" indicator
            float plusSize = 3.0f;
            drawList->AddLine(
                {screenPos.x - plusSize, screenPos.y},
                {screenPos.x + plusSize, screenPos.y},
                IM_COL32(255, 255, 255, 200), 2.0f);
            drawList->AddLine(
                {screenPos.x, screenPos.y - plusSize},
                {screenPos.x, screenPos.y + plusSize},
                IM_COL32(255, 255, 255, 200), 2.0f);
        }

        // Draw nodes
        for (const auto& [id, layout] : nodeLayouts_) {
            ImU32 fillColor = COLOR_NODE;
            if (id == draggedNode_) {
                if (isInvalidDragPosition_) {
                    fillColor = IM_COL32(255, 80, 80, 255);  // Red for invalid position
                } else {
                    fillColor = COLOR_NODE_DRAG;
                }
            } else if (id == selectedNode_) {
                fillColor = COLOR_NODE_SELECTED;
            } else if (id == hoveredNode_) {
                fillColor = COLOR_NODE_HOVER;
            }

            ImVec2 p1 = {layout.position.x + offset_.x, layout.position.y + offset_.y};
            ImVec2 p2 = {p1.x + layout.size.width, p1.y + layout.size.height};

            drawList->AddRectFilled(p1, p2, fillColor, 5.0f);
            drawList->AddRect(p1, p2, COLOR_NODE_BORDER, 5.0f, 0, 2.0f);

            // Draw label
            const NodeData& node = graph_.getNode(id);
            if (!node.label.empty()) {
                ImVec2 textSize = ImGui::CalcTextSize(node.label.c_str());
                ImVec2 textPos = {p1.x + (layout.size.width - textSize.x) / 2,
                                 p1.y + (layout.size.height - textSize.y) / 2};
                drawList->AddText(textPos, COLOR_TEXT, node.label.c_str());
            }

            // Draw snap points if show enabled
            if (showSnapPoints_) {
                drawSnapPoints(drawList, layout);
            }
        }

        // Draw snap point candidates during drag
        if (draggingSnapPoint_.isValid() && snapController_.isDragging()) {
            drawSnapCandidates(drawList);
        }

        // Draw A* path preview during snap point drag
        if (hasSnapPreview_ && draggingSnapPoint_.isValid()) {
            drawSnapPreviewPath(drawList);
        }
    }

    void drawSnapCandidates(ImDrawList* drawList) {
        const auto& candidates = snapController_.getCandidates();
        for (size_t i = 0; i < candidates.size(); ++i) {
            const auto& candidate = candidates[i];
            ImVec2 screenPos = {candidate.position.x + offset_.x,
                                candidate.position.y + offset_.y};

            bool isSnapped = (static_cast<int>(i) == snappedCandidateIndex_);

            if (isSnapped) {
                // Highlight the snapped candidate
                drawList->AddCircleFilled(screenPos, 8.0f, IM_COL32(255, 100, 100, 255));
                drawList->AddCircle(screenPos, 8.0f, IM_COL32(255, 255, 255, 255), 0, 2.0f);
            } else {
                // Draw other candidates as smaller circles
                drawList->AddCircleFilled(screenPos, 4.0f, IM_COL32(150, 150, 255, 150));
                drawList->AddCircle(screenPos, 4.0f, IM_COL32(100, 100, 200, 200), 0, 1.0f);
            }
        }
    }

    void drawSnapPreviewPath(ImDrawList* drawList) {
        // Draw preview path in a distinct color (cyan, semi-transparent)
        ImU32 previewColor = IM_COL32(0, 200, 255, 180);
        float thickness = 3.0f;

        auto points = snapController_.getPreviewLayout().allPoints();
        for (size_t i = 1; i < points.size(); ++i) {
            ImVec2 p1 = {points[i-1].x + offset_.x, points[i-1].y + offset_.y};
            ImVec2 p2 = {points[i].x + offset_.x, points[i].y + offset_.y};
            drawList->AddLine(p1, p2, previewColor, thickness);
        }

        // Draw arrow at the target
        if (points.size() >= 2) {
            ImVec2 p1 = {points[points.size()-2].x + offset_.x, points[points.size()-2].y + offset_.y};
            ImVec2 p2 = {points.back().x + offset_.x, points.back().y + offset_.y};

            float dx = p2.x - p1.x;
            float dy = p2.y - p1.y;
            float len = std::sqrt(dx*dx + dy*dy);
            if (len > 0.01f) {
                dx /= len;
                dy /= len;
                float arrowSize = 8.0f;
                ImVec2 arrow1 = {p2.x - arrowSize * (dx + dy * 0.5f),
                                 p2.y - arrowSize * (dy - dx * 0.5f)};
                ImVec2 arrow2 = {p2.x - arrowSize * (dx - dy * 0.5f),
                                 p2.y - arrowSize * (dy + dx * 0.5f)};
                drawList->AddTriangleFilled(p2, arrow1, arrow2, previewColor);
            }
        }
    }

    void drawSnapPoints(ImDrawList* drawList, const NodeLayout& nodeLayout) {
        // Draw actual edge connection points with snap indices (for both modes)
        // Use foreground draw list for labels to ensure they're on top
        ImDrawList* fgDrawList = ImGui::GetForegroundDrawList();

        for (const auto& [edgeId, edgeLayout] : edgeLayouts_) {
            // Skip snap points for affected edges when in invalid drag position
            bool isAffectedEdge = std::find(affectedEdges_.begin(), affectedEdges_.end(), edgeId)
                                  != affectedEdges_.end();
            if (isAffectedEdge && isInvalidDragPosition_) {
                continue;
            }

            // Skip snap points during drag and until A* completes when HideUntilDrop mode is active
            if (isAffectedEdge &&
                layoutOptions_.optimizationOptions.dragAlgorithm == DragAlgorithm::HideUntilDrop) {
                continue;
            }

            // Source point on this node (outgoing - green)
            if (edgeLayout.from == nodeLayout.id) {
                ImVec2 screenPos = {edgeLayout.sourcePoint.x + offset_.x,
                                    edgeLayout.sourcePoint.y + offset_.y};

                // Determine colors based on hover/drag state
                bool isSourceHovered = (hoveredSnapPoint_.edgeId == edgeId && hoveredSnapPoint_.isSource);
                bool isSourceDragging = (draggingSnapPoint_.edgeId == edgeId && draggingSnapPoint_.isSource);
                float radius = 5.0f;
                ImU32 fillColor = IM_COL32(100, 200, 100, 255);
                ImU32 borderColor = IM_COL32(50, 150, 50, 255);

                if (isSourceDragging) {
                    radius = 8.0f;
                    fillColor = IM_COL32(255, 180, 80, 255);  // Orange for dragging
                    borderColor = IM_COL32(200, 130, 50, 255);
                } else if (isSourceHovered) {
                    radius = 7.0f;
                    fillColor = IM_COL32(255, 220, 100, 255);  // Yellow for hover
                    borderColor = IM_COL32(200, 170, 50, 255);
                }

                drawList->AddCircleFilled(screenPos, radius, fillColor);
                drawList->AddCircle(screenPos, radius, borderColor, 0, 1.5f);

                // Draw snap index label for source
                if (showSnapIndices_) {
                    char label[32];
                    const char* edgeName = "";
                    switch (edgeLayout.sourceEdge) {
                        case NodeEdge::Top: edgeName = "T"; break;
                        case NodeEdge::Bottom: edgeName = "B"; break;
                        case NodeEdge::Left: edgeName = "L"; break;
                        case NodeEdge::Right: edgeName = "R"; break;
                    }
                    snprintf(label, sizeof(label), "%s%d", edgeName, edgeLayout.sourceSnapIndex);

                    // Position label: centered on snap point, inside node (away from arrow)
                    ImVec2 textSize = ImGui::CalcTextSize(label);
                    ImVec2 textPos = screenPos;

                    // Center text horizontally/vertically and move inside node
                    switch (edgeLayout.sourceEdge) {
                        case NodeEdge::Top:
                            textPos.x -= textSize.x / 2.0f;
                            textPos.y += 8;  // Move down into node
                            break;
                        case NodeEdge::Bottom:
                            textPos.x -= textSize.x / 2.0f;
                            textPos.y -= textSize.y + 8;  // Move up into node
                            break;
                        case NodeEdge::Left:
                            textPos.x += 8;  // Move right into node
                            textPos.y -= textSize.y / 2.0f;
                            break;
                        case NodeEdge::Right:
                            textPos.x -= textSize.x + 8;  // Move left into node
                            textPos.y -= textSize.y / 2.0f;
                            break;
                    }

                    // Draw bold text with white background
                    ImVec2 bgMin = {textPos.x - 3, textPos.y - 2};
                    ImVec2 bgMax = {textPos.x + textSize.x + 3, textPos.y + textSize.y + 2};
                    fgDrawList->AddRectFilled(bgMin, bgMax, IM_COL32(255, 255, 255, 240), 3.0f);
                    fgDrawList->AddRect(bgMin, bgMax, IM_COL32(40, 140, 40, 255), 3.0f, 0, 2.0f);
                    // Bold effect: draw text twice with offset
                    fgDrawList->AddText({textPos.x + 1, textPos.y}, IM_COL32(20, 100, 20, 255), label);
                    fgDrawList->AddText(textPos, IM_COL32(20, 100, 20, 255), label);
                }
            }
            // Target point on this node (incoming - red)
            if (edgeLayout.to == nodeLayout.id) {
                ImVec2 screenPos = {edgeLayout.targetPoint.x + offset_.x,
                                    edgeLayout.targetPoint.y + offset_.y};

                // Determine colors based on hover/drag state
                bool isTargetHovered = (hoveredSnapPoint_.edgeId == edgeId && !hoveredSnapPoint_.isSource);
                bool isTargetDragging = (draggingSnapPoint_.edgeId == edgeId && !draggingSnapPoint_.isSource);
                float radius = 5.0f;
                ImU32 fillColor = IM_COL32(200, 100, 100, 255);
                ImU32 borderColor = IM_COL32(150, 50, 50, 255);

                if (isTargetDragging) {
                    radius = 8.0f;
                    fillColor = IM_COL32(255, 180, 80, 255);  // Orange for dragging
                    borderColor = IM_COL32(200, 130, 50, 255);
                } else if (isTargetHovered) {
                    radius = 7.0f;
                    fillColor = IM_COL32(255, 220, 100, 255);  // Yellow for hover
                    borderColor = IM_COL32(200, 170, 50, 255);
                }

                drawList->AddCircleFilled(screenPos, radius, fillColor);
                drawList->AddCircle(screenPos, radius, borderColor, 0, 1.5f);

                // Draw snap index label for target
                if (showSnapIndices_) {
                    char label[32];
                    const char* edgeName = "";
                    switch (edgeLayout.targetEdge) {
                        case NodeEdge::Top: edgeName = "T"; break;
                        case NodeEdge::Bottom: edgeName = "B"; break;
                        case NodeEdge::Left: edgeName = "L"; break;
                        case NodeEdge::Right: edgeName = "R"; break;
                    }
                    snprintf(label, sizeof(label), "%s%d", edgeName, edgeLayout.targetSnapIndex);

                    // Position label: centered on snap point, inside node (away from arrow)
                    ImVec2 textSize = ImGui::CalcTextSize(label);
                    ImVec2 textPos = screenPos;

                    // Center text horizontally/vertically and move inside node
                    switch (edgeLayout.targetEdge) {
                        case NodeEdge::Top:
                            textPos.x -= textSize.x / 2.0f;
                            textPos.y += 8;  // Move down into node
                            break;
                        case NodeEdge::Bottom:
                            textPos.x -= textSize.x / 2.0f;
                            textPos.y -= textSize.y + 8;  // Move up into node
                            break;
                        case NodeEdge::Left:
                            textPos.x += 8;  // Move right into node
                            textPos.y -= textSize.y / 2.0f;
                            break;
                        case NodeEdge::Right:
                            textPos.x -= textSize.x + 8;  // Move left into node
                            textPos.y -= textSize.y / 2.0f;
                            break;
                    }

                    // Draw bold text with white background
                    ImVec2 bgMin = {textPos.x - 3, textPos.y - 2};
                    ImVec2 bgMax = {textPos.x + textSize.x + 3, textPos.y + textSize.y + 2};
                    fgDrawList->AddRectFilled(bgMin, bgMax, IM_COL32(255, 255, 255, 240), 3.0f);
                    fgDrawList->AddRect(bgMin, bgMax, IM_COL32(180, 40, 40, 255), 3.0f, 0, 2.0f);
                    // Bold effect: draw text twice with offset
                    fgDrawList->AddText({textPos.x + 1, textPos.y}, IM_COL32(160, 20, 20, 255), label);
                    fgDrawList->AddText(textPos, IM_COL32(160, 20, 20, 255), label);
                }
            }
        }
    }

    void drawArrowhead(ImDrawList* drawList, const Point& from, const Point& to, ImU32 color) {
        float dx = to.x - from.x;
        float dy = to.y - from.y;
        float len = std::sqrt(dx * dx + dy * dy);
        if (len < 0.001f) return;

        dx /= len;
        dy /= len;

        float arrowSize = 10.0f;
        ImVec2 tip = {to.x + offset_.x, to.y + offset_.y};
        ImVec2 left = {tip.x - arrowSize * dx + arrowSize * 0.5f * dy,
                       tip.y - arrowSize * dy - arrowSize * 0.5f * dx};
        ImVec2 right = {tip.x - arrowSize * dx - arrowSize * 0.5f * dy,
                        tip.y - arrowSize * dy + arrowSize * 0.5f * dx};

        drawList->AddTriangleFilled(tip, left, right, color);
    }

    void renderUI() {
        ImGui::SetNextWindowPos(ImVec2(10, 10), ImGuiCond_FirstUseEver);
        ImGui::SetNextWindowSize(ImVec2(300, 500), ImGuiCond_FirstUseEver);

        ImGui::Begin("ArborVia Demo");

        // Edge Routing Options
        if (ImGui::TreeNode("Edge Routing Options")) {
                bool changed = false;

                if (ImGui::SliderFloat("Spacing", &layoutOptions_.channelRouting.channelSpacing, 5.0f, 30.0f)) {
                    changed = true;
                }
                if (ImGui::IsItemHovered()) {
                    ImGui::SetTooltip("Spacing between parallel edge channels");
                }

                if (ImGui::SliderFloat("Offset", &layoutOptions_.channelRouting.channelOffset, 10.0f, 50.0f)) {
                    changed = true;
                }
                if (ImGui::IsItemHovered()) {
                    ImGui::SetTooltip("Minimum offset from layer boundary");
                }

                ImGui::Checkbox("Center Single Edge", &layoutOptions_.channelRouting.centerSingleEdge);

                // Self-loop direction
                ImGui::Text("Self-Loop Direction:");
                const char* loopDirs[] = {"Right", "Left", "Top", "Bottom", "Auto"};
                int currentDir = static_cast<int>(layoutOptions_.channelRouting.selfLoop.preferredDirection);
                if (ImGui::Combo("##LoopDir", &currentDir, loopDirs, 5)) {
                    layoutOptions_.channelRouting.selfLoop.preferredDirection =
                        static_cast<SelfLoopDirection>(currentDir);
                    changed = true;
                }

                if (ImGui::SliderFloat("Loop Offset", &layoutOptions_.channelRouting.selfLoop.loopOffset, 10.0f, 50.0f)) {
                    changed = true;
                }

                if (changed) {
                    reRouteEdgesOnly();
                }
                ImGui::TreePop();
            }

        ImGui::Separator();
        ImGui::Text("Drag nodes to see edge re-routing");
        ImGui::Separator();
        ImGui::Text("Nodes: %zu", graph_.nodeCount());
        ImGui::Text("Edges: %zu", graph_.edgeCount());

        if (draggedNode_ != INVALID_NODE) {
            if (auto node = graph_.tryGetNode(draggedNode_)) {
                ImGui::TextColored(ImVec4(1, 0.5f, 0.5f, 1),
                    "Dragging: %s", node->label.c_str());
            }
            ImGui::Text("Affected edges: %zu", affectedEdges_.size());
        }

        ImGui::Separator();
        ImGui::Checkbox("Show Snap Points", &showSnapPoints_);
        if (showSnapPoints_) {
            ImGui::SameLine();
            ImGui::Checkbox("Show Indices", &showSnapIndices_);
            if (ImGui::IsItemHovered()) {
                ImGui::SetTooltip("T=Top, B=Bottom, L=Left, R=Right\nGreen=Outgoing, Red=Incoming");
            }
        }
        ImGui::Checkbox("Show Blocked Cells", &showBlockedCells_);
        if (ImGui::IsItemHovered()) {
            ImGui::SetTooltip("Show no-drag zones while dragging (5 grid units margin)");
        }

        // A* debug visualization
        if (ImGui::Checkbox("Show A* Grid", &showAStarDebug_)) {
            if (showAStarDebug_) {
                // Build obstacle map for visualization
                debugObstacles_ = std::make_shared<ObstacleMap>();
                debugObstacles_->buildFromNodes(nodeLayouts_, gridSize_);
                // Add edge segments, excluding selected edge if any
                debugObstacles_->addEdgeSegments(edgeLayouts_, selectedEdge_);

                // Set start/goal from selected edge
                if (selectedEdge_ != INVALID_EDGE) {
                    auto it = edgeLayouts_.find(selectedEdge_);
                    if (it != edgeLayouts_.end()) {
                        astarStart_ = it->second.sourcePoint;
                        astarGoal_ = it->second.targetPoint;
                    }
                } else {
                    astarStart_ = {-1, -1};
                    astarGoal_ = {-1, -1};
                }
            }
        }
        if (ImGui::IsItemHovered()) {
            ImGui::SetTooltip("Show A* pathfinding grid\nYellow=Node, Blue=H-Edge, Green=V-Edge");
        }

        // Snap point configuration for selected node (manual mode feature - disabled)
        if (false && selectedNode_ != INVALID_NODE) {
            auto selectedNode = graph_.tryGetNode(selectedNode_);
            if (!selectedNode) {
                selectedNode_ = INVALID_NODE;
                ImGui::End();
                return;
            }

            ImGui::Separator();
            ImGui::Text("Snap Points: %s", selectedNode->label.c_str());

            SnapPointConfig config = manualManager_->getSnapConfig(selectedNode_);
            bool changed = false;

            if (ImGui::SliderInt("Top", &config.topCount, 1, 5)) changed = true;
            if (ImGui::SliderInt("Bottom", &config.bottomCount, 1, 5)) changed = true;
            if (ImGui::SliderInt("Left", &config.leftCount, 1, 5)) changed = true;
            if (ImGui::SliderInt("Right", &config.rightCount, 1, 5)) changed = true;

            if (changed) {
                manualManager_->setSnapConfig(selectedNode_, config);
                doLayout();
            }
        }

        // Edge routing configuration for selected edge (manual mode feature - disabled)
        if (false && selectedEdge_ != INVALID_EDGE) {
            auto edgeOpt = graph_.tryGetEdge(selectedEdge_);
            if (!edgeOpt) {
                selectedEdge_ = INVALID_EDGE;
                ImGui::End();
                return;
            }
            const EdgeData& edgeData = *edgeOpt;

            auto fromNode = graph_.tryGetNode(edgeData.from);
            auto toNode = graph_.tryGetNode(edgeData.to);
            if (!fromNode || !toNode) {
                selectedEdge_ = INVALID_EDGE;
                ImGui::End();
                return;
            }

            ImGui::Separator();
            ImGui::Text("Edge: %s -> %s",
                fromNode->label.c_str(),
                toNode->label.c_str());
            if (!edgeData.label.empty()) {
                ImGui::Text("Label: %s", edgeData.label.c_str());
            }

            EdgeRoutingConfig routing = manualManager_->getEdgeRouting(selectedEdge_);
            bool routingChanged = false;

            // Source edge selection
            ImGui::Text("Source Edge:");
            const char* edgeNames[] = {"Top", "Bottom", "Left", "Right"};
            int srcEdge = static_cast<int>(routing.sourceEdge);
            if (ImGui::Combo("##SrcEdge", &srcEdge, edgeNames, 4)) {
                routing.sourceEdge = static_cast<NodeEdge>(srcEdge);
                routing.sourceSnapIndex = 0;  // Reset index when edge changes
                routingChanged = true;
            }

            // Source snap point index
            SnapPointConfig srcConfig = manualManager_->getSnapConfig(edgeData.from);
            int srcMaxSnap = srcConfig.getCount(routing.sourceEdge) - 1;
            if (srcMaxSnap > 0) {
                if (ImGui::SliderInt("Src Snap", &routing.sourceSnapIndex, 0, srcMaxSnap)) {
                    routingChanged = true;
                }
            }

            // Target edge selection
            ImGui::Text("Target Edge:");
            int tgtEdge = static_cast<int>(routing.targetEdge);
            if (ImGui::Combo("##TgtEdge", &tgtEdge, edgeNames, 4)) {
                routing.targetEdge = static_cast<NodeEdge>(tgtEdge);
                routing.targetSnapIndex = 0;  // Reset index when edge changes
                routingChanged = true;
            }

            // Target snap point index
            SnapPointConfig tgtConfig = manualManager_->getSnapConfig(edgeData.to);
            int tgtMaxSnap = tgtConfig.getCount(routing.targetEdge) - 1;
            if (tgtMaxSnap > 0) {
                if (ImGui::SliderInt("Tgt Snap", &routing.targetSnapIndex, 0, tgtMaxSnap)) {
                    routingChanged = true;
                }
            }

            if (routingChanged) {
                manualManager_->setEdgeRouting(selectedEdge_, routing);
                doLayout();
            }

            // Bend point management UI
            ImGui::Separator();
            ImGui::Text("Bend Points:");

            const auto& bps = manualManager_->getBendPoints(selectedEdge_);
            if (bps.empty()) {
                ImGui::TextDisabled("None (auto routing)");
                ImGui::TextDisabled("Click on edge to add");
            } else {
                ImGui::Text("Count: %zu", bps.size());
                for (size_t i = 0; i < bps.size(); ++i) {
                    ImGui::PushID(static_cast<int>(i));
                    bool isSelected = (selectedBendPoint_.edgeId == selectedEdge_ &&
                                      selectedBendPoint_.bendPointIndex == static_cast<int>(i));
                    if (isSelected) {
                        ImGui::TextColored(ImVec4(0.4f, 1.0f, 0.4f, 1.0f),
                            "[%zu] (%.0f, %.0f)", i, bps[i].position.x, bps[i].position.y);
                    } else {
                        ImGui::Text("[%zu] (%.0f, %.0f)", i, bps[i].position.x, bps[i].position.y);
                    }
                    ImGui::PopID();
                }
                if (ImGui::Button("Clear All Bends")) {
                    manualManager_->clearBendPoints(selectedEdge_);
                    selectedBendPoint_.clear();
                    doLayout();
                }
            }

            if (selectedBendPoint_.isValid() && selectedBendPoint_.edgeId == selectedEdge_) {
                ImGui::Text("Selected: [%d]", selectedBendPoint_.bendPointIndex);
                ImGui::TextDisabled("Press Delete to remove");
            }
        }

        ImGui::Separator();
        if (ImGui::Button("Reset Layout")) {
            manualManager_->clearManualState();
            doLayout();
        }

        ImGui::SameLine();
        if (ImGui::Button("Save")) {
            saveLayout("layout.json");
        }

        ImGui::SameLine();
        if (ImGui::Button("Load")) {
            loadLayout("layout.json");
        }

        ImGui::End();
    }

private:
    Graph graph_;
    std::unordered_map<NodeId, NodeLayout> nodeLayouts_;
    std::unordered_map<EdgeId, EdgeLayout> edgeLayouts_;
    LayoutResult layoutResult_;
    std::shared_ptr<ManualLayoutManager> manualManager_;
    std::shared_ptr<PathRoutingCoordinator> routingCoordinator_;
    std::unique_ptr<LayoutController> layoutController_;  // Centralized constraint enforcement
    LayoutOptions layoutOptions_;

    Point offset_ = {0, 0};
    float gridSize_ = 20.0f;  // Grid cell size (0 = disabled)

    NodeId hoveredNode_ = INVALID_NODE;
    NodeId draggedNode_ = INVALID_NODE;
    NodeId selectedNode_ = INVALID_NODE;
    EdgeId hoveredEdge_ = INVALID_EDGE;
    EdgeId selectedEdge_ = INVALID_EDGE;
    Point dragOffset_ = {0, 0};
    std::vector<EdgeId> affectedEdges_;
    bool showSnapPoints_ = true;
    bool showSnapIndices_ = true;
    bool showBlockedCells_ = true;  // Show node obstacle areas in red

    // Drag constraint state
    bool isInvalidDragPosition_ = false;      // Current drag position is invalid
    Point lastValidPosition_ = {0, 0};        // Last known valid position during drag
    Point lastRoutedPosition_ = {-9999, -9999};  // Last position where edges were routed
    std::vector<ForbiddenZone> forbiddenZones_;  // Pre-calculated forbidden zones for drag

    // Bend point interaction state
    HoveredBendPoint hoveredBendPoint_;
    HoveredBendPoint selectedBendPoint_;
    HoveredBendPoint draggingBendPoint_;
    Point bendPointDragOffset_ = {0, 0};
    BendPointPreview bendPointPreview_;

    // Snap point interaction state
    HoveredSnapPoint hoveredSnapPoint_;
    HoveredSnapPoint draggingSnapPoint_;
    Point snapPointDragOffset_ = {0, 0};
    Point snapPointDragStart_ = {0, 0};  // Original snap point position at drag start

    // Snap point controller (handles drag preview, swap, etc.)
    SnapPointController snapController_;
    int snappedCandidateIndex_ = -1;                  // Currently snapped candidate (cached for rendering)
    bool hasSnapPreview_ = false;                      // Whether preview is valid

    // A* debug visualization state
    bool showAStarDebug_ = false;               // 'A' key toggles
    EdgeId debugEdgeId_ = INVALID_EDGE;         // Edge to debug (-1 = selected edge)
    std::shared_ptr<ObstacleMap> debugObstacles_;  // Current obstacle map for visualization
    Point astarStart_ = {-1, -1};               // A* start point
    Point astarGoal_ = {-1, -1};                // A* goal point

    // External command server
    CommandServer commandServer_;
    int port_;
    bool paused_ = false;
    std::string pauseMessage_;
    bool shouldQuit_ = false;

public:
    // Start command server
    bool startCommandServer() {
        return commandServer_.start();
    }

    // Process pending commands (call from main loop)
    void processCommands() {
        while (commandServer_.hasCommand()) {
            Command cmd = commandServer_.popCommand();
            processCommand(cmd);
        }
    }

    bool isPaused() const { return paused_; }
    const std::string& pauseMessage() const { return pauseMessage_; }
    bool shouldQuit() const { return shouldQuit_; }
    int port() const { return port_; }

private:
    void processCommand(const Command& cmd) {
        if (cmd.name == "drag" && cmd.args.size() >= 3) {
            // drag <node_id> <dx> <dy> - uses moveNode() API with validation
            NodeId nodeId = std::stoi(cmd.args[0]);
            float dx = std::stof(cmd.args[1]);
            float dy = std::stof(cmd.args[2]);

            if (nodeLayouts_.count(nodeId)) {
                Point proposedPos = {
                    nodeLayouts_[nodeId].position.x + dx,
                    nodeLayouts_[nodeId].position.y + dy
                };

                auto result = LayoutUtils::moveNode(
                    nodeId, proposedPos, nodeLayouts_, edgeLayouts_,
                    graph_, layoutOptions_);

                if (result.success) {
                    // Collect ALL edges for A* optimization (same as GUI drag)
                    std::vector<EdgeId> allEdges;
                    allEdges.reserve(edgeLayouts_.size());
                    for (const auto& [edgeId, layout] : edgeLayouts_) {
                        allEdges.push_back(edgeId);
                    }
                    // Trigger A* optimization callback (same as GUI drag end)
                    routingCoordinator_->onDragStart(allEdges);
                    routingCoordinator_->onDragEnd({nodeId});
                    commandServer_.sendResponse("OK drag " + std::to_string(nodeId));
                } else {
                    commandServer_.sendResponse("BLOCKED " + result.reason);
                }
            } else {
                commandServer_.sendResponse("ERROR node not found");
            }
        }
        else if (cmd.name == "set_pos" && cmd.args.size() >= 3) {
            // set_pos <node_id> <x> <y> - simulates full drag-drop cycle with A* optimization
            NodeId nodeId = std::stoi(cmd.args[0]);
            float x = std::stof(cmd.args[1]);
            float y = std::stof(cmd.args[2]);

            if (nodeLayouts_.count(nodeId)) {
                Point proposedPos = {x, y};

                // Collect affected edges before move
                std::vector<EdgeId> affected;
                for (const auto& [edgeId, layout] : edgeLayouts_) {
                    if (layout.from == nodeId || layout.to == nodeId) {
                        affected.push_back(edgeId);
                    }
                }

                // Simulate drag start
                routingCoordinator_->onDragStart(affected);

                auto result = LayoutUtils::moveNode(
                    nodeId, proposedPos, nodeLayouts_, edgeLayouts_,
                    graph_, layoutOptions_);

                if (result.success) {
                    // Update affectedEdges_ for coordinator callback
                    affectedEdges_ = affected;
                    
                    // Trigger drag end - this schedules A* optimization
                    routingCoordinator_->onDragEnd({nodeId});
                    
                    commandServer_.sendResponse("OK set_pos " + std::to_string(nodeId));
                } else {
                    routingCoordinator_->cancelPendingOptimization();
                    commandServer_.sendResponse("BLOCKED " + result.reason);
                }
            } else {
                commandServer_.sendResponse("ERROR node not found");
            }
        }
        else if (cmd.name == "test_snap_drag" && cmd.args.size() >= 4) {
            // test_snap_drag <edge_id> <is_source:0|1> <dx> <dy> - Snap point drag simulation
            EdgeId edgeId = std::stoi(cmd.args[0]);
            bool isSource = std::stoi(cmd.args[1]) != 0;
            float dx = std::stof(cmd.args[2]);
            float dy = std::stof(cmd.args[3]);

            if (edgeLayouts_.count(edgeId)) {
                std::cout << "\n========== TEST_SNAP_DRAG START ==========" << std::endl;
                std::cout << "[test_snap_drag] Edge " << edgeId << " isSource=" << isSource
                          << " delta=(" << dx << "," << dy << ")" << std::endl;

                EdgeLayout& layout = edgeLayouts_[edgeId];
                Point currentPos = isSource ? layout.sourcePoint : layout.targetPoint;
                int currentSnapIdx = isSource ? layout.sourceSnapIndex : layout.targetSnapIndex;
                NodeEdge currentEdge = isSource ? layout.sourceEdge : layout.targetEdge;

                std::cout << "[test_snap_drag] BEFORE:" << std::endl;
                std::cout << "  snapPoint=(" << currentPos.x << "," << currentPos.y << ")"
                          << " snapIndex=" << currentSnapIdx
                          << " nodeEdge=" << static_cast<int>(currentEdge) << std::endl;

                // Log all edge layouts BEFORE
                std::cout << "[test_snap_drag] All edge layouts BEFORE:" << std::endl;
                for (const auto& [eid, elayout] : edgeLayouts_) {
                    std::cout << "  Edge " << eid << ": snapIdx=(" << elayout.sourceSnapIndex
                              << "," << elayout.targetSnapIndex << ") src=(" << elayout.sourcePoint.x
                              << "," << elayout.sourcePoint.y << ") tgt=(" << elayout.targetPoint.x
                              << "," << elayout.targetPoint.y << ") bends=" << elayout.bendPoints.size() << std::endl;
                }

                Point newPos = {currentPos.x + dx, currentPos.y + dy};
                std::cout << "[test_snap_drag] newPos=(" << newPos.x << "," << newPos.y << ")" << std::endl;

                std::cout << "[test_snap_drag] Calling LayoutUtils::moveSnapPoint()..." << std::endl;
                auto result = LayoutUtils::moveSnapPoint(
                    edgeId, isSource, newPos,
                    nodeLayouts_, edgeLayouts_, graph_, layoutOptions_);

                std::cout << "[test_snap_drag] Result: success=" << result.success
                          << " reason=" << result.reason << std::endl;

                if (result.success) {
                    EdgeLayout& afterLayout = edgeLayouts_[edgeId];
                    Point afterPos = isSource ? afterLayout.sourcePoint : afterLayout.targetPoint;
                    int afterSnapIdx = isSource ? afterLayout.sourceSnapIndex : afterLayout.targetSnapIndex;
                    NodeEdge afterEdge = isSource ? afterLayout.sourceEdge : afterLayout.targetEdge;

                    std::cout << "[test_snap_drag] AFTER:" << std::endl;
                    std::cout << "  snapPoint=(" << afterPos.x << "," << afterPos.y << ")"
                              << " snapIndex=" << afterSnapIdx
                              << " nodeEdge=" << static_cast<int>(afterEdge) << std::endl;
                    std::cout << "  actualPosition=(" << result.actualPosition.x << "," << result.actualPosition.y << ")"
                              << " newEdge=" << static_cast<int>(result.newEdge) << std::endl;
                    std::cout << "  redistributedEdges=" << result.redistributedEdges.size() << std::endl;
                }

                // Log all edge layouts AFTER
                std::cout << "[test_snap_drag] All edge layouts AFTER:" << std::endl;
                for (const auto& [eid, elayout] : edgeLayouts_) {
                    std::cout << "  Edge " << eid << ": snapIdx=(" << elayout.sourceSnapIndex
                              << "," << elayout.targetSnapIndex << ") src=(" << elayout.sourcePoint.x
                              << "," << elayout.sourcePoint.y << ") tgt=(" << elayout.targetPoint.x
                              << "," << elayout.targetPoint.y << ") bends=" << elayout.bendPoints.size() << std::endl;
                }

                std::cout << "========== TEST_SNAP_DRAG END ==========\n" << std::endl;
                commandServer_.sendResponse(result.success ? "OK test_snap_drag" : "FAILED " + result.reason);
            } else {
                commandServer_.sendResponse("ERROR edge not found");
            }
        }
        else if (cmd.name == "test_coop_reroute" && cmd.args.size() >= 1) {
            // test_coop_reroute <edge_id> - Test cooperative rerouting for single edge
            // New algorithm: find my path, identify blocking edges that overlap (not cross),
            // reroute blocking edges with my path as obstacle to force them around
            EdgeId edgeId = std::stoi(cmd.args[0]);

            if (edgeLayouts_.count(edgeId)) {
                std::cout << "\n========== TEST_COOP_REROUTE START ==========" << std::endl;
                std::cout << "[coop_reroute] Edge " << edgeId << std::endl;

                const EdgeLayout& currentLayout = edgeLayouts_[edgeId];
                std::cout << "[coop_reroute] BEFORE: src=(" << currentLayout.sourcePoint.x 
                          << "," << currentLayout.sourcePoint.y << ") srcEdge=" 
                          << static_cast<int>(currentLayout.sourceEdge) << std::endl;
                std::cout << "[coop_reroute] BEFORE: tgt=(" << currentLayout.targetPoint.x 
                          << "," << currentLayout.targetPoint.y << ") tgtEdge=" 
                          << static_cast<int>(currentLayout.targetEdge) << std::endl;
                std::cout << "[coop_reroute] BEFORE: bendPoints=" << currentLayout.bendPoints.size() << std::endl;

                // Build other layouts (excluding this edge)
                std::unordered_map<EdgeId, EdgeLayout> otherLayouts;
                for (const auto& [eid, layout] : edgeLayouts_) {
                    if (eid != edgeId) {
                        otherLayouts[eid] = layout;
                    }
                }

                // Create cooperative rerouter
                CooperativeRerouter rerouter(nullptr, gridSize_);
                auto result = rerouter.rerouteWithCooperation(edgeId, currentLayout, otherLayouts, nodeLayouts_);

                std::cout << "[coop_reroute] Result: success=" << result.success 
                          << " attempts=" << result.attempts
                          << " reroutedEdges=" << result.reroutedEdges.size() << std::endl;

                if (result.success) {
                    std::cout << "[coop_reroute] AFTER: src=(" << result.layout.sourcePoint.x 
                              << "," << result.layout.sourcePoint.y << ") srcEdge=" 
                              << static_cast<int>(result.layout.sourceEdge) << std::endl;
                    std::cout << "[coop_reroute] AFTER: tgt=(" << result.layout.targetPoint.x 
                              << "," << result.layout.targetPoint.y << ") tgtEdge=" 
                              << static_cast<int>(result.layout.targetEdge) << std::endl;
                    std::cout << "[coop_reroute] AFTER: bendPoints=" << result.layout.bendPoints.size() << std::endl;
                    
                    // Apply the result (my layout)
                    edgeLayouts_[edgeId] = result.layout;
                    std::cout << "[coop_reroute] My layout applied." << std::endl;

                    // Apply rerouted blocking edges
                    for (const auto& reroutedLayout : result.reroutedEdges) {
                        edgeLayouts_[reroutedLayout.id] = reroutedLayout;
                        std::cout << "[coop_reroute] Blocking edge " << reroutedLayout.id 
                                  << " rerouted." << std::endl;
                    }

                    // Also update otherLayouts back to edgeLayouts_ (they may have been modified)
                    for (const auto& [eid, layout] : otherLayouts) {
                        edgeLayouts_[eid] = layout;
                    }
                } else {
                    std::cout << "[coop_reroute] Failed: " << result.failureReason << std::endl;
                }

                std::cout << "========== TEST_COOP_REROUTE END ==========\n" << std::endl;
                commandServer_.sendResponse(result.success ? "OK test_coop_reroute" : "FAILED " + result.failureReason);
            } else {
                commandServer_.sendResponse("ERROR edge not found");
            }
        }
        else if (cmd.name == "test_coop_pair" && cmd.args.size() >= 2) {
            // test_coop_pair <edge_id_a> <edge_id_b> - Check if two edges have segment overlap
            // This is now a simple overlap check, not rerouting
            EdgeId edgeIdA = std::stoi(cmd.args[0]);
            EdgeId edgeIdB = std::stoi(cmd.args[1]);

            if (edgeLayouts_.count(edgeIdA) && edgeLayouts_.count(edgeIdB)) {
                std::cout << "\n========== TEST_COOP_PAIR START ==========" << std::endl;
                std::cout << "[coop_pair] Checking overlap between edges " << edgeIdA << " and " << edgeIdB << std::endl;

                const EdgeLayout& layoutA = edgeLayouts_[edgeIdA];
                const EdgeLayout& layoutB = edgeLayouts_[edgeIdB];

                bool hasOverlap = CooperativeRerouter::hasSegmentOverlap(layoutA, layoutB);

                std::cout << "[coop_pair] Result: hasOverlap=" << (hasOverlap ? "true" : "false") << std::endl;
                std::cout << "[coop_pair] (Cross intersections allowed, parallel segment overlaps forbidden)" << std::endl;

                std::cout << "========== TEST_COOP_PAIR END ==========\n" << std::endl;
                commandServer_.sendResponse(hasOverlap ? "OVERLAP" : "NO_OVERLAP");
            } else {
                commandServer_.sendResponse("ERROR one or both edges not found");
            }
        }
        else if (cmd.name == "test_drag" && cmd.args.size() >= 3) {
            // test_drag <node_id> <dx> <dy> - Full drag simulation with detailed logging
            NodeId nodeId = std::stoi(cmd.args[0]);
            float dx = std::stof(cmd.args[1]);
            float dy = std::stof(cmd.args[2]);

            if (nodeLayouts_.count(nodeId)) {
                std::cout << "\n========== TEST_DRAG START ==========" << std::endl;
                std::cout << "[test_drag] Node " << nodeId << " delta=(" << dx << "," << dy << ")" << std::endl;

                // Log BEFORE state
                std::cout << "[test_drag] BEFORE node position: ("
                          << nodeLayouts_[nodeId].position.x << ","
                          << nodeLayouts_[nodeId].position.y << ")" << std::endl;
                std::cout << "[test_drag] BEFORE edge layouts:" << std::endl;
                for (const auto& [edgeId, layout] : edgeLayouts_) {
                    std::cout << "  Edge " << edgeId << ": snapIdx=(" << layout.sourceSnapIndex
                              << "," << layout.targetSnapIndex << ") src=(" << layout.sourcePoint.x
                              << "," << layout.sourcePoint.y << ") tgt=(" << layout.targetPoint.x
                              << "," << layout.targetPoint.y << ") bends=" << layout.bendPoints.size() << std::endl;
                }

                Point proposedPos = {
                    nodeLayouts_[nodeId].position.x + dx,
                    nodeLayouts_[nodeId].position.y + dy
                };
                std::cout << "[test_drag] Proposed position: (" << proposedPos.x << "," << proposedPos.y << ")" << std::endl;

                // Collect affected edges
                std::vector<EdgeId> affected;
                for (const auto& [edgeId, layout] : edgeLayouts_) {
                    if (layout.from == nodeId || layout.to == nodeId) {
                        affected.push_back(edgeId);
                    }
                }
                std::cout << "[test_drag] Affected edges: " << affected.size() << std::endl;

                // Simulate drag start
                std::cout << "[test_drag] Calling onDragStart()..." << std::endl;
                routingCoordinator_->onDragStart(affected);
                std::cout << "[test_drag] routingCoordinator state after onDragStart: "
                          << static_cast<int>(routingCoordinator_->state()) << std::endl;

                // Actually move the node (bypass moveNode validation for testing)
                nodeLayouts_[nodeId].position = proposedPos;
                std::cout << "[test_drag] Node position updated" << std::endl;

                // Save affectedEdges_ for callback
                affectedEdges_ = affected;

                // Trigger drag end
                std::cout << "[test_drag] Calling onDragEnd()..." << std::endl;
                std::cout << "[test_drag] dragAlgorithm=" << static_cast<int>(layoutOptions_.optimizationOptions.dragAlgorithm)
                          << " postDragAlgorithm=" << static_cast<int>(layoutOptions_.optimizationOptions.postDragAlgorithm) << std::endl;
                routingCoordinator_->onDragEnd({nodeId});
                std::cout << "[test_drag] routingCoordinator state after onDragEnd: "
                          << static_cast<int>(routingCoordinator_->state()) << std::endl;

                // Force update() to trigger callback (debounceDelay=0)
                routingCoordinator_->update(SDL_GetTicks());

                // Log AFTER state
                std::cout << "[test_drag] AFTER edge layouts:" << std::endl;
                for (const auto& [edgeId, layout] : edgeLayouts_) {
                    std::cout << "  Edge " << edgeId << ": snapIdx=(" << layout.sourceSnapIndex
                              << "," << layout.targetSnapIndex << ") src=(" << layout.sourcePoint.x
                              << "," << layout.sourcePoint.y << ") tgt=(" << layout.targetPoint.x
                              << "," << layout.targetPoint.y << ") bends=" << layout.bendPoints.size() << std::endl;
                }
                std::cout << "========== TEST_DRAG END ==========\n" << std::endl;

                commandServer_.sendResponse("OK test_drag " + std::to_string(nodeId));
            } else {
                commandServer_.sendResponse("ERROR node not found");
            }
        }
        else if (cmd.name == "pause") {
            paused_ = true;
            pauseMessage_ = cmd.args.empty() ? "Paused by external command" : cmd.args[0];
            commandServer_.sendResponse("OK paused");
        }
        else if (cmd.name == "resume") {
            paused_ = false;
            pauseMessage_.clear();
            commandServer_.sendResponse("OK resumed");
        }
        else if (cmd.name == "get_state") {
            // Return current node positions and edge info (brief)
            std::ostringstream oss;
            oss << "STATE nodes=" << nodeLayouts_.size() << " edges=" << edgeLayouts_.size();
            for (const auto& [id, layout] : nodeLayouts_) {
                oss << " n" << id << "=(" << layout.position.x << "," << layout.position.y << ")";
            }
            commandServer_.sendResponse(oss.str());
        }
        else if (cmd.name == "get_layout") {
            // Return full layout as JSON (same as save format)
            LayoutResult currentResult;
            for (const auto& [id, layout] : nodeLayouts_) {
                currentResult.setNodeLayout(id, layout);
            }
            for (const auto& [id, layout] : edgeLayouts_) {
                currentResult.setEdgeLayout(id, layout);
            }
            currentResult.setLayerCount(layoutResult_.layerCount());

            std::string layoutJson = LayoutSerializer::toJson(currentResult);
            commandServer_.sendResponse(layoutJson);
        }
        else if (cmd.name == "check_penetration") {
            // Check if any edge penetrates any node
            int count = 0;
            std::ostringstream oss;
            oss << "PENETRATION";
            for (const auto& [edgeId, el] : edgeLayouts_) {
                for (const auto& [nodeId, nl] : nodeLayouts_) {
                    if (nodeId == el.from || nodeId == el.to) continue;

                    // Check if edge path intersects node bounds
                    auto checkSegment = [&](const Point& p1, const Point& p2) {
                        // Simple bounding box check
                        float minX = std::min(p1.x, p2.x);
                        float maxX = std::max(p1.x, p2.x);
                        float minY = std::min(p1.y, p2.y);
                        float maxY = std::max(p1.y, p2.y);

                        float nodeMinX = nl.position.x;
                        float nodeMaxX = nl.position.x + nl.size.width;
                        float nodeMinY = nl.position.y;
                        float nodeMaxY = nl.position.y + nl.size.height;

                        // Check overlap
                        if (maxX > nodeMinX && minX < nodeMaxX &&
                            maxY > nodeMinY && minY < nodeMaxY) {
                            // More precise check for orthogonal segments
                            if (p1.x == p2.x) { // Vertical segment
                                if (p1.x > nodeMinX && p1.x < nodeMaxX) {
                                    float segMinY = std::min(p1.y, p2.y);
                                    float segMaxY = std::max(p1.y, p2.y);
                                    if (segMaxY > nodeMinY && segMinY < nodeMaxY) {
                                        return true;
                                    }
                                }
                            } else if (p1.y == p2.y) { // Horizontal segment
                                if (p1.y > nodeMinY && p1.y < nodeMaxY) {
                                    float segMinX = std::min(p1.x, p2.x);
                                    float segMaxX = std::max(p1.x, p2.x);
                                    if (segMaxX > nodeMinX && segMinX < nodeMaxX) {
                                        return true;
                                    }
                                }
                            }
                        }
                        return false;
                    };

                    std::vector<Point> path;
                    path.push_back(el.sourcePoint);
                    for (const auto& bp : el.bendPoints) path.push_back(bp.position);
                    path.push_back(el.targetPoint);

                    for (size_t i = 0; i + 1 < path.size(); ++i) {
                        if (checkSegment(path[i], path[i+1])) {
                            oss << " e" << edgeId << "->n" << nodeId;
                            count++;
                            break;
                        }
                    }
                }
            }
            oss << " count=" << count;
            commandServer_.sendResponse(oss.str());
        }
        else if (cmd.name == "get_log") {
            // Return captured logs (non-destructive read)
            std::string logs = LogCapture::instance().get();
            if (logs.empty()) {
                commandServer_.sendResponse("LOG_EMPTY");
            } else {
                // Replace newlines with special marker for TCP transmission
                std::string escaped;
                for (char c : logs) {
                    if (c == '\n') {
                        escaped += "\\n";
                    } else {
                        escaped += c;
                    }
                }
                commandServer_.sendResponse("LOG " + escaped);
            }
        }
        else if (cmd.name == "clear_log") {
            // Clear log buffer
            LogCapture::instance().clear();
            commandServer_.sendResponse("OK cleared");
        }
        else if (cmd.name == "wait_idle") {
            // Wait until optimization is complete (with timeout)
            int timeoutMs = 5000;  // Default 5 seconds
            if (!cmd.args.empty()) {
                timeoutMs = std::stoi(cmd.args[0]);
            }
            
            auto startTime = std::chrono::steady_clock::now();
            while (routingCoordinator_->state() != RoutingState::Idle) {
                auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                    std::chrono::steady_clock::now() - startTime).count();
                if (elapsed >= timeoutMs) {
                    commandServer_.sendResponse("TIMEOUT state=" + 
                        std::to_string(static_cast<int>(routingCoordinator_->state())));
                    return;
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
            commandServer_.sendResponse("OK idle");
        }
        else if (cmd.name == "get_state_full") {
            // Extended state including optimization status
            std::ostringstream oss;
            oss << "STATE nodes=" << nodeLayouts_.size()
                << " edges=" << edgeLayouts_.size()
                << " routing=" << static_cast<int>(routingCoordinator_->state())
                << " pending=" << (routingCoordinator_->isPendingOptimization() ? "yes" : "no");
            commandServer_.sendResponse(oss.str());
        }
        else if (cmd.name == "astar_viz") {
            // Enable A* visualization, optionally for specific edge
            showAStarDebug_ = true;
            debugEdgeId_ = INVALID_EDGE;

            if (!cmd.args.empty()) {
                debugEdgeId_ = std::stoi(cmd.args[0]);
            }

            // Build obstacle map
            debugObstacles_ = std::make_shared<ObstacleMap>();
            debugObstacles_->buildFromNodes(nodeLayouts_, gridSize_);
            debugObstacles_->addEdgeSegments(edgeLayouts_, debugEdgeId_);

            // Set start/goal from specified edge
            if (debugEdgeId_ != INVALID_EDGE) {
                auto it = edgeLayouts_.find(debugEdgeId_);
                if (it != edgeLayouts_.end()) {
                    astarStart_ = it->second.sourcePoint;
                    astarGoal_ = it->second.targetPoint;
                }
            } else {
                astarStart_ = {-1, -1};
                astarGoal_ = {-1, -1};
            }

            std::ostringstream oss;
            oss << "OK astar_viz edge=" << debugEdgeId_
                << " grid=" << debugObstacles_->width() << "x" << debugObstacles_->height();
            commandServer_.sendResponse(oss.str());
        }
        else if (cmd.name == "astar_viz_off") {
            showAStarDebug_ = false;
            debugObstacles_.reset();
            commandServer_.sendResponse("OK astar_viz_off");
        }
        else if (cmd.name == "quit") {
            commandServer_.sendResponse("OK bye");
            shouldQuit_ = true;
        }
        else {
            commandServer_.sendResponse("ERROR unknown command: " + cmd.name);
        }
    }
};

int main(int argc, char* argv[]) {
    // Install log capture for TCP polling
    LogCapture::instance().install();

    // Parse command line arguments
    int port = 9999;
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg.find("--port=") == 0) {
            port = std::stoi(arg.substr(7));
        } else if (arg == "-p" && i + 1 < argc) {
            port = std::stoi(argv[++i]);
        }
    }

    if (!SDL_Init(SDL_INIT_VIDEO)) {
        SDL_Log("SDL_Init failed: %s", SDL_GetError());
        return 1;
    }

    SDL_Window* window = SDL_CreateWindow(
        "ArborVia Interactive Demo",
        1024, 768,
        SDL_WINDOW_RESIZABLE
    );
    if (!window) {
        SDL_Log("SDL_CreateWindow failed: %s", SDL_GetError());
        return 1;
    }

    SDL_Renderer* renderer = SDL_CreateRenderer(window, nullptr);
    if (!renderer) {
        SDL_Log("SDL_CreateRenderer failed: %s", SDL_GetError());
        return 1;
    }

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;

    ImGui::StyleColorsDark();

    ImGui_ImplSDL3_InitForSDLRenderer(window, renderer);
    ImGui_ImplSDLRenderer3_Init(renderer);

    InteractiveDemo demo(port);

    // Start command server for external control
    if (!demo.startCommandServer()) {
        std::cerr << "Warning: Failed to start command server on port " << demo.port() << std::endl;
    } else {
        std::cout << "Command server started on port " << demo.port() << std::endl;
        std::cout << "Commands: drag <id> <dx> <dy>, set_pos <id> <x> <y>, pause [msg], resume, get_state, get_layout, check_penetration, quit" << std::endl;
    }

    bool running = true;
    while (running) {
        // Process external commands
        demo.processCommands();
        if (demo.shouldQuit()) {
            running = false;
            break;
        }

        SDL_Event event;
        while (SDL_PollEvent(&event)) {
            ImGui_ImplSDL3_ProcessEvent(&event);
            if (event.type == SDL_EVENT_QUIT) {
                running = false;
            }
            if (event.type == SDL_EVENT_KEY_DOWN &&
                event.key.key == SDLK_ESCAPE) {
                running = false;
            }
        }

        ImGui_ImplSDLRenderer3_NewFrame();
        ImGui_ImplSDL3_NewFrame();
        ImGui::NewFrame();

        if (!demo.isPaused()) {
            demo.update();
        }

        // Draw graph on background (must be before Render())
        ImDrawList* drawList = ImGui::GetBackgroundDrawList();
        demo.render(drawList);

        demo.renderUI();

        // Display pause overlay if paused
        if (demo.isPaused()) {
            ImGui::SetNextWindowPos(ImVec2(ImGui::GetIO().DisplaySize.x * 0.5f,
                                           ImGui::GetIO().DisplaySize.y * 0.5f),
                                    ImGuiCond_Always, ImVec2(0.5f, 0.5f));
            ImGui::Begin("Paused", nullptr,
                        ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove |
                        ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_AlwaysAutoResize);
            ImGui::Text("PAUSED");
            if (!demo.pauseMessage().empty()) {
                ImGui::Text("%s", demo.pauseMessage().c_str());
            }
            ImGui::Text("Send 'resume' command to continue");
            ImGui::End();
        }

        ImGui::Render();

        SDL_SetRenderDrawColor(renderer, 245, 245, 245, 255);
        SDL_RenderClear(renderer);

        ImGui_ImplSDLRenderer3_RenderDrawData(ImGui::GetDrawData(), renderer);
        SDL_RenderPresent(renderer);
    }

    ImGui_ImplSDLRenderer3_Shutdown();
    ImGui_ImplSDL3_Shutdown();
    ImGui::DestroyContext();

    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();

    return 0;
}
