#include <SDL3/SDL.h>
#include <imgui.h>
#include <imgui_impl_sdl3.h>
#include <imgui_impl_sdlrenderer3.h>

#include <arborvia/arborvia.h>
#include <arborvia/common/Logger.h>
#include <arborvia/layout/interactive/PathRoutingCoordinator.h>
#include <arborvia/layout/api/LayoutController.h>
#include <arborvia/layout/config/ConstraintConfig.h>
#include <arborvia/layout/constraints/ConstraintGateway.h>
#include "../../src/layout/interactive/ConstraintManager.h"
#include <arborvia/layout/interactive/SnapPointController.h>
#include "../../src/layout/pathfinding/ObstacleMap.h"
#include "../../src/layout/snap/GridSnapCalculator.h"
#include "../../src/layout/pathfinding/AStarPathFinder.h"
#include "../../src/layout/routing/OrthogonalRouter.h"
#include "../../src/layout/routing/CooperativeRerouter.h"
#include "CommandServer.h"
#include <arborvia/core/TaskExecutor.h>

// SCXML test infrastructure
#include "SCXMLTestLoader.h"
#include "SCXMLGraph.h"
#include "SCXMLTypes.h"

using arborvia::test::scxml::SCXMLTestLoader;
using arborvia::test::scxml::SCXMLGraph;
using arborvia::test::scxml::SCXMLNodeType;
using arborvia::test::scxml::TestInfo;
using arborvia::test::scxml::ConvertOptions;

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
#include <queue>
#include <future>

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

        int sync() override {
            if (original_) {
                original_->pubsync();
            }
            return 0;
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

// Point node visual radius (used for both rendering and hit detection)
constexpr float POINT_NODE_RADIUS = 10.0f;

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

        // Configure routing coordinator with async mode
        routingCoordinator_->setDebounceDelay(0);  // Immediate A* after drop (no delay)
        routingCoordinator_->setListener(this);
        routingCoordinator_->setAsyncMode(true);  // Enable async optimization
        
        // Sync callback (used internally, not for async result delivery)
        routingCoordinator_->setOptimizationCallback(
            [this](const std::vector<EdgeId>& affectedEdges,
                   const std::unordered_set<NodeId>& movedNodes) {
            // In async mode, this callback triggers async optimization start
            startAsyncOptimization(affectedEdges, movedNodes);
        });

        setupGraph();
        doLayout();
    }

    ~InteractiveDemo() {
        // Ensure all async tasks complete before destroying this
        if (executor_) {
            executor_->shutdown();
        }
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

    void update() {
        // Update routing coordinator for debounce timing (triggers optimization after delay)
        routingCoordinator_->update(SDL_GetTicks());

        ImGuiIO& io = ImGui::GetIO();

        // Handle zoom (mouse wheel) - works even when ImGui wants mouse for scroll
        if (!io.WantCaptureMouse && io.MouseWheel != 0) {
            // Zoom centered on mouse position
            ImVec2 mousePos = io.MousePos;
            Point worldBeforeZoom = screenToWorld(mousePos);

            float zoomDelta = io.MouseWheel * 0.1f;
            float oldZoom = zoom_;
            zoom_ = std::clamp(zoom_ + zoomDelta, 0.1f, 5.0f);

            // Adjust pan to keep mouse position fixed in world coordinates
            if (zoom_ != oldZoom) {
                Point worldAfterZoom = screenToWorld(mousePos);
                panOffset_.x += worldAfterZoom.x - worldBeforeZoom.x;
                panOffset_.y += worldAfterZoom.y - worldBeforeZoom.y;
            }
        }

        // Handle pan (middle mouse drag - works anywhere)
        if (ImGui::IsMouseDragging(ImGuiMouseButton_Middle) && !io.WantCaptureMouse) {
            panOffset_.x += io.MouseDelta.x / zoom_;
            panOffset_.y += io.MouseDelta.y / zoom_;
            isPanning_ = true;
        } else if (!emptyAreaPanStarted_) {
            isPanning_ = false;
        }

        // Skip other graph interaction if ImGui wants mouse or middle-mouse panning
        if (io.WantCaptureMouse || (isPanning_ && !emptyAreaPanStarted_)) {
            return;
        }

        ImVec2 mousePos = io.MousePos;

        // Convert to graph coordinates (using pan/zoom transform)
        Point graphMouse = screenToWorld(mousePos);

        // Find hovered node
        hoveredNode_ = INVALID_NODE;
        for (const auto& [id, layout] : nodeLayouts_) {
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
                // Compute snap indices from positions
                auto srcNodeIt = nodeLayouts_.find(edgeLayout.from);
                auto tgtNodeIt = nodeLayouts_.find(edgeLayout.to);
                float gridSize = layoutOptions_.gridConfig.cellSize > 0 
                    ? layoutOptions_.gridConfig.cellSize : 10.0f;
                if (srcNodeIt != nodeLayouts_.end()) {
                    routing.sourceSnapIndex = GridSnapCalculator::getCandidateIndexFromPosition(
                        srcNodeIt->second, edgeLayout.sourceEdge, edgeLayout.sourcePoint, gridSize);
                }
                if (tgtNodeIt != nodeLayouts_.end()) {
                    routing.targetSnapIndex = GridSnapCalculator::getCandidateIndexFromPosition(
                        tgtNodeIt->second, edgeLayout.targetEdge, edgeLayout.targetPoint, gridSize);
                }
                manualManager_->setEdgeRouting(edgeId, routing);
            }

            const auto& existingBps = manualManager_->getBendPoints(edgeId);

            // Use library API to calculate bend point pair
            auto bpResult = OrthogonalRouter::calculateBendPointPair(
                edgeLayout, existingBps, clickPos, bendPointPreview_.insertIndex);

            // Insert both points (convert Point to GridPoint)
            manualManager_->addBendPoint(edgeId, bpResult.insertIndex, 
                GridPoint::fromPixel(bpResult.first, gridSize_), gridSize_);
            manualManager_->addBendPoint(edgeId, bpResult.insertIndex + 1, 
                GridPoint::fromPixel(bpResult.second, gridSize_), gridSize_);

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
            // Create constraint manager for drag validation
            LOG_DEBUG("[Demo::dragStart] Creating constraint manager with layoutOptions_.optimizationOptions.dragAlgorithm={}",
                      static_cast<int>(layoutOptions_.optimizationOptions.dragAlgorithm));
            auto defaultConfig = ConstraintConfig::createDefault(&layoutOptions_);
            LOG_DEBUG("[Demo::dragStart] ConstraintConfig has {} constraints", defaultConfig.constraints.size());
            for (const auto& c : defaultConfig.constraints) {
                LOG_DEBUG("[Demo::dragStart]   constraint: type={} enabled={}", c.type, c.enabled);
            }
            constraintManager_ = ConstraintFactory::create(defaultConfig);
            LOG_DEBUG("[Demo::dragStart] ConstraintManager created with {} constraints", constraintManager_->constraintCount());
            // Pre-calculate blocked regions for visualization
            ConstraintContext ctx{draggedNode_, layout.position, nodeLayouts_, edgeLayouts_, &graph_, gridSize_};
            blockedRegions_ = constraintManager_->getAllBlockedRegions(ctx);
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
        } else if (ImGui::IsMouseClicked(0) && hoveredNode_ == INVALID_NODE && hoveredEdge_ == INVALID_EDGE && !hoveredBendPoint_.isValid() && !hoveredSnapPoint_.isValid()) {
            // Left click on empty area - start potential pan
            emptyAreaPanStarted_ = true;
        }

        // Handle empty area pan (left-drag on empty)
        if (emptyAreaPanStarted_ && ImGui::IsMouseDragging(ImGuiMouseButton_Left)) {
            panOffset_.x += io.MouseDelta.x / zoom_;
            panOffset_.y += io.MouseDelta.y / zoom_;
            isPanning_ = true;
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

            // Handle empty area pan release
            if (emptyAreaPanStarted_) {
                if (!isPanning_) {
                    // Was just a click (no drag) - deselect everything
                    selectedNode_ = INVALID_NODE;
                    selectedEdge_ = INVALID_EDGE;
                    selectedBendPoint_.clear();
                }
                emptyAreaPanStarted_ = false;
                isPanning_ = false;
            }
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

                // Apply the calculated positions (convert Point to GridPoint)
                if (dragResult.nextAdjusted) {
                    manualManager_->moveBendPoint(edgeId, static_cast<size_t>(bpIdx + 1), 
                        GridPoint::fromPixel(dragResult.adjustedNextPos, gridSize_), gridSize_);
                }
                manualManager_->moveBendPoint(edgeId, static_cast<size_t>(bpIdx), 
                    GridPoint::fromPixel(dragResult.newCurrentPos, gridSize_), gridSize_);

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

            // Early exit if position hasn't changed (prevents unnecessary work every frame)
            constexpr float EPSILON = 0.001f;
            Point proposedPosition = {newX, newY};
            if (std::abs(proposedPosition.x - layout.position.x) < EPSILON &&
                std::abs(proposedPosition.y - layout.position.y) < EPSILON) {
                // Position unchanged - skip all processing
                // This is critical because ImGui::IsMouseDragging returns true every frame while mouse button is held
                return;
            }

            // Position changed - do full validation using ConstraintManager
            LOG_DEBUG("[Demo::drag] graphMouse=({},{}) dragOffset=({},{}) newX={} newY={} proposedPosition=({},{})",
                      graphMouse.x, graphMouse.y, dragOffset_.x, dragOffset_.y, newX, newY, proposedPosition.x, proposedPosition.y);

            ConstraintContext ctx{draggedNode_, proposedPosition, nodeLayouts_, edgeLayouts_, &graph_, gridSize_};
            auto validation = constraintManager_->validate(ctx);
            LOG_DEBUG("[Demo::drag] validation result: valid={} failedConstraint={} reason={}",
                      validation.valid, validation.failedConstraint, validation.reason);

            // Always update position during drag (visual feedback)
            layout.position.x = newX;
            layout.position.y = newY;

            if (validation.valid) {
                // Valid position - save as last valid and re-route edges
                isInvalidDragPosition_ = false;
                lastValidPosition_ = proposedPosition;
                manualManager_->setNodePosition(draggedNode_, layout.position);

                // Re-route connected edges (position change already verified above)
                // Skip rerouting in HideUntilDrop mode - edges will be calculated on drop
                if (layoutOptions_.optimizationOptions.dragAlgorithm != DragAlgorithm::HideUntilDrop) {
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
        // Re-route ALL edges after position change
        // Algorithm used depends on layoutOptions_.optimizationOptions.postDragAlgorithm
        std::vector<EdgeId> allEdges;
        allEdges.reserve(edgeLayouts_.size());
        for (const auto& [edgeId, layout] : edgeLayouts_) {
            allEdges.push_back(edgeId);
        }
        const char* algoName = "Unknown";
        switch (layoutOptions_.optimizationOptions.postDragAlgorithm) {
            case PostDragAlgorithm::None: algoName = "None"; break;
            case PostDragAlgorithm::AStar: algoName = "AStar"; break;
            case PostDragAlgorithm::Geometric: algoName = "Geometric"; break;
        }
        std::cout << "[rerouteAffectedEdges] Processing " << allEdges.size()
                  << " edges with " << algoName << " (draggedNode=" << draggedNode_ << ")" << std::endl;
        std::unordered_set<NodeId> movedNodes = {draggedNode_};
        LayoutUtils::updateEdgePositions(
            edgeLayouts_, nodeLayouts_, allEdges,
            layoutOptions_, movedNodes);
    }

    void drawGrid(ImDrawList* drawList, float width, float height) {
        if (gridSize_ <= 0.0f) return;

        ImU32 gridColor = IM_COL32(200, 200, 200, 80);  // Light gray, semi-transparent
        ImU32 gridColorMajor = IM_COL32(180, 180, 180, 120);  // Slightly darker for major lines

        // Calculate zoomed grid spacing
        float zoomedGridSize = gridSize_ * zoom_;

        // Calculate grid bounds with pan/zoom
        // Find the world coordinate of the screen origin
        Point worldOrigin = screenToWorld({0, 0});
        // Find where the first grid line should be in world coordinates
        float worldStartX = std::floor(worldOrigin.x / gridSize_) * gridSize_;
        float worldStartY = std::floor(worldOrigin.y / gridSize_) * gridSize_;
        // Convert back to screen coordinates
        ImVec2 screenStart = worldToScreen({worldStartX, worldStartY});

        // Draw vertical lines
        int lineCount = static_cast<int>(std::floor(worldOrigin.x / gridSize_));
        for (float x = screenStart.x; x < width; x += zoomedGridSize) {
            if (x >= 0) {
                ImU32 color = (std::abs(lineCount) % 5 == 0) ? gridColorMajor : gridColor;
                drawList->AddLine({x, 0}, {x, height}, color, 1.0f);
            }
            lineCount++;
        }

        // Draw horizontal lines
        lineCount = static_cast<int>(std::floor(worldOrigin.y / gridSize_));
        for (float y = screenStart.y; y < height; y += zoomedGridSize) {
            if (y >= 0) {
                ImU32 color = (std::abs(lineCount) % 5 == 0) ? gridColorMajor : gridColor;
                drawList->AddLine({0, y}, {width, y}, color, 1.0f);
            }
            lineCount++;
        }
    }

    void drawBlockedCells(ImDrawList* drawList) {
        // Only show during drag, and skip if disabled or no blocked regions
        if (!showBlockedCells_ || gridSize_ <= 0.0f || draggedNode_ == INVALID_NODE) return;

        // Verify dragged node exists
        if (nodeLayouts_.find(draggedNode_) == nodeLayouts_.end()) return;

        // Use pre-calculated blocked regions from ConstraintManager
        for (const auto& region : blockedRegions_) {
            // Convert region bounds to grid cells
            int leftCell = static_cast<int>(std::ceil(region.x / gridSize_));
            int topCell = static_cast<int>(std::ceil(region.y / gridSize_));
            int rightCell = static_cast<int>(std::ceil((region.x + region.width) / gridSize_));
            int bottomCell = static_cast<int>(std::ceil((region.y + region.height) / gridSize_));

            // Draw blocked cells
            for (int gx = leftCell; gx < rightCell; ++gx) {
                for (int gy = topCell; gy < bottomCell; ++gy) {
                    ImVec2 p1 = worldToScreen({gx * gridSize_, gy * gridSize_});
                    ImVec2 p2 = worldToScreen({(gx + 1) * gridSize_, (gy + 1) * gridSize_});

                    drawList->AddRectFilled(p1, p2, COLOR_BLOCKED_CELL);
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
        float zoomedCellSize = cellSize * zoom_;

        // Draw all grid cells with obstacles
        for (int gy = 0; gy < debugObstacles_->height(); ++gy) {
            for (int gx = 0; gx < debugObstacles_->width(); ++gx) {
                int gridX = gx + obsOffsetX;
                int gridY = gy + obsOffsetY;
                auto info = debugObstacles_->getCellVisInfo(gridX, gridY);

                ImVec2 p1 = worldToScreen({gridX * cellSize, gridY * cellSize});
                ImVec2 p2 = worldToScreen({(gridX + 1) * cellSize, (gridY + 1) * cellSize});

                // Node obstacles (yellow fill)
                if (info.isNodeBlocked) {
                    drawList->AddRectFilled(p1, p2, nodeBlockColor);
                }
                // Horizontal edge segments (blue horizontal bar)
                if (info.hasHorizontalSegment) {
                    drawList->AddRectFilled({p1.x, p1.y + zoomedCellSize*0.35f},
                                           {p2.x, p2.y - zoomedCellSize*0.35f}, hSegmentColor);
                }
                // Vertical edge segments (green vertical bar)
                if (info.hasVerticalSegment) {
                    drawList->AddRectFilled({p1.x + zoomedCellSize*0.35f, p1.y},
                                           {p2.x - zoomedCellSize*0.35f, p2.y}, vSegmentColor);
                }
            }
        }

        // Highlight start/goal points
        if (astarStart_.x >= 0) {
            ImVec2 p = worldToScreen(astarStart_);
            float radius = 8 * zoom_;
            drawList->AddCircleFilled(p, radius, startColor);
            drawList->AddText({p.x + 10, p.y - 5}, IM_COL32_WHITE, "START");
        }
        if (astarGoal_.x >= 0) {
            ImVec2 p = worldToScreen(astarGoal_);
            float radius = 8 * zoom_;
            drawList->AddCircleFilled(p, radius, goalColor);
            drawList->AddText({p.x + 10, p.y - 5}, IM_COL32_WHITE, "GOAL");
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
            float scaledThickness = thickness * zoom_;
            for (size_t i = 1; i < points.size(); ++i) {
                ImVec2 p1 = worldToScreen(points[i-1]);
                ImVec2 p2 = worldToScreen(points[i]);
                drawList->AddLine(p1, p2, color, scaledThickness);
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
                ImVec2 labelPos = worldToScreen(layout.labelPosition);
                ImVec2 textPos = {labelPos.x - 20, labelPos.y - 15};
                drawList->AddText(textPos, COLOR_TEXT, edge.label.c_str());
            }
        }

        // Draw bend point insertion preview
        if (bendPointPreview_.active) {
            ImVec2 screenPos = worldToScreen(bendPointPreview_.position);
            float size = 5.0f * zoom_;
            ImVec2 pts[4] = {
                {screenPos.x, screenPos.y - size},
                {screenPos.x + size, screenPos.y},
                {screenPos.x, screenPos.y + size},
                {screenPos.x - size, screenPos.y}
            };
            drawList->AddConvexPolyFilled(pts, 4, COLOR_BEND_POINT_PREVIEW);
            drawList->AddPolyline(pts, 4, IM_COL32(100, 200, 255, 200), ImDrawFlags_Closed, 1.0f);

            // Draw "+" indicator
            float plusSize = 3.0f * zoom_;
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

            // Check if this is a point node (size = 0,0)
            // For point nodes, position IS the center (used for Initial, Final, History)
            bool isPointNode = layout.isPointNode();
            
            ImVec2 p1 = worldToScreen(layout.position);
            ImVec2 p2 = worldToScreen({layout.position.x + layout.size.width,
                                       layout.position.y + layout.size.height});
            
            // Get center and radius for circular nodes
            float centerX, centerY, radius;
            if (isPointNode) {
                // Point node: position is center, use visual radius
                ImVec2 centerScreen = worldToScreen(layout.position);
                centerX = centerScreen.x;
                centerY = centerScreen.y;
                radius = POINT_NODE_RADIUS * zoom_;  // Scaled for screen space
            } else {
                // Regular node: center is middle of bounding box
                centerX = (p1.x + p2.x) / 2.0f;
                centerY = (p1.y + p2.y) / 2.0f;
                radius = std::min(p2.x - p1.x, p2.y - p1.y) / 2.0f;
            }

            // Check SCXML node type for special rendering
            bool drawnAsSpecial = false;
            if (scxmlModeActive_ && scxmlGraph_) {
                SCXMLNodeType nodeType = scxmlGraph_->getNodeType(id);
                
                switch (nodeType) {
                    case SCXMLNodeType::Initial: {
                        // Initial pseudo-state: filled black circle at grid point
                        ImU32 initialColor = IM_COL32(30, 30, 30, 255);
                        drawList->AddCircleFilled({centerX, centerY}, radius, initialColor);
                        drawnAsSpecial = true;
                        break;
                    }
                    case SCXMLNodeType::Final: {
                        // Final state: double circle at grid point
                        ImU32 outerColor = COLOR_NODE_BORDER;
                        float outerRadius = radius;
                        float innerRadius = radius * 0.6f;
                        drawList->AddCircle({centerX, centerY}, outerRadius, outerColor, 0, 2.0f * zoom_);
                        drawList->AddCircleFilled({centerX, centerY}, innerRadius, IM_COL32(30, 30, 30, 255));
                        drawnAsSpecial = true;
                        break;
                    }
                    case SCXMLNodeType::History:
                    case SCXMLNodeType::HistoryDeep: {
                        // History: circle with "H" or "H*" at grid point
                        drawList->AddCircle({centerX, centerY}, radius, COLOR_NODE_BORDER, 0, 2.0f * zoom_);
                        drawList->AddCircleFilled({centerX, centerY}, radius - 1.0f * zoom_, fillColor);
                        const char* histText = (nodeType == SCXMLNodeType::HistoryDeep) ? "H*" : "H";
                        ImVec2 textSize = ImGui::CalcTextSize(histText);
                        ImVec2 textPos = {centerX - textSize.x / 2, centerY - textSize.y / 2};
                        drawList->AddText(textPos, COLOR_TEXT, histText);
                        drawnAsSpecial = true;
                        break;
                    }
                    case SCXMLNodeType::Parallel: {
                        // Parallel state: rectangle with dashed border
                        float scaledRounding = 5.0f * zoom_;
                        drawList->AddRectFilled(p1, p2, fillColor, scaledRounding);
                        // Dashed border effect: double line
                        drawList->AddRect(p1, p2, COLOR_NODE_BORDER, scaledRounding, 0, 2.0f * zoom_);
                        drawList->AddRect({p1.x + 3*zoom_, p1.y + 3*zoom_}, 
                                         {p2.x - 3*zoom_, p2.y - 3*zoom_}, 
                                         IM_COL32(100, 100, 100, 150), scaledRounding, 0, 1.0f * zoom_);
                        drawnAsSpecial = true;
                        // Fall through to draw label
                    }
                    default:
                        break;
                }
            }

            // Standard rectangular node rendering
            if (!drawnAsSpecial) {
                float scaledRounding = 5.0f * zoom_;
                drawList->AddRectFilled(p1, p2, fillColor, scaledRounding);
                drawList->AddRect(p1, p2, COLOR_NODE_BORDER, scaledRounding, 0, 2.0f * zoom_);
            }

            // Draw label (for State, Parallel types)
            const NodeData& node = graph_.getNode(id);
            if (!node.label.empty()) {
                // Skip label for Initial/Final/History (already drawn or no label)
                bool skipLabel = false;
                if (scxmlModeActive_ && scxmlGraph_) {
                    SCXMLNodeType nodeType = scxmlGraph_->getNodeType(id);
                    if (nodeType == SCXMLNodeType::Initial || 
                        nodeType == SCXMLNodeType::Final ||
                        nodeType == SCXMLNodeType::History ||
                        nodeType == SCXMLNodeType::HistoryDeep) {
                        skipLabel = true;
                    }
                }
                if (!skipLabel) {
                    ImVec2 textSize = ImGui::CalcTextSize(node.label.c_str());
                    float scaledWidth = layout.size.width * zoom_;
                    float scaledHeight = layout.size.height * zoom_;
                    ImVec2 textPos = {p1.x + (scaledWidth - textSize.x) / 2,
                                     p1.y + (scaledHeight - textSize.y) / 2};
                    drawList->AddText(textPos, COLOR_TEXT, node.label.c_str());
                }
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
            ImVec2 screenPos = worldToScreen(candidate.position);

            bool isSnapped = (static_cast<int>(i) == snappedCandidateIndex_);

            if (isSnapped) {
                // Highlight the snapped candidate
                float radius = 8.0f * zoom_;
                drawList->AddCircleFilled(screenPos, radius, IM_COL32(255, 100, 100, 255));
                drawList->AddCircle(screenPos, radius, IM_COL32(255, 255, 255, 255), 0, 2.0f);
            } else {
                // Draw other candidates as smaller circles
                float radius = 4.0f * zoom_;
                drawList->AddCircleFilled(screenPos, radius, IM_COL32(150, 150, 255, 150));
                drawList->AddCircle(screenPos, radius, IM_COL32(100, 100, 200, 200), 0, 1.0f);
            }
        }
    }

    void drawSnapPreviewPath(ImDrawList* drawList) {
        // Draw preview path in a distinct color (cyan, semi-transparent)
        ImU32 previewColor = IM_COL32(0, 200, 255, 180);
        float thickness = 3.0f * zoom_;

        auto points = snapController_.getPreviewLayout().allPoints();
        for (size_t i = 1; i < points.size(); ++i) {
            ImVec2 p1 = worldToScreen(points[i-1]);
            ImVec2 p2 = worldToScreen(points[i]);
            drawList->AddLine(p1, p2, previewColor, thickness);
        }

        // Draw arrow at the target
        if (points.size() >= 2) {
            ImVec2 p1 = worldToScreen(points[points.size()-2]);
            ImVec2 p2 = worldToScreen(points.back());

            float dx = p2.x - p1.x;
            float dy = p2.y - p1.y;
            float len = std::sqrt(dx*dx + dy*dy);
            if (len > 0.01f) {
                dx /= len;
                dy /= len;
                float arrowSize = 8.0f * zoom_;
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
                ImVec2 screenPos = worldToScreen(edgeLayout.sourcePoint);

                // Determine colors based on hover/drag state
                bool isSourceHovered = (hoveredSnapPoint_.edgeId == edgeId && hoveredSnapPoint_.isSource);
                bool isSourceDragging = (draggingSnapPoint_.edgeId == edgeId && draggingSnapPoint_.isSource);
                float radius = 5.0f * zoom_;
                ImU32 fillColor = IM_COL32(100, 200, 100, 255);
                ImU32 borderColor = IM_COL32(50, 150, 50, 255);

                if (isSourceDragging) {
                    radius = 8.0f * zoom_;
                    fillColor = IM_COL32(255, 180, 80, 255);  // Orange for dragging
                    borderColor = IM_COL32(200, 130, 50, 255);
                } else if (isSourceHovered) {
                    radius = 7.0f * zoom_;
                    fillColor = IM_COL32(255, 220, 100, 255);  // Yellow for hover
                    borderColor = IM_COL32(200, 170, 50, 255);
                }

                drawList->AddCircleFilled(screenPos, radius, fillColor);
                drawList->AddCircle(screenPos, radius, borderColor, 0, 1.5f);

                // Draw snap index label for source
                // Skip for Point nodes (they only have one snap point, label would be redundant)
                bool isSourcePointNode = nodeLayout.isPointNode();
                if (showSnapIndices_ && !isSourcePointNode) {
                    char label[32];
                    const char* edgeName = "";
                    switch (edgeLayout.sourceEdge) {
                        case NodeEdge::Top: edgeName = "T"; break;
                        case NodeEdge::Bottom: edgeName = "B"; break;
                        case NodeEdge::Left: edgeName = "L"; break;
                        case NodeEdge::Right: edgeName = "R"; break;
                    }
                    // Compute snap index from position
                    int srcSnapIdx = GridSnapCalculator::getCandidateIndexFromPosition(
                        nodeLayout, edgeLayout.sourceEdge, edgeLayout.sourcePoint, gridSize_);
                    snprintf(label, sizeof(label), "%s%d", edgeName, srcSnapIdx);

                    // Position label: centered on snap point, inside node (away from arrow)
                    ImVec2 textSize = ImGui::CalcTextSize(label);
                    ImVec2 textPos = screenPos;
                    float labelOffset = 8.0f * zoom_;

                    // Center text horizontally/vertically and move inside node
                    switch (edgeLayout.sourceEdge) {
                        case NodeEdge::Top:
                            textPos.x -= textSize.x / 2.0f;
                            textPos.y += labelOffset;  // Move down into node
                            break;
                        case NodeEdge::Bottom:
                            textPos.x -= textSize.x / 2.0f;
                            textPos.y -= textSize.y + labelOffset;  // Move up into node
                            break;
                        case NodeEdge::Left:
                            textPos.x += labelOffset;  // Move right into node
                            textPos.y -= textSize.y / 2.0f;
                            break;
                        case NodeEdge::Right:
                            textPos.x -= textSize.x + labelOffset;  // Move left into node
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
                ImVec2 screenPos = worldToScreen(edgeLayout.targetPoint);

                // Determine colors based on hover/drag state
                bool isTargetHovered = (hoveredSnapPoint_.edgeId == edgeId && !hoveredSnapPoint_.isSource);
                bool isTargetDragging = (draggingSnapPoint_.edgeId == edgeId && !draggingSnapPoint_.isSource);
                float radius = 5.0f * zoom_;
                ImU32 fillColor = IM_COL32(200, 100, 100, 255);
                ImU32 borderColor = IM_COL32(150, 50, 50, 255);

                if (isTargetDragging) {
                    radius = 8.0f * zoom_;
                    fillColor = IM_COL32(255, 180, 80, 255);  // Orange for dragging
                    borderColor = IM_COL32(200, 130, 50, 255);
                } else if (isTargetHovered) {
                    radius = 7.0f * zoom_;
                    fillColor = IM_COL32(255, 220, 100, 255);  // Yellow for hover
                    borderColor = IM_COL32(200, 170, 50, 255);
                }

                drawList->AddCircleFilled(screenPos, radius, fillColor);
                drawList->AddCircle(screenPos, radius, borderColor, 0, 1.5f);

                // Draw snap index label for target
                // Skip for Point nodes (they only have one snap point, label would be redundant)
                auto tgtNodeIt = nodeLayouts_.find(edgeLayout.to);
                bool isTargetPointNode = (tgtNodeIt != nodeLayouts_.end() &&
                    tgtNodeIt->second.isPointNode());
                if (showSnapIndices_ && !isTargetPointNode) {
                    char label[32];
                    const char* edgeName = "";
                    switch (edgeLayout.targetEdge) {
                        case NodeEdge::Top: edgeName = "T"; break;
                        case NodeEdge::Bottom: edgeName = "B"; break;
                        case NodeEdge::Left: edgeName = "L"; break;
                        case NodeEdge::Right: edgeName = "R"; break;
                    }
                    // Compute snap index from position (use target node)
                    int tgtSnapIdx = 0;
                    if (tgtNodeIt != nodeLayouts_.end()) {
                        tgtSnapIdx = GridSnapCalculator::getCandidateIndexFromPosition(
                            tgtNodeIt->second, edgeLayout.targetEdge, edgeLayout.targetPoint, gridSize_);
                    }
                    snprintf(label, sizeof(label), "%s%d", edgeName, tgtSnapIdx);

                    // Position label: centered on snap point, inside node (away from arrow)
                    ImVec2 textSize = ImGui::CalcTextSize(label);
                    ImVec2 textPos = screenPos;
                    float labelOffset = 8.0f * zoom_;

                    // Center text horizontally/vertically and move inside node
                    switch (edgeLayout.targetEdge) {
                        case NodeEdge::Top:
                            textPos.x -= textSize.x / 2.0f;
                            textPos.y += labelOffset;  // Move down into node
                            break;
                        case NodeEdge::Bottom:
                            textPos.x -= textSize.x / 2.0f;
                            textPos.y -= textSize.y + labelOffset;  // Move up into node
                            break;
                        case NodeEdge::Left:
                            textPos.x += labelOffset;  // Move right into node
                            textPos.y -= textSize.y / 2.0f;
                            break;
                        case NodeEdge::Right:
                            textPos.x -= textSize.x + labelOffset;  // Move left into node
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
        // Convert to screen coordinates first
        ImVec2 screenFrom = worldToScreen(from);
        ImVec2 screenTo = worldToScreen(to);

        float dx = screenTo.x - screenFrom.x;
        float dy = screenTo.y - screenFrom.y;
        float len = std::sqrt(dx * dx + dy * dy);
        if (len < 0.001f) return;

        dx /= len;
        dy /= len;

        float arrowSize = 10.0f * zoom_;
        ImVec2 tip = screenTo;
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

        // Pan/Zoom controls
        ImGui::Separator();
        ImGui::Text("View: %.0f%% zoom", zoom_ * 100);
        ImGui::SameLine();
        if (ImGui::Button("Reset View")) {
            panOffset_ = {0, 0};
            zoom_ = 1.0f;
        }
        if (ImGui::IsItemHovered()) {
            ImGui::SetTooltip("Pan: Drag empty area or Middle mouse\nZoom: Mouse wheel");
        }
        ImGui::SliderFloat("Zoom", &zoom_, 0.1f, 5.0f, "%.1f");

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

        // SCXML Test Visualization Section
        ImGui::Separator();
        if (scxmlLoader_ && scxmlLoader_->getTestCount() > 0) {
            if (ImGui::CollapsingHeader("SCXML Tests", ImGuiTreeNodeFlags_DefaultOpen)) {
                // Current test info
                if (scxmlModeActive_ && currentTestIndex_ >= 0) {
                    const auto* testInfo = getCurrentTestInfo();
                    if (testInfo) {
                        ImGui::TextColored(ImVec4(0.4f, 0.8f, 1.0f, 1.0f), "Test %s", testInfo->id.c_str());
                        ImGui::TextWrapped("%s", testInfo->file.c_str());
                        if (!testInfo->description.empty()) {
                            ImGui::TextWrapped("%s", testInfo->description.c_str());
                        }
                    }
                    ImGui::Text("Index: %d / %zu", currentTestIndex_ + 1, scxmlLoader_->getTestCount());
                } else {
                    ImGui::TextDisabled("No test loaded");
                    ImGui::Text("Tests available: %zu", scxmlLoader_->getTestCount());
                }

                // Navigation buttons
                ImGui::Spacing();
                if (ImGui::Button("Prev")) {
                    prevTest();
                }
                ImGui::SameLine();
                if (ImGui::Button("Next")) {
                    nextTest();
                }
                ImGui::SameLine();
                if (scxmlModeActive_) {
                    if (ImGui::Button("Exit SCXML")) {
                        exitSCXMLMode();
                    }
                } else {
                    if (ImGui::Button("Load First")) {
                        loadSCXMLTest(0);
                    }
                }

                // Test selection combo
                ImGui::Spacing();
                if (ImGui::BeginCombo("Select Test", currentTestIndex_ >= 0 ?
                    scxmlLoader_->getTest(static_cast<size_t>(currentTestIndex_))->id.c_str() : "None")) {
                    for (size_t i = 0; i < scxmlLoader_->getTestCount(); ++i) {
                        const auto* info = scxmlLoader_->getTest(i);
                        bool isSelected = (static_cast<int>(i) == currentTestIndex_);
                        std::string label = info->id + " - " + info->file;
                        if (ImGui::Selectable(label.c_str(), isSelected)) {
                            loadSCXMLTest(i);
                        }
                        if (isSelected) {
                            ImGui::SetItemDefaultFocus();
                        }
                    }
                    ImGui::EndCombo();
                }
            }
        } else {
            ImGui::TextDisabled("SCXML tests not loaded");
            if (ImGui::IsItemHovered()) {
                ImGui::SetTooltip("Place SCXML tests in resources/scxml/");
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

    // Pan/Zoom state
    Point panOffset_ = {0, 0};  // Pan offset in world coordinates
    float zoom_ = 1.0f;         // Zoom level (1.0 = 100%)
    bool isPanning_ = false;           // Currently panning
    bool emptyAreaPanStarted_ = false; // Left-click started on empty area (potential pan)

    // Coordinate transformation helpers
    ImVec2 worldToScreen(const Point& world) const {
        return {
            (world.x + panOffset_.x) * zoom_ + offset_.x,
            (world.y + panOffset_.y) * zoom_ + offset_.y
        };
    }

    Point screenToWorld(const ImVec2& screen) const {
        return {
            (screen.x - offset_.x) / zoom_ - panOffset_.x,
            (screen.y - offset_.y) / zoom_ - panOffset_.y
        };
    }

    float worldToScreenScale(float worldSize) const {
        return worldSize * zoom_;
    }

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
    std::unique_ptr<ConstraintManager> constraintManager_;  // Constraint manager for drag validation
    std::vector<Rect> blockedRegions_;  // Pre-calculated blocked regions for visualization

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

    // Async optimization state
    std::queue<std::function<void()>> mainThreadQueue_;
    std::mutex queueMutex_;
    std::unique_ptr<ITaskExecutor> executor_ = std::make_unique<JThreadExecutor>();

    // SCXML test visualization state
    std::unique_ptr<SCXMLTestLoader> scxmlLoader_;
    std::unique_ptr<SCXMLGraph> scxmlGraph_;
    int currentTestIndex_ = -1;
    bool scxmlModeActive_ = false;
    std::string scxmlBasePath_;

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

    // SCXML test visualization methods
    bool initSCXML(const std::string& basePath) {
        scxmlBasePath_ = basePath;
        scxmlLoader_ = std::make_unique<SCXMLTestLoader>(basePath);
        if (!scxmlLoader_->loadIndex()) {
            std::cerr << "Failed to load SCXML test index from " << basePath << std::endl;
            scxmlLoader_.reset();
            return false;
        }
        std::cout << "Loaded " << scxmlLoader_->getTestCount() << " SCXML tests from " << basePath << std::endl;
        return true;
    }

    bool loadSCXMLTest(size_t index) {
        if (!scxmlLoader_ || index >= scxmlLoader_->getTestCount()) {
            return false;
        }

        auto newGraph = scxmlLoader_->loadGraph(index);
        if (!newGraph) {
            std::cerr << "Failed to load SCXML test " << index << std::endl;
            return false;
        }

        const auto* testInfo = scxmlLoader_->getTest(index);
        std::cout << "Loading SCXML test " << testInfo->id << ": " << testInfo->file << std::endl;

        // Replace the current graph with SCXMLGraph
        scxmlGraph_ = std::move(newGraph);
        currentTestIndex_ = static_cast<int>(index);
        scxmlModeActive_ = true;

        // Copy to main graph_ and do layout
        graph_ = *scxmlGraph_;
        manualManager_->clearManualState();
        doLayout();

        return true;
    }

    void nextTest() {
        if (!scxmlLoader_ || scxmlLoader_->getTestCount() == 0) return;
        size_t next = (currentTestIndex_ < 0) ? 0 :
                      static_cast<size_t>((currentTestIndex_ + 1) % static_cast<int>(scxmlLoader_->getTestCount()));
        loadSCXMLTest(next);
    }

    void prevTest() {
        if (!scxmlLoader_ || scxmlLoader_->getTestCount() == 0) return;
        size_t prev = (currentTestIndex_ <= 0) ?
                      scxmlLoader_->getTestCount() - 1 :
                      static_cast<size_t>(currentTestIndex_ - 1);
        loadSCXMLTest(prev);
    }

    void exitSCXMLMode() {
        scxmlModeActive_ = false;
        scxmlGraph_.reset();
        currentTestIndex_ = -1;
        // Restore default graph
        graph_ = Graph();
        setupGraph();
        doLayout();
    }

    bool isSCXMLModeActive() const { return scxmlModeActive_; }
    size_t getSCXMLTestCount() const { return scxmlLoader_ ? scxmlLoader_->getTestCount() : 0; }
    const TestInfo* getCurrentTestInfo() const {
        if (!scxmlLoader_ || currentTestIndex_ < 0) return nullptr;
        return scxmlLoader_->getTest(static_cast<size_t>(currentTestIndex_));
    }

    // Process async optimization results (call from main loop)
    void processMainThreadQueue() {
        std::queue<std::function<void()>> toProcess;
        {
            std::lock_guard<std::mutex> lock(queueMutex_);
            std::swap(toProcess, mainThreadQueue_);
        }
        while (!toProcess.empty()) {
            toProcess.front()();
            toProcess.pop();
        }
    }

private:
    void postToMainThread(std::function<void()> fn) {
        std::lock_guard<std::mutex> lock(queueMutex_);
        mainThreadQueue_.push(std::move(fn));
    }

    void startAsyncOptimization(
        const std::vector<EdgeId>& /*affectedEdges*/,
        const std::unordered_set<NodeId>& movedNodes)
    {
        // Get current generation for this optimization
        uint64_t gen = routingCoordinator_->currentGeneration();

        // Snapshot current state
        auto snapshotEdges = edgeLayouts_;
        auto snapshotNodes = nodeLayouts_;
        auto options = layoutOptions_;

        // Collect all edges for optimization
        std::vector<EdgeId> allEdges;
        allEdges.reserve(snapshotEdges.size());
        for (const auto& [edgeId, layout] : snapshotEdges) {
            allEdges.push_back(edgeId);
        }

        std::cout << "[Async] Starting optimization gen=" << gen
                  << " edges=" << allEdges.size()
                  << " movedNodes=" << movedNodes.size() << std::endl;

        // Start async optimization via executor
        executor_->submit([this, gen, allEdges, snapshotEdges, snapshotNodes, options, movedNodes]() {
            // Worker thread: run optimization on snapshot
            auto workingEdges = snapshotEdges;

            LayoutUtils::updateEdgePositions(
                workingEdges, snapshotNodes, allEdges,
                options, movedNodes);

            // Post result to main thread
            postToMainThread([this, gen, workingEdges = std::move(workingEdges)]() {
                onAsyncOptimizationComplete(workingEdges, gen);
            });
        });
    }

    void onAsyncOptimizationComplete(
        const std::unordered_map<EdgeId, EdgeLayout>& result,
        uint64_t generation)
    {
        // Check if result is still valid
        if (!routingCoordinator_->canApplyResult(generation)) {
            std::cout << "[Async] Result discarded (gen=" << generation 
                      << ", current=" << routingCoordinator_->currentGeneration() 
                      << ", state=" << static_cast<int>(routingCoordinator_->state())
                      << ")" << std::endl;
            return;
        }
        
        // Apply result
        std::vector<EdgeId> optimizedEdges;
        optimizedEdges.reserve(result.size());
        for (const auto& [id, layout] : result) {
            edgeLayouts_[id] = layout;
            optimizedEdges.push_back(id);
        }
        
        // Notify listener (this clears affectedEdges_ via onOptimizationComplete)
        routingCoordinator_->notifyOptimizationComplete(optimizedEdges);
        
        std::cout << "[Async] Result applied (gen=" << generation 
                  << ", edges=" << result.size() << ")" << std::endl;
    }

    void processCommand(const Command& cmd) {
        if (cmd.name == "drag" && cmd.args.size() >= 3) {
            // drag <node_id> <dx> <dy> - uses LayoutController API with validation
            NodeId nodeId = std::stoi(cmd.args[0]);
            float dx = std::stof(cmd.args[1]);
            float dy = std::stof(cmd.args[2]);

            if (nodeLayouts_.count(nodeId)) {
                Point proposedPos = {
                    nodeLayouts_[nodeId].position.x + dx,
                    nodeLayouts_[nodeId].position.y + dy
                };

                // Sync controller state and use LayoutController for move
                layoutController_->initializeFrom(nodeLayouts_, edgeLayouts_);
                auto result = layoutController_->moveNode(nodeId, proposedPos);

                if (result.success) {
                    // Sync layouts from controller
                    for (const auto& [id, layout] : layoutController_->nodeLayouts()) {
                        nodeLayouts_[id] = layout;
                    }
                    for (const auto& [id, layout] : layoutController_->edgeLayouts()) {
                        edgeLayouts_[id] = layout;
                    }
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

                // Sync controller state and use LayoutController for move
                layoutController_->initializeFrom(nodeLayouts_, edgeLayouts_);
                auto result = layoutController_->moveNode(nodeId, proposedPos);

                if (result.success) {
                    // Sync layouts from controller
                    for (const auto& [id, layout] : layoutController_->nodeLayouts()) {
                        nodeLayouts_[id] = layout;
                    }
                    for (const auto& [id, layout] : layoutController_->edgeLayouts()) {
                        edgeLayouts_[id] = layout;
                    }
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
                NodeEdge currentEdge = isSource ? layout.sourceEdge : layout.targetEdge;
                // Compute snap index from position
                NodeId nodeId = isSource ? layout.from : layout.to;
                int currentSnapIdx = 0;
                auto nodeIt = nodeLayouts_.find(nodeId);
                if (nodeIt != nodeLayouts_.end()) {
                    currentSnapIdx = GridSnapCalculator::getCandidateIndexFromPosition(
                        nodeIt->second, currentEdge, currentPos, gridSize_);
                }

                std::cout << "[test_snap_drag] BEFORE:" << std::endl;
                std::cout << "  snapPoint=(" << currentPos.x << "," << currentPos.y << ")"
                          << " snapIndex=" << currentSnapIdx
                          << " nodeEdge=" << static_cast<int>(currentEdge) << std::endl;

                // Log all edge layouts BEFORE (positions only, snapIdx computed from position)
                std::cout << "[test_snap_drag] All edge layouts BEFORE:" << std::endl;
                for (const auto& [eid, elayout] : edgeLayouts_) {
                    std::cout << "  Edge " << eid << ": src=(" << elayout.sourcePoint.x
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
                    NodeEdge afterEdge = isSource ? afterLayout.sourceEdge : afterLayout.targetEdge;
                    // Compute snap index from position
                    NodeId afterNodeId = isSource ? afterLayout.from : afterLayout.to;
                    int afterSnapIdx = 0;
                    auto afterNodeIt = nodeLayouts_.find(afterNodeId);
                    if (afterNodeIt != nodeLayouts_.end()) {
                        afterSnapIdx = GridSnapCalculator::getCandidateIndexFromPosition(
                            afterNodeIt->second, afterEdge, afterPos, gridSize_);
                    }

                    std::cout << "[test_snap_drag] AFTER:" << std::endl;
                    std::cout << "  snapPoint=(" << afterPos.x << "," << afterPos.y << ")"
                              << " snapIndex=" << afterSnapIdx
                              << " nodeEdge=" << static_cast<int>(afterEdge) << std::endl;
                    std::cout << "  actualPosition=(" << result.actualPosition.x << "," << result.actualPosition.y << ")"
                              << " newEdge=" << static_cast<int>(result.newEdge) << std::endl;
                    std::cout << "  redistributedEdges=" << result.redistributedEdges.size() << std::endl;
                }

                // Log all edge layouts AFTER (positions only, snapIdx computed from position)
                std::cout << "[test_snap_drag] All edge layouts AFTER:" << std::endl;
                for (const auto& [eid, elayout] : edgeLayouts_) {
                    std::cout << "  Edge " << eid << ": src=(" << elayout.sourcePoint.x
                              << "," << elayout.sourcePoint.y << ") tgt=(" << elayout.targetPoint.x
                              << "," << elayout.targetPoint.y << ") bends=" << elayout.bendPoints.size() << std::endl;
                }

                std::cout << "========== TEST_SNAP_DRAG END ==========\n" << std::endl;
                commandServer_.sendResponse(result.success ? "OK test_snap_drag" : "FAILED " + result.reason);
            } else {
                commandServer_.sendResponse("ERROR edge not found");
            }
        }
        else if (cmd.name == "test_snap_preview" && cmd.args.size() >= 4) {
            // test_snap_preview <edge_id> <is_source:0|1> <dx> <dy>
            // Test snap point drag PREVIEW (not final drop)
            EdgeId edgeId = std::stoi(cmd.args[0]);
            bool isSource = std::stoi(cmd.args[1]) != 0;
            float dx = std::stof(cmd.args[2]);
            float dy = std::stof(cmd.args[3]);

            if (edgeLayouts_.count(edgeId)) {
                std::cout << "\n========== TEST_SNAP_PREVIEW START ==========" << std::endl;
                std::cout << "[test_snap_preview] Edge " << edgeId << " isSource=" << isSource
                          << " delta=(" << dx << "," << dy << ")" << std::endl;

                EdgeLayout& layout = edgeLayouts_[edgeId];
                Point currentPos = isSource ? layout.sourcePoint : layout.targetPoint;
                std::cout << "[test_snap_preview] Current snap pos=(" << currentPos.x << "," << currentPos.y << ")" << std::endl;

                // Start drag
                auto startResult = snapController_.startDrag(
                    edgeId, isSource, nodeLayouts_, edgeLayouts_, gridSize_);

                if (startResult.success) {
                    std::cout << "[test_snap_preview] startDrag SUCCESS, candidates=" << startResult.candidates.size() << std::endl;

                    // Update drag to target position
                    Point targetPos = {currentPos.x + dx, currentPos.y + dy};
                    std::cout << "[test_snap_preview] Target pos=(" << targetPos.x << "," << targetPos.y << ")" << std::endl;

                    auto updateResult = snapController_.updateDrag(targetPos, nodeLayouts_, gridSize_);
                    std::cout << "[test_snap_preview] updateDrag: snappedIdx=" << updateResult.snappedCandidateIndex
                              << " hasValidPreview=" << updateResult.hasValidPreview << std::endl;

                    // Log preview layout
                    const auto& preview = snapController_.getPreviewLayout();
                    std::cout << "[test_snap_preview] Preview layout:" << std::endl;
                    std::cout << "  src=(" << preview.sourcePoint.x << "," << preview.sourcePoint.y << ")" << std::endl;
                    std::cout << "  tgt=(" << preview.targetPoint.x << "," << preview.targetPoint.y << ")" << std::endl;
                    std::cout << "  bends=" << preview.bendPoints.size() << std::endl;

                    // Check orthogonality
                    auto points = preview.allPoints();
                    std::cout << "[test_snap_preview] All points:" << std::endl;
                    for (size_t i = 0; i < points.size(); ++i) {
                        std::cout << "  [" << i << "]: (" << points[i].x << "," << points[i].y << ")" << std::endl;
                    }
                    std::cout << "[test_snap_preview] Orthogonality check:" << std::endl;
                    bool allOrthogonal = true;
                    for (size_t i = 1; i < points.size(); ++i) {
                        float segDx = points[i].x - points[i-1].x;
                        float segDy = points[i].y - points[i-1].y;
                        bool isOrtho = (std::abs(segDx) < 0.1f || std::abs(segDy) < 0.1f);
                        std::cout << "  [" << (i-1) << "->" << i << "]: dx=" << segDx << " dy=" << segDy
                                  << " orthogonal=" << (isOrtho ? "YES" : "NO") << std::endl;
                        if (!isOrtho) allOrthogonal = false;
                    }

                    // Cancel drag (don't complete)
                    snapController_.cancelDrag();
                    std::cout << "[test_snap_preview] Drag cancelled" << std::endl;

                    std::cout << "========== TEST_SNAP_PREVIEW END ==========\n" << std::endl;
                    commandServer_.sendResponse(allOrthogonal ? "OK all_orthogonal" : "FAILED diagonal_detected");
                } else {
                    std::cout << "[test_snap_preview] startDrag FAILED: " << startResult.reason << std::endl;
                    std::cout << "========== TEST_SNAP_PREVIEW END ==========\n" << std::endl;
                    commandServer_.sendResponse("FAILED " + startResult.reason);
                }
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
                    std::cout << "  Edge " << edgeId << ": src=(" << layout.sourcePoint.x
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

                // Log AFTER state (positions only, snapIdx computed from position)
                std::cout << "[test_drag] AFTER edge layouts:" << std::endl;
                for (const auto& [edgeId, layout] : edgeLayouts_) {
                    std::cout << "  Edge " << edgeId << ": src=(" << layout.sourcePoint.x
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
            // Use ConstraintGateway for centralized constraint checking
            ConstraintGateway gateway;
            auto results = gateway.validateAll(edgeLayouts_, nodeLayouts_, gridSize_);

            std::ostringstream oss;
            oss << "VIOLATIONS";
            int totalCount = 0;

            for (const auto& [edgeId, result] : results) {
                for (const auto& violation : result.violations) {
                    totalCount++;
                    oss << " e" << edgeId << ":";
                    switch (violation.type) {
                        case ConstraintViolationType::NodePenetration:
                            oss << "NodePen";
                            if (violation.nodeId) oss << "->n" << *violation.nodeId;
                            break;
                        case ConstraintViolationType::DirectionalSourcePenetration:
                            oss << "SrcPen";
                            if (violation.nodeId) oss << "->n" << *violation.nodeId;
                            break;
                        case ConstraintViolationType::DirectionalTargetPenetration:
                            oss << "TgtPen";
                            if (violation.nodeId) oss << "->n" << *violation.nodeId;
                            break;
                        case ConstraintViolationType::Orthogonality:
                            oss << "Diagonal";
                            if (violation.segmentIndex >= 0) oss << "[" << violation.segmentIndex << "]";
                            break;
                        case ConstraintViolationType::SegmentOverlap:
                            oss << "Overlap";
                            if (violation.otherEdgeId) oss << "->e" << *violation.otherEdgeId;
                            break;
                    }
                }
            }
            oss << " count=" << totalCount;
            commandServer_.sendResponse(oss.str());
        }
        else if (cmd.name == "get_log") {
            // Return captured logs with optional filtering and pagination
            // Usage: get_log [pattern] [tail_lines]
            // Examples:
            //   get_log                    - all logs (max 200 lines)
            //   get_log SnapCalc           - lines containing "SnapCalc"
            //   get_log SnapCalc 50        - last 50 lines containing "SnapCalc"
            //   get_log * 100              - last 100 lines (all patterns)
            
            std::string pattern = "";
            size_t tailLines = 200;  // Default max lines
            
            if (!cmd.args.empty()) {
                pattern = cmd.args[0];
                if (pattern == "*") pattern = "";  // Wildcard means no filter
            }
            if (cmd.args.size() >= 2) {
                tailLines = static_cast<size_t>(std::stoi(cmd.args[1]));
            }
            
            // Get logs from library API
            auto loggerLogs = Logger::getCapturedLogs(pattern, tailLines);
            
            // Also get cout logs and filter
            std::string coutLogs = LogCapture::instance().get();
            std::vector<std::string> coutLines;
            std::istringstream iss(coutLogs);
            std::string line;
            while (std::getline(iss, line)) {
                if (pattern.empty() || line.find(pattern) != std::string::npos) {
                    coutLines.push_back(line);
                }
            }
            
            // Combine and apply tail limit
            std::vector<std::string> allLines;
            allLines.insert(allLines.end(), loggerLogs.begin(), loggerLogs.end());
            allLines.insert(allLines.end(), coutLines.begin(), coutLines.end());
            
            if (allLines.size() > tailLines) {
                allLines.erase(allLines.begin(), allLines.begin() + (allLines.size() - tailLines));
            }
            
            if (allLines.empty()) {
                commandServer_.sendResponse("LOG_EMPTY");
            } else {
                // Build response with line count info
                std::ostringstream result;
                result << "LOG [" << allLines.size() << " lines]\\n";
                for (const auto& l : allLines) {
                    for (char c : l) {
                        if (c == '\n') {
                            result << "\\n";
                        } else {
                            result << c;
                        }
                    }
                    result << "\\n";
                }
                commandServer_.sendResponse(result.str());
            }
        }
        else if (cmd.name == "clear_log") {
            // Clear both log buffers
            Logger::clearCapturedLogs();
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
        else if (cmd.name == "scxml_load" && !cmd.args.empty()) {
            // Load SCXML test by index or ID
            if (!scxmlLoader_) {
                commandServer_.sendResponse("ERROR SCXML not initialized");
            } else {
                bool success = false;
                // Try as numeric index first
                try {
                    size_t index = std::stoul(cmd.args[0]);
                    success = loadSCXMLTest(index);
                } catch (...) {
                    // Try as test ID
                    for (size_t i = 0; i < scxmlLoader_->getTestCount(); ++i) {
                        if (scxmlLoader_->getTest(i)->id == cmd.args[0]) {
                            success = loadSCXMLTest(i);
                            break;
                        }
                    }
                }
                if (success) {
                    commandServer_.sendResponse("OK scxml_load " + std::to_string(currentTestIndex_));
                } else {
                    commandServer_.sendResponse("ERROR test not found: " + cmd.args[0]);
                }
            }
        }
        else if (cmd.name == "scxml_next") {
            if (!scxmlLoader_) {
                commandServer_.sendResponse("ERROR SCXML not initialized");
            } else {
                nextTest();
                commandServer_.sendResponse("OK scxml_next " + std::to_string(currentTestIndex_));
            }
        }
        else if (cmd.name == "scxml_prev") {
            if (!scxmlLoader_) {
                commandServer_.sendResponse("ERROR SCXML not initialized");
            } else {
                prevTest();
                commandServer_.sendResponse("OK scxml_prev " + std::to_string(currentTestIndex_));
            }
        }
        else if (cmd.name == "scxml_list") {
            if (!scxmlLoader_) {
                commandServer_.sendResponse("ERROR SCXML not initialized");
            } else {
                std::ostringstream oss;
                oss << "SCXML_TESTS count=" << scxmlLoader_->getTestCount() << "\\n";
                for (size_t i = 0; i < scxmlLoader_->getTestCount(); ++i) {
                    const auto* info = scxmlLoader_->getTest(i);
                    oss << i << ": " << info->id << " - " << info->file << "\\n";
                }
                commandServer_.sendResponse(oss.str());
            }
        }
        else if (cmd.name == "scxml_info") {
            if (!scxmlLoader_) {
                commandServer_.sendResponse("ERROR SCXML not initialized");
            } else if (currentTestIndex_ < 0) {
                commandServer_.sendResponse("SCXML_INFO none loaded");
            } else {
                const auto* info = getCurrentTestInfo();
                std::ostringstream oss;
                oss << "SCXML_INFO index=" << currentTestIndex_
                    << " id=" << info->id
                    << " file=" << info->file
                    << " nodes=" << graph_.nodeCount()
                    << " edges=" << graph_.edgeCount();
                commandServer_.sendResponse(oss.str());
            }
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
    // Enable log capture for TCP retrieval (library API)
    Logger::enableCapture(true);
    
    // Install cout capture for non-Logger output (std::cout << ...)
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

    // Try to load SCXML tests from resources directory
    // Look for resources/scxml relative to executable or common paths
    std::vector<std::string> scxmlPaths = {
        "resources/scxml",
        "../resources/scxml",
        "../../resources/scxml",
        "../../../resources/scxml"
    };
    for (const auto& path : scxmlPaths) {
        if (demo.initSCXML(path)) {
            break;
        }
    }

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
        
        // Process async optimization results
        demo.processMainThreadQueue();
        
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
