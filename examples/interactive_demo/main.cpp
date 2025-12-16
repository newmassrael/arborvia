#include <SDL3/SDL.h>
#include <imgui.h>
#include <imgui_impl_sdl3.h>
#include <imgui_impl_sdlrenderer3.h>

#include "DemoState.h"
#include "DemoRenderer.h"
#include "DemoColors.h"
#include "DemoInputHandler.h"
#include "DemoCommandProcessor.h"

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

// Use centralized colors and visual constants from DemoColors.h
// Aliases for backward compatibility with existing render code
#define COLOR_NODE          DemoColors::NODE
#define COLOR_NODE_HOVER    DemoColors::NODE_HOVER
#define COLOR_NODE_DRAG     DemoColors::NODE_DRAG
#define COLOR_NODE_BORDER   DemoColors::NODE_BORDER
#define COLOR_NODE_SELECTED DemoColors::NODE_SELECTED
#define COLOR_EDGE          DemoColors::EDGE
#define COLOR_EDGE_AFFECTED DemoColors::EDGE_AFFECTED
#define COLOR_EDGE_SELECTED DemoColors::EDGE_SELECTED
#define COLOR_TEXT          DemoColors::TEXT
#define COLOR_BEND_POINT_PREVIEW DemoColors::BEND_POINT_PREVIEW
#define COLOR_BLOCKED_CELL  DemoColors::BLOCKED_CELL
#define POINT_NODE_RADIUS   DemoVisuals::POINT_NODE_RADIUS

// HoveredBendPoint, BendPointPreview, HoveredSnapPoint are now in DemoState.h

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

        // Initialize extracted components
        inputHandler_ = std::make_unique<DemoInputHandler>(manualManager_);
        commandProcessor_ = std::make_unique<DemoCommandProcessor>(
            commandServer_, manualManager_, routingCoordinator_);

        // Set up input handler callbacks
        inputHandler_->setDoLayoutCallback([this]() { doLayout(); });
        inputHandler_->setRerouteEdgesCallback([this]() { rerouteAffectedEdges(); });
        inputHandler_->setStartDragCallback([this](NodeId nodeId) {
            startNodeDrag(nodeId);
        });

        // Set up command processor callbacks
        commandProcessor_->setGetLogCallback([](const std::string& pattern, int tailLines) {
            // Get logs from library API
            auto loggerLogs = Logger::getCapturedLogs(pattern, static_cast<size_t>(tailLines));
            
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
            
            if (static_cast<int>(allLines.size()) > tailLines) {
                allLines.erase(allLines.begin(), allLines.begin() + (allLines.size() - tailLines));
            }
            
            if (allLines.empty()) {
                return std::string("LOG_EMPTY");
            }
            
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
            return result.str();
        });
        commandProcessor_->setClearLogCallback([]() {
            Logger::clearCapturedLogs();
        });
        commandProcessor_->setWaitIdleCallback([this](int timeoutMs) {
            return waitForOptimizationIdle(timeoutMs);
        });
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

    /// Start node drag operation (called when drag threshold is exceeded)
    void startNodeDrag(NodeId nodeId) {
        draggedNode_ = nodeId;
        auto& layout = nodeLayouts_[draggedNode_];
        affectedEdges_ = graph_.getConnectedEdges(draggedNode_);

        // Initialize drag constraint state
        lastValidPosition_ = layout.position;
        isInvalidDragPosition_ = false;

        // Create constraint manager for drag validation
        LOG_DEBUG("[Demo::startNodeDrag] Creating constraint manager");
        auto defaultConfig = ConstraintConfig::createDefault(&layoutOptions_);
        constraintManager_ = ConstraintFactory::create(defaultConfig);

        // Pre-calculate blocked regions for visualization
        ConstraintContext ctx{draggedNode_, layout.position, nodeLayouts_, edgeLayouts_, &graph_, gridSize_};
        blockedRegions_ = constraintManager_->getAllBlockedRegions(ctx);

        // Sync LayoutController state before drag
        if (layoutController_) {
            layoutController_->initializeFrom(nodeLayouts_, edgeLayouts_);
        }

        // Notify coordinator that drag started
        routingCoordinator_->onDragStart(affectedEdges_);
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
            // Handle click for node selection (drag starts after threshold)
            selectedNode_ = hoveredNode_;
            selectedEdge_ = INVALID_EDGE;
            selectedBendPoint_.clear();
            // Store click position for drag threshold detection
            pendingNodeDrag_ = true;
            nodeClickStart_ = {io.MousePos.x, io.MousePos.y};
            auto& layout = nodeLayouts_[hoveredNode_];
            dragOffset_ = {graphMouse.x - layout.position.x,
                          graphMouse.y - layout.position.y};
        } else if (ImGui::IsMouseClicked(0) && hoveredEdge_ != INVALID_EDGE) {
            selectedEdge_ = hoveredEdge_;
            selectedNode_ = INVALID_NODE;
            selectedBendPoint_.clear();
        } else if (ImGui::IsMouseClicked(0) && hoveredNode_ == INVALID_NODE && hoveredEdge_ == INVALID_EDGE && !hoveredBendPoint_.isValid() && !hoveredSnapPoint_.isValid()) {
            // Left click on empty area - start potential pan
            emptyAreaPanStarted_ = true;
        }

        // Handle pending node drag - check if threshold exceeded to start actual drag
        if (pendingNodeDrag_ && ImGui::IsMouseDragging(ImGuiMouseButton_Left)) {
            float dx = io.MousePos.x - nodeClickStart_.x;
            float dy = io.MousePos.y - nodeClickStart_.y;
            float distance = std::sqrt(dx * dx + dy * dy);

            if (distance > DRAG_THRESHOLD) {
                pendingNodeDrag_ = false;
                startNodeDrag(selectedNode_);
            }
        }

        // Handle empty area pan (left-drag on empty)
        if (emptyAreaPanStarted_ && ImGui::IsMouseDragging(ImGuiMouseButton_Left)) {
            panOffset_.x += io.MouseDelta.x / zoom_;
            panOffset_.y += io.MouseDelta.y / zoom_;
            isPanning_ = true;
        }

        if (ImGui::IsMouseReleased(0)) {
            // Clear pending drag state (click without exceeding threshold = just selection)
            pendingNodeDrag_ = false;

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

    void render(ImDrawList* drawList) {
        // Get state for DemoRenderer calls
        ViewTransform view = getViewTransform();
        RenderOptions options = getRenderOptions();
        ImGuiIO& io = ImGui::GetIO();
        ImDrawList* fgDrawList = ImGui::GetForegroundDrawList();

        // Draw grid background
        DemoRenderer::drawGrid(drawList, view, io.DisplaySize.x, io.DisplaySize.y, gridSize_);

        // Draw blocked cells (node obstacle areas)
        if (options.showBlockedCells) {
            DemoRenderer::drawBlockedCells(drawList, view, blockedRegions_, gridSize_);
        }

        // Draw A* debug visualization
        if (options.showAStarDebug) {
            updateAStarDebugState();
            DemoRenderer::drawAStarDebug(drawList, view, getAStarDebugState(), gridSize_);
        }

        // Draw edges
        for (const auto& [id, layout] : edgeLayouts_) {
            bool isAffected = std::find(affectedEdges_.begin(), affectedEdges_.end(), id)
                             != affectedEdges_.end();

            // Skip logic for edge visibility
            if (isAffected && isInvalidDragPosition_) continue;
            if (isAffected && layoutOptions_.optimizationOptions.dragAlgorithm == DragAlgorithm::HideUntilDrop) continue;
            if (draggingSnapPoint_.isValid() && draggingSnapPoint_.edgeId == id && hasSnapPreview_) continue;

            bool isSelected = (id == selectedEdge_);
            bool isHovered = (id == hoveredEdge_);
            const EdgeData* edgeData = &graph_.getEdge(id);

            DemoRenderer::drawEdge(drawList, view, layout, edgeData, isSelected, isHovered, isAffected);
        }

        // Draw bend point insertion preview
        DemoRenderer::drawBendPointPreview(drawList, view, bendPointPreview_);

        // Draw nodes
        for (const auto& [id, layout] : nodeLayouts_) {
            bool isSelected = (id == selectedNode_);
            bool isHovered = (id == hoveredNode_);
            bool isDragged = (id == draggedNode_);
            const NodeData* nodeData = &graph_.getNode(id);
            test::scxml::SCXMLGraph* scxmlGraph = scxmlModeActive_ ? scxmlGraph_.get() : nullptr;

            DemoRenderer::drawNode(drawList, view, id, layout, nodeData,
                                   isSelected, isHovered, isDragged, isInvalidDragPosition_,
                                   scxmlGraph);

            // Draw snap points per node
            if (showSnapPoints_) {
                DemoRenderer::drawSnapPoints(drawList, fgDrawList, view, layout,
                                            edgeLayouts_, nodeLayouts_, options,
                                            hoveredSnapPoint_, draggingSnapPoint_);
            }
        }

        // Draw snap point candidates during drag
        if (draggingSnapPoint_.isValid() && snapController_.isDragging()) {
            DemoRenderer::drawSnapCandidates(drawList, view, snapController_, snappedCandidateIndex_);
        }

        // Draw A* path preview during snap point drag
        if (hasSnapPreview_ && draggingSnapPoint_.isValid()) {
            DemoRenderer::drawSnapPreviewPath(drawList, view, snapController_);
        }
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

        // Selected node info and type toggle
        if (selectedNode_ != INVALID_NODE && draggedNode_ == INVALID_NODE) {
            auto nodeIt = nodeLayouts_.find(selectedNode_);
            if (nodeIt != nodeLayouts_.end()) {
                const auto& layout = nodeIt->second;
                auto graphNode = graph_.tryGetNode(selectedNode_);
                
                ImGui::Separator();
                ImGui::TextColored(ImVec4(0.5f, 1.0f, 0.5f, 1),
                    "Selected: %s", graphNode ? graphNode->label.c_str() : "Unknown");
                
                bool isPoint = layout.isPointNode();
                ImGui::Text("Type: %s", isPoint ? "Point (dot)" : "Normal (rect)");
                ImGui::Text("Position: (%.0f, %.0f)", layout.position.x, layout.position.y);
                if (!isPoint) {
                    ImGui::Text("Size: %.0f x %.0f", layout.size.width, layout.size.height);
                }
                
                // Node type toggle button
                const char* buttonLabel = isPoint ? "To Normal Node" : "To Point Node";
                if (ImGui::Button(buttonLabel)) {
                    NodeType newType = isPoint ? NodeType::Regular : NodeType::Point;
                    if (layoutController_) {
                        layoutController_->initializeFrom(nodeLayouts_, edgeLayouts_);
                        auto result = layoutController_->setNodeType(selectedNode_, newType);
                        if (result.success) {
                            // Sync back from controller
                            for (const auto& [id, nl] : layoutController_->nodeLayouts()) {
                                nodeLayouts_[id] = nl;
                            }
                            for (const auto& [id, el] : layoutController_->edgeLayouts()) {
                                edgeLayouts_[id] = el;
                            }
                            std::cout << "[Demo] Node " << selectedNode_ 
                                      << " converted to " << (newType == NodeType::Point ? "Point" : "Normal") 
                                      << std::endl;
                        } else {
                            std::cout << "[Demo] Type conversion failed: " << result.reason << std::endl;
                        }
                    }
                }
            }
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

    // Extracted components for separation of concerns
    std::unique_ptr<DemoInputHandler> inputHandler_;
    std::unique_ptr<DemoCommandProcessor> commandProcessor_;

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

    /// Get current view transform for DemoRenderer calls
    ViewTransform getViewTransform() const {
        ViewTransform vt;
        vt.panOffset = panOffset_;
        vt.zoom = zoom_;
        vt.screenOffset = offset_;
        return vt;
    }

    /// Get render options for DemoRenderer calls
    RenderOptions getRenderOptions() const {
        RenderOptions opts;
        opts.showSnapPoints = showSnapPoints_;
        opts.showSnapIndices = showSnapIndices_;
        opts.showBlockedCells = showBlockedCells_;
        opts.showAStarDebug = showAStarDebug_;
        opts.gridSize = gridSize_;
        return opts;
    }

    /// Get A* debug state for DemoRenderer calls
    AStarDebugState getAStarDebugState() const {
        AStarDebugState debug;
        debug.debugEdgeId = debugEdgeId_;
        debug.obstacles = debugObstacles_;
        debug.start = astarStart_;
        debug.goal = astarGoal_;
        return debug;
    }

    /// Update A* debug state (obstacle map, start/goal points)
    void updateAStarDebugState() {
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
    }

    /// Get DemoState for passing to extracted components (DemoInputHandler, etc.)
    /// Note: This creates a view into InteractiveDemo's state, not a copy.
    DemoState getDemoState() {
        DemoState state;
        
        // Core data (non-owning pointers)
        state.graph = &graph_;
        state.nodeLayouts = &nodeLayouts_;
        state.edgeLayouts = &edgeLayouts_;
        state.layoutOptions = &layoutOptions_;
        
        // Controllers (non-owning pointers)
        state.layoutController = layoutController_.get();
        state.routingCoordinator = routingCoordinator_.get();
        state.constraintManager = constraintManager_.get();
        state.snapController = &snapController_;
        
        // View transform
        state.view.panOffset = panOffset_;
        state.view.zoom = zoom_;
        state.view.screenOffset = offset_;
        
        // Interaction state
        state.interaction.selectedNode = selectedNode_;
        state.interaction.selectedEdge = selectedEdge_;
        state.interaction.selectedBendPoint = selectedBendPoint_;
        state.interaction.hoveredNode = hoveredNode_;
        state.interaction.hoveredEdge = hoveredEdge_;
        state.interaction.hoveredBendPoint = hoveredBendPoint_;
        state.interaction.hoveredSnapPoint = hoveredSnapPoint_;
        state.interaction.draggedNode = draggedNode_;
        state.interaction.dragOffset = dragOffset_;
        state.interaction.affectedEdges = affectedEdges_;
        state.interaction.isInvalidDragPosition = isInvalidDragPosition_;
        state.interaction.lastValidPosition = lastValidPosition_;
        state.interaction.lastRoutedPosition = lastRoutedPosition_;
        state.interaction.pendingNodeDrag = pendingNodeDrag_;
        state.interaction.nodeClickStart = nodeClickStart_;
        state.interaction.draggingBendPoint = draggingBendPoint_;
        state.interaction.bendPointDragOffset = bendPointDragOffset_;
        state.interaction.bendPointPreview = bendPointPreview_;
        state.interaction.draggingSnapPoint = draggingSnapPoint_;
        state.interaction.snapPointDragOffset = snapPointDragOffset_;
        state.interaction.snapPointDragStart = snapPointDragStart_;
        state.interaction.snappedCandidateIndex = snappedCandidateIndex_;
        state.interaction.hasSnapPreview = hasSnapPreview_;
        state.interaction.isPanning = isPanning_;
        state.interaction.emptyAreaPanStarted = emptyAreaPanStarted_;
        
        // Render options
        state.renderOptions.showSnapPoints = showSnapPoints_;
        state.renderOptions.showSnapIndices = showSnapIndices_;
        state.renderOptions.showBlockedCells = showBlockedCells_;
        state.renderOptions.showAStarDebug = showAStarDebug_;
        state.renderOptions.gridSize = gridSize_;
        
        // A* debug state
        state.astarDebug.debugEdgeId = debugEdgeId_;
        state.astarDebug.obstacles = debugObstacles_;
        state.astarDebug.start = astarStart_;
        state.astarDebug.goal = astarGoal_;
        
        // Blocked regions
        state.blockedRegions = blockedRegions_;
        
        return state;
    }

    /// Update interaction state from external source (for extracted components)
    void setInteractionState(const InteractionState& state) {
        selectedNode_ = state.selectedNode;
        selectedEdge_ = state.selectedEdge;
        selectedBendPoint_ = state.selectedBendPoint;
        hoveredNode_ = state.hoveredNode;
        hoveredEdge_ = state.hoveredEdge;
        hoveredBendPoint_ = state.hoveredBendPoint;
        hoveredSnapPoint_ = state.hoveredSnapPoint;
        draggedNode_ = state.draggedNode;
        dragOffset_ = state.dragOffset;
        affectedEdges_ = state.affectedEdges;
        isInvalidDragPosition_ = state.isInvalidDragPosition;
        lastValidPosition_ = state.lastValidPosition;
        lastRoutedPosition_ = state.lastRoutedPosition;
        pendingNodeDrag_ = state.pendingNodeDrag;
        nodeClickStart_ = state.nodeClickStart;
        draggingBendPoint_ = state.draggingBendPoint;
        bendPointDragOffset_ = state.bendPointDragOffset;
        bendPointPreview_ = state.bendPointPreview;
        draggingSnapPoint_ = state.draggingSnapPoint;
        snapPointDragOffset_ = state.snapPointDragOffset;
        snapPointDragStart_ = state.snapPointDragStart;
        snappedCandidateIndex_ = state.snappedCandidateIndex;
        hasSnapPreview_ = state.hasSnapPreview;
        isPanning_ = state.isPanning;
        emptyAreaPanStarted_ = state.emptyAreaPanStarted;
    }

    /// Update view transform from external source
    void setViewTransform(const ViewTransform& view) {
        panOffset_ = view.panOffset;
        zoom_ = view.zoom;
        offset_ = view.screenOffset;
    }

    /// Update render options from external source
    void setRenderOptions(const RenderOptions& opts) {
        showSnapPoints_ = opts.showSnapPoints;
        showSnapIndices_ = opts.showSnapIndices;
        showBlockedCells_ = opts.showBlockedCells;
        showAStarDebug_ = opts.showAStarDebug;
        // Note: gridSize_ is not updated from RenderOptions as it's a layout parameter
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

    // Drag threshold detection (distinguish click from drag)
    static constexpr float DRAG_THRESHOLD = 5.0f;  // Pixels before drag starts
    bool pendingNodeDrag_ = false;                 // Clicked but not yet dragging
    Point nodeClickStart_ = {0, 0};               // Screen position at click

    // Drag constraint state
    bool isInvalidDragPosition_ = false;      // Current drag position is invalid
    Point lastValidPosition_ = {0, 0};        // Last known valid position during drag
    Point lastRoutedPosition_ = {-9999, -9999};  // Last position where edges were routed
    std::unique_ptr<IConstraintValidator> constraintManager_;  // Constraint validator for drag validation
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

    /// Wait for optimization to become idle (with timeout)
    /// @param timeoutMs Timeout in milliseconds
    /// @return true if idle, false if timeout
    bool waitForOptimizationIdle(int timeoutMs) {
        auto startTime = std::chrono::steady_clock::now();
        while (routingCoordinator_->state() != RoutingState::Idle) {
            auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now() - startTime).count();
            if (elapsed >= timeoutMs) {
                return false;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        return true;
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
