#include <SDL3/SDL.h>
#include <imgui.h>
#include <imgui_impl_sdl3.h>
#include <imgui_impl_sdlrenderer3.h>

#include "DemoState.h"
#include "DemoRenderer.h"
#include "DemoColors.h"
#include "DemoInputHandler.h"
#include "DemoCommandProcessor.h"
#include "DemoUIPanel.h"
#include "LogCapture.h"

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
        inputHandler_->setEndDragCallback([this](NodeId nodeId) {
            endNodeDrag(nodeId);
        });
        inputHandler_->setValidateDragCallback([this](NodeId nodeId, Point proposedPos) -> bool {
            if (!constraintManager_) return true;
            ConstraintContext ctx{nodeId, proposedPos, nodeLayouts_, edgeLayouts_, &graph_, gridSize_};
            auto validation = constraintManager_->validate(ctx);
            return validation.valid;
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
            LogCapture::instance().clear();
        });
        commandProcessor_->setWaitIdleCallback([this](int timeoutMs) {
            return waitForOptimizationIdle(timeoutMs);
        });

        // SCXML callbacks
        commandProcessor_->setInitSCXMLCallback([this](const std::string& path) {
            return initSCXML(path);
        });
        commandProcessor_->setLoadSCXMLTestCallback([this](int index) {
            return loadSCXMLTest(static_cast<size_t>(index));
        });
        commandProcessor_->setNextTestCallback([this]() {
            nextTest();
        });
        commandProcessor_->setPrevTestCallback([this]() {
            prevTest();
        });
        commandProcessor_->setGetSCXMLTestCountCallback([this]() {
            return static_cast<int>(getSCXMLTestCount());
        });
        commandProcessor_->setGetCurrentTestInfoCallback([this]() {
            const auto* info = getCurrentTestInfo();
            if (!info) {
                return std::string("ERROR no test loaded");
            }
            std::ostringstream oss;
            oss << "SCXML_INFO index=" << currentTestIndex_
                << " id=" << info->id
                << " file=" << info->file
                << " desc=" << info->description;
            return oss.str();
        });

        // Initialize UI panel
        uiPanel_ = std::make_unique<DemoUIPanel>(manualManager_);
        uiPanel_->setDoLayoutCallback([this]() { doLayout(); });
        uiPanel_->setReRouteEdgesCallback([this]() { reRouteEdgesOnly(); });
        uiPanel_->setSaveLayoutCallback([this](const std::string& path) { saveLayout(path); });
        uiPanel_->setLoadLayoutCallback([this](const std::string& path) { loadLayout(path); });
        uiPanel_->setLoadSCXMLTestCallback([this](size_t index) { loadSCXMLTest(index); });
        uiPanel_->setNextTestCallback([this]() { nextTest(); });
        uiPanel_->setPrevTestCallback([this]() { prevTest(); });
        uiPanel_->setExitSCXMLModeCallback([this]() { exitSCXMLMode(); });
        uiPanel_->setSetNodeTypeCallback([this](NodeId nodeId, NodeType newType) -> bool {
            if (!layoutController_) return false;
            layoutController_->initializeFrom(nodeLayouts_, edgeLayouts_);
            auto result = layoutController_->setNodeType(nodeId, newType);
            if (result.success) {
                for (const auto& [id, nl] : layoutController_->nodeLayouts()) {
                    nodeLayouts_[id] = nl;
                }
                for (const auto& [id, el] : layoutController_->edgeLayouts()) {
                    edgeLayouts_[id] = el;
                }
                return true;
            }
            std::cout << "[Demo] Type conversion failed: " << result.reason << std::endl;
            return false;
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

    /// End node drag operation (called when mouse is released)
    void endNodeDrag(NodeId nodeId) {
        // Signal drag end - coordinator will schedule optimization after delay
        routingCoordinator_->onDragEnd({nodeId});
        
        // Clear drag-only resources
        blockedRegions_.clear();
        constraintManager_.reset();
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

        // Process input via extracted handler
        ImGuiIO& io = ImGui::GetIO();
        DemoState state = getDemoState();
        InputResult result = inputHandler_->processInput(state, io);
        
        // Sync back state changes
        setInteractionState(state.interaction);
        setViewTransform(state.view);
        
        // Handle requested actions
        switch (result.action) {
            case InputAction::DoLayout:
                doLayout();
                break;
            case InputAction::RerouteEdges:
                rerouteAffectedEdges();
                break;
            case InputAction::StartAsyncOptimization:
                // This is typically handled via callbacks already
                break;
            case InputAction::None:
            default:
                break;
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

        // Draw blocked cells (node obstacle areas) - only during active drag
        if (options.showBlockedCells && draggedNode_ != INVALID_NODE) {
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
                // Hide snap points for affected edges:
                // 1. In HideUntilDrop mode (edges hidden during drag)
                // 2. When in invalid drag position (edges hidden to indicate error)
                const std::vector<EdgeId>& hiddenEdges =
                    (layoutOptions_.optimizationOptions.dragAlgorithm == DragAlgorithm::HideUntilDrop ||
                     isInvalidDragPosition_)
                    ? affectedEdges_ : std::vector<EdgeId>{};
                DemoRenderer::drawSnapPoints(drawList, fgDrawList, view, layout,
                                            edgeLayouts_, nodeLayouts_, options,
                                            hoveredSnapPoint_, draggingSnapPoint_,
                                            hiddenEdges);
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
        // Set SCXML loader on first call (after initSCXML may have been called)
        if (scxmlLoader_ && uiPanel_) {
            uiPanel_->setSCXMLLoader(scxmlLoader_.get());
        }

        DemoState state = getDemoState();
        [[maybe_unused]] UIResult result = uiPanel_->render(state);

        // Sync back state changes from UI
        setViewTransform(state.view);
        setRenderOptions(state.renderOptions);

        // Handle A* debug state update when checkbox changes
        if (state.renderOptions.showAStarDebug != showAStarDebug_) {
            showAStarDebug_ = state.renderOptions.showAStarDebug;
            if (showAStarDebug_) {
                updateAStarDebugState();
            }
        }
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
    std::unique_ptr<DemoUIPanel> uiPanel_;

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

        // SCXML state
        state.scxml.modeActive = scxmlModeActive_;
        state.scxml.currentTestIndex = currentTestIndex_;
        
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
        // affectedEdges_ sync rules:
        // - For node drag: set by startNodeDrag() callback, cleared by onOptimizationComplete()
        // - For snap point drag: sync from state when drag ends (detected by draggingSnapPoint transition)
        if (draggingSnapPoint_.isValid() && !state.draggingSnapPoint.isValid()) {
            // Snap point drag just ended - sync affectedEdges for optimization
            affectedEdges_ = state.affectedEdges;
        }
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
            
            // Create state view and process via commandProcessor_
            DemoState state = getDemoState();
            if (commandProcessor_->processCommand(cmd, state)) {
                // Sync back interaction state changes
                setInteractionState(state.interaction);
            }
        }
    }

    bool isPaused() const { return commandProcessor_->isPaused(); }
    const std::string& pauseMessage() const { return commandProcessor_->pauseMessage(); }
    bool shouldQuit() const { return commandProcessor_->shouldQuit(); }
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
