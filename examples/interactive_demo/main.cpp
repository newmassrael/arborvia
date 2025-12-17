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
#include "SCXMLManager.h"
#include "AsyncOptimizer.h"

using arborvia::test::scxml::TestInfo;

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
        // Enable grid by default (use state_.renderOptions.gridSize as source of truth)
        state_.renderOptions.gridSize = 20.0f;
        layoutOptions_.gridConfig.cellSize = state_.renderOptions.gridSize;

        // Configure routing coordinator with async mode
        routingCoordinator_->setDebounceDelay(0);  // Immediate A* after drop (no delay)
        routingCoordinator_->setListener(this);
        routingCoordinator_->setAsyncMode(true);  // Enable async optimization
        
        // Initialize async optimizer
        asyncOptimizer_ = std::make_unique<AsyncOptimizer>(routingCoordinator_);
        asyncOptimizer_->setGetSnapshotCallback([this]() {
            return AsyncOptimizer::LayoutSnapshot{edgeLayouts_, nodeLayouts_, layoutOptions_};
        });
        asyncOptimizer_->setApplyResultCallback([this](const auto& result) {
            for (const auto& [id, layout] : result) {
                edgeLayouts_[id] = layout;
            }
        });

        // Sync callback (used internally, not for async result delivery)
        routingCoordinator_->setOptimizationCallback(
            [this](const std::vector<EdgeId>& /*affectedEdges*/,
                   const std::unordered_set<NodeId>& movedNodes) {
            // In async mode, this callback triggers async optimization start
            asyncOptimizer_->startOptimization(movedNodes);
        });

        setupGraph();

        // Initialize centralized state pointers BEFORE doLayout()
        // (doLayout sets state_.layoutController and state_.view.screenOffset)
        state_.graph = &graph_;
        state_.nodeLayouts = &nodeLayouts_;
        state_.edgeLayouts = &edgeLayouts_;
        state_.layoutOptions = &layoutOptions_;
        state_.routingCoordinator = routingCoordinator_.get();
        state_.snapController = &snapController_;

        doLayout();  // Sets state_.layoutController and state_.view.screenOffset

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
            ConstraintContext ctx{nodeId, proposedPos, nodeLayouts_, edgeLayouts_, &graph_, state_.renderOptions.gridSize};
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
            return asyncOptimizer_->waitForIdle(timeoutMs);
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
            oss << "SCXML_INFO index=" << state_.scxml.currentTestIndex
                << " id=" << info->id
                << " file=" << info->file
                << " desc=" << info->description;
            return oss.str();
        });

        // Initialize SCXML manager (before UI panel callbacks that use it)
        scxmlManager_ = std::make_unique<SCXMLManager>(state_.scxml);
        scxmlManager_->setOnTestLoadedCallback([this](Graph& graph) {
            graph_ = graph;
            manualManager_->clearManualState();
            doLayout();
        });
        scxmlManager_->setOnModeExitCallback([this]() {
            graph_ = Graph();
            setupGraph();
            doLayout();
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
                
                // Update graph label when converting Point → Regular
                if (newType == NodeType::Regular) {
                    // Use SCXML ID if available, otherwise use node ID
                    std::string label = scxmlManager_ ? scxmlManager_->getScxmlId(nodeId) : "";
                    if (label.empty()) {
                        label = "Node " + std::to_string(static_cast<int>(nodeId));
                    }
                    graph_.getNode(nodeId).label = label;
                } else {
                    // Point nodes don't show label
                    graph_.getNode(nodeId).label = "";
                }
                
                return true;
            }
            std::cout << "[Demo] Type conversion failed: " << result.reason << std::endl;
            return false;
        });
        
        // Check if a node can be converted to Point (only Final allowed)
        uiPanel_->setCanConvertToPointCallback([this](NodeId nodeId) -> bool {
            return scxmlManager_->canConvertToPoint(nodeId);
        });

        // Check if a node can be converted to Regular (only Final allowed)
        uiPanel_->setCanConvertToRegularCallback([this](NodeId nodeId) -> bool {
            return scxmlManager_->canConvertToRegular(nodeId);
        });
    }

    ~InteractiveDemo() {
        // Ensure all async tasks complete before destroying this
        if (asyncOptimizer_) {
            asyncOptimizer_->shutdown();
        }
    }

    // IRoutingListener implementation
    void onOptimizationComplete(const std::vector<EdgeId>& optimizedEdges) override {
        std::cout << "Optimization complete: " << optimizedEdges.size() << " edges re-routed" << std::endl;
        // Clear affected edges now that optimization is complete - edges become visible
        state_.interaction.affectedEdges.clear();
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
        state_.interaction.draggedNode = nodeId;
        auto& layout = nodeLayouts_[state_.interaction.draggedNode];
        state_.interaction.affectedEdges = graph_.getConnectedEdges(state_.interaction.draggedNode);

        // Initialize drag constraint state
        state_.interaction.lastValidPosition = layout.position;
        state_.interaction.isInvalidDragPosition = false;

        // Create constraint manager for drag validation
        LOG_DEBUG("[Demo::startNodeDrag] Creating constraint manager");
        auto defaultConfig = ConstraintConfig::createDefault(&layoutOptions_);
        constraintManager_ = ConstraintFactory::create(defaultConfig);

        // Pre-calculate blocked regions for visualization
        ConstraintContext ctx{state_.interaction.draggedNode, layout.position, nodeLayouts_, edgeLayouts_, &graph_, state_.renderOptions.gridSize};
        state_.blockedRegions = constraintManager_->getAllBlockedRegions(ctx);

        // Sync LayoutController state before drag
        if (layoutController_) {
            layoutController_->initializeFrom(nodeLayouts_, edgeLayouts_);
        }

        // Notify coordinator that drag started
        routingCoordinator_->onDragStart(state_.interaction.affectedEdges);
    }

    /// End node drag operation (called when mouse is released)
    void endNodeDrag(NodeId nodeId) {
        // Signal drag end - coordinator will schedule optimization after delay
        routingCoordinator_->onDragEnd({nodeId});
        
        // Clear drag-only resources
        state_.blockedRegions.clear();
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
        state_.layoutController = layoutController_.get();

        // Offset for centering
        state_.view.screenOffset = {100.0f, 50.0f};
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
        InputResult result = inputHandler_->processInput(state_, io);
        
        // State changes are applied directly via state_ reference
        
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
                  << " edges with " << algoName << " (draggedNode=" << state_.interaction.draggedNode << ")" << std::endl;
        std::unordered_set<NodeId> movedNodes = {state_.interaction.draggedNode};
        LayoutUtils::updateEdgePositions(
            edgeLayouts_, nodeLayouts_, allEdges,
            layoutOptions_, movedNodes);
    }

    void render(ImDrawList* drawList) {
        // Get state for DemoRenderer calls
        const ViewTransform& view = state_.view;
        const RenderOptions& options = state_.renderOptions;
        const InteractionState& interaction = state_.interaction;
        ImGuiIO& io = ImGui::GetIO();
        ImDrawList* fgDrawList = ImGui::GetForegroundDrawList();

        // Draw grid background
        DemoRenderer::drawGrid(drawList, view, io.DisplaySize.x, io.DisplaySize.y, options.gridSize);

        // Draw blocked cells (node obstacle areas) - only during active drag
        if (options.showBlockedCells && interaction.draggedNode != INVALID_NODE) {
            DemoRenderer::drawBlockedCells(drawList, view, state_.blockedRegions, options.gridSize);
        }

        // Draw A* debug visualization
        if (options.showAStarDebug) {
            updateAStarDebugState();
            DemoRenderer::drawAStarDebug(drawList, view, state_.astarDebug, options.gridSize);
        }

        // Draw edges
        for (const auto& [id, layout] : edgeLayouts_) {
            bool isAffected = std::find(interaction.affectedEdges.begin(), interaction.affectedEdges.end(), id)
                             != interaction.affectedEdges.end();

            // Skip logic for edge visibility
            if (isAffected && interaction.isInvalidDragPosition) continue;
            if (isAffected && layoutOptions_.optimizationOptions.dragAlgorithm == DragAlgorithm::HideUntilDrop) continue;
            if (interaction.draggingSnapPoint.isValid() && interaction.draggingSnapPoint.edgeId == id && interaction.hasSnapPreview) continue;

            bool isSelected = (id == interaction.selectedEdge);
            bool isHovered = (id == interaction.hoveredEdge);
            const EdgeData* edgeData = &graph_.getEdge(id);

            DemoRenderer::drawEdge(drawList, view, layout, edgeData, isSelected, isHovered, isAffected);
        }

        // Draw bend point insertion preview
        DemoRenderer::drawBendPointPreview(drawList, view, interaction.bendPointPreview);

        // Draw nodes
        for (const auto& [id, layout] : nodeLayouts_) {
            bool isSelected = (id == interaction.selectedNode);
            bool isHovered = (id == interaction.hoveredNode);
            bool isDragged = (id == interaction.draggedNode);
            const NodeData* nodeData = &graph_.getNode(id);
            test::scxml::SCXMLGraph* scxmlGraph = state_.scxml.modeActive && scxmlManager_ ? scxmlManager_->getCurrentGraph() : nullptr;

            DemoRenderer::drawNode(drawList, view, id, layout, nodeData,
                                   isSelected, isHovered, isDragged, interaction.isInvalidDragPosition,
                                   scxmlGraph);

            // Draw snap points per node
            if (options.showSnapPoints) {
                // Hide snap points for affected edges:
                // 1. In HideUntilDrop mode (edges hidden during drag)
                // 2. When in invalid drag position (edges hidden to indicate error)
                const std::vector<EdgeId>& hiddenEdges =
                    (layoutOptions_.optimizationOptions.dragAlgorithm == DragAlgorithm::HideUntilDrop ||
                     interaction.isInvalidDragPosition)
                    ? interaction.affectedEdges : std::vector<EdgeId>{};
                DemoRenderer::drawSnapPoints(drawList, fgDrawList, view, layout,
                                            edgeLayouts_, nodeLayouts_, options,
                                            interaction.hoveredSnapPoint, interaction.draggingSnapPoint,
                                            hiddenEdges);
            }
        }

        // Draw snap point candidates during drag
        if (interaction.draggingSnapPoint.isValid() && snapController_.isDragging()) {
            DemoRenderer::drawSnapCandidates(drawList, view, snapController_, interaction.snappedCandidateIndex);
        }

        // Draw A* path preview during snap point drag
        if (interaction.hasSnapPreview && interaction.draggingSnapPoint.isValid()) {
            DemoRenderer::drawSnapPreviewPath(drawList, view, snapController_);
        }
    }

    void renderUI() {
        // Set SCXML loader on first call (after initSCXML may have been called)
        if (scxmlManager_ && scxmlManager_->getLoader() && uiPanel_) {
            uiPanel_->setSCXMLLoader(scxmlManager_->getLoader());
        }

        // UI modifies state_ directly through reference
        [[maybe_unused]] UIResult result = uiPanel_->render(state_);

        // State changes are applied directly via state_ reference

        // Handle A* debug state update when checkbox changes
        if (state_.renderOptions.showAStarDebug != prevShowAStarDebug_) {
            prevShowAStarDebug_ = state_.renderOptions.showAStarDebug;
            if (state_.renderOptions.showAStarDebug) {
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

    // Centralized state (single source of truth)
    DemoState state_;

    /// Update A* debug state (obstacle map, start/goal points)
    void updateAStarDebugState() {
        // Rebuild obstacle map every frame to reflect current state
        state_.astarDebug.obstacles = std::make_shared<ObstacleMap>();
        state_.astarDebug.obstacles->buildFromNodes(nodeLayouts_, state_.renderOptions.gridSize);
        state_.astarDebug.obstacles->addEdgeSegments(edgeLayouts_, state_.astarDebug.debugEdgeId);

        // Update start/goal from selected or debug edge
        EdgeId targetEdge = (state_.astarDebug.debugEdgeId != INVALID_EDGE) 
            ? state_.astarDebug.debugEdgeId 
            : state_.interaction.selectedEdge;
        if (targetEdge != INVALID_EDGE) {
            auto it = edgeLayouts_.find(targetEdge);
            if (it != edgeLayouts_.end()) {
                state_.astarDebug.start = it->second.sourcePoint;
                state_.astarDebug.goal = it->second.targetPoint;
            }
        }
    }

    /// Get DemoState for passing to extracted components (DemoInputHandler, etc.)
    /// Returns reference to internal state_ (single source of truth).
    DemoState& getDemoState() {
        return state_;
    }

    // Snap point controller (handles drag preview, swap, etc.)
    SnapPointController snapController_;

    // Constraint validator for drag validation (created on drag start, cleared on drag end)
    std::unique_ptr<IConstraintValidator> constraintManager_;

    // A* debug state change tracking
    bool prevShowAStarDebug_ = false;

    // External command server
    CommandServer commandServer_;
    int port_;


    // Async optimizer
    std::unique_ptr<AsyncOptimizer> asyncOptimizer_;

    // SCXML test manager
    std::unique_ptr<SCXMLManager> scxmlManager_;

public:
    // Start command server
    bool startCommandServer() {
        return commandServer_.start();
    }

    // Process pending commands (call from main loop)
    void processCommands() {
        while (commandServer_.hasCommand()) {
            Command cmd = commandServer_.popCommand();
            
            // Process command - state changes are applied directly via state_ reference
            commandProcessor_->processCommand(cmd, state_);
        }
    }

    bool isPaused() const { return commandProcessor_->isPaused(); }
    const std::string& pauseMessage() const { return commandProcessor_->pauseMessage(); }
    bool shouldQuit() const { return commandProcessor_->shouldQuit(); }
    int port() const { return port_; }

    // SCXML wrapper methods (delegate to SCXMLManager)
    bool initSCXML(const std::string& basePath) {
        return scxmlManager_ && scxmlManager_->init(basePath);
    }
    bool loadSCXMLTest(size_t index) {
        return scxmlManager_ && scxmlManager_->loadTest(index);
    }
    void nextTest() {
        if (scxmlManager_) scxmlManager_->nextTest();
    }
    void prevTest() {
        if (scxmlManager_) scxmlManager_->prevTest();
    }
    void exitSCXMLMode() {
        if (scxmlManager_) scxmlManager_->exitMode();
    }
    bool isSCXMLModeActive() const {
        return scxmlManager_ && scxmlManager_->isModeActive();
    }
    size_t getSCXMLTestCount() const {
        return scxmlManager_ ? scxmlManager_->getTestCount() : 0;
    }
    const TestInfo* getCurrentTestInfo() const {
        return scxmlManager_ ? scxmlManager_->getCurrentTestInfo() : nullptr;
    }

    // Process async optimization results (call from main loop)
    void processMainThreadQueue() {
        if (asyncOptimizer_) {
            asyncOptimizer_->processMainThreadQueue();
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
