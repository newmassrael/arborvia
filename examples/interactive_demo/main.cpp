#include <SDL3/SDL.h>
#include <imgui.h>
#include <imgui_impl_sdl3.h>
#include <imgui_impl_sdlrenderer3.h>

#include <arborvia/arborvia.h>

#include <unordered_map>
#include <unordered_set>
#include <algorithm>
#include <string>
#include <cmath>
#include <fstream>
#include <map>
#include <iostream>

using namespace arborvia;

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

class InteractiveDemo {
public:
    InteractiveDemo() {
        setupGraph();
        doLayout();
    }
    
    LayoutMode getLayoutMode() const { return manualManager_.getMode(); }
    
    void setLayoutMode(LayoutMode mode) {
        if (mode == LayoutMode::Manual && manualManager_.getMode() == LayoutMode::Auto) {
            // Switching to Manual: capture current layout
            manualManager_.captureFromResult(layoutResult_);
        }
        manualManager_.setMode(mode);
        layoutOptions_.mode = mode;
        doLayout();
    }
    
    void saveLayout(const std::string& path) {
        manualManager_.saveToFile(path);
    }
    
    void loadLayout(const std::string& path) {
        if (manualManager_.loadFromFile(path)) {
            doLayout();
        }
    }
    
    void setupGraph() {
        // Create sample state machine graph
        auto idle = graph_.addNode(Size{100, 50}, "Idle");
        auto running = graph_.addNode(Size{100, 50}, "Running");
        auto paused = graph_.addNode(Size{100, 50}, "Paused");
        auto stopped = graph_.addNode(Size{100, 50}, "Stopped");
        auto error = graph_.addNode(Size{100, 50}, "Error");
        
        graph_.addEdge(idle, running, "start");
        graph_.addEdge(running, paused, "pause");
        graph_.addEdge(paused, running, "resume");
        graph_.addEdge(running, stopped, "stop");
        graph_.addEdge(paused, stopped, "stop");
        graph_.addEdge(running, error, "fail");
        graph_.addEdge(error, idle, "reset");
    }
    
    void doLayout() {
        SugiyamaLayout layout;
        layout.setOptions(layoutOptions_);
        layout.setManualLayoutManager(&manualManager_);
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
        
        // Offset for centering
        offset_ = {100.0f, 50.0f};
    }
    
    void syncSnapConfigsFromEdges() {
        // Count snap points needed per node edge based on actual edge connections
        std::map<std::pair<NodeId, NodeEdge>, int> snapCounts;
        
        for (const auto& [edgeId, layout] : edgeLayouts_) {
            // Track max snap index for each node edge
            auto& srcCount = snapCounts[{layout.from, layout.sourceEdge}];
            srcCount = std::max(srcCount, layout.sourceSnapIndex + 1);
            
            auto& tgtCount = snapCounts[{layout.to, layout.targetEdge}];
            tgtCount = std::max(tgtCount, layout.targetSnapIndex + 1);
        }
        
        // Update snap configs (only if greater than current to preserve manual settings)
        for (const auto& [key, count] : snapCounts) {
            auto [nodeId, edge] = key;
            SnapPointConfig config = manualManager_.getSnapConfig(nodeId);
            if (count > config.getCount(edge)) {
                config.setCount(edge, count);
                manualManager_.setSnapConfig(nodeId, config);
            }
        }
    }
    
    void update() {
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
        
        // Find hovered edge
        hoveredEdge_ = INVALID_EDGE;
        if (hoveredNode_ == INVALID_NODE) {
            for (const auto& [id, layout] : edgeLayouts_) {
                if (isPointNearEdge(graphMouse, layout, 8.0f)) {
                    hoveredEdge_ = id;
                    break;
                }
            }
        }
        
        // Handle click for selection
        if (ImGui::IsMouseClicked(0) && hoveredNode_ != INVALID_NODE) {
            selectedNode_ = hoveredNode_;
            selectedEdge_ = INVALID_EDGE;
            draggedNode_ = hoveredNode_;
            auto& layout = nodeLayouts_[draggedNode_];
            dragOffset_ = {graphMouse.x - layout.position.x, 
                          graphMouse.y - layout.position.y};
            affectedEdges_ = graph_.getConnectedEdges(draggedNode_);
        } else if (ImGui::IsMouseClicked(0) && hoveredEdge_ != INVALID_EDGE) {
            selectedEdge_ = hoveredEdge_;
            selectedNode_ = INVALID_NODE;
        } else if (ImGui::IsMouseClicked(0) && hoveredNode_ == INVALID_NODE && hoveredEdge_ == INVALID_EDGE) {
            selectedNode_ = INVALID_NODE;
            selectedEdge_ = INVALID_EDGE;
        }
        
        if (ImGui::IsMouseReleased(0)) {
            if (draggedNode_ != INVALID_NODE) {
                // Save node positions before layout recalculation
                std::unordered_map<NodeId, Point> savedPositions;
                for (const auto& [id, layout] : nodeLayouts_) {
                    savedPositions[id] = layout.position;
                    manualManager_.setNodePosition(id, layout.position);
                }
                
                // Get all edge IDs for update
                std::vector<EdgeId> allEdges;
                for (const auto& [edgeId, _] : edgeLayouts_) {
                    allEdges.push_back(edgeId);
                }
                
                // Recalculate layout (this gives fresh edge routing)
                doLayout();
                
                // Restore node positions
                for (auto& [id, layout] : nodeLayouts_) {
                    layout.position = savedPositions[id];
                }
                
                // Update all edge positions using library API
                // Pass empty set to update ALL endpoints (not just moved nodes)
                LayoutUtils::updateEdgePositions(
                    edgeLayouts_, nodeLayouts_, allEdges,
                    layoutOptions_.snapDistribution, {});
            }
            draggedNode_ = INVALID_NODE;
            affectedEdges_.clear();
        }
        
        if (draggedNode_ != INVALID_NODE && ImGui::IsMouseDragging(0)) {
            // Update node position
            auto& layout = nodeLayouts_[draggedNode_];
            layout.position.x = graphMouse.x - dragOffset_.x;
            layout.position.y = graphMouse.y - dragOffset_.y;
            
            // Save position (for both modes, so snap distribution works after drag)
            manualManager_.setNodePosition(draggedNode_, layout.position);
            
            // Re-route connected edges
            rerouteAffectedEdges();
        }
    }
    
    bool isPointNearEdge(const Point& p, const EdgeLayout& edge, float threshold) {
        auto points = edge.allPoints();
        for (size_t i = 1; i < points.size(); ++i) {
            Point a = points[i-1];
            Point b = points[i];
            
            // Distance from point to line segment
            float dx = b.x - a.x;
            float dy = b.y - a.y;
            float len2 = dx * dx + dy * dy;
            
            float t = 0.0f;
            if (len2 > 0.0001f) {
                t = std::max(0.0f, std::min(1.0f, 
                    ((p.x - a.x) * dx + (p.y - a.y) * dy) / len2));
            }
            
            Point closest = {a.x + t * dx, a.y + t * dy};
            float dist = std::sqrt((p.x - closest.x) * (p.x - closest.x) + 
                                   (p.y - closest.y) * (p.y - closest.y));
            
            if (dist < threshold) return true;
        }
        return false;
    }
    
    void rerouteAffectedEdges() {
        // Use library API for edge position updates
        // Only update endpoints on the dragged node, not on connected nodes
        std::unordered_set<NodeId> movedNodes = {draggedNode_};
        LayoutUtils::updateEdgePositions(
            edgeLayouts_, nodeLayouts_, affectedEdges_, 
            layoutOptions_.snapDistribution, movedNodes);
    }
    
    void render(ImDrawList* drawList) {
        // Draw edges first
        for (const auto& [id, layout] : edgeLayouts_) {
            bool isAffected = std::find(affectedEdges_.begin(), affectedEdges_.end(), id) 
                             != affectedEdges_.end();
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
            
            // Draw edge label
            const EdgeData& edge = graph_.getEdge(id);
            if (!edge.label.empty()) {
                Point mid = {(layout.sourcePoint.x + layout.targetPoint.x) / 2,
                            (layout.sourcePoint.y + layout.targetPoint.y) / 2};
                ImVec2 textPos = {mid.x + offset_.x - 20, mid.y + offset_.y - 15};
                drawList->AddText(textPos, COLOR_TEXT, edge.label.c_str());
            }
        }
        
        // Draw nodes
        for (const auto& [id, layout] : nodeLayouts_) {
            ImU32 fillColor = COLOR_NODE;
            if (id == draggedNode_) {
                fillColor = COLOR_NODE_DRAG;
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
    }
    
    void drawSnapPoints(ImDrawList* drawList, const NodeLayout& nodeLayout) {
        // In Manual mode, draw configured snap points
        if (manualManager_.getMode() == LayoutMode::Manual) {
            SnapPointConfig config = manualManager_.getSnapConfig(nodeLayout.id);
            
            auto drawEdgeSnaps = [&](NodeEdge edge, int count) {
                for (int i = 0; i < count; ++i) {
                    Point p = ManualLayoutManager::calculateSnapPoint(nodeLayout, edge, i, count);
                    ImVec2 screenPos = {p.x + offset_.x, p.y + offset_.y};
                    drawList->AddCircleFilled(screenPos, 4.0f, COLOR_SNAP_POINT);
                }
            };
            
            drawEdgeSnaps(NodeEdge::Top, config.topCount);
            drawEdgeSnaps(NodeEdge::Bottom, config.bottomCount);
            drawEdgeSnaps(NodeEdge::Left, config.leftCount);
            drawEdgeSnaps(NodeEdge::Right, config.rightCount);
        }
        
        // Draw actual edge connection points (for both modes)
        for (const auto& [edgeId, edgeLayout] : edgeLayouts_) {
            // Source point on this node
            if (edgeLayout.from == nodeLayout.id) {
                ImVec2 screenPos = {edgeLayout.sourcePoint.x + offset_.x, 
                                    edgeLayout.sourcePoint.y + offset_.y};
                drawList->AddCircleFilled(screenPos, 5.0f, IM_COL32(100, 200, 100, 255));
                drawList->AddCircle(screenPos, 5.0f, IM_COL32(50, 150, 50, 255), 0, 1.5f);
            }
            // Target point on this node
            if (edgeLayout.to == nodeLayout.id) {
                ImVec2 screenPos = {edgeLayout.targetPoint.x + offset_.x, 
                                    edgeLayout.targetPoint.y + offset_.y};
                drawList->AddCircleFilled(screenPos, 5.0f, IM_COL32(200, 100, 100, 255));
                drawList->AddCircle(screenPos, 5.0f, IM_COL32(150, 50, 50, 255), 0, 1.5f);
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
        
        // Mode selection
        ImGui::Text("Layout Mode:");
        int mode = (manualManager_.getMode() == LayoutMode::Auto) ? 0 : 1;
        if (ImGui::RadioButton("Auto", &mode, 0)) {
            setLayoutMode(LayoutMode::Auto);
        }
        ImGui::SameLine();
        if (ImGui::RadioButton("Manual", &mode, 1)) {
            setLayoutMode(LayoutMode::Manual);
        }
        
        // Snap Distribution (only in Auto mode)
        if (manualManager_.getMode() == LayoutMode::Auto) {
            ImGui::Text("Snap Distribution:");
            int snapDist = (layoutOptions_.snapDistribution == SnapDistribution::Unified) ? 0 : 1;
            if (ImGui::RadioButton("Unified", &snapDist, 0)) {
                layoutOptions_.snapDistribution = SnapDistribution::Unified;
                doLayout();
            }
            ImGui::SameLine();
            if (ImGui::RadioButton("Separated", &snapDist, 1)) {
                layoutOptions_.snapDistribution = SnapDistribution::Separated;
                doLayout();
            }
            if (ImGui::IsItemHovered()) {
                ImGui::SetTooltip("Unified: all edges evenly distributed\nSeparated: incoming left, outgoing right");
            }
        }
        
        ImGui::Separator();
        ImGui::Text("Drag nodes to see edge re-routing");
        ImGui::Separator();
        ImGui::Text("Nodes: %zu", graph_.nodeCount());
        ImGui::Text("Edges: %zu", graph_.edgeCount());
        
        if (draggedNode_ != INVALID_NODE) {
            ImGui::TextColored(ImVec4(1, 0.5f, 0.5f, 1), 
                "Dragging: %s", graph_.getNode(draggedNode_).label.c_str());
            ImGui::Text("Affected edges: %zu", affectedEdges_.size());
        }
        
        ImGui::Separator();
        ImGui::Checkbox("Show Snap Points", &showSnapPoints_);
        
        // Snap point configuration for selected node
        if (selectedNode_ != INVALID_NODE && manualManager_.getMode() == LayoutMode::Manual) {
            ImGui::Separator();
            ImGui::Text("Snap Points: %s", graph_.getNode(selectedNode_).label.c_str());
            
            SnapPointConfig config = manualManager_.getSnapConfig(selectedNode_);
            bool changed = false;
            
            if (ImGui::SliderInt("Top", &config.topCount, 1, 5)) changed = true;
            if (ImGui::SliderInt("Bottom", &config.bottomCount, 1, 5)) changed = true;
            if (ImGui::SliderInt("Left", &config.leftCount, 1, 5)) changed = true;
            if (ImGui::SliderInt("Right", &config.rightCount, 1, 5)) changed = true;
            
            if (changed) {
                manualManager_.setSnapConfig(selectedNode_, config);
                doLayout();
            }
        }
        
        // Edge routing configuration for selected edge
        if (selectedEdge_ != INVALID_EDGE && manualManager_.getMode() == LayoutMode::Manual) {
            ImGui::Separator();
            const EdgeData& edgeData = graph_.getEdge(selectedEdge_);
            ImGui::Text("Edge: %s -> %s", 
                graph_.getNode(edgeData.from).label.c_str(),
                graph_.getNode(edgeData.to).label.c_str());
            if (!edgeData.label.empty()) {
                ImGui::Text("Label: %s", edgeData.label.c_str());
            }
            
            EdgeRoutingConfig routing = manualManager_.getEdgeRouting(selectedEdge_);
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
            SnapPointConfig srcConfig = manualManager_.getSnapConfig(edgeData.from);
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
            SnapPointConfig tgtConfig = manualManager_.getSnapConfig(edgeData.to);
            int tgtMaxSnap = tgtConfig.getCount(routing.targetEdge) - 1;
            if (tgtMaxSnap > 0) {
                if (ImGui::SliderInt("Tgt Snap", &routing.targetSnapIndex, 0, tgtMaxSnap)) {
                    routingChanged = true;
                }
            }
            
            if (routingChanged) {
                manualManager_.setEdgeRouting(selectedEdge_, routing);
                doLayout();
            }
        }
        
        ImGui::Separator();
        if (ImGui::Button("Reset Layout")) {
            manualManager_.clearManualState();
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
    ManualLayoutManager manualManager_;
    LayoutOptions layoutOptions_;
    
    Point offset_ = {0, 0};
    NodeId hoveredNode_ = INVALID_NODE;
    NodeId draggedNode_ = INVALID_NODE;
    NodeId selectedNode_ = INVALID_NODE;
    EdgeId hoveredEdge_ = INVALID_EDGE;
    EdgeId selectedEdge_ = INVALID_EDGE;
    Point dragOffset_ = {0, 0};
    std::vector<EdgeId> affectedEdges_;
    bool showSnapPoints_ = true;
};

int main(int argc, char* argv[]) {
    (void)argc; (void)argv;
    
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
    
    InteractiveDemo demo;
    
    bool running = true;
    while (running) {
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
        
        demo.update();

        // Draw graph on background (must be before Render())
        ImDrawList* drawList = ImGui::GetBackgroundDrawList();
        demo.render(drawList);

        demo.renderUI();

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
