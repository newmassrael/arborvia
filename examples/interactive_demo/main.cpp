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
        
        // Find hovered bend point (only in Manual mode)
        hoveredBendPoint_.clear();
        bendPointPreview_.clear();
        if (hoveredNode_ == INVALID_NODE && manualManager_.getMode() == LayoutMode::Manual) {
            for (const auto& [edgeId, layout] : edgeLayouts_) {
                const auto& bps = manualManager_.getBendPoints(edgeId);
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
                int insertIdx = -1;
                Point insertPos = {0, 0};
                if (isPointNearEdgeWithInsertInfo(graphMouse, layout, 8.0f, insertIdx, insertPos)) {
                    hoveredEdge_ = id;
                    // Show insert preview if edge is selected
                    if (id == selectedEdge_ && manualManager_.getMode() == LayoutMode::Manual) {
                        bendPointPreview_.edgeId = id;
                        bendPointPreview_.insertIndex = insertIdx;
                        bendPointPreview_.position = insertPos;
                        bendPointPreview_.active = true;
                    }
                    break;
                }
            }
        }
        
        // Handle bend point interaction
        if (hoveredBendPoint_.isValid() && ImGui::IsMouseClicked(0)) {
            // Start dragging bend point
            selectedBendPoint_ = hoveredBendPoint_;
            selectedEdge_ = hoveredBendPoint_.edgeId;
            draggingBendPoint_ = hoveredBendPoint_;
            const auto& bps = manualManager_.getBendPoints(draggingBendPoint_.edgeId);
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
            if (!manualManager_.hasManualBendPoints(edgeId)) {
                EdgeRoutingConfig routing;
                routing.sourceEdge = edgeLayout.sourceEdge;
                routing.targetEdge = edgeLayout.targetEdge;
                routing.sourceSnapIndex = edgeLayout.sourceSnapIndex;
                routing.targetSnapIndex = edgeLayout.targetSnapIndex;
                manualManager_.setEdgeRouting(edgeId, routing);
            }
            
            const auto& existingBps = manualManager_.getBendPoints(edgeId);
            
            Point bp1, bp2;
            size_t insertIdx = 0;
            
            if (existingBps.empty()) {
                // First insertion - calculate relative to source and target only
                // (Auto bend points will be removed when manual bends are added)
                Point source = edgeLayout.sourcePoint;
                Point target = edgeLayout.targetPoint;
                
                float dx = std::abs(target.x - source.x);
                float dy = std::abs(target.y - source.y);
                
                if (dx > dy) {
                    // More horizontal - create vertical step
                    bp1 = {clickPos.x, source.y};
                    bp2 = {clickPos.x, target.y};
                } else {
                    // More vertical - create horizontal step
                    bp1 = {source.x, clickPos.y};
                    bp2 = {target.x, clickPos.y};
                }
                insertIdx = 0;
            } else {
                // Existing manual bends - use current path structure
                int segmentIdx = bendPointPreview_.insertIndex;
                
                // Build path: source → [manual bends] → target
                std::vector<Point> path;
                path.push_back(edgeLayout.sourcePoint);
                for (const auto& bp : existingBps) {
                    path.push_back(bp.position);
                }
                path.push_back(edgeLayout.targetPoint);
                
                Point prevPoint = path[segmentIdx];
                Point nextPoint = path[segmentIdx + 1];
                
                float dx = std::abs(nextPoint.x - prevPoint.x);
                float dy = std::abs(nextPoint.y - prevPoint.y);
                
                if (dx > dy) {
                    // More horizontal segment
                    bp1 = {clickPos.x, prevPoint.y};
                    bp2 = {clickPos.x, nextPoint.y};
                    if (std::abs(prevPoint.y - nextPoint.y) < 1.0f) {
                        bp2 = {clickPos.x, clickPos.y};
                    }
                } else {
                    // More vertical segment
                    bp1 = {prevPoint.x, clickPos.y};
                    bp2 = {nextPoint.x, clickPos.y};
                    if (std::abs(prevPoint.x - nextPoint.x) < 1.0f) {
                        bp2 = {clickPos.x, clickPos.y};
                    }
                }
                insertIdx = static_cast<size_t>(segmentIdx);
            }
            
            // Insert both points
            manualManager_.addBendPoint(edgeId, insertIdx, bp1);
            manualManager_.addBendPoint(edgeId, insertIdx + 1, bp2);
            
            selectedBendPoint_.edgeId = edgeId;
            selectedBendPoint_.bendPointIndex = static_cast<int>(insertIdx) + 1;
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
            // Handle bend point drag release
            if (draggingBendPoint_.isValid()) {
                doLayout();
            }
            draggingBendPoint_.clear();
            
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
        
        // Handle bend point dragging with orthogonal constraint
        if (draggingBendPoint_.isValid() && ImGui::IsMouseDragging(0)) {
            EdgeId edgeId = draggingBendPoint_.edgeId;
            int bpIdx = draggingBendPoint_.bendPointIndex;
            
            // Get current bend points from manager (source of truth)
            const auto& bps = manualManager_.getBendPoints(edgeId);
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
                auto dragResult = ManualLayoutManager::calculateOrthogonalDrag(
                    prevPoint, currentPos, nextPoint, dragTarget, hasNextBend, isLastBend);
                
                // Apply the calculated positions
                if (dragResult.nextAdjusted) {
                    manualManager_.moveBendPoint(edgeId, static_cast<size_t>(bpIdx + 1), dragResult.adjustedNextPos);
                }
                manualManager_.moveBendPoint(edgeId, static_cast<size_t>(bpIdx), dragResult.newCurrentPos);
                
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
            layout.position.x = graphMouse.x - dragOffset_.x;
            layout.position.y = graphMouse.y - dragOffset_.y;
            
            // Save position (for both modes, so snap distribution works after drag)
            manualManager_.setNodePosition(draggedNode_, layout.position);
            
            // Re-route connected edges
            rerouteAffectedEdges();
        }
        
        // Handle Delete key for bend point removal
        if (selectedBendPoint_.isValid() && ImGui::IsKeyPressed(ImGuiKey_Delete)) {
            manualManager_.removeBendPoint(
                selectedBendPoint_.edgeId,
                static_cast<size_t>(selectedBendPoint_.bendPointIndex)
            );
            selectedBendPoint_.clear();
            doLayout();
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
    
    // Extended version that returns insertion point info
    bool isPointNearEdgeWithInsertInfo(const Point& p, const EdgeLayout& edge, float threshold,
                                        int& outInsertIndex, Point& outInsertPos) {
        auto points = edge.allPoints();
        for (size_t i = 1; i < points.size(); ++i) {
            Point a = points[i-1];
            Point b = points[i];
            
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
            
            if (dist < threshold) {
                // Segment i-1 to i means bend point index i-1 
                // (insert after (i-1)th bend point, before ith)
                outInsertIndex = static_cast<int>(i) - 1;
                outInsertPos = closest;
                return true;
            }
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
            
            // Draw bend points as diamonds (Manual mode only)
            if (manualManager_.getMode() == LayoutMode::Manual) {
                const auto& bps = layout.bendPoints;
                for (size_t i = 0; i < bps.size(); ++i) {
                    Point bpPos = bps[i].position;
                    ImVec2 screenPos = {bpPos.x + offset_.x, bpPos.y + offset_.y};
                    
                    // Determine color based on state
                    ImU32 bpColor = COLOR_BEND_POINT;
                    float size = 6.0f;
                    if (draggingBendPoint_.edgeId == id && 
                        draggingBendPoint_.bendPointIndex == static_cast<int>(i)) {
                        bpColor = COLOR_BEND_POINT_DRAGGING;
                        size = 8.0f;
                    } else if (selectedBendPoint_.edgeId == id && 
                               selectedBendPoint_.bendPointIndex == static_cast<int>(i)) {
                        bpColor = COLOR_BEND_POINT_SELECTED;
                        size = 7.0f;
                    } else if (hoveredBendPoint_.edgeId == id && 
                               hoveredBendPoint_.bendPointIndex == static_cast<int>(i)) {
                        bpColor = COLOR_BEND_POINT_HOVER;
                        size = 7.0f;
                    }
                    
                    // Draw diamond shape
                    ImVec2 pts[4] = {
                        {screenPos.x, screenPos.y - size},  // Top
                        {screenPos.x + size, screenPos.y},  // Right
                        {screenPos.x, screenPos.y + size},  // Bottom
                        {screenPos.x - size, screenPos.y}   // Left
                    };
                    drawList->AddConvexPolyFilled(pts, 4, bpColor);
                    drawList->AddPolyline(pts, 4, IM_COL32(50, 80, 150, 255), ImDrawFlags_Closed, 1.5f);
                }
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
            
            // Bend point management UI
            ImGui::Separator();
            ImGui::Text("Bend Points:");
            
            const auto& bps = manualManager_.getBendPoints(selectedEdge_);
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
                    manualManager_.clearBendPoints(selectedEdge_);
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
    
    // Bend point interaction state
    HoveredBendPoint hoveredBendPoint_;
    HoveredBendPoint selectedBendPoint_;
    HoveredBendPoint draggingBendPoint_;
    Point bendPointDragOffset_ = {0, 0};
    BendPointPreview bendPointPreview_;
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
