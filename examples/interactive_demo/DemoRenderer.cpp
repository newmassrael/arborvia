#include "DemoRenderer.h"
#include "DemoColors.h"
#include "../../src/layout/snap/GridSnapCalculator.h"

// SCXML types for node visualization
#include "SCXMLGraph.h"
#include "SCXMLTypes.h"

#include <cmath>
#include <algorithm>

namespace arborvia {

// Aliases for shorter code
namespace Colors = DemoColors;
namespace Visuals = DemoVisuals;

// Anonymous namespace for helper functions
namespace {

/// Draw SCXML Initial pseudo-state (filled black circle)
void drawSCXMLInitialNode(ImDrawList* drawList, float centerX, float centerY, float radius) {
    constexpr ImU32 initialColor = IM_COL32(30, 30, 30, 255);
    drawList->AddCircleFilled({centerX, centerY}, radius, initialColor);
}

/// Draw SCXML Final state (double circle)
void drawSCXMLFinalNode(ImDrawList* drawList, float centerX, float centerY, float radius, float zoom) {
    constexpr ImU32 innerColor = IM_COL32(30, 30, 30, 255);
    float outerRadius = radius;
    float innerRadius = radius * 0.6f;
    drawList->AddCircle({centerX, centerY}, outerRadius, Colors::NODE_BORDER, 0, Visuals::NODE_BORDER_THICKNESS * zoom);
    drawList->AddCircleFilled({centerX, centerY}, innerRadius, innerColor);
}

/// Draw SCXML History state (circle with "H" or "H*")
void drawSCXMLHistoryNode(ImDrawList* drawList, float centerX, float centerY, float radius,
                          float zoom, ImU32 fillColor, bool isDeep) {
    drawList->AddCircle({centerX, centerY}, radius, Colors::NODE_BORDER, 0, Visuals::NODE_BORDER_THICKNESS * zoom);
    drawList->AddCircleFilled({centerX, centerY}, radius - 1.0f * zoom, fillColor);
    const char* histText = isDeep ? "H*" : "H";
    ImVec2 textSize = ImGui::CalcTextSize(histText);
    ImVec2 textPos = {centerX - textSize.x / 2, centerY - textSize.y / 2};
    drawList->AddText(textPos, Colors::TEXT, histText);
}

/// Draw SCXML Parallel state (rectangle with dashed border)
void drawSCXMLParallelNode(ImDrawList* drawList, const ImVec2& p1, const ImVec2& p2,
                           float zoom, ImU32 fillColor) {
    float scaledRounding = Visuals::NODE_ROUNDING * zoom;
    drawList->AddRectFilled(p1, p2, fillColor, scaledRounding);
    drawList->AddRect(p1, p2, Colors::NODE_BORDER, scaledRounding, 0, Visuals::NODE_BORDER_THICKNESS * zoom);
    // Inner dashed border effect
    constexpr ImU32 innerBorderColor = IM_COL32(100, 100, 100, 150);
    constexpr float innerOffset = 3.0f;
    drawList->AddRect({p1.x + innerOffset * zoom, p1.y + innerOffset * zoom},
                     {p2.x - innerOffset * zoom, p2.y - innerOffset * zoom},
                     innerBorderColor, scaledRounding, 0, 1.0f * zoom);
}

/// Draw standard rectangular node
void drawStandardNode(ImDrawList* drawList, const ImVec2& p1, const ImVec2& p2,
                      float zoom, ImU32 fillColor) {
    float scaledRounding = Visuals::NODE_ROUNDING * zoom;
    drawList->AddRectFilled(p1, p2, fillColor, scaledRounding);
    drawList->AddRect(p1, p2, Colors::NODE_BORDER, scaledRounding, 0, Visuals::NODE_BORDER_THICKNESS * zoom);
}

/// Draw node ID label
void drawNodeIdLabel(ImDrawList* drawList, NodeId id, float centerX, float centerY,
                     float radius, bool isPointNode, const ImVec2& p1) {
    char idStr[16];
    snprintf(idStr, sizeof(idStr), "[%d]", static_cast<int>(id));
    ImVec2 idSize = ImGui::CalcTextSize(idStr);
    ImVec2 idPos;
    if (isPointNode) {
        idPos = {centerX - idSize.x / 2, centerY - radius - idSize.y - 4};
    } else {
        idPos = {p1.x + 3, p1.y + 3};
    }
    float padding = Visuals::LABEL_PADDING;
    ImVec2 bgMin = {idPos.x - padding, idPos.y - padding};
    ImVec2 bgMax = {idPos.x + idSize.x + padding, idPos.y + idSize.y + padding};
    drawList->AddRectFilled(bgMin, bgMax, Colors::ID_LABEL_BG, 2.0f);
    drawList->AddRect(bgMin, bgMax, Colors::ID_LABEL_BORDER, 2.0f);
    drawList->AddText(idPos, IM_COL32(0, 0, 0, 255), idStr);
}

/// Draw node text label
void drawNodeTextLabel(ImDrawList* drawList, const NodeData* nodeData, const NodeLayout& layout,
                       const ImVec2& p1, float zoom) {
    if (!nodeData || nodeData->label.empty()) return;
    
    ImVec2 textSize = ImGui::CalcTextSize(nodeData->label.c_str());
    float scaledWidth = layout.size.width * zoom;
    float scaledHeight = layout.size.height * zoom;
    ImVec2 textPos = {p1.x + (scaledWidth - textSize.x) / 2,
                     p1.y + (scaledHeight - textSize.y) / 2};
    drawList->AddText(textPos, Colors::TEXT, nodeData->label.c_str());
}

/// Check if SCXML node type should skip label
bool shouldSkipLabelForSCXMLType(test::scxml::SCXMLNodeType nodeType) {
    using SCXMLNodeType = test::scxml::SCXMLNodeType;
    return nodeType == SCXMLNodeType::Initial ||
           nodeType == SCXMLNodeType::Final ||
           nodeType == SCXMLNodeType::History ||
           nodeType == SCXMLNodeType::HistoryDeep;
}

}  // anonymous namespace

void DemoRenderer::drawGrid(ImDrawList* drawList, const ViewTransform& view,
                           float width, float height, float gridSize) {
    if (gridSize <= 0.0f) return;

    float zoomedGridSize = gridSize * view.zoom;

    // Calculate grid bounds with pan/zoom
    Point worldOrigin = view.screenToWorld({0, 0});
    float worldStartX = std::floor(worldOrigin.x / gridSize) * gridSize;
    float worldStartY = std::floor(worldOrigin.y / gridSize) * gridSize;
    ImVec2 screenStart = view.worldToScreen({worldStartX, worldStartY});

    // Draw vertical lines
    int lineCount = static_cast<int>(std::floor(worldOrigin.x / gridSize));
    for (float x = screenStart.x; x < width; x += zoomedGridSize) {
        if (x >= 0) {
            ImU32 color = (std::abs(lineCount) % 5 == 0) ? Colors::GRID_MAJOR : Colors::GRID;
            drawList->AddLine({x, 0}, {x, height}, color, 1.0f);
        }
        lineCount++;
    }

    // Draw horizontal lines
    lineCount = static_cast<int>(std::floor(worldOrigin.y / gridSize));
    for (float y = screenStart.y; y < height; y += zoomedGridSize) {
        if (y >= 0) {
            ImU32 color = (std::abs(lineCount) % 5 == 0) ? Colors::GRID_MAJOR : Colors::GRID;
            drawList->AddLine({0, y}, {width, y}, color, 1.0f);
        }
        lineCount++;
    }
}

void DemoRenderer::drawBlockedCells(ImDrawList* drawList, const ViewTransform& view,
                                    const std::vector<Rect>& blockedRegions, float gridSize) {
    if (gridSize <= 0.0f) return;

    for (const auto& region : blockedRegions) {
        int leftCell = static_cast<int>(std::ceil(region.x / gridSize));
        int topCell = static_cast<int>(std::ceil(region.y / gridSize));
        int rightCell = static_cast<int>(std::ceil((region.x + region.width) / gridSize));
        int bottomCell = static_cast<int>(std::ceil((region.y + region.height) / gridSize));

        for (int gx = leftCell; gx < rightCell; ++gx) {
            for (int gy = topCell; gy < bottomCell; ++gy) {
                ImVec2 p1 = view.worldToScreen({gx * gridSize, gy * gridSize});
                ImVec2 p2 = view.worldToScreen({(gx + 1) * gridSize, (gy + 1) * gridSize});
                drawList->AddRectFilled(p1, p2, Colors::BLOCKED_CELL);
            }
        }
    }
}

void DemoRenderer::drawAStarDebug(ImDrawList* drawList, const ViewTransform& view,
                                  const AStarDebugState& debug, [[maybe_unused]] float gridSize) {
    if (!debug.obstacles) return;

    constexpr float SEGMENT_INSET = 0.35f;  // Inset ratio for segment visualization
    constexpr float DEBUG_POINT_RADIUS = 8.0f;

    float cellSize = debug.obstacles->gridSize();
    int obsOffsetX = debug.obstacles->offsetX();
    int obsOffsetY = debug.obstacles->offsetY();
    float zoomedCellSize = cellSize * view.zoom;

    // Draw all grid cells with obstacles
    for (int gy = 0; gy < debug.obstacles->height(); ++gy) {
        for (int gx = 0; gx < debug.obstacles->width(); ++gx) {
            int gridX = gx + obsOffsetX;
            int gridY = gy + obsOffsetY;
            auto info = debug.obstacles->getCellVisInfo(gridX, gridY);

            ImVec2 p1 = view.worldToScreen({gridX * cellSize, gridY * cellSize});
            ImVec2 p2 = view.worldToScreen({(gridX + 1) * cellSize, (gridY + 1) * cellSize});

            if (info.isNodeBlocked) {
                drawList->AddRectFilled(p1, p2, Colors::ASTAR_NODE_BLOCK);
            }
            if (info.hasHorizontalSegment) {
                drawList->AddRectFilled({p1.x, p1.y + zoomedCellSize * SEGMENT_INSET},
                                       {p2.x, p2.y - zoomedCellSize * SEGMENT_INSET}, Colors::ASTAR_H_SEGMENT);
            }
            if (info.hasVerticalSegment) {
                drawList->AddRectFilled({p1.x + zoomedCellSize * SEGMENT_INSET, p1.y},
                                       {p2.x - zoomedCellSize * SEGMENT_INSET, p2.y}, Colors::ASTAR_V_SEGMENT);
            }
        }
    }

    // Highlight start/goal points
    if (debug.start.x >= 0) {
        ImVec2 p = view.worldToScreen(debug.start);
        float radius = DEBUG_POINT_RADIUS * view.zoom;
        drawList->AddCircleFilled(p, radius, Colors::ASTAR_START);
        drawList->AddText({p.x + 10, p.y - 5}, IM_COL32_WHITE, "START");
    }
    if (debug.goal.x >= 0) {
        ImVec2 p = view.worldToScreen(debug.goal);
        float radius = DEBUG_POINT_RADIUS * view.zoom;
        drawList->AddCircleFilled(p, radius, Colors::ASTAR_GOAL);
        drawList->AddText({p.x + 10, p.y - 5}, IM_COL32_WHITE, "GOAL");
    }
}

void DemoRenderer::drawArrowhead(ImDrawList* drawList, const ViewTransform& view,
                                 const Point& from, const Point& to, ImU32 color) {
    ImVec2 screenFrom = view.worldToScreen(from);
    ImVec2 screenTo = view.worldToScreen(to);

    float dx = screenTo.x - screenFrom.x;
    float dy = screenTo.y - screenFrom.y;
    float len = std::sqrt(dx * dx + dy * dy);
    constexpr float MIN_LENGTH = 0.001f;
    if (len < MIN_LENGTH) return;

    dx /= len;
    dy /= len;

    constexpr float ARROW_WIDTH_RATIO = 0.5f;
    float arrowSize = Visuals::ARROW_SIZE * view.zoom;
    ImVec2 tip = screenTo;
    ImVec2 left = {tip.x - arrowSize * dx + arrowSize * ARROW_WIDTH_RATIO * dy,
                   tip.y - arrowSize * dy - arrowSize * ARROW_WIDTH_RATIO * dx};
    ImVec2 right = {tip.x - arrowSize * dx - arrowSize * ARROW_WIDTH_RATIO * dy,
                    tip.y - arrowSize * dy + arrowSize * ARROW_WIDTH_RATIO * dx};

    drawList->AddTriangleFilled(tip, left, right, color);
}

void DemoRenderer::drawEdge(ImDrawList* drawList, const ViewTransform& view,
                           const EdgeLayout& layout, const EdgeData* edgeData,
                           bool isSelected, bool isHovered, bool isAffected) {
    ImU32 color = Colors::EDGE;
    float thickness = Visuals::EDGE_THICKNESS;

    if (isSelected) {
        color = Colors::EDGE_SELECTED;
        thickness = Visuals::EDGE_THICKNESS_SELECTED;
    } else if (isHovered) {
        color = Colors::NODE_HOVER;
        thickness = Visuals::EDGE_THICKNESS_HOVERED;
    } else if (isAffected) {
        color = Colors::EDGE_AFFECTED;
    }

    auto points = layout.allPoints();
    float scaledThickness = thickness * view.zoom;

    for (size_t i = 1; i < points.size(); ++i) {
        ImVec2 p1 = view.worldToScreen(points[i-1]);
        ImVec2 p2 = view.worldToScreen(points[i]);
        drawList->AddLine(p1, p2, color, scaledThickness);
    }

    // Draw arrowhead
    if (points.size() >= 2) {
        Point last = points.back();
        Point prev = points[points.size() - 2];
        drawArrowhead(drawList, view, prev, last, color);
    }

    // Draw edge label
    if (edgeData && !edgeData->label.empty()) {
        constexpr float LABEL_OFFSET_X = 20.0f;
        constexpr float LABEL_OFFSET_Y = 15.0f;
        ImVec2 labelPos = view.worldToScreen(layout.labelPosition);
        ImVec2 textPos = {labelPos.x - LABEL_OFFSET_X, labelPos.y - LABEL_OFFSET_Y};
        drawList->AddText(textPos, Colors::TEXT, edgeData->label.c_str());
    }
}

void DemoRenderer::drawBendPointPreview(ImDrawList* drawList, const ViewTransform& view,
                                        const BendPointPreview& preview) {
    if (!preview.active) return;

    constexpr ImU32 PREVIEW_BORDER = IM_COL32(100, 200, 255, 200);
    constexpr ImU32 PLUS_COLOR = IM_COL32(255, 255, 255, 200);
    constexpr float PLUS_THICKNESS = 2.0f;

    ImVec2 screenPos = view.worldToScreen(preview.position);
    float size = Visuals::BEND_PREVIEW_SIZE * view.zoom;
    ImVec2 pts[4] = {
        {screenPos.x, screenPos.y - size},
        {screenPos.x + size, screenPos.y},
        {screenPos.x, screenPos.y + size},
        {screenPos.x - size, screenPos.y}
    };
    drawList->AddConvexPolyFilled(pts, 4, Colors::BEND_POINT_PREVIEW);
    drawList->AddPolyline(pts, 4, PREVIEW_BORDER, ImDrawFlags_Closed, 1.0f);

    // Draw "+" indicator
    float plusSize = Visuals::BEND_PLUS_SIZE * view.zoom;
    drawList->AddLine(
        {screenPos.x - plusSize, screenPos.y},
        {screenPos.x + plusSize, screenPos.y},
        PLUS_COLOR, PLUS_THICKNESS);
    drawList->AddLine(
        {screenPos.x, screenPos.y - plusSize},
        {screenPos.x, screenPos.y + plusSize},
        PLUS_COLOR, PLUS_THICKNESS);
}

void DemoRenderer::drawNode(ImDrawList* drawList, const ViewTransform& view,
                           NodeId id, const NodeLayout& layout,
                           const NodeData* nodeData,
                           bool isSelected, bool isHovered, bool isDragged,
                           bool isInvalidPosition,
                           const test::scxml::SCXMLGraph* scxmlGraph) {
    // Determine fill color based on state
    ImU32 fillColor = Colors::NODE;
    if (isDragged) {
        fillColor = isInvalidPosition ? Colors::NODE_INVALID : Colors::NODE_DRAG;
    } else if (isSelected) {
        fillColor = Colors::NODE_SELECTED;
    } else if (isHovered) {
        fillColor = Colors::NODE_HOVER;
    }

    bool isPointNode = layout.isPointNode();

    // Calculate screen coordinates
    ImVec2 p1 = view.worldToScreen(layout.position);
    ImVec2 p2 = view.worldToScreen({layout.position.x + layout.size.width,
                                    layout.position.y + layout.size.height});

    // Calculate center and radius
    float centerX, centerY, radius;
    if (isPointNode) {
        ImVec2 centerScreen = view.worldToScreen(layout.position);
        centerX = centerScreen.x;
        centerY = centerScreen.y;
        radius = Visuals::POINT_NODE_RADIUS * view.zoom;
    } else {
        centerX = (p1.x + p2.x) / 2.0f;
        centerY = (p1.y + p2.y) / 2.0f;
        radius = std::min(p2.x - p1.x, p2.y - p1.y) / 2.0f;
    }

    // Draw node shape based on SCXML type
    bool drawnAsSpecial = false;
    test::scxml::SCXMLNodeType nodeType = test::scxml::SCXMLNodeType::State;
    
    if (scxmlGraph && isPointNode) {
        nodeType = scxmlGraph->getNodeType(id);

        using SCXMLNodeType = test::scxml::SCXMLNodeType;
        switch (nodeType) {
            case SCXMLNodeType::Initial:
                drawSCXMLInitialNode(drawList, centerX, centerY, radius);
                drawnAsSpecial = true;
                break;
            case SCXMLNodeType::Final:
                drawSCXMLFinalNode(drawList, centerX, centerY, radius, view.zoom);
                drawnAsSpecial = true;
                break;
            case SCXMLNodeType::History:
                drawSCXMLHistoryNode(drawList, centerX, centerY, radius, view.zoom, fillColor, false);
                drawnAsSpecial = true;
                break;
            case SCXMLNodeType::HistoryDeep:
                drawSCXMLHistoryNode(drawList, centerX, centerY, radius, view.zoom, fillColor, true);
                drawnAsSpecial = true;
                break;
            case SCXMLNodeType::Parallel:
                drawSCXMLParallelNode(drawList, p1, p2, view.zoom, fillColor);
                drawnAsSpecial = true;
                break;
            default:
                break;
        }
    }

    // Standard rectangular node rendering
    if (!drawnAsSpecial) {
        drawStandardNode(drawList, p1, p2, view.zoom, fillColor);
    }

    // Draw node ID label
    drawNodeIdLabel(drawList, id, centerX, centerY, radius, isPointNode, p1);

    // Draw text label (skip for certain SCXML types)
    if (nodeData && !nodeData->label.empty()) {
        bool skipLabel = (scxmlGraph && isPointNode && shouldSkipLabelForSCXMLType(nodeType));
        if (!skipLabel) {
            drawNodeTextLabel(drawList, nodeData, layout, p1, view.zoom);
        }
    }
}

void DemoRenderer::drawSnapPoints(ImDrawList* drawList, ImDrawList* fgDrawList,
                                  const ViewTransform& view,
                                  const NodeLayout& nodeLayout,
                                  const std::unordered_map<EdgeId, EdgeLayout>& edgeLayouts,
                                  const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
                                  const RenderOptions& options,
                                  const HoveredSnapPoint& hoveredSnapPoint,
                                  const HoveredSnapPoint& draggingSnapPoint) {
    for (const auto& [edgeId, edgeLayout] : edgeLayouts) {
        // Source point on this node (outgoing - green)
        if (edgeLayout.from == nodeLayout.id) {
            ImVec2 screenPos = view.worldToScreen(edgeLayout.sourcePoint);

            bool isSourceHovered = (hoveredSnapPoint.edgeId == edgeId && hoveredSnapPoint.isSource);
            bool isSourceDragging = (draggingSnapPoint.edgeId == edgeId && draggingSnapPoint.isSource);
            float radius = Visuals::SNAP_RADIUS * view.zoom;
            ImU32 fillColor = Colors::SNAP_SOURCE;
            ImU32 borderColor = Colors::SNAP_SOURCE_BORDER;

            if (isSourceDragging) {
                radius = Visuals::SNAP_RADIUS_DRAGGING * view.zoom;
                fillColor = Colors::SNAP_DRAG;
                borderColor = Colors::SNAP_DRAG_BORDER;
            } else if (isSourceHovered) {
                radius = Visuals::SNAP_RADIUS_HOVERED * view.zoom;
                fillColor = Colors::SNAP_HOVER;
                borderColor = Colors::SNAP_HOVER_BORDER;
            }

            drawList->AddCircleFilled(screenPos, radius, fillColor);
            drawList->AddCircle(screenPos, radius, borderColor, 0, Visuals::SNAP_BORDER_THICKNESS);

            // Draw snap index label
            bool isSourcePointNode = nodeLayout.isPointNode();
            if (options.showSnapIndices && !isSourcePointNode) {
                drawSnapIndexLabel(fgDrawList, view, screenPos,
                                  nodeLayout, edgeLayout.sourceEdge, edgeLayout.sourcePoint,
                                  options.gridSize, true);
            }
        }

        // Target point on this node (incoming - red)
        if (edgeLayout.to == nodeLayout.id) {
            ImVec2 screenPos = view.worldToScreen(edgeLayout.targetPoint);

            bool isTargetHovered = (hoveredSnapPoint.edgeId == edgeId && !hoveredSnapPoint.isSource);
            bool isTargetDragging = (draggingSnapPoint.edgeId == edgeId && !draggingSnapPoint.isSource);
            float radius = Visuals::SNAP_RADIUS * view.zoom;
            ImU32 fillColor = Colors::SNAP_TARGET;
            ImU32 borderColor = Colors::SNAP_TARGET_BORDER;

            if (isTargetDragging) {
                radius = Visuals::SNAP_RADIUS_DRAGGING * view.zoom;
                fillColor = Colors::SNAP_DRAG;
                borderColor = Colors::SNAP_DRAG_BORDER;
            } else if (isTargetHovered) {
                radius = Visuals::SNAP_RADIUS_HOVERED * view.zoom;
                fillColor = Colors::SNAP_HOVER;
                borderColor = Colors::SNAP_HOVER_BORDER;
            }

            drawList->AddCircleFilled(screenPos, radius, fillColor);
            drawList->AddCircle(screenPos, radius, borderColor, 0, Visuals::SNAP_BORDER_THICKNESS);

            // Draw snap index label
            auto tgtNodeIt = nodeLayouts.find(edgeLayout.to);
            bool isTargetPointNode = (tgtNodeIt != nodeLayouts.end() && tgtNodeIt->second.isPointNode());
            if (options.showSnapIndices && !isTargetPointNode && tgtNodeIt != nodeLayouts.end()) {
                drawSnapIndexLabel(fgDrawList, view, screenPos,
                                  tgtNodeIt->second, edgeLayout.targetEdge, edgeLayout.targetPoint,
                                  options.gridSize, false);
            }
        }
    }
}

void DemoRenderer::drawSnapIndexLabel(ImDrawList* drawList, const ViewTransform& view,
                                      const ImVec2& screenPos,
                                      const NodeLayout& nodeLayout, NodeEdge edge,
                                      const Point& snapPoint, float gridSize, bool isSource) {
    char label[32];
    const char* edgeName = "";
    switch (edge) {
        case NodeEdge::Top: edgeName = "T"; break;
        case NodeEdge::Bottom: edgeName = "B"; break;
        case NodeEdge::Left: edgeName = "L"; break;
        case NodeEdge::Right: edgeName = "R"; break;
    }

    int snapIdx = GridSnapCalculator::getCandidateIndexFromPosition(
        nodeLayout, edge, snapPoint, gridSize);
    snprintf(label, sizeof(label), "%s%d", edgeName, snapIdx);

    ImVec2 textSize = ImGui::CalcTextSize(label);
    ImVec2 textPos = screenPos;
    float labelOffset = Visuals::SNAP_LABEL_OFFSET * view.zoom;

    switch (edge) {
        case NodeEdge::Top:
            textPos.x -= textSize.x / 2.0f;
            textPos.y += labelOffset;
            break;
        case NodeEdge::Bottom:
            textPos.x -= textSize.x / 2.0f;
            textPos.y -= textSize.y + labelOffset;
            break;
        case NodeEdge::Left:
            textPos.x += labelOffset;
            textPos.y -= textSize.y / 2.0f;
            break;
        case NodeEdge::Right:
            textPos.x -= textSize.x + labelOffset;
            textPos.y -= textSize.y / 2.0f;
            break;
    }

    ImVec2 bgMin = {textPos.x - Visuals::SNAP_LABEL_PADDING_X, textPos.y - Visuals::SNAP_LABEL_PADDING_Y};
    ImVec2 bgMax = {textPos.x + textSize.x + Visuals::SNAP_LABEL_PADDING_X,
                    textPos.y + textSize.y + Visuals::SNAP_LABEL_PADDING_Y};

    ImU32 bgBorderColor = isSource ? Colors::SNAP_LABEL_SOURCE_BORDER : Colors::SNAP_LABEL_TARGET_BORDER;
    ImU32 textColor = isSource ? Colors::SNAP_LABEL_SOURCE_TEXT : Colors::SNAP_LABEL_TARGET_TEXT;

    drawList->AddRectFilled(bgMin, bgMax, Colors::SNAP_LABEL_BG, Visuals::SNAP_LABEL_ROUNDING);
    drawList->AddRect(bgMin, bgMax, bgBorderColor, Visuals::SNAP_LABEL_ROUNDING, 0, Visuals::SNAP_LABEL_BORDER_THICKNESS);
    drawList->AddText({textPos.x + 1, textPos.y}, textColor, label);
    drawList->AddText(textPos, textColor, label);
}

void DemoRenderer::drawSnapCandidates(ImDrawList* drawList, const ViewTransform& view,
                                      const SnapPointController& controller,
                                      int snappedIndex) {
    const auto& candidates = controller.getCandidates();
    for (size_t i = 0; i < candidates.size(); ++i) {
        const auto& candidate = candidates[i];
        ImVec2 screenPos = view.worldToScreen(candidate.position);

        bool isSnapped = (static_cast<int>(i) == snappedIndex);

        if (isSnapped) {
            float radius = Visuals::SNAP_CANDIDATE_RADIUS_SNAPPED * view.zoom;
            drawList->AddCircleFilled(screenPos, radius, Colors::SNAP_CANDIDATE_SNAPPED);
            drawList->AddCircle(screenPos, radius, Colors::SNAP_CANDIDATE_SNAPPED_BORDER,
                               0, Visuals::SNAP_CANDIDATE_BORDER_THICKNESS_SNAPPED);
        } else {
            float radius = Visuals::SNAP_CANDIDATE_RADIUS * view.zoom;
            drawList->AddCircleFilled(screenPos, radius, Colors::SNAP_CANDIDATE);
            drawList->AddCircle(screenPos, radius, Colors::SNAP_CANDIDATE_BORDER,
                               0, Visuals::SNAP_CANDIDATE_BORDER_THICKNESS);
        }
    }
}

void DemoRenderer::drawSnapPreviewPath(ImDrawList* drawList, const ViewTransform& view,
                                       const SnapPointController& controller) {
    float thickness = Visuals::SNAP_PREVIEW_THICKNESS * view.zoom;

    auto points = controller.getPreviewLayout().allPoints();
    for (size_t i = 1; i < points.size(); ++i) {
        ImVec2 p1 = view.worldToScreen(points[i-1]);
        ImVec2 p2 = view.worldToScreen(points[i]);
        drawList->AddLine(p1, p2, Colors::SNAP_PREVIEW_PATH, thickness);
    }

    // Draw arrow at the target
    if (points.size() >= 2) {
        ImVec2 p1 = view.worldToScreen(points[points.size()-2]);
        ImVec2 p2 = view.worldToScreen(points.back());

        float dx = p2.x - p1.x;
        float dy = p2.y - p1.y;
        float len = std::sqrt(dx*dx + dy*dy);
        if (len > 0.01f) {
            dx /= len;
            dy /= len;
            float arrowSize = Visuals::SNAP_PREVIEW_ARROW_SIZE * view.zoom;
            ImVec2 arrow1 = {p2.x - arrowSize * (dx + dy * 0.5f),
                             p2.y - arrowSize * (dy - dx * 0.5f)};
            ImVec2 arrow2 = {p2.x - arrowSize * (dx - dy * 0.5f),
                             p2.y - arrowSize * (dy + dx * 0.5f)};
            drawList->AddTriangleFilled(p2, arrow1, arrow2, Colors::SNAP_PREVIEW_PATH);
        }
    }
}

}  // namespace arborvia
