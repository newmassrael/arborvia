#pragma once

#include "DemoState.h"
#include <imgui.h>

// Forward declarations for SCXML types
namespace arborvia::test::scxml {
class SCXMLGraph;
enum class SCXMLNodeType;
}

namespace arborvia {

/// Renderer for interactive demo visualization
///
/// Extracts rendering logic from InteractiveDemo for better separation of concerns.
/// All methods are static and take necessary state as parameters.
class DemoRenderer {
public:
    /// Draw background grid
    static void drawGrid(ImDrawList* drawList, const ViewTransform& view,
                        float width, float height, float gridSize);

    /// Draw blocked cells (obstacle visualization)
    static void drawBlockedCells(ImDrawList* drawList, const ViewTransform& view,
                                 const std::vector<Rect>& blockedRegions, float gridSize);

    /// Draw A* debug visualization
    static void drawAStarDebug(ImDrawList* drawList, const ViewTransform& view,
                               const AStarDebugState& debug, float gridSize);

    /// Draw arrowhead at edge endpoint
    static void drawArrowhead(ImDrawList* drawList, const ViewTransform& view,
                             const Point& from, const Point& to, ImU32 color);

    /// Draw a single edge with its bend points
    static void drawEdge(ImDrawList* drawList, const ViewTransform& view,
                        const EdgeLayout& layout, const EdgeData* edgeData,
                        bool isSelected, bool isHovered, bool isAffected);

    /// Draw bend point insertion preview
    static void drawBendPointPreview(ImDrawList* drawList, const ViewTransform& view,
                                     const BendPointPreview& preview);

    /// Draw a single node
    static void drawNode(ImDrawList* drawList, const ViewTransform& view,
                        NodeId id, const NodeLayout& layout,
                        const NodeData* nodeData,
                        bool isSelected, bool isHovered, bool isDragged,
                        bool isInvalidPosition,
                        const test::scxml::SCXMLGraph* scxmlGraph);

    /// Draw snap points for a node
    static void drawSnapPoints(ImDrawList* drawList, ImDrawList* fgDrawList,
                              const ViewTransform& view,
                              const NodeLayout& nodeLayout,
                              const std::unordered_map<EdgeId, EdgeLayout>& edgeLayouts,
                              const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
                              const RenderOptions& options,
                              const HoveredSnapPoint& hoveredSnapPoint,
                              const HoveredSnapPoint& draggingSnapPoint,
                              const std::vector<EdgeId>& hiddenEdges = {});

    /// Draw snap index label
    static void drawSnapIndexLabel(ImDrawList* drawList, const ViewTransform& view,
                                   const ImVec2& screenPos,
                                   const NodeLayout& nodeLayout, NodeEdge edge,
                                   const Point& snapPoint, float gridSize, bool isSource);

    /// Draw snap candidates during snap point drag
    static void drawSnapCandidates(ImDrawList* drawList, const ViewTransform& view,
                                   const SnapPointController& controller,
                                   int snappedIndex);

    /// Draw snap preview path
    static void drawSnapPreviewPath(ImDrawList* drawList, const ViewTransform& view,
                                    const SnapPointController& controller);
};

}  // namespace arborvia
