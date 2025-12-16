#pragma once

#include <arborvia/arborvia.h>
#include <arborvia/layout/api/IConstraintValidator.h>
#include <arborvia/layout/api/LayoutController.h>
#include <arborvia/layout/interactive/PathRoutingCoordinator.h>
#include <arborvia/layout/interactive/SnapPointController.h>
#include "../../src/layout/pathfinding/ObstacleMap.h"

#include <imgui.h>
#include <memory>
#include <unordered_map>
#include <vector>

namespace arborvia {

/// Hovered bend point information
struct HoveredBendPoint {
    EdgeId edgeId = INVALID_EDGE;
    int bendPointIndex = -1;  // -1 = invalid, 0+ = bend point index

    bool isValid() const { return edgeId != INVALID_EDGE && bendPointIndex >= 0; }
    void clear() { edgeId = INVALID_EDGE; bendPointIndex = -1; }
};

/// Hovered snap point information
struct HoveredSnapPoint {
    EdgeId edgeId = INVALID_EDGE;
    bool isSource = true;  // true = source snap, false = target snap

    bool isValid() const { return edgeId != INVALID_EDGE; }
    void clear() { edgeId = INVALID_EDGE; }
};

/// Bend point preview during edge creation
struct BendPointPreview {
    EdgeId edgeId = INVALID_EDGE;
    int insertIndex = -1;
    Point position{0, 0};
    bool active = false;

    void clear() {
        edgeId = INVALID_EDGE;
        insertIndex = -1;
        active = false;
    }
};

/// View transformation state (pan/zoom)
struct ViewTransform {
    Point panOffset = {0, 0};
    float zoom = 1.0f;
    Point screenOffset = {0, 0};  // Canvas offset in screen coordinates

    ImVec2 worldToScreen(const Point& world) const {
        return {
            (world.x + panOffset.x) * zoom + screenOffset.x,
            (world.y + panOffset.y) * zoom + screenOffset.y
        };
    }

    Point screenToWorld(const ImVec2& screen) const {
        return {
            (screen.x - screenOffset.x) / zoom - panOffset.x,
            (screen.y - screenOffset.y) / zoom - panOffset.y
        };
    }

    float worldToScreenScale(float worldSize) const {
        return worldSize * zoom;
    }
};

/// Interactive state (selection, drag, hover)
struct InteractionState {
    // Selection
    NodeId selectedNode = INVALID_NODE;
    EdgeId selectedEdge = INVALID_EDGE;
    HoveredBendPoint selectedBendPoint;

    // Hover
    NodeId hoveredNode = INVALID_NODE;
    EdgeId hoveredEdge = INVALID_EDGE;
    HoveredBendPoint hoveredBendPoint;
    HoveredSnapPoint hoveredSnapPoint;

    // Node drag
    NodeId draggedNode = INVALID_NODE;
    Point dragOffset = {0, 0};
    std::vector<EdgeId> affectedEdges;
    bool isInvalidDragPosition = false;
    Point lastValidPosition = {0, 0};
    Point lastRoutedPosition = {-9999, -9999};

    // Drag threshold
    static constexpr float DRAG_THRESHOLD = 5.0f;
    bool pendingNodeDrag = false;
    Point nodeClickStart = {0, 0};

    // Bend point drag
    HoveredBendPoint draggingBendPoint;
    Point bendPointDragOffset = {0, 0};
    BendPointPreview bendPointPreview;

    // Snap point drag
    HoveredSnapPoint draggingSnapPoint;
    Point snapPointDragOffset = {0, 0};
    Point snapPointDragStart = {0, 0};
    int snappedCandidateIndex = -1;
    bool hasSnapPreview = false;

    // Pan state
    bool isPanning = false;
    bool emptyAreaPanStarted = false;
};

/// Rendering options
struct RenderOptions {
    bool showSnapPoints = true;
    bool showSnapIndices = true;
    bool showBlockedCells = true;
    bool showAStarDebug = false;
    float gridSize = 20.0f;
};

/// A* debug visualization state
struct AStarDebugState {
    EdgeId debugEdgeId = INVALID_EDGE;
    std::shared_ptr<ObstacleMap> obstacles;
    Point start = {-1, -1};
    Point goal = {-1, -1};
};

/// Complete demo state shared between components
struct DemoState {
    // Core data
    Graph* graph = nullptr;
    std::unordered_map<NodeId, NodeLayout>* nodeLayouts = nullptr;
    std::unordered_map<EdgeId, EdgeLayout>* edgeLayouts = nullptr;
    LayoutOptions* layoutOptions = nullptr;

    // Controllers
    LayoutController* layoutController = nullptr;
    PathRoutingCoordinator* routingCoordinator = nullptr;
    SnapPointController* snapController = nullptr;

    // State
    ViewTransform view;
    InteractionState interaction;
    RenderOptions renderOptions;
    AStarDebugState astarDebug;
    std::vector<Rect> blockedRegions;

    // SCXML test state
    struct SCXMLState {
        bool modeActive = false;
        int currentTestIndex = -1;
    } scxml;
};

}  // namespace arborvia
