#pragma once

#include <imgui.h>

namespace arborvia {

/// Centralized color constants for demo rendering
/// All rendering components should use these colors for consistency
namespace DemoColors {
    // Node colors
    constexpr ImU32 NODE = IM_COL32(200, 200, 200, 255);
    constexpr ImU32 NODE_HOVER = IM_COL32(150, 200, 255, 255);
    constexpr ImU32 NODE_DRAG = IM_COL32(100, 180, 255, 255);
    constexpr ImU32 NODE_BORDER = IM_COL32(80, 80, 80, 255);
    constexpr ImU32 NODE_SELECTED = IM_COL32(100, 255, 100, 255);
    constexpr ImU32 NODE_INVALID = IM_COL32(255, 80, 80, 255);

    // Edge colors
    constexpr ImU32 EDGE = IM_COL32(100, 100, 100, 255);
    constexpr ImU32 EDGE_SELECTED = IM_COL32(0, 200, 100, 255);
    constexpr ImU32 EDGE_AFFECTED = IM_COL32(255, 100, 100, 255);

    // Text
    constexpr ImU32 TEXT = IM_COL32(0, 0, 0, 255);

    // Snap points
    constexpr ImU32 SNAP_POINT = IM_COL32(0, 150, 255, 200);
    constexpr ImU32 SNAP_POINT_HOVER = IM_COL32(255, 200, 0, 255);
    // Source snap point (green)
    constexpr ImU32 SNAP_SOURCE = IM_COL32(100, 200, 100, 255);
    constexpr ImU32 SNAP_SOURCE_BORDER = IM_COL32(50, 150, 50, 255);
    // Target snap point (red)
    constexpr ImU32 SNAP_TARGET = IM_COL32(200, 100, 100, 255);
    constexpr ImU32 SNAP_TARGET_BORDER = IM_COL32(150, 50, 50, 255);
    // Hovered/dragging snap point (yellow/orange)
    constexpr ImU32 SNAP_HOVER = IM_COL32(255, 220, 100, 255);
    constexpr ImU32 SNAP_HOVER_BORDER = IM_COL32(200, 170, 50, 255);
    constexpr ImU32 SNAP_DRAG = IM_COL32(255, 180, 80, 255);
    constexpr ImU32 SNAP_DRAG_BORDER = IM_COL32(200, 130, 50, 255);

    // Grid
    constexpr ImU32 GRID = IM_COL32(200, 200, 200, 80);
    constexpr ImU32 GRID_MAJOR = IM_COL32(180, 180, 180, 120);

    // Blocked cells (obstacle visualization)
    constexpr ImU32 BLOCKED_CELL = IM_COL32(255, 100, 100, 60);

    // Bend points
    constexpr ImU32 BEND_POINT = IM_COL32(100, 150, 255, 200);
    constexpr ImU32 BEND_POINT_HOVER = IM_COL32(150, 200, 255, 255);
    constexpr ImU32 BEND_POINT_SELECTED = IM_COL32(100, 255, 100, 255);
    constexpr ImU32 BEND_POINT_DRAGGING = IM_COL32(255, 180, 100, 255);
    constexpr ImU32 BEND_POINT_PREVIEW = IM_COL32(100, 200, 255, 128);

    // A* debug visualization
    constexpr ImU32 ASTAR_NODE_BLOCK = IM_COL32(255, 200, 0, 80);
    constexpr ImU32 ASTAR_H_SEGMENT = IM_COL32(100, 100, 255, 100);
    constexpr ImU32 ASTAR_V_SEGMENT = IM_COL32(100, 255, 100, 100);
    constexpr ImU32 ASTAR_START = IM_COL32(0, 255, 0, 200);
    constexpr ImU32 ASTAR_GOAL = IM_COL32(255, 0, 0, 200);

    // Snap preview path
    constexpr ImU32 SNAP_PREVIEW_PATH = IM_COL32(0, 200, 255, 180);

    // Snap candidates
    constexpr ImU32 SNAP_CANDIDATE = IM_COL32(150, 150, 255, 150);
    constexpr ImU32 SNAP_CANDIDATE_BORDER = IM_COL32(100, 100, 200, 200);
    constexpr ImU32 SNAP_CANDIDATE_SNAPPED = IM_COL32(255, 100, 100, 255);
    constexpr ImU32 SNAP_CANDIDATE_SNAPPED_BORDER = IM_COL32(255, 255, 255, 255);

    // Node ID label background
    constexpr ImU32 ID_LABEL_BG = IM_COL32(255, 255, 200, 220);
    constexpr ImU32 ID_LABEL_BORDER = IM_COL32(100, 100, 0, 200);

    // Snap index label
    constexpr ImU32 SNAP_LABEL_BG = IM_COL32(255, 255, 255, 240);
    constexpr ImU32 SNAP_LABEL_SOURCE_BORDER = IM_COL32(40, 140, 40, 255);
    constexpr ImU32 SNAP_LABEL_SOURCE_TEXT = IM_COL32(20, 100, 20, 255);
    constexpr ImU32 SNAP_LABEL_TARGET_BORDER = IM_COL32(180, 40, 40, 255);
    constexpr ImU32 SNAP_LABEL_TARGET_TEXT = IM_COL32(160, 20, 20, 255);
}

/// Visual constants for demo rendering
namespace DemoVisuals {
    // Point node visual radius (used for both rendering and hit detection)
    constexpr float POINT_NODE_RADIUS = 10.0f;

    // Node rendering
    constexpr float NODE_ROUNDING = 5.0f;
    constexpr float NODE_BORDER_THICKNESS = 2.0f;

    // Edge rendering
    constexpr float EDGE_THICKNESS = 2.0f;
    constexpr float EDGE_THICKNESS_SELECTED = 3.0f;
    constexpr float EDGE_THICKNESS_HOVERED = 2.5f;
    constexpr float ARROW_SIZE = 10.0f;

    // Snap point rendering
    constexpr float SNAP_RADIUS = 5.0f;
    constexpr float SNAP_RADIUS_HOVERED = 7.0f;
    constexpr float SNAP_RADIUS_DRAGGING = 8.0f;
    constexpr float SNAP_BORDER_THICKNESS = 1.5f;
    constexpr float SNAP_LABEL_OFFSET = 8.0f;
    constexpr float SNAP_LABEL_PADDING_X = 3.0f;
    constexpr float SNAP_LABEL_PADDING_Y = 2.0f;
    constexpr float SNAP_LABEL_ROUNDING = 3.0f;
    constexpr float SNAP_LABEL_BORDER_THICKNESS = 2.0f;
    constexpr float SNAP_CANDIDATE_RADIUS = 4.0f;
    constexpr float SNAP_CANDIDATE_RADIUS_SNAPPED = 8.0f;
    constexpr float SNAP_CANDIDATE_BORDER_THICKNESS = 1.0f;
    constexpr float SNAP_CANDIDATE_BORDER_THICKNESS_SNAPPED = 2.0f;
    constexpr float SNAP_PREVIEW_THICKNESS = 3.0f;
    constexpr float SNAP_PREVIEW_ARROW_SIZE = 8.0f;

    // Bend point rendering
    constexpr float BEND_PREVIEW_SIZE = 5.0f;
    constexpr float BEND_PLUS_SIZE = 3.0f;

    // Label padding
    constexpr float LABEL_PADDING = 2.0f;
}

}  // namespace arborvia
