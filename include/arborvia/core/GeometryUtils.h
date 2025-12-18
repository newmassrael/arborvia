#pragma once

#include "Types.h"
#include "../layout/config/LayoutResult.h"

#include <cmath>

namespace arborvia {

/// Geometry utility functions for collision detection and spatial queries
namespace geometry {

/// Check if a line segment intersects an axis-aligned bounding box
/// Uses separating axis theorem for orthogonal segments
/// @param p1 Start point of segment
/// @param p2 End point of segment
/// @param boxPos Top-left corner of the box
/// @param boxSize Size of the box
/// @return true if segment intersects the box
bool segmentIntersectsAABB(
    const Point& p1,
    const Point& p2,
    const Point& boxPos,
    const Size& boxSize);

/// Check if any segment of an edge path intersects the given AABB
/// @param layout The edge layout containing the path
/// @param boxPos Top-left corner of the box
/// @param boxSize Size of the box
/// @return true if any segment of the path intersects the box
bool edgePathIntersectsAABB(
    const EdgeLayout& layout,
    const Point& boxPos,
    const Size& boxSize);

/// Calculate the distance from a point to an axis-aligned bounding box
/// @param point The query point
/// @param boxPos Top-left corner of the box
/// @param boxSize Size of the box
/// @return Distance to the box (0 if point is inside)
float pointToAABBDistance(
    const Point& point,
    const Point& boxPos,
    const Size& boxSize);

/// Check if two AABBs overlap
/// @param pos1 Top-left of first box
/// @param size1 Size of first box
/// @param pos2 Top-left of second box
/// @param size2 Size of second box
/// @return true if boxes overlap
bool aabbOverlap(
    const Point& pos1, const Size& size1,
    const Point& pos2, const Size& size2);

/// Check if a segment penetrates a node's interior (not just boundary touch)
/// This is the single source of truth for directional penetration checking.
/// The node bounds are shrunk by tolerance to allow boundary touches.
/// @param p1 Start point of segment
/// @param p2 End point of segment
/// @param node The node layout to check against
/// @param tolerance Amount to shrink node bounds (default 1.0f for 1 pixel margin)
/// @return true if segment penetrates the node's interior
bool segmentPenetratesNodeInterior(
    const Point& p1,
    const Point& p2,
    const NodeLayout& node,
    float tolerance = 1.0f);

/// Calculate AABB gap distance (minimum distance between bounding boxes)
/// @param pos1 Top-left of first box
/// @param size1 Size of first box
/// @param pos2 Top-left of second box
/// @param size2 Size of second box
/// @param outDx Output: horizontal gap (0 if overlapping)
/// @param outDy Output: vertical gap (0 if overlapping)
void aabbGapDistance(
    const Point& pos1, const Size& size1,
    const Point& pos2, const Size& size2,
    float& outDx, float& outDy);

}  // namespace geometry

/// Layout constraint constants
///
/// Grid Size System:
/// - PATHFINDING_GRID_SIZE (10.0f): Default for A* pathfinding when no value specified
/// - User gridSize = 0.0f: Indicates "use default" (PATHFINDING_GRID_SIZE)
/// - User gridSize > 0.0f: Use explicit value
/// - OptimizerConfig presets: aggressive=30, balanced=20, conservative=10
/// - Use effectiveGridSize(gridSize) to resolve 0.0f to PATHFINDING_GRID_SIZE
namespace constants {

/// Floating-point comparison tolerance
/// Used across geometry and layout calculations
constexpr float EPSILON = 1e-6f;

/// Minimum distance between nodes in grid units
/// Used by DirectionAwareMarginConstraint and other drag constraints
constexpr float MIN_NODE_GRID_DISTANCE = 5.0f;

/// Internal grid size for A* pathfinding algorithms
/// Independent of user's GridConfig.cellSize setting
/// Used when user grid is disabled (cellSize <= 0)
constexpr float PATHFINDING_GRID_SIZE = 10.0f;

/// Get effective grid size, using PATHFINDING_GRID_SIZE when input is invalid
/// This is the single source of truth for gridSize fallback logic.
/// @param gridSize User-specified grid size (0, negative, or NaN = use default)
/// @return Effective grid size to use (always valid positive value)
inline float effectiveGridSize(float gridSize) noexcept {
    return (gridSize > 0.0f && std::isfinite(gridSize)) ? gridSize : PATHFINDING_GRID_SIZE;
}

/// Snap index value indicating redistribution is needed
/// Used when edge routing changes and snap points must be recalculated
constexpr int SNAP_INDEX_UNASSIGNED = -1;

/// Snap index for Point node center position
/// Point nodes always have all edges at center, sharing this index.
/// This is intentional - do NOT use snapIndex for collision detection on Point nodes.
constexpr int SNAP_INDEX_POINT_NODE_CENTER = 0;

}  // namespace constants

/// Grid coordinate conversion utilities
/// These complement GridPoint::fromPixel/toPixel methods in Types.h
namespace grid {

/// Convert pixel distance to grid cells (ceil for safety margin)
/// Use this when calculating clearance or margin in grid units
/// @param pixels Distance in pixels
/// @param gridSize Size of each grid cell
/// @return Number of cells (rounded up to ensure sufficient space)
inline int pixelToCells(float pixels, float gridSize) noexcept {
    if (gridSize <= 0.0f) return 0;
    return static_cast<int>(std::ceil(pixels / gridSize));
}

}  // namespace grid

}  // namespace arborvia
