#pragma once

#include "Types.h"
#include "../layout/LayoutResult.h"

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
namespace constants {

/// Floating-point comparison tolerance
/// Used across geometry and layout calculations
constexpr float EPSILON = 1e-6f;

/// Minimum distance between nodes in grid units
/// Used by MinDistanceConstraint and ValidRegionCalculator
constexpr float MIN_NODE_GRID_DISTANCE = 5.0f;

/// Internal grid size for A* pathfinding algorithms
/// Independent of user's GridConfig.cellSize setting
/// Used when user grid is disabled (cellSize <= 0)
constexpr float PATHFINDING_GRID_SIZE = 10.0f;

}  // namespace constants

}  // namespace arborvia
