#pragma once

#include "arborvia/core/Types.h"
#include "arborvia/layout/config/LayoutResult.h"

#include <unordered_map>
#include <unordered_set>

namespace arborvia {


/// Utilities for detecting path and segment intersections
/// Uses CCW (Counter-Clockwise) algorithm for robust intersection detection
namespace PathIntersection {

    /// Check if two line segments intersect (excluding shared endpoints)
    /// Uses CCW orientation test for numerical stability
    /// @param a1 Start point of first segment
    /// @param a2 End point of first segment
    /// @param b1 Start point of second segment
    /// @param b2 End point of second segment
    /// @return true if segments cross each other (not just touch at endpoints)
    bool segmentsIntersect(const Point& a1, const Point& a2,
                           const Point& b1, const Point& b2);

    /// Count intersections between two edge paths
    /// Compares all segments of both paths against each other
    /// @param e1 First edge layout
    /// @param e2 Second edge layout
    /// @return Number of intersection points between the two paths
    int countPathIntersections(const EdgeLayout& e1, const EdgeLayout& e2);

    /// Count total intersections of one edge against all other edges
    /// @param edge Edge to check
    /// @param otherEdges Map of other edge layouts (excludes self automatically)
    /// @param excludeEdgeId Edge ID to exclude from checking (usually the edge being evaluated)
    /// @return Total intersection count
    int countIntersectionsWithOthers(
        const EdgeLayout& edge,
        const std::unordered_map<EdgeId, EdgeLayout>& otherEdges,
        EdgeId excludeEdgeId = INVALID_EDGE);

    // =========================================================================
    // Segment Overlap Detection (CONSTRAINT: segments must not share same path)
    // =========================================================================

    /// Check if two collinear segments overlap (share any portion of the same line)
    /// CONSTRAINT: Partial overlap is also forbidden
    /// @param a1 Start point of first segment
    /// @param a2 End point of first segment
    /// @param b1 Start point of second segment
    /// @param b2 End point of second segment
    /// @return true if segments overlap (share same line portion)
    bool segmentsOverlap(const Point& a1, const Point& a2,
                         const Point& b1, const Point& b2);

    /// Check if two edge paths have any overlapping segments
    /// CONSTRAINT: Any segment overlap means the paths cannot coexist
    /// @param e1 First edge layout
    /// @param e2 Second edge layout
    /// @return true if any segments overlap
    bool hasSegmentOverlap(const EdgeLayout& e1, const EdgeLayout& e2);

    /// Check if two edge paths have overlapping segments, excluding the last N segments
    /// This allows edges going to the same target to share the final approach
    /// @param e1 First edge layout
    /// @param e2 Second edge layout
    /// @param excludeLastN Number of segments from the end to exclude from checking
    /// @return true if any non-excluded segments overlap
    bool hasSegmentOverlapExcludingLast(const EdgeLayout& e1, const EdgeLayout& e2, int excludeLastN);

    /// Check if an edge has segment overlaps with any other assigned edges
    /// @param edge Edge to check
    /// @param otherEdges Map of other edge layouts
    /// @param excludeEdgeId Edge ID to exclude from checking
    /// @return true if overlap found with any other edge
    bool hasOverlapWithOthers(
        const EdgeLayout& edge,
        const std::unordered_map<EdgeId, EdgeLayout>& otherEdges,
        EdgeId excludeEdgeId = INVALID_EDGE);

    // =========================================================================
    // Overlap Avoidance Helpers
    // =========================================================================

    /// Information about an overlapping segment (grid-based for internal calculations)
    struct OverlapInfo {
        bool found = false;
        bool isVertical = false;      // true: vertical (same x), false: horizontal (same y)
        int sharedGridCoord = 0;      // The grid x (if vertical) or y (if horizontal) where overlap occurs
        int overlapStartGrid = 0;     // Start of overlap range (grid units)
        int overlapEndGrid = 0;       // End of overlap range (grid units)
    };

    /// Find detailed overlap information between a candidate path and assigned edges
    /// Returns grid-based overlap info for use with grid calculation functions
    /// @param candidate Edge layout to check
    /// @param assignedLayouts Already assigned edges
    /// @param excludeEdgeId Edge to exclude from check
    /// @param gridSize Grid cell size for coordinate conversion (default 20.0f)
    /// @return OverlapInfo with grid coordinates about the first overlap found
    OverlapInfo findOverlapInfo(
        const EdgeLayout& candidate,
        const std::unordered_map<EdgeId, EdgeLayout>& assignedLayouts,
        EdgeId excludeEdgeId = INVALID_EDGE,
        float gridSize = 20.0f);

    /// Find all grid X coordinates used by vertical segments in assigned edges
    /// @param assignedLayouts Assigned edge layouts
    /// @param yMinGrid Minimum Y to consider (grid units)
    /// @param yMaxGrid Maximum Y to consider (grid units)
    /// @param gridSize Grid cell size for coordinate conversion
    /// @return Set of grid X coordinates that have vertical segments in the Y range
    std::unordered_set<int> findUsedVerticalGridX(
        const std::unordered_map<EdgeId, EdgeLayout>& assignedLayouts,
        int yMinGrid, int yMaxGrid,
        float gridSize);

    /// Find all grid Y coordinates used by horizontal segments in assigned edges
    /// @param assignedLayouts Assigned edge layouts
    /// @param xMinGrid Minimum X to consider (grid units)
    /// @param xMaxGrid Maximum X to consider (grid units)
    /// @param gridSize Grid cell size for coordinate conversion
    /// @return Set of grid Y coordinates that have horizontal segments in the X range
    std::unordered_set<int> findUsedHorizontalGridY(
        const std::unordered_map<EdgeId, EdgeLayout>& assignedLayouts,
        int xMinGrid, int xMaxGrid,
        float gridSize);

    // =========================================================================
    // Overlap Adjustment (move segments to avoid overlap)
    // =========================================================================

    /// Adjust bend points to avoid overlap with assigned edges
    /// @param candidate Edge layout to adjust
    /// @param assignedLayouts Already assigned edges
    /// @param gridSpacing Grid spacing for offset calculation
    /// @return Adjusted bend points that avoid overlap
    std::vector<BendPoint> adjustPathToAvoidOverlap(
        const EdgeLayout& candidate,
        const std::unordered_map<EdgeId, EdgeLayout>& assignedLayouts,
        float gridSpacing = 20.0f);

    /// Find alternative grid X coordinate that doesn't overlap with existing vertical segments
    /// Works entirely in grid space for guaranteed alignment
    /// @param originalGridX The grid X coordinate that causes overlap
    /// @param yMinGrid Minimum Y of the segment (grid units)
    /// @param yMaxGrid Maximum Y of the segment (grid units)
    /// @param assignedLayouts Already assigned edges
    /// @param gridSize Grid cell size for coordinate conversion
    /// @return Alternative grid X coordinate (int)
    int findAlternativeGridX(
        int originalGridX,
        int yMinGrid,
        int yMaxGrid,
        const std::unordered_map<EdgeId, EdgeLayout>& assignedLayouts,
        float gridSize);

    /// Find alternative grid Y coordinate that doesn't overlap with existing horizontal segments
    /// Works entirely in grid space for guaranteed alignment
    /// @param originalGridY The grid Y coordinate that causes overlap
    /// @param xMinGrid Minimum X of the segment (grid units)
    /// @param xMaxGrid Maximum X of the segment (grid units)
    /// @param assignedLayouts Already assigned edges
    /// @param gridSize Grid cell size for coordinate conversion
    /// @return Alternative grid Y coordinate (int)
    int findAlternativeGridY(
        int originalGridY,
        int xMinGrid,
        int xMaxGrid,
        const std::unordered_map<EdgeId, EdgeLayout>& assignedLayouts,
        float gridSize);

    // =========================================================================
    // Bulk Overlap Detection (with bounding box optimization)
    // =========================================================================

    /// Find all pairs of edges that have overlapping segments
    /// Uses bounding box pre-filtering for O(n²) performance with early rejection
    /// @param edgeLayouts All edge layouts to check
    /// @return Vector of edge ID pairs that overlap
    std::vector<std::pair<EdgeId, EdgeId>> findAllOverlappingPairs(
        const std::unordered_map<EdgeId, EdgeLayout>& edgeLayouts);

}  // namespace PathIntersection


}  // namespace arborvia
