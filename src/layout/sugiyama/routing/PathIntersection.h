#pragma once

#include "arborvia/core/Types.h"
#include "arborvia/layout/config/LayoutResult.h"

#include <unordered_map>

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

    /// Information about an overlapping segment
    struct OverlapInfo {
        bool found = false;
        bool isVertical = false;      // true: vertical (same x), false: horizontal (same y)
        float sharedCoordinate = 0;   // The x (if vertical) or y (if horizontal) where overlap occurs
        float overlapStart = 0;       // Start of overlap range
        float overlapEnd = 0;         // End of overlap range
    };

    /// Find detailed overlap information between a candidate path and assigned edges
    /// @param candidate Edge layout to check
    /// @param assignedLayouts Already assigned edges
    /// @param excludeEdgeId Edge to exclude from check
    /// @return OverlapInfo with details about the first overlap found
    OverlapInfo findOverlapInfo(
        const EdgeLayout& candidate,
        const std::unordered_map<EdgeId, EdgeLayout>& assignedLayouts,
        EdgeId excludeEdgeId = INVALID_EDGE);

    /// Find all X coordinates used by vertical segments in assigned edges
    /// @param assignedLayouts Assigned edge layouts
    /// @param yMin Minimum Y to consider
    /// @param yMax Maximum Y to consider
    /// @return Set of X coordinates that have vertical segments in the Y range
    std::vector<float> findUsedVerticalX(
        const std::unordered_map<EdgeId, EdgeLayout>& assignedLayouts,
        float yMin, float yMax);

    /// Find all Y coordinates used by horizontal segments in assigned edges
    /// @param assignedLayouts Assigned edge layouts
    /// @param xMin Minimum X to consider
    /// @param xMax Maximum X to consider
    /// @return Set of Y coordinates that have horizontal segments in the X range
    std::vector<float> findUsedHorizontalY(
        const std::unordered_map<EdgeId, EdgeLayout>& assignedLayouts,
        float xMin, float xMax);

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

    /// Find alternative X coordinate that doesn't overlap with existing vertical segments
    /// @param originalX The X coordinate that causes overlap
    /// @param yMin Minimum Y of the segment
    /// @param yMax Maximum Y of the segment
    /// @param assignedLayouts Already assigned edges
    /// @param gridSpacing Grid spacing for offset calculation
    /// @return Alternative X coordinate
    float findAlternativeX(
        float originalX,
        float yMin,
        float yMax,
        const std::unordered_map<EdgeId, EdgeLayout>& assignedLayouts,
        float gridSpacing = 20.0f);

    /// Find alternative Y coordinate that doesn't overlap with existing horizontal segments
    /// @param originalY The Y coordinate that causes overlap
    /// @param xMin Minimum X of the segment
    /// @param xMax Maximum X of the segment
    /// @param assignedLayouts Already assigned edges
    /// @param gridSpacing Grid spacing for offset calculation
    /// @return Alternative Y coordinate
    float findAlternativeY(
        float originalY,
        float xMin,
        float xMax,
        const std::unordered_map<EdgeId, EdgeLayout>& assignedLayouts,
        float gridSpacing = 20.0f);

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
