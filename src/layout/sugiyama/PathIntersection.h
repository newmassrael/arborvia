#pragma once

#include "arborvia/core/Types.h"
#include "arborvia/layout/LayoutResult.h"

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

}  // namespace PathIntersection


}  // namespace arborvia
