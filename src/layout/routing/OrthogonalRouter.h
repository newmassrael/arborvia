#pragma once

#include "arborvia/layout/config/LayoutResult.h"
#include <vector>

namespace arborvia {

/// Orthogonal routing calculations for bend points.
/// Provides utility functions for creating and maintaining
/// orthogonal (90-degree) edge routing in graph layouts.
namespace OrthogonalRouter {

/// Constants for geometric calculations
constexpr float EPSILON = 0.001f;           ///< Tolerance for floating-point comparison
constexpr float COLLINEAR_THRESHOLD = 1.0f; ///< Threshold for collinear point detection

/// Result of bend point pair calculation for orthogonal insertion
struct BendPointPairResult {
    Point first{0, 0};      ///< First bend point to insert
    Point second{0, 0};     ///< Second bend point to insert
    size_t insertIndex = 0; ///< Index at which to insert (first goes here, second at insertIndex+1)
};

/// Result of orthogonal drag constraint calculation
struct OrthogonalDragResult {
    Point newCurrentPos{0, 0};      ///< Constrained position for the dragged bend point
    Point adjustedNextPos{0, 0};    ///< Adjusted position for the next bend point (if applicable)
    bool nextAdjusted = false;      ///< True if next bend point position was adjusted
};

/// Calculate a pair of bend points for orthogonal routing at click position.
/// This creates two bend points that maintain orthogonality when inserted into the path.
/// @param edgeLayout The edge layout (provides source/target points)
/// @param existingBendPoints Current manual bend points (may be empty)
/// @param clickPosition Where the user clicked to insert
/// @param segmentIndex Which segment was clicked (from EdgeHitResult)
/// @return BendPointPairResult with calculated positions and insert index
BendPointPairResult calculateBendPointPair(
    const EdgeLayout& edgeLayout,
    const std::vector<BendPoint>& existingBendPoints,
    const Point& clickPosition,
    int segmentIndex);

/// Calculate constrained drag position to maintain orthogonality.
/// This is the core logic for orthogonal bend point dragging.
/// @param prevPoint The previous point in the path (source or previous bend)
/// @param currentPos Current position of the bend point being dragged
/// @param nextPoint The next point in the path (next bend or target)
/// @param dragTarget Where the user is trying to drag to
/// @param hasNextBend True if there is a next bend point that can be adjusted
/// @param isLastBend True if this is the last bend point (next is target)
/// @return Constrained positions that maintain orthogonality
OrthogonalDragResult calculateOrthogonalDrag(
    const Point& prevPoint,
    const Point& currentPos,
    const Point& nextPoint,
    const Point& dragTarget,
    bool hasNextBend,
    bool isLastBend);

}  // namespace OrthogonalRouter

}  // namespace arborvia
