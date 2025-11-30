#include "arborvia/layout/OrthogonalRouter.h"
#include <cmath>
#include <vector>

namespace arborvia {
namespace OrthogonalRouter {

OrthogonalDragResult calculateOrthogonalDrag(
    const Point& prevPoint,
    const Point& currentPos,
    const Point& nextPoint,
    const Point& dragTarget,
    bool hasNextBend,
    bool isLastBend)
{
    OrthogonalDragResult result;

    // Determine incoming segment direction
    // Using EPSILON to handle edge case where dx == dy (45 degrees)
    // In that case, we default to treating it as horizontal
    float incomingDx = std::abs(currentPos.x - prevPoint.x);
    float incomingDy = std::abs(currentPos.y - prevPoint.y);
    bool incomingHorizontal = (incomingDx > incomingDy + EPSILON) || 
                               (std::abs(incomingDx - incomingDy) <= EPSILON);

    if (isLastBend) {
        // For the last bend point, must maintain orthogonality with BOTH prev and target
        float outgoingDx = std::abs(currentPos.x - nextPoint.x);
        float outgoingDy = std::abs(currentPos.y - nextPoint.y);
        bool outgoingHorizontal = (outgoingDx > outgoingDy + EPSILON) ||
                                   (std::abs(outgoingDx - outgoingDy) <= EPSILON);

        if (incomingHorizontal && !outgoingHorizontal) {
            // Incoming horizontal, outgoing vertical
            // Keep Y = prev.y (for horizontal) and X = target.x (for vertical)
            result.newCurrentPos = {nextPoint.x, prevPoint.y};
        } else if (!incomingHorizontal && outgoingHorizontal) {
            // Incoming vertical, outgoing horizontal
            // Keep X = prev.x (for vertical) and Y = target.y (for horizontal)
            result.newCurrentPos = {prevPoint.x, nextPoint.y};
        } else if (incomingHorizontal && outgoingHorizontal) {
            // Both horizontal - only change X freely
            result.newCurrentPos = {dragTarget.x, prevPoint.y};
        } else {
            // Both vertical - only change Y freely
            result.newCurrentPos = {prevPoint.x, dragTarget.y};
        }
    } else {
        // Not the last bend - constrain based on incoming direction only
        if (incomingHorizontal) {
            // Incoming is horizontal - keep Y same as prevPoint
            result.newCurrentPos = {dragTarget.x, prevPoint.y};
        } else {
            // Incoming is vertical - keep X same as prevPoint
            result.newCurrentPos = {prevPoint.x, dragTarget.y};
        }
    }

    // Adjust the next bend point to maintain orthogonality on outgoing segment
    if (hasNextBend) {
        if (incomingHorizontal) {
            // Outgoing should be vertical, so next.x = current.x
            result.adjustedNextPos = {result.newCurrentPos.x, nextPoint.y};
        } else {
            // Outgoing should be horizontal, so next.y = current.y
            result.adjustedNextPos = {nextPoint.x, result.newCurrentPos.y};
        }
        result.nextAdjusted = true;
    }

    return result;
}

BendPointPairResult calculateBendPointPair(
    const EdgeLayout& edgeLayout,
    const std::vector<BendPoint>& existingBendPoints,
    const Point& clickPosition,
    int segmentIndex)
{
    BendPointPairResult result;
    Point bp1, bp2;

    if (existingBendPoints.empty()) {
        // First insertion - calculate relative to source and target only
        Point source = edgeLayout.sourcePoint;
        Point target = edgeLayout.targetPoint;

        float dx = std::abs(target.x - source.x);
        float dy = std::abs(target.y - source.y);

        if (dx > dy) {
            // More horizontal - create vertical step
            bp1 = {clickPosition.x, source.y};
            bp2 = {clickPosition.x, target.y};
        } else {
            // More vertical - create horizontal step
            bp1 = {source.x, clickPosition.y};
            bp2 = {target.x, clickPosition.y};
        }
        result.insertIndex = 0;
    } else {
        // Existing manual bends - use current path structure
        // Build path: source -> [manual bends] -> target
        std::vector<Point> path;
        path.push_back(edgeLayout.sourcePoint);
        for (const auto& bp : existingBendPoints) {
            path.push_back(bp.position);
        }
        path.push_back(edgeLayout.targetPoint);

        // Validate segmentIndex range
        int maxSegmentIndex = static_cast<int>(path.size()) - 2;
        if (segmentIndex < 0 || segmentIndex > maxSegmentIndex) {
            // Invalid segment index - use first segment as fallback
            segmentIndex = 0;
        }

        Point prevPoint = path[segmentIndex];
        Point nextPoint = path[segmentIndex + 1];

        float dx = std::abs(nextPoint.x - prevPoint.x);
        float dy = std::abs(nextPoint.y - prevPoint.y);

        if (dx > dy) {
            // More horizontal segment
            bp1 = {clickPosition.x, prevPoint.y};
            bp2 = {clickPosition.x, nextPoint.y};
            if (std::abs(prevPoint.y - nextPoint.y) < COLLINEAR_THRESHOLD) {
                bp2 = {clickPosition.x, clickPosition.y};
            }
        } else {
            // More vertical segment
            bp1 = {prevPoint.x, clickPosition.y};
            bp2 = {nextPoint.x, clickPosition.y};
            if (std::abs(prevPoint.x - nextPoint.x) < COLLINEAR_THRESHOLD) {
                bp2 = {clickPosition.x, clickPosition.y};
            }
        }
        result.insertIndex = static_cast<size_t>(segmentIndex);
    }

    result.first = bp1;
    result.second = bp2;
    return result;
}

}  // namespace OrthogonalRouter
}  // namespace arborvia
