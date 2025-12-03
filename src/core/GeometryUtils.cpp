#include "arborvia/core/GeometryUtils.h"

#include <algorithm>
#include <cmath>

namespace arborvia::geometry {

bool segmentIntersectsAABB(
    const Point& p1,
    const Point& p2,
    const Point& boxPos,
    const Size& boxSize) {

    float boxRight = boxPos.x + boxSize.width;
    float boxBottom = boxPos.y + boxSize.height;

    // Get segment bounding box
    float segMinX = std::min(p1.x, p2.x);
    float segMaxX = std::max(p1.x, p2.x);
    float segMinY = std::min(p1.y, p2.y);
    float segMaxY = std::max(p1.y, p2.y);

    // Quick rejection: bounding boxes don't overlap
    if (segMaxX < boxPos.x || segMinX > boxRight ||
        segMaxY < boxPos.y || segMinY > boxBottom) {
        return false;
    }

    // For orthogonal segments, if bounding boxes overlap, segment intersects
    bool isVertical = std::abs(p1.x - p2.x) < 0.001f;
    bool isHorizontal = std::abs(p1.y - p2.y) < 0.001f;

    if (isVertical || isHorizontal) {
        return true;
    }

    // For diagonal segments, check if corners are on opposite sides
    auto sign = [](float v) { return v > 0 ? 1 : (v < 0 ? -1 : 0); };

    float dx = p2.x - p1.x;
    float dy = p2.y - p1.y;

    auto crossSign = [&](float cx, float cy) {
        return sign((cx - p1.x) * dy - (cy - p1.y) * dx);
    };

    int s1 = crossSign(boxPos.x, boxPos.y);
    int s2 = crossSign(boxRight, boxPos.y);
    int s3 = crossSign(boxRight, boxBottom);
    int s4 = crossSign(boxPos.x, boxBottom);

    if (s1 == s2 && s2 == s3 && s3 == s4 && s1 != 0) {
        return false;
    }

    return true;
}

bool edgePathIntersectsAABB(
    const EdgeLayout& layout,
    const Point& boxPos,
    const Size& boxSize) {

    bool intersects = false;
    layout.forEachSegment([&](const Point& p1, const Point& p2) {
        if (!intersects && segmentIntersectsAABB(p1, p2, boxPos, boxSize)) {
            intersects = true;
        }
    });
    return intersects;
}

float pointToAABBDistance(
    const Point& point,
    const Point& boxPos,
    const Size& boxSize) {

    float boxRight = boxPos.x + boxSize.width;
    float boxBottom = boxPos.y + boxSize.height;

    // Calculate closest point on box to query point
    float closestX = std::clamp(point.x, boxPos.x, boxRight);
    float closestY = std::clamp(point.y, boxPos.y, boxBottom);

    // Calculate distance
    float dx = point.x - closestX;
    float dy = point.y - closestY;

    return std::sqrt(dx * dx + dy * dy);
}

bool aabbOverlap(
    const Point& pos1, const Size& size1,
    const Point& pos2, const Size& size2) {

    float right1 = pos1.x + size1.width;
    float bottom1 = pos1.y + size1.height;
    float right2 = pos2.x + size2.width;
    float bottom2 = pos2.y + size2.height;

    return !(right1 <= pos2.x || pos1.x >= right2 ||
             bottom1 <= pos2.y || pos1.y >= bottom2);
}

void aabbGapDistance(
    const Point& pos1, const Size& size1,
    const Point& pos2, const Size& size2,
    float& outDx, float& outDy) {

    float right1 = pos1.x + size1.width;
    float bottom1 = pos1.y + size1.height;
    float right2 = pos2.x + size2.width;
    float bottom2 = pos2.y + size2.height;

    // Calculate horizontal gap
    if (right1 <= pos2.x) {
        outDx = pos2.x - right1;
    } else if (right2 <= pos1.x) {
        outDx = pos1.x - right2;
    } else {
        outDx = 0.0f;  // Overlapping horizontally
    }

    // Calculate vertical gap
    if (bottom1 <= pos2.y) {
        outDy = pos2.y - bottom1;
    } else if (bottom2 <= pos1.y) {
        outDy = pos1.y - bottom2;
    } else {
        outDy = 0.0f;  // Overlapping vertically
    }
}

}  // namespace arborvia::geometry
