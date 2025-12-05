#include "PathIntersection.h"
#include "arborvia/core/GeometryUtils.h"

#include <cmath>
#include <algorithm>

namespace arborvia {

using constants::EPSILON;

namespace {

    /// Counter-Clockwise orientation test
    /// @return >0 if CCW (left turn), <0 if CW (right turn), =0 if collinear
    float ccw(const Point& a, const Point& b, const Point& c) {
        // Cross product of vectors (b-a) and (c-a)
        // Positive = counter-clockwise, Negative = clockwise, Zero = collinear
        return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
    }

    /// Check if point c lies on segment ab (when collinear)
    bool onSegment(const Point& a, const Point& b, const Point& c) {
        // Check if point c lies on segment ab (assumes collinearity)
        return c.x >= std::min(a.x, b.x) - EPSILON &&
               c.x <= std::max(a.x, b.x) + EPSILON &&
               c.y >= std::min(a.y, b.y) - EPSILON &&
               c.y <= std::max(a.y, b.y) + EPSILON;
    }
}  // anonymous namespace


namespace PathIntersection {

bool segmentsIntersect(const Point& a1, const Point& a2,
                       const Point& b1, const Point& b2) {
    // Skip if segments share an endpoint (adjacent segments in path)
    if ((a1 == b1) || (a1 == b2) || (a2 == b1) || (a2 == b2)) {
        return false;
    }

    float d1 = ccw(b1, b2, a1);
    float d2 = ccw(b1, b2, a2);
    float d3 = ccw(a1, a2, b1);
    float d4 = ccw(a1, a2, b2);

    // General case: segments straddle each other
    if (((d1 > EPSILON && d2 < -EPSILON) || (d1 < -EPSILON && d2 > EPSILON)) &&
        ((d3 > EPSILON && d4 < -EPSILON) || (d3 < -EPSILON && d4 > EPSILON))) {
        return true;
    }

    // Collinear cases: check if endpoints lie on the other segment
    if (std::abs(d1) <= EPSILON && onSegment(b1, b2, a1)) return true;
    if (std::abs(d2) <= EPSILON && onSegment(b1, b2, a2)) return true;
    if (std::abs(d3) <= EPSILON && onSegment(a1, a2, b1)) return true;
    if (std::abs(d4) <= EPSILON && onSegment(a1, a2, b2)) return true;

    return false;
}

int countPathIntersections(const EdgeLayout& e1, const EdgeLayout& e2) {
    int count = 0;

    // Build segment lists for both edges
    std::vector<std::pair<Point, Point>> segments1, segments2;

    // Collect segments from edge 1
    e1.forEachSegment([&](const Point& p1, const Point& p2) {
        segments1.emplace_back(p1, p2);
    });

    // Collect segments from edge 2
    e2.forEachSegment([&](const Point& p1, const Point& p2) {
        segments2.emplace_back(p1, p2);
    });

    // Compare all segment pairs
    for (const auto& [a1, a2] : segments1) {
        for (const auto& [b1, b2] : segments2) {
            if (segmentsIntersect(a1, a2, b1, b2)) {
                ++count;
            }
        }
    }

    return count;
}

int countIntersectionsWithOthers(
    const EdgeLayout& edge,
    const std::unordered_map<EdgeId, EdgeLayout>& otherEdges,
    EdgeId excludeEdgeId) {

    int total = 0;

    for (const auto& [edgeId, otherLayout] : otherEdges) {
        if (edgeId == excludeEdgeId) {
            continue;
        }
        total += countPathIntersections(edge, otherLayout);
    }

    return total;
}

}  // namespace PathIntersection


}  // namespace arborvia
