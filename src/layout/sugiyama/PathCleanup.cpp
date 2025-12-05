#include "PathCleanup.h"
#include <cmath>

namespace arborvia {

bool PathCleanup::isPointDuplicate(const Point& a, const Point& b) {
    return std::abs(a.x - b.x) < EPSILON && std::abs(a.y - b.y) < EPSILON;
}

bool PathCleanup::isSpike(const Point& a, const Point& b, const Point& c) {
    // Check vertical spike (all on same X)
    bool sameX = (std::abs(a.x - b.x) < EPSILON && std::abs(b.x - c.x) < EPSILON);
    if (sameX) {
        bool aToB_down = b.y > a.y;
        bool bToC_down = c.y > b.y;
        if (aToB_down != bToC_down) {
            return true;
        }
    }

    // Check horizontal spike (all on same Y)
    bool sameY = (std::abs(a.y - b.y) < EPSILON && std::abs(b.y - c.y) < EPSILON);
    if (sameY) {
        bool aToB_right = b.x > a.x;
        bool bToC_right = c.x > b.x;
        if (aToB_right != bToC_right) {
            return true;
        }
    }

    return false;
}

void PathCleanup::removeSpikesAndDuplicates(std::vector<Point>& points) {
    bool modified = true;
    while (modified && points.size() > 2) {
        modified = false;

        // Remove duplicate consecutive points
        for (size_t i = 0; i + 1 < points.size(); ++i) {
            if (isPointDuplicate(points[i], points[i + 1])) {
                points.erase(points.begin() + static_cast<long>(i + 1));
                modified = true;
                break;
            }
        }
        if (modified) continue;

        // Remove spike points (direction reversal on same line)
        for (size_t i = 0; i + 2 < points.size(); ++i) {
            if (isSpike(points[i], points[i + 1], points[i + 2])) {
                points.erase(points.begin() + static_cast<long>(i + 1));
                modified = true;
                break;
            }
        }
        if (modified) continue;

        // Remove redundant collinear intermediate points
        // If three consecutive points are on the same line (vertical or horizontal),
        // the middle point can be removed as it doesn't affect the path
        for (size_t i = 0; i + 2 < points.size(); ++i) {
            const Point& a = points[i];
            const Point& b = points[i + 1];
            const Point& c = points[i + 2];

            // Check if all three are on same vertical line
            bool sameX = (std::abs(a.x - b.x) < EPSILON && std::abs(b.x - c.x) < EPSILON);
            // Check if all three are on same horizontal line
            bool sameY = (std::abs(a.y - b.y) < EPSILON && std::abs(b.y - c.y) < EPSILON);

            if (sameX || sameY) {
                // Middle point is redundant, remove it
                points.erase(points.begin() + static_cast<long>(i + 1));
                modified = true;
                break;
            }
        }
        if (modified) continue;

        // Remove non-consecutive duplicate points and the circular detour between them
        // This is safe because if two points are identical, the path between them
        // is a closed loop that accomplishes nothing
        for (size_t i = 0; i + 2 < points.size(); ++i) {
            for (size_t j = i + 2; j < points.size(); ++j) {
                if (isPointDuplicate(points[i], points[j])) {
                    // Found duplicate at i and j - remove all points from i+1 to j (inclusive)
                    // This eliminates the circular detour between them
                    points.erase(points.begin() + static_cast<long>(i + 1),
                                 points.begin() + static_cast<long>(j + 1));
                    modified = true;
                    break;
                }
            }
            if (modified) break;
        }
    }
}

}  // namespace arborvia
