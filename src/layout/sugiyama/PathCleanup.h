#pragma once

#include "arborvia/core/Types.h"
#include <vector>

namespace arborvia {

/// Utility functions for cleaning up edge paths
/// Removes spikes, duplicates, and redundant points from orthogonal paths
class PathCleanup {
public:
    /// Tolerance for floating point comparisons
    static constexpr float EPSILON = 0.1f;

    /// Check if two points are essentially the same (within tolerance)
    /// @param a First point
    /// @param b Second point
    /// @return true if points are within EPSILON distance
    static bool isPointDuplicate(const Point& a, const Point& b);

    /// Check if three collinear points form a spike (direction reversal)
    /// @param a First point
    /// @param b Middle point (potential spike)
    /// @param c Third point
    /// @return true if b is a spike between a and c
    static bool isSpike(const Point& a, const Point& b, const Point& c);

    /// Remove spikes, duplicate points, and circular detours from a path
    /// A circular detour is when the path returns to a previously visited point
    /// @param points Path points to clean (modified in place)
    static void removeSpikesAndDuplicates(std::vector<Point>& points);
};

}  // namespace arborvia
