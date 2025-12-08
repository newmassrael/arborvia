#pragma once

#include "arborvia/core/Types.h"
#include "arborvia/layout/LayoutResult.h"
#include <vector>

namespace arborvia {

/// Utility functions for cleaning up edge paths
/// Removes spikes, duplicates, and redundant points from orthogonal paths
class PathCleanup {
public:
    /// Tolerance for floating point comparisons
    static constexpr float EPSILON = 0.1f;

    /// Default margin for keeping bends outside nodes
    static constexpr float DEFAULT_MARGIN = 20.0f;

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

    /// Remove bend points that duplicate sourcePoint or targetPoint
    /// Also removes consecutive duplicate bend points
    /// @param layout Edge layout to clean (modified in place)
    static void removeEndpointDuplicates(EdgeLayout& layout);

    /// Move bend points that are inside node boundaries to be outside
    /// @param layout Edge layout to clean (modified in place)
    /// @param targetNode Target node layout (for boundary checking)
    /// @param margin Minimum distance to keep from node boundary
    static void moveBendsOutsideNode(EdgeLayout& layout, const NodeLayout& targetNode, float margin = DEFAULT_MARGIN);
};

}  // namespace arborvia
