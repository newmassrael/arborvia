#pragma once

#include "arborvia/core/Types.h"
#include "arborvia/layout/LayoutResult.h"
#include "arborvia/layout/LayoutOptions.h"

#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace arborvia {

/// Utility functions for interactive layout manipulation
class LayoutUtils {
public:
    /// Update edge positions when nodes move (for interactive drag)
    /// Preserves edge routing (sourceEdge, targetEdge) but recalculates snap positions
    /// @param edgeLayouts The edge layouts to update (modified in place)
    /// @param nodeLayouts Current node positions
    /// @param affectedEdges Edges that need updating (connected to moved nodes)
    /// @param distribution Snap distribution mode
    /// @param movedNodes Optional set of nodes that actually moved. If provided, only endpoints
    ///                   on these nodes will be recalculated. If empty, all endpoints are updated.
    static void updateEdgePositions(
        std::unordered_map<EdgeId, EdgeLayout>& edgeLayouts,
        const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
        const std::vector<EdgeId>& affectedEdges,
        SnapDistribution distribution = SnapDistribution::Separated,
        const std::unordered_set<NodeId>& movedNodes = {},
        float gridSize = 0.0f);

    /// Result of edge hit test with insertion info
    struct EdgeHitResult {
        bool hit = false;              ///< True if point is near the edge
        int segmentIndex = -1;         ///< Index of the segment hit (0 = source to first bend)
        Point closestPoint{0, 0};      ///< Closest point on the edge segment
        float distance = 0.0f;         ///< Distance from query point to closest point
    };

    /// Check if a point is near an edge and get insertion info
    /// @param point The query point
    /// @param edge The edge layout to test against
    /// @param threshold Maximum distance for a hit
    /// @return EdgeHitResult with hit info and insertion position
    static EdgeHitResult hitTestEdge(const Point& point, const EdgeLayout& edge, float threshold);

    /// Calculate distance from a point to a line segment
    /// @param point The query point
    /// @param segmentStart Start of the line segment
    /// @param segmentEnd End of the line segment
    /// @param outClosestPoint Output: closest point on segment to query point
    /// @return Distance from point to the closest point on segment
    static float pointToSegmentDistance(
        const Point& point,
        const Point& segmentStart,
        const Point& segmentEnd,
        Point& outClosestPoint);

    /// Calculate snap point position on a node edge from index
    /// @param node The node layout
    /// @param edge Which edge of the node (Top, Bottom, Left, Right)
    /// @param snapIndex Index of the snap point (0-based)
    /// @param totalSnapPoints Total number of snap points on this edge
    /// @return Calculated snap point position
    static Point calculateSnapPoint(
        const NodeLayout& node,
        NodeEdge edge,
        int snapIndex,
        int totalSnapPoints);

    /// Calculate snap point position on a node edge from relative position
    /// @param node The node layout
    /// @param edge Which edge of the node (Top, Bottom, Left, Right)
    /// @param position Relative position along the edge (0.0 to 1.0)
    /// @return Calculated snap point position
    static Point calculateSnapPointFromPosition(
        const NodeLayout& node,
        NodeEdge edge,
        float position);

    /// Calculate optimal label position for an edge
    /// For edges with bend points: uses the middle of the "main" segment (between bends)
    /// For straight edges: uses the midpoint of the path
    /// @param edge The edge layout
    /// @return Optimal label position
    static Point calculateEdgeLabelPosition(const EdgeLayout& edge);
};

}  // namespace arborvia
