#pragma once

#include "arborvia/core/Types.h"
#include "arborvia/layout/LayoutResult.h"
#include "arborvia/layout/LayoutOptions.h"
#include "arborvia/layout/ConstraintManager.h"
#include "arborvia/layout/ValidRegionCalculator.h"

#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <memory>

namespace arborvia {

class PathRoutingCoordinator;

/// Utility functions for interactive layout manipulation
class LayoutUtils {
public:
    /// Update edge positions when nodes move (for interactive drag)
    /// Preserves edge routing (sourceEdge, targetEdge) but recalculates snap positions
    /// @param edgeLayouts The edge layouts to update (modified in place)
    /// @param nodeLayouts Current node positions
    /// @param affectedEdges Edges that need updating (connected to moved nodes)
    /// @param movedNodes Optional set of nodes that actually moved. If provided, only endpoints
    ///                   on these nodes will be recalculated. If empty, all endpoints are updated.
    /// @param gridSize Grid size for snapping (0 = no snapping)
    static void updateEdgePositions(
        std::unordered_map<EdgeId, EdgeLayout>& edgeLayouts,
        const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
        const std::vector<EdgeId>& affectedEdges,
        const std::unordered_set<NodeId>& movedNodes = {},
        float gridSize = 0.0f);

    /// Update edge positions using a routing coordinator for algorithm selection
    /// The coordinator determines which pathfinder to use based on drag state
    /// @param edgeLayouts The edge layouts to update (modified in place)
    /// @param nodeLayouts Current node positions
    /// @param affectedEdges Edges that need updating (connected to moved nodes)
    /// @param coordinator Routing coordinator for pathfinder selection
    /// @param movedNodes Optional set of nodes that actually moved
    /// @param gridSize Grid size for snapping (0 = no snapping)
    static void updateEdgePositions(
        std::unordered_map<EdgeId, EdgeLayout>& edgeLayouts,
        const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
        const std::vector<EdgeId>& affectedEdges,
        PathRoutingCoordinator& coordinator,
        const std::unordered_set<NodeId>& movedNodes = {},
        float gridSize = 0.0f);

    /// Update edge positions with full layout options (supports greedy optimization)
    /// Use this when you want greedy optimization during drag for better path quality
    /// @param edgeLayouts The edge layouts to update (modified in place)
    /// @param nodeLayouts Current node positions
    /// @param affectedEdges Edges that need updating (connected to moved nodes)
    /// @param options Layout options including optimization settings
    /// @param movedNodes Optional set of nodes that actually moved
    static void updateEdgePositions(
        std::unordered_map<EdgeId, EdgeLayout>& edgeLayouts,
        const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
        const std::vector<EdgeId>& affectedEdges,
        const LayoutOptions& options,
        const std::unordered_set<NodeId>& movedNodes = {});

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

    /// Result of drag validation check
    struct DragValidation {
        bool valid = true;              ///< True if node can be moved to this position
        std::vector<EdgeId> invalidEdges;  ///< Edges that would have invalid routing
    };

    /// Check if a node can be moved to a specific position
    /// Uses ValidRegionCalculator to check forbidden zones based on edge counts
    /// @param nodeId Node being moved
    /// @param newPosition Proposed new position
    /// @param nodeLayouts Current node layouts
    /// @param edgeLayouts Current edge layouts (used to count edges per direction)
    /// @param gridSize Grid size for margin calculations
    /// @return DragValidation indicating if move is valid
    static DragValidation canMoveNodeTo(
        NodeId nodeId,
        Point newPosition,
        const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
        const std::unordered_map<EdgeId, EdgeLayout>& edgeLayouts,
        float gridSize = 20.0f);

    /// Check if a node can be moved to a specific position with custom constraints
    /// @param nodeId Node being moved
    /// @param newPosition Proposed new position
    /// @param nodeLayouts Current node layouts
    /// @param edgeLayouts Current edge layouts
    /// @param constraintManager Custom constraint manager to use
    /// @param gridSize Grid size for routing calculations
    /// @return DragValidation indicating if move is valid
    static DragValidation canMoveNodeTo(
        NodeId nodeId,
        Point newPosition,
        const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
        const std::unordered_map<EdgeId, EdgeLayout>& edgeLayouts,
        const ConstraintManager& constraintManager,
        float gridSize = 20.0f);

    /// Check if a node can be moved to a specific position using ConstraintConfig
    /// @param nodeId Node being moved
    /// @param newPosition Proposed new position
    /// @param nodeLayouts Current node layouts
    /// @param edgeLayouts Current edge layouts
    /// @param config Constraint configuration
    /// @param gridSize Grid size for routing calculations
    /// @return DragValidation indicating if move is valid
    static DragValidation canMoveNodeTo(
        NodeId nodeId,
        Point newPosition,
        const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
        const std::unordered_map<EdgeId, EdgeLayout>& edgeLayouts,
        const ConstraintConfig& config,
        float gridSize = 20.0f);

    /// Check if a node can be moved using pre-calculated forbidden zones
    /// Use this for consistent validation with visualization during drag
    /// @param nodeId Node being moved
    /// @param newPosition Proposed new position
    /// @param nodeLayouts Current node layouts (for node size lookup)
    /// @param preCalculatedZones Forbidden zones calculated at drag start
    /// @return DragValidation indicating if move is valid
    static DragValidation canMoveNodeTo(
        NodeId nodeId,
        Point newPosition,
        const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
        const std::vector<ForbiddenZone>& preCalculatedZones);

    /// Get edges connected to a node
    /// @param nodeId The node to query
    /// @param edgeLayouts All edge layouts
    /// @return Vector of edge IDs connected to this node
    static std::vector<EdgeId> getConnectedEdges(
        NodeId nodeId,
        const std::unordered_map<EdgeId, EdgeLayout>& edgeLayouts);

    /// Fast check if a node position would overlap with other nodes
    /// Much faster than canMoveNodeTo - use for real-time drag feedback
    /// @param nodeId Node being moved
    /// @param newPosition Proposed new position
    /// @param nodeLayouts Current node layouts
    /// @param margin Minimum gap between nodes (default 0)
    /// @return true if position is valid (no overlap)
    static bool canMoveNodeToFast(
        NodeId nodeId,
        Point newPosition,
        const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
        float margin = 0.0f);
};

}  // namespace arborvia
