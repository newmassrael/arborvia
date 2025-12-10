#pragma once

#include "arborvia/core/Types.h"
#include "arborvia/core/Graph.h"
#include "arborvia/layout/config/LayoutResult.h"
#include "arborvia/layout/config/LayoutOptions.h"
#include "arborvia/layout/config/LayoutTypes.h"

#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <memory>
#include <string>
#include <functional>

#include "arborvia/layout/config/MoveDirection.h"

namespace arborvia {

// Forward declarations for internal types
class PathRoutingCoordinator;
class ConstraintManager;

/// Utility functions for interactive layout manipulation
/// NOTE: For node moves, use LayoutController::moveNode() instead
class LayoutUtils {
public:
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

    // ========== Direction Helpers ==========

    /// Get required exit direction from source node edge
    /// @param sourceEdge Which edge the connection exits from
    /// @return Direction the path must initially move in
    static MoveDirection getRequiredSourceDirection(NodeEdge sourceEdge);

    /// Get required arrival direction to target node edge
    /// @param targetEdge Which edge the connection enters
    /// @return Direction the path must arrive from
    static MoveDirection getRequiredTargetArrivalDirection(NodeEdge targetEdge);

    /// Get the direction of the first segment in an edge path
    /// @param edge The edge layout
    /// @return Direction of the first segment, or None if empty
    static MoveDirection getFirstSegmentDirection(const EdgeLayout& edge);

    /// Get the direction of the last segment in an edge path
    /// @param edge The edge layout
    /// @return Direction of the last segment, or None if empty
    static MoveDirection getLastSegmentDirection(const EdgeLayout& edge);

    // ========== Snap Point Manipulation API ==========

    /// Result of snap point move operation
    struct SnapMoveResult {
        bool success = false;           ///< True if move was successful
        Point actualPosition{0, 0};     ///< Actual snap point position after move
        NodeEdge newEdge = NodeEdge::Top;  ///< Edge the snap point is now on
        int newSnapIndex = 0;           ///< New snap index on that edge
        std::vector<EdgeId> redistributedEdges;  ///< Other edges that were pushed/redistributed
        std::string reason;             ///< Failure reason if !success
    };

    /// Callback type for bend point regeneration (decouples from EdgeRouting)
    using BendPointRegenerator = std::function<void(
        std::unordered_map<EdgeId, EdgeLayout>& edgeLayouts,
        const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
        const std::vector<EdgeId>& affectedEdges,
        const LayoutOptions& options)>;

    /// Move an edge's snap point to a new position on the node
    /// This will:
    /// 1. Calculate which edge and position the new point falls on
    /// 2. Swap with existing edge at target position (if any)
    /// 3. Re-route the affected edge paths
    /// @param edgeId Edge whose snap point to move
    /// @param isSource True to move source snap, false to move target snap
    /// @param newPosition Desired new position (will be snapped to node edge)
    /// @param nodeLayouts Node layouts
    /// @param edgeLayouts Edge layouts (modified in place)
    /// @param graph Graph for connectivity
    /// @param options Layout options
    /// @param regenerator Optional callback for bend point regeneration (default uses EdgeRouting)
    /// @return SnapMoveResult with actual position and swapped/redistributed edges
    static SnapMoveResult moveSnapPoint(
        EdgeId edgeId,
        bool isSource,
        Point newPosition,
        const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
        std::unordered_map<EdgeId, EdgeLayout>& edgeLayouts,
        const Graph& graph,
        const LayoutOptions& options,
        const BendPointRegenerator& regenerator = nullptr);

    /// Calculate which node edge a point is closest to
    /// @param point The query point
    /// @param node The node layout
    /// @return Pair of (edge, relative position 0-1 along that edge)
    static std::pair<NodeEdge, float> findClosestNodeEdge(
        const Point& point,
        const NodeLayout& node);

    /// Redistribute snap points on a node edge to accommodate a fixed position
    /// @param nodeId The node whose edge to redistribute
    /// @param edge Which edge (Top, Bottom, Left, Right)
    /// @param fixedEdgeId Edge that has a fixed position (won't be moved)
    /// @param fixedPosition The fixed relative position (0-1) for that edge
    /// @param nodeLayouts Node layouts
    /// @param edgeLayouts Edge layouts (modified in place)
    /// @param graph Graph for connectivity
    /// @param options Layout options
    /// @return List of edge IDs that were redistributed
    static std::vector<EdgeId> redistributeSnapPoints(
        NodeId nodeId,
        NodeEdge edge,
        EdgeId fixedEdgeId,
        float fixedPosition,
        const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
        std::unordered_map<EdgeId, EdgeLayout>& edgeLayouts,
        const Graph& graph,
        const LayoutOptions& options);

    /// Get edges connected to a node
    /// @param nodeId The node to query
    /// @param edgeLayouts All edge layouts
    /// @return Vector of edge IDs connected to this node
    static std::vector<EdgeId> getConnectedEdges(
        NodeId nodeId,
        const std::unordered_map<EdgeId, EdgeLayout>& edgeLayouts);

    /// Fast check if a node position would overlap with other nodes
    /// Much faster than full validation - use for real-time drag feedback
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

    // ========== Internal API (for advanced use) ==========
    // These are exposed for cases where you need fine-grained control
    // Prefer using LayoutController for node moves

    /// Result of drag validation check
    struct DragValidation {
        bool valid = true;              ///< True if node can be moved to this position
        std::vector<EdgeId> invalidEdges;  ///< Edges that would have invalid routing
    };

    /// Update edge positions when nodes move (internal - prefer moveNode())
    static void updateEdgePositions(
        std::unordered_map<EdgeId, EdgeLayout>& edgeLayouts,
        const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
        const std::vector<EdgeId>& affectedEdges,
        const std::unordered_set<NodeId>& movedNodes = {},
        float gridSize = 0.0f);

    /// Update edge positions using a routing coordinator (internal)
    static void updateEdgePositions(
        std::unordered_map<EdgeId, EdgeLayout>& edgeLayouts,
        const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
        const std::vector<EdgeId>& affectedEdges,
        PathRoutingCoordinator& coordinator,
        const std::unordered_set<NodeId>& movedNodes = {},
        float gridSize = 0.0f);

    /// Update edge positions with full layout options (internal)
    static void updateEdgePositions(
        std::unordered_map<EdgeId, EdgeLayout>& edgeLayouts,
        const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
        const std::vector<EdgeId>& affectedEdges,
        const LayoutOptions& options,
        const std::unordered_set<NodeId>& movedNodes = {});

    /// Update edge positions with constraint-satisfying node adjustment
    /// If A* fails for any edge, automatically adjusts moved node positions
    /// to satisfy all constraints (A* paths exist, no diagonal segments)
    /// @param edgeLayouts Edge layouts to update
    /// @param nodeLayouts Node layouts (may be modified if adjustment needed)
    /// @param affectedEdges Edges to route
    /// @param graph Graph for connectivity info
    /// @param options Layout options
    /// @param movedNodes Nodes that were moved (will be adjusted if needed)
    /// @return true if all edges have valid paths (possibly with node adjustments)
    static bool updateEdgePositionsWithConstraints(
        std::unordered_map<EdgeId, EdgeLayout>& edgeLayouts,
        std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
        const std::vector<EdgeId>& affectedEdges,
        const Graph& graph,
        const LayoutOptions& options,
        const std::unordered_set<NodeId>& movedNodes = {});

    /// Check if a node can be moved to a specific position (internal)
    static DragValidation canMoveNodeTo(
        NodeId nodeId,
        Point newPosition,
        const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
        const std::unordered_map<EdgeId, EdgeLayout>& edgeLayouts,
        float gridSize = 20.0f);

    /// Check if a node can be moved with custom constraints (internal)
    static DragValidation canMoveNodeTo(
        NodeId nodeId,
        Point newPosition,
        const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
        const std::unordered_map<EdgeId, EdgeLayout>& edgeLayouts,
        const ConstraintManager& constraintManager,
        float gridSize = 20.0f);

    /// Check if a node can be moved using ConstraintConfig (internal)
    static DragValidation canMoveNodeTo(
        NodeId nodeId,
        Point newPosition,
        const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
        const std::unordered_map<EdgeId, EdgeLayout>& edgeLayouts,
        const ConstraintConfig& config,
        float gridSize = 20.0f);

    /// Check if a node can be moved using pre-calculated zones (internal)
    static DragValidation canMoveNodeTo(
        NodeId nodeId,
        Point newPosition,
        const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
        const std::vector<ForbiddenZone>& preCalculatedZones);
};

}  // namespace arborvia
