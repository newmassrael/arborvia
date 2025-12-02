#pragma once

#include "arborvia/core/Graph.h"
#include "arborvia/layout/LayoutOptions.h"
#include "arborvia/layout/LayoutResult.h"

#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <map>

namespace arborvia {
namespace algorithms {

// Forward declaration
class SnapIndexManager;

/// Channel assignment info for an edge
struct ChannelAssignment {
    int channel = 0;         // Channel index (0-based)
    float yPosition = 0.0f;  // Computed position for channel segment (Y for vertical, X for horizontal layout)
    int sourceLayer = 0;     // Source node's layer
    int targetLayer = 0;     // Target node's layer
    bool isSelfLoop = false; // True if source == target
};

/// Channel region between two adjacent layers
struct ChannelRegion {
    int fromLayer = 0;       // Source layer index
    int toLayer = 0;         // Target layer index
    float regionStart = 0;   // Coordinate where region starts (Y for vertical, X for horizontal layout)
    float regionEnd = 0;     // Coordinate where region ends (Y for vertical, X for horizontal layout)
    std::vector<EdgeId> edges;  // Edges routed through this region
};

/// Routes edges with appropriate bend points
class EdgeRouting {
public:
    /// Result of edge routing operation
    struct Result {
        std::unordered_map<EdgeId, EdgeLayout> edgeLayouts;
        // Optional: channel assignments for debugging/visualization
        std::unordered_map<EdgeId, ChannelAssignment> channelAssignments;
        
        // === Helper methods ===
        
        /// Get edge layout by ID, returns nullptr if not found
        const EdgeLayout* getEdgeLayout(EdgeId id) const {
            auto it = edgeLayouts.find(id);
            return it != edgeLayouts.end() ? &it->second : nullptr;
        }
        
        /// Get mutable edge layout by ID, returns nullptr if not found
        EdgeLayout* getEdgeLayout(EdgeId id) {
            auto it = edgeLayouts.find(id);
            return it != edgeLayouts.end() ? &it->second : nullptr;
        }
        
        /// Check if edge layout exists
        bool hasEdgeLayout(EdgeId id) const {
            return edgeLayouts.count(id) > 0;
        }
        
        /// Get channel assignment by ID, returns nullptr if not found
        const ChannelAssignment* getChannelAssignment(EdgeId id) const {
            auto it = channelAssignments.find(id);
            return it != channelAssignments.end() ? &it->second : nullptr;
        }
        
        /// Get number of edges
        size_t edgeCount() const { return edgeLayouts.size(); }
        
        /// Clear all layouts
        void clear() {
            edgeLayouts.clear();
            channelAssignments.clear();
        }
    };
    
    /// Route edges based on node positions
    Result route(const Graph& graph,
                const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
                const std::unordered_set<EdgeId>& reversedEdges,
                const LayoutOptions& options);
    
    /// Distribute snap points evenly for edges connecting to same node edge (Auto mode)
    static void distributeAutoSnapPoints(
        Result& result,
        const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
        SnapDistribution distribution = SnapDistribution::Separated,
        float gridSize = 0.0f);
    
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

    // === Utility functions for edge routing (public for internal helper use) ===
    
    /// Check if an orthogonal segment intersects a node's interior
    /// @param p1 Start point of segment
    /// @param p2 End point of segment
    /// @param node The node to check intersection with
    /// @return True if segment passes through node interior (excluding boundary)
    static bool segmentIntersectsNode(
        const Point& p1,
        const Point& p2,
        const NodeLayout& node);

    /// Calculate coordinate to avoid blocking node
    /// @param nodeMin Minimum coordinate of blocking node
    /// @param nodeMax Maximum coordinate of blocking node
    /// @param goPositive True to go right/down, false to go left/up
    /// @param gridSize Grid cell size for margin calculation
    /// @return Safe coordinate outside blocking node with margin
    static float calculateAvoidanceCoordinate(
        float nodeMin,
        float nodeMax,
        bool goPositive,
        float gridSize = 0.0f);

private:
    // === Helper functions for snap point calculation ===
    
    /// Calculate snap point position on a node edge
    /// @param node The node layout
    /// @param edge Which edge of the node (Top, Bottom, Left, Right)
    /// @param position Relative position along the edge (0.0 to 1.0)
    /// @return Absolute point coordinates
    static Point calculateSnapPosition(const NodeLayout& node, NodeEdge edge, float position);
    
    /// Calculate relative position for a snap point
    /// @param snapIdx The snap index (0-based)
    /// @param count Total number of snap points on this range
    /// @param rangeStart Start of the range (0.0 to 1.0)
    /// @param rangeEnd End of the range (0.0 to 1.0)
    /// @return Relative position (0.0 to 1.0)
    static float calculateRelativePosition(int snapIdx, int count, float rangeStart, float rangeEnd);

    /// Convert unified snap index to local index within a group
    /// In Separated mode, indices are unified: incoming [0, inCount), outgoing [inCount, total)
    /// This function converts to local index [0, count) for position calculation
    /// @param unifiedIdx The unified snap index from EdgeLayout
    /// @param offset The offset for this group (0 for incoming, inCount for outgoing)
    /// @param count The count of items in this group
    /// @return Local index clamped to [0, count)
    static int unifiedToLocalIndex(int unifiedIdx, int offset, int count);

    /// Recalculate bend points for an edge based on source and target points
    /// @param layout The edge layout to recalculate bend points for
    /// @param nodeLayouts All node layouts for intersection checking
    /// @param gridSize Grid cell size for coordinate snapping (0 = disabled)
    static void recalculateBendPoints(
        EdgeLayout& layout,
        const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
        float gridSize = 0.0f);

    // === recalculateBendPoints helper methods (SRP decomposition) ===

    /// Perform channel-based routing for edges with assigned channels
    /// @param layout The edge layout to route
    /// @param isHorizontal True for horizontal layout direction
    /// @param nodeLayouts All node layouts for intersection checking
    /// @param gridSize Grid cell size for coordinate snapping
    static void performChannelBasedRouting(
        EdgeLayout& layout,
        bool isHorizontal,
        const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
        float gridSize);

    /// Perform fallback routing for edges without channels
    /// @param layout The edge layout to route
    /// @param isHorizontal True for horizontal layout direction
    /// @param nodeLayouts All node layouts for intersection checking
    /// @param gridSize Grid cell size for coordinate snapping
    static void performFallbackRouting(
        EdgeLayout& layout,
        bool isHorizontal,
        const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
        float gridSize);

    /// Move intermediate bend points that are inside nodes to outside
    /// @param allPoints All path points (source, bends, target)
    /// @param nodeLayouts All node layouts
    /// @param gridSize Grid cell size for coordinate snapping
    static void moveIntermediatePointsOutsideNodes(
        std::vector<Point>& allPoints,
        const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
        float gridSize);

    /// Fix non-orthogonal segments by inserting intermediate points
    /// @param allPoints All path points (modified in place)
    /// @param nodeLayouts All node layouts for validation
    static void fixNonOrthogonalSegments(
        std::vector<Point>& allPoints,
        const std::unordered_map<NodeId, NodeLayout>& nodeLayouts);

    /// Fix segments that intersect nodes by routing around them
    /// @param allPoints All path points (modified in place)
    /// @param nodeLayouts All node layouts
    /// @param gridSize Grid cell size for coordinate snapping
    static void fixSegmentNodeIntersections(
        std::vector<Point>& allPoints,
        const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
        float gridSize);

    /// Final pass: fix segments intersecting intermediate nodes
    /// @param allPoints All path points (modified in place)
    /// @param layout The edge layout (for source/target node IDs)
    /// @param nodeLayouts All node layouts
    static void fixIntermediateNodeIntersections(
        std::vector<Point>& allPoints,
        const EdgeLayout& layout,
        const std::unordered_map<NodeId, NodeLayout>& nodeLayouts);

    /// Finalize edge path with direction corrections and cleanup
    /// @param allPoints All path points (modified in place)
    /// @param layout The edge layout (for edge info)
    /// @param nodeLayouts All node layouts
    /// @param gridSize Grid cell size for coordinate snapping
    static void finalizeEdgePath(
        std::vector<Point>& allPoints,
        const EdgeLayout& layout,
        const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
        float gridSize);

    /// Count total connections on a node edge from all edge layouts
    /// @param edgeLayouts All edge layouts to count from
    /// @param nodeId The node to count connections for
    /// @param nodeEdge Which edge of the node
    /// @return Pair of (incoming count, outgoing count)
    static std::pair<int, int> countConnectionsOnNodeEdge(
        const std::unordered_map<EdgeId, EdgeLayout>& edgeLayouts,
        NodeId nodeId,
        NodeEdge nodeEdge);
    
    // Compute snap point (centered on edge of node)
    Point computeSnapPoint(const NodeLayout& node,
                          Direction direction,
                          bool isSource);

    // === Channel-based routing methods ===

    /// Compute channel regions between adjacent layers
    std::vector<ChannelRegion> computeChannelRegions(
        const Graph& graph,
        const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
        const std::unordered_set<EdgeId>& reversedEdges,
        Direction direction);

    /// Allocate edges to channels
    std::unordered_map<EdgeId, ChannelAssignment> allocateChannels(
        const Graph& graph,
        const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
        const std::unordered_set<EdgeId>& reversedEdges,
        const LayoutOptions& options);

    /// Compute channel position (Y for vertical, X for horizontal layout)
    /// When gridSize > 0, returns grid-aligned value using quantized arithmetic
    float computeChannelY(const ChannelRegion& region,
                         int channelIndex,
                         const ChannelRoutingOptions& opts,
                         float gridSize = 0.0f);

    /// Build edge lookup map for bidirectional detection from Graph
    /// Returns map: (from, to) -> edgeId (skips self-loops)
    template<typename Hash>
    static std::unordered_map<std::pair<NodeId, NodeId>, EdgeId, Hash>
    buildEdgeMapFromGraph(const Graph& graph) {
        std::unordered_map<std::pair<NodeId, NodeId>, EdgeId, Hash> edgeMap;
        for (EdgeId edgeId : graph.edges()) {
            const EdgeData edge = graph.tryGetEdge(edgeId).value();
            if (edge.from != edge.to) {  // Skip self-loops
                edgeMap[{edge.from, edge.to}] = edgeId;
            }
        }
        return edgeMap;
    }
    
    /// Build edge lookup map for bidirectional detection from EdgeLayouts
    /// Returns map: (from, to) -> edgeId (skips self-loops)
    template<typename Hash>
    static std::unordered_map<std::pair<NodeId, NodeId>, EdgeId, Hash>
    buildEdgeMapFromLayouts(const std::unordered_map<EdgeId, EdgeLayout>& edgeLayouts) {
        std::unordered_map<std::pair<NodeId, NodeId>, EdgeId, Hash> edgeMap;
        for (const auto& [edgeId, layout] : edgeLayouts) {
            if (layout.from != layout.to) {  // Skip self-loops
                edgeMap[{layout.from, layout.to}] = edgeId;
            }
        }
        return edgeMap;
    }

    /// Route edge using channel assignment
    /// @param edge The edge data
    /// @param fromLayout Source node layout
    /// @param toLayout Target node layout
    /// @param isReversed Whether edge is reversed
    /// @param channel Channel assignment info
    /// @param options Layout options
    /// @param allNodeLayouts Optional: all node layouts for segment-node intersection checking
    EdgeLayout routeChannelOrthogonal(
        const EdgeData& edge,
        const NodeLayout& fromLayout,
        const NodeLayout& toLayout,
        bool isReversed,
        const ChannelAssignment& channel,
        const LayoutOptions& options,
        const std::unordered_map<NodeId, NodeLayout>* allNodeLayouts = nullptr);

    /// Route self-loop edge
    EdgeLayout routeSelfLoop(
        const EdgeData& edge,
        const NodeLayout& nodeLayout,
        int loopIndex,
        const LayoutOptions& options);

    /// Analyze best direction for self-loop
    SelfLoopDirection analyzeSelfLoopDirection(
        const NodeLayout& node,
        const std::unordered_map<EdgeId, EdgeLayout>& existingLayouts,
        const std::unordered_map<NodeId, NodeLayout>& allNodes);
};

}  // namespace algorithms
}  // namespace arborvia
