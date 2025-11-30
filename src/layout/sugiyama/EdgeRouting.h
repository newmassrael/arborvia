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
    float yPosition = 0.0f;  // Computed Y coordinate for horizontal segment
    int sourceLayer = 0;     // Source node's layer
    int targetLayer = 0;     // Target node's layer
    bool isSelfLoop = false; // True if source == target
};

/// Channel region between two adjacent layers
struct ChannelRegion {
    int fromLayer = 0;       // Source layer index
    int toLayer = 0;         // Target layer index
    float regionStart = 0;   // Y coordinate where region starts
    float regionEnd = 0;     // Y coordinate where region ends
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
        SnapDistribution distribution = SnapDistribution::Separated);
    
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
        const std::unordered_set<NodeId>& movedNodes = {});

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
    static void recalculateBendPoints(EdgeLayout& layout);
    
    /// Count total connections on a node edge from all edge layouts
    /// @param edgeLayouts All edge layouts to count from
    /// @param nodeId The node to count connections for
    /// @param nodeEdge Which edge of the node
    /// @return Pair of (incoming count, outgoing count)
    static std::pair<int, int> countConnectionsOnNodeEdge(
        const std::unordered_map<EdgeId, EdgeLayout>& edgeLayouts,
        NodeId nodeId,
        NodeEdge nodeEdge);
    
    // === Existing private methods ===
    
    // Orthogonal edge routing (right-angle bends)
    EdgeLayout routeOrthogonal(const EdgeData& edge,
                               const NodeLayout& fromLayout,
                               const NodeLayout& toLayout,
                               bool isReversed,
                               const LayoutOptions& options);
    
    // Polyline edge routing (direct with bend points)
    EdgeLayout routePolyline(const EdgeData& edge,
                            const NodeLayout& fromLayout,
                            const NodeLayout& toLayout,
                            bool isReversed,
                            const LayoutOptions& options);
    
    // Compute connection point on node boundary
    Point computeConnectionPoint(const NodeLayout& node,
                                const Point& targetCenter,
                                bool isSource);
    
    // Compute snap point (centered on edge of node)
    Point computeSnapPoint(const NodeLayout& node,
                          Direction direction,
                          bool isSource);

    // === Channel-based routing methods ===

    /// Compute channel regions between adjacent layers
    std::vector<ChannelRegion> computeChannelRegions(
        const Graph& graph,
        const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
        const std::unordered_set<EdgeId>& reversedEdges);

    /// Allocate edges to channels
    std::unordered_map<EdgeId, ChannelAssignment> allocateChannels(
        const Graph& graph,
        const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
        const std::unordered_set<EdgeId>& reversedEdges,
        const LayoutOptions& options);

    /// Compute channel Y position
    float computeChannelY(const ChannelRegion& region,
                         int channelIndex,
                         const ChannelRoutingOptions& opts);

    /// Route edge using channel assignment
    EdgeLayout routeChannelOrthogonal(
        const EdgeData& edge,
        const NodeLayout& fromLayout,
        const NodeLayout& toLayout,
        bool isReversed,
        const ChannelAssignment& channel,
        const LayoutOptions& options);

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
