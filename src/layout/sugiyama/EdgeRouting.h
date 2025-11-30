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

/// Routes edges with appropriate bend points
class EdgeRouting {
public:
    struct Result {
        std::unordered_map<EdgeId, EdgeLayout> edgeLayouts;
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
};

}  // namespace algorithms
}  // namespace arborvia
