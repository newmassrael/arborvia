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
        const std::unordered_map<NodeId, NodeLayout>& nodeLayouts);

private:
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
