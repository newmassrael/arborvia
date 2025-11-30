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
        const std::unordered_set<NodeId>& movedNodes = {});
};

}  // namespace arborvia
