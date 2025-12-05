#pragma once

#include "arborvia/layout/ILayerAssignment.h"

#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace arborvia {


/// Longest-path based layer assignment algorithm
///
/// This is the default implementation of ILayerAssignment using the
/// longest path algorithm to minimize the height of the layout.
class LongestPathLayerAssignment : public ILayerAssignment {
public:
    /// Type alias for backward compatibility
    using Result = LayerAssignmentResult;

    LongestPathLayerAssignment() = default;

    /// Get algorithm name
    const char* algorithmName() const override { return "LongestPath"; }

    /// Assign layers using longest path algorithm
    LayerAssignmentResult assignLayers(
        const Graph& graph,
        const std::unordered_set<EdgeId>& reversedEdges) const override;

    /// Assign layers with specified root nodes (they become layer 0)
    LayerAssignmentResult assignLayersWithRoots(
        const Graph& graph,
        const std::vector<NodeId>& roots,
        const std::unordered_set<EdgeId>& reversedEdges) const override;

private:
    void longestPathAssignment(const Graph& graph,
                               const std::unordered_set<EdgeId>& reversedEdges,
                               LayerAssignmentResult& result) const;

    int computeLayerDFS(NodeId node,
                        const Graph& graph,
                        const std::unordered_set<EdgeId>& reversedEdges,
                        std::unordered_map<NodeId, int>& memo) const;
};

/// Backward compatibility alias
using LayerAssignment = LongestPathLayerAssignment;


}  // namespace arborvia
