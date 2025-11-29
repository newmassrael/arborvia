#pragma once

#include "arborvia/core/Graph.h"
#include "arborvia/layout/LayoutOptions.h"

#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace arborvia {
namespace algorithms {

/// Assigns nodes to layers (ranks) in a hierarchical layout
class LayerAssignment {
public:
    struct Result {
        std::unordered_map<NodeId, int> nodeLayer;  // Node -> layer index
        int layerCount = 0;
        std::vector<std::vector<NodeId>> layers;    // Layer -> nodes in that layer
    };
    
    /// Assign layers using longest path algorithm
    /// reversedEdges: edges that should be treated as reversed for layer calculation
    Result assignLayers(const Graph& graph, 
                       const std::unordered_set<EdgeId>& reversedEdges,
                       arborvia::LayerAssignment strategy = arborvia::LayerAssignment::LongestPath);
    
    /// Assign layers with specified root nodes (they become layer 0)
    Result assignLayersWithRoots(const Graph& graph,
                                const std::vector<NodeId>& roots,
                                const std::unordered_set<EdgeId>& reversedEdges);

private:
    void longestPathAssignment(const Graph& graph,
                               const std::unordered_set<EdgeId>& reversedEdges,
                               Result& result);
    
    int computeLayerDFS(NodeId node, 
                        const Graph& graph,
                        const std::unordered_set<EdgeId>& reversedEdges,
                        std::unordered_map<NodeId, int>& memo);
};

}  // namespace algorithms
}  // namespace arborvia
