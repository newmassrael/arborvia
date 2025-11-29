#pragma once

#include "arborvia/core/Graph.h"

#include <unordered_set>
#include <vector>

namespace arborvia {
namespace algorithms {

/// Removes cycles from a directed graph by reversing edges
/// Uses depth-first search to detect back edges
class CycleRemoval {
public:
    struct Result {
        std::vector<EdgeId> reversedEdges;  // Edges that were logically reversed
        bool isAcyclic = false;
    };
    
    /// Detect and mark edges to reverse for making graph acyclic
    /// Does not modify the graph, returns edges to reverse
    Result findEdgesToReverse(const Graph& graph);
    
    /// Check if graph has cycles
    bool hasCycles(const Graph& graph);
    
private:
    enum class NodeState { White, Gray, Black };
    
    void dfs(NodeId node, const Graph& graph,
             const std::unordered_map<NodeId, size_t>& nodeIndex,
             std::vector<NodeState>& state,
             std::unordered_set<EdgeId>& backEdges);
};

}  // namespace algorithms
}  // namespace arborvia
