#include "CycleRemoval.h"

namespace arborvia {
namespace algorithms {

CycleRemoval::Result CycleRemoval::findEdgesToReverse(const Graph& graph) {
    Result result;
    result.isAcyclic = true;
    
    std::vector<NodeId> nodes = graph.nodes();
    if (nodes.empty()) {
        return result;
    }
    
    // Build node index map once - O(n)
    std::unordered_map<NodeId, size_t> nodeIndex;
    nodeIndex.reserve(nodes.size());
    for (size_t i = 0; i < nodes.size(); ++i) {
        nodeIndex[nodes[i]] = i;
    }
    
    // Initialize all nodes as unvisited (White)
    std::vector<NodeState> state(nodes.size(), NodeState::White);
    std::unordered_set<EdgeId> backEdges;
    
    // Run DFS from each unvisited node - O(n+e) total
    for (NodeId node : nodes) {
        size_t idx = nodeIndex[node];
        if (state[idx] == NodeState::White) {
            dfs(node, graph, nodeIndex, state, backEdges);
        }
    }
    
    // Convert back edges to result
    result.reversedEdges = std::vector<EdgeId>(backEdges.begin(), backEdges.end());
    result.isAcyclic = backEdges.empty();
    
    return result;
}

bool CycleRemoval::hasCycles(const Graph& graph) {
    return !findEdgesToReverse(graph).isAcyclic;
}

void CycleRemoval::dfs(NodeId node, const Graph& graph,
                       const std::unordered_map<NodeId, size_t>& nodeIndex,
                       std::vector<NodeState>& state,
                       std::unordered_set<EdgeId>& backEdges) {
    size_t nodeIdx = nodeIndex.at(node);
    state[nodeIdx] = NodeState::Gray;  // Currently being processed
    
    // Visit all outgoing edges
    for (EdgeId edgeId : graph.outEdges(node)) {
        NodeId successor = graph.getEdge(edgeId).to;
        size_t succIdx = nodeIndex.at(successor);
        
        if (state[succIdx] == NodeState::Gray) {
            // Back edge found - this creates a cycle
            backEdges.insert(edgeId);
        } else if (state[succIdx] == NodeState::White) {
            // Unvisited node - recurse
            dfs(successor, graph, nodeIndex, state, backEdges);
        }
        // If Black, already fully processed - no action needed
    }
    
    state[nodeIdx] = NodeState::Black;  // Fully processed
}

}  // namespace algorithms
}  // namespace arborvia
