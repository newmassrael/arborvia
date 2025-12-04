#include "LayerAssignment.h"

#include <algorithm>
#include <queue>

namespace arborvia {
namespace algorithms {

LayerAssignmentResult LongestPathLayerAssignment::assignLayers(
    const Graph& graph,
    const std::unordered_set<EdgeId>& reversedEdges) const {

    LayerAssignmentResult result;

    std::vector<NodeId> nodes = graph.nodes();
    if (nodes.empty()) {
        return result;
    }

    longestPathAssignment(graph, reversedEdges, result);

    return result;
}

LayerAssignmentResult LongestPathLayerAssignment::assignLayersWithRoots(
    const Graph& graph,
    const std::vector<NodeId>& roots,
    const std::unordered_set<EdgeId>& reversedEdges) const {

    LayerAssignmentResult result;

    if (roots.empty()) {
        return assignLayers(graph, reversedEdges);
    }

    // BFS from roots to assign layers
    std::queue<NodeId> queue;
    for (NodeId root : roots) {
        result.nodeLayer[root] = 0;
        queue.push(root);
    }

    while (!queue.empty()) {
        NodeId current = queue.front();
        queue.pop();

        int currentLayer = result.nodeLayer[current];

        for (EdgeId edgeId : graph.outEdges(current)) {
            const EdgeData edge = graph.getEdge(edgeId);
            NodeId successor = edge.to;

            // Handle reversed edges
            bool isReversed = reversedEdges.count(edgeId) > 0;
            if (isReversed) {
                continue;  // Skip reversed edges in forward pass
            }

            if (result.nodeLayer.find(successor) == result.nodeLayer.end()) {
                result.nodeLayer[successor] = currentLayer + 1;
                queue.push(successor);
            }
        }
    }

    // Calculate layer count and organize nodes into layers
    result.layerCount = 0;
    for (const auto& [node, layer] : result.nodeLayer) {
        result.layerCount = std::max(result.layerCount, layer + 1);
    }

    result.layers.resize(result.layerCount);
    for (const auto& [node, layer] : result.nodeLayer) {
        result.layers[layer].push_back(node);
    }

    return result;
}

void LongestPathLayerAssignment::longestPathAssignment(
    const Graph& graph,
    const std::unordered_set<EdgeId>& reversedEdges,
    LayerAssignmentResult& result) const {

    std::vector<NodeId> nodes = graph.nodes();
    std::unordered_map<NodeId, int> memo;

    // Find sink nodes (nodes with no outgoing edges, considering reversed edges)
    std::vector<NodeId> sinks;
    for (NodeId node : nodes) {
        bool hasSucessor = false;
        for (EdgeId edgeId : graph.outEdges(node)) {
            if (reversedEdges.count(edgeId) == 0) {
                hasSucessor = true;
                break;
            }
        }
        if (!hasSucessor) {
            sinks.push_back(node);
        }
    }

    // If no sinks found, use all nodes (graph might be cyclic or empty)
    if (sinks.empty()) {
        sinks = nodes;
    }

    // Assign sinks to layer 0 and compute layers bottom-up
    for (NodeId sink : sinks) {
        memo[sink] = 0;
    }

    // Compute layer for each node using DFS
    for (NodeId node : nodes) {
        if (memo.find(node) == memo.end()) {
            computeLayerDFS(node, graph, reversedEdges, memo);
        }
    }

    // Find max layer to flip the assignment (we computed bottom-up)
    int maxLayer = 0;
    for (const auto& [node, layer] : memo) {
        maxLayer = std::max(maxLayer, layer);
    }

    // Flip layers so sources are at layer 0
    for (const auto& [node, layer] : memo) {
        result.nodeLayer[node] = maxLayer - layer;
    }

    result.layerCount = maxLayer + 1;
    result.layers.resize(result.layerCount);

    for (const auto& [node, layer] : result.nodeLayer) {
        result.layers[layer].push_back(node);
    }
}

int LongestPathLayerAssignment::computeLayerDFS(
    NodeId node,
    const Graph& graph,
    const std::unordered_set<EdgeId>& reversedEdges,
    std::unordered_map<NodeId, int>& memo) const {

    // Already computed
    auto it = memo.find(node);
    if (it != memo.end()) {
        return it->second;
    }

    // Compute based on successors
    int maxSuccessorLayer = -1;

    for (EdgeId edgeId : graph.outEdges(node)) {
        if (reversedEdges.count(edgeId) > 0) {
            continue;  // Skip reversed edges
        }

        const EdgeData edge = graph.getEdge(edgeId);
        int successorLayer = computeLayerDFS(edge.to, graph, reversedEdges, memo);
        maxSuccessorLayer = std::max(maxSuccessorLayer, successorLayer);
    }

    // This node is one layer above its highest successor
    int layer = maxSuccessorLayer + 1;
    memo[node] = layer;

    return layer;
}

}  // namespace algorithms
}  // namespace arborvia
