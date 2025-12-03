#include "CrossingMinimization.h"

#include <algorithm>
#include <numeric>

namespace arborvia {
namespace algorithms {

CrossingMinimizationResult BarycenterCrossingMinimization::minimize(
    const Graph& graph,
    std::vector<std::vector<NodeId>> layers,
    const std::unordered_set<EdgeId>& reversedEdges,
    arborvia::CrossingMinimization strategy,
    int passes) const {
    
    CrossingMinimizationResult result;
    result.layers = std::move(layers);
    
    if (result.layers.size() < 2) {
        result.crossingCount = 0;
        return result;
    }
    
    if (strategy == arborvia::CrossingMinimization::None) {
        result.crossingCount = countTotalCrossings(graph, result.layers, reversedEdges);
        return result;
    }
    
    // Perform alternating up/down sweeps
    for (int pass = 0; pass < passes; ++pass) {
        // Downward sweep (fix upper layer, reorder lower)
        if (strategy == arborvia::CrossingMinimization::BarycenterHeuristic) {
            barycenterSweep(graph, result.layers, reversedEdges, true);
        } else {
            medianSweep(graph, result.layers, reversedEdges, true);
        }
        
        // Upward sweep (fix lower layer, reorder upper)
        if (strategy == arborvia::CrossingMinimization::BarycenterHeuristic) {
            barycenterSweep(graph, result.layers, reversedEdges, false);
        } else {
            medianSweep(graph, result.layers, reversedEdges, false);
        }
    }
    
    result.crossingCount = countTotalCrossings(graph, result.layers, reversedEdges);
    return result;
}

int BarycenterCrossingMinimization::countCrossings(
    const Graph& graph,
    const std::vector<NodeId>& upperLayer,
    const std::vector<NodeId>& lowerLayer,
    const std::unordered_set<EdgeId>& reversedEdges) const {
    
    // Build position maps
    std::unordered_map<NodeId, int> upperPos, lowerPos;
    for (size_t i = 0; i < upperLayer.size(); ++i) {
        upperPos[upperLayer[i]] = static_cast<int>(i);
    }
    for (size_t i = 0; i < lowerLayer.size(); ++i) {
        lowerPos[lowerLayer[i]] = static_cast<int>(i);
    }
    
    // Collect edges between these layers
    std::vector<std::pair<int, int>> edges;
    
    for (NodeId upper : upperLayer) {
        for (EdgeId edgeId : graph.outEdges(upper)) {
            const EdgeData edge = graph.getEdge(edgeId);
            NodeId target = edge.to;
            
            // Handle reversed edges
            if (reversedEdges.count(edgeId) > 0) {
                continue;
            }
            
            auto it = lowerPos.find(target);
            if (it != lowerPos.end()) {
                edges.emplace_back(upperPos[upper], it->second);
            }
        }
    }
    
    // Count inversions (crossings)
    int crossings = 0;
    for (size_t i = 0; i < edges.size(); ++i) {
        for (size_t j = i + 1; j < edges.size(); ++j) {
            // Two edges cross if their positions are inverted
            bool upperInverted = edges[i].first > edges[j].first;
            bool lowerInverted = edges[i].second > edges[j].second;
            if (upperInverted != lowerInverted) {
                ++crossings;
            }
        }
    }
    
    return crossings;
}

int BarycenterCrossingMinimization::countTotalCrossings(
    const Graph& graph,
    const std::vector<std::vector<NodeId>>& layers,
    const std::unordered_set<EdgeId>& reversedEdges) const {
    
    int total = 0;
    for (size_t i = 0; i + 1 < layers.size(); ++i) {
        total += countCrossings(graph, layers[i], layers[i + 1], reversedEdges);
    }
    return total;
}

void BarycenterCrossingMinimization::barycenterSweep(
    const Graph& graph,
    std::vector<std::vector<NodeId>>& layers,
    const std::unordered_set<EdgeId>& reversedEdges,
    bool downward) const {
    
    if (downward) {
        // Process layers from top to bottom
        for (size_t i = 1; i < layers.size(); ++i) {
            std::vector<std::pair<float, NodeId>> nodeWeights;
            
            for (NodeId node : layers[i]) {
                float bc = computeBarycenter(node, graph, layers[i - 1], 
                                            reversedEdges, false);
                nodeWeights.emplace_back(bc, node);
            }
            
            // Sort by barycenter
            std::sort(nodeWeights.begin(), nodeWeights.end());
            
            layers[i].clear();
            for (const auto& [weight, node] : nodeWeights) {
                layers[i].push_back(node);
            }
        }
    } else {
        // Process layers from bottom to top
        for (int i = static_cast<int>(layers.size()) - 2; i >= 0; --i) {
            std::vector<std::pair<float, NodeId>> nodeWeights;
            
            for (NodeId node : layers[i]) {
                float bc = computeBarycenter(node, graph, layers[i + 1], 
                                            reversedEdges, true);
                nodeWeights.emplace_back(bc, node);
            }
            
            std::sort(nodeWeights.begin(), nodeWeights.end());
            
            layers[i].clear();
            for (const auto& [weight, node] : nodeWeights) {
                layers[i].push_back(node);
            }
        }
    }
}

void BarycenterCrossingMinimization::medianSweep(
    const Graph& graph,
    std::vector<std::vector<NodeId>>& layers,
    const std::unordered_set<EdgeId>& reversedEdges,
    bool downward) const {
    
    // Similar to barycenter but use median instead of mean
    // For simplicity, reuse barycenter for now (median is similar for small graphs)
    barycenterSweep(graph, layers, reversedEdges, downward);
}

float BarycenterCrossingMinimization::computeBarycenter(
    NodeId node,
    const Graph& graph,
    const std::vector<NodeId>& adjacentLayer,
    const std::unordered_set<EdgeId>& reversedEdges,
    bool useSuccessors) const {
    
    // Build position map for adjacent layer
    std::unordered_map<NodeId, int> pos;
    for (size_t i = 0; i < adjacentLayer.size(); ++i) {
        pos[adjacentLayer[i]] = static_cast<int>(i);
    }
    
    std::vector<int> neighborPositions;
    
    if (useSuccessors) {
        for (EdgeId edgeId : graph.outEdges(node)) {
            if (reversedEdges.count(edgeId) > 0) continue;
            const EdgeData edge = graph.getEdge(edgeId);
            auto it = pos.find(edge.to);
            if (it != pos.end()) {
                neighborPositions.push_back(it->second);
            }
        }
    } else {
        for (EdgeId edgeId : graph.inEdges(node)) {
            if (reversedEdges.count(edgeId) > 0) continue;
            const EdgeData edge = graph.getEdge(edgeId);
            auto it = pos.find(edge.from);
            if (it != pos.end()) {
                neighborPositions.push_back(it->second);
            }
        }
    }
    
    if (neighborPositions.empty()) {
        // No neighbors in adjacent layer - keep original position
        auto it = pos.find(node);
        return it != pos.end() ? static_cast<float>(it->second) : 0.0f;
    }
    
    // Return average position
    float sum = std::accumulate(neighborPositions.begin(), neighborPositions.end(), 0.0f);
    return sum / static_cast<float>(neighborPositions.size());
}

}  // namespace algorithms
}  // namespace arborvia
