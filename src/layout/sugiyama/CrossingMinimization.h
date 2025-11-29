#pragma once

#include "arborvia/core/Graph.h"
#include "arborvia/layout/LayoutOptions.h"

#include <unordered_set>
#include <vector>

namespace arborvia {
namespace algorithms {

/// Minimizes edge crossings by reordering nodes within layers
class CrossingMinimization {
public:
    struct Result {
        std::vector<std::vector<NodeId>> layers;  // Reordered layers
        int crossingCount = 0;
    };
    
    /// Minimize crossings using specified strategy
    Result minimize(const Graph& graph,
                   std::vector<std::vector<NodeId>> layers,
                   const std::unordered_set<EdgeId>& reversedEdges,
                   arborvia::CrossingMinimization strategy,
                   int passes = 4);
    
    /// Count edge crossings between two adjacent layers
    int countCrossings(const Graph& graph,
                      const std::vector<NodeId>& upperLayer,
                      const std::vector<NodeId>& lowerLayer,
                      const std::unordered_set<EdgeId>& reversedEdges);
    
    /// Count total crossings in the entire layout
    int countTotalCrossings(const Graph& graph,
                           const std::vector<std::vector<NodeId>>& layers,
                           const std::unordered_set<EdgeId>& reversedEdges);

private:
    // Barycenter heuristic: order by average position of neighbors
    void barycenterSweep(const Graph& graph,
                        std::vector<std::vector<NodeId>>& layers,
                        const std::unordered_set<EdgeId>& reversedEdges,
                        bool downward);
    
    // Median heuristic: order by median position of neighbors
    void medianSweep(const Graph& graph,
                    std::vector<std::vector<NodeId>>& layers,
                    const std::unordered_set<EdgeId>& reversedEdges,
                    bool downward);
    
    float computeBarycenter(NodeId node,
                           const Graph& graph,
                           const std::vector<NodeId>& adjacentLayer,
                           const std::unordered_set<EdgeId>& reversedEdges,
                           bool useSuccessors);
};

}  // namespace algorithms
}  // namespace arborvia
