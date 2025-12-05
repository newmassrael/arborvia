#pragma once

#include "arborvia/layout/ICrossingMinimization.h"

#include <unordered_set>
#include <vector>

namespace arborvia {


/// Barycenter-based crossing minimization algorithm
/// 
/// This is the default implementation of ICrossingMinimization using
/// barycenter and median heuristics for layer sweeping.
class BarycenterCrossingMinimization : public ICrossingMinimization {
public:
    /// Type alias for backward compatibility
    using Result = CrossingMinimizationResult;

    BarycenterCrossingMinimization() = default;

    /// Get algorithm name
    const char* algorithmName() const override { return "Barycenter"; }

    /// Minimize crossings using specified strategy
    CrossingMinimizationResult minimize(
        const Graph& graph,
        std::vector<std::vector<NodeId>> layers,
        const std::unordered_set<EdgeId>& reversedEdges,
        arborvia::CrossingMinimization strategy,
        int passes = 4) const override;

    /// Count edge crossings between two adjacent layers
    int countCrossings(
        const Graph& graph,
        const std::vector<NodeId>& upperLayer,
        const std::vector<NodeId>& lowerLayer,
        const std::unordered_set<EdgeId>& reversedEdges) const override;

    /// Count total crossings in the entire layout
    int countTotalCrossings(
        const Graph& graph,
        const std::vector<std::vector<NodeId>>& layers,
        const std::unordered_set<EdgeId>& reversedEdges) const override;

private:
    // Barycenter heuristic: order by average position of neighbors
    void barycenterSweep(const Graph& graph,
                        std::vector<std::vector<NodeId>>& layers,
                        const std::unordered_set<EdgeId>& reversedEdges,
                        bool downward) const;

    // Median heuristic: order by median position of neighbors
    void medianSweep(const Graph& graph,
                    std::vector<std::vector<NodeId>>& layers,
                    const std::unordered_set<EdgeId>& reversedEdges,
                    bool downward) const;

    float computeBarycenter(NodeId node,
                           const Graph& graph,
                           const std::vector<NodeId>& adjacentLayer,
                           const std::unordered_set<EdgeId>& reversedEdges,
                           bool useSuccessors) const;
};

/// Backward compatibility alias
// Note: CrossingMinimization is an enum class in LayoutOptions.h
// Use BarycenterCrossingMinimization directly


}  // namespace arborvia
