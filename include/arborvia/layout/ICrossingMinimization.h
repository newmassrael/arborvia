#pragma once

#include "../core/Types.h"
#include "../core/Graph.h"
#include "LayoutOptions.h"

#include <vector>
#include <unordered_set>

namespace arborvia {
namespace algorithms {

/// Result of crossing minimization operation
struct CrossingMinimizationResult {
    std::vector<std::vector<NodeId>> layers;  ///< Reordered layers
    int crossingCount = 0;                     ///< Number of crossings after minimization
};

/// Abstract interface for crossing minimization algorithms
/// 
/// Implementations can provide different strategies:
/// - Barycenter (default, fast heuristic)
/// - Median (alternative heuristic)
/// - Sifting (more sophisticated)
/// - ILP-based (optimal but slow)
/// 
/// Use this interface to swap crossing minimization algorithms without
/// modifying SugiyamaLayout or other dependent code.
class ICrossingMinimization {
public:
    virtual ~ICrossingMinimization() = default;

    /// Minimize crossings using the implemented algorithm
    /// @param graph The input graph
    /// @param layers Initial layer assignment (will be reordered)
    /// @param reversedEdges Edges that are logically reversed
    /// @param strategy Strategy hint (implementation may ignore if not supported)
    /// @param passes Number of optimization passes
    /// @return Result containing reordered layers
    virtual CrossingMinimizationResult minimize(
        const Graph& graph,
        std::vector<std::vector<NodeId>> layers,
        const std::unordered_set<EdgeId>& reversedEdges,
        arborvia::CrossingMinimization strategy,
        int passes) const = 0;

    /// Count edge crossings between two adjacent layers
    /// @param graph The input graph
    /// @param upperLayer Nodes in upper layer
    /// @param lowerLayer Nodes in lower layer
    /// @param reversedEdges Edges that are logically reversed
    /// @return Number of crossings between the layers
    virtual int countCrossings(
        const Graph& graph,
        const std::vector<NodeId>& upperLayer,
        const std::vector<NodeId>& lowerLayer,
        const std::unordered_set<EdgeId>& reversedEdges) const = 0;

    /// Count total crossings in the entire layout
    /// @param graph The input graph
    /// @param layers All layers in order
    /// @param reversedEdges Edges that are logically reversed
    /// @return Total number of crossings
    virtual int countTotalCrossings(
        const Graph& graph,
        const std::vector<std::vector<NodeId>>& layers,
        const std::unordered_set<EdgeId>& reversedEdges) const = 0;

    /// Get algorithm name for debugging/logging
    virtual const char* algorithmName() const = 0;
};

}  // namespace algorithms
}  // namespace arborvia
