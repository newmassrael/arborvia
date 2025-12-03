#pragma once

#include "../core/Types.h"
#include "../core/Graph.h"
#include "LayoutOptions.h"

#include <vector>
#include <unordered_map>
#include <unordered_set>

namespace arborvia {
namespace algorithms {

/// Result of layer assignment operation
struct LayerAssignmentResult {
    std::unordered_map<NodeId, int> nodeLayer;  ///< Node -> layer index
    int layerCount = 0;                          ///< Total number of layers
    std::vector<std::vector<NodeId>> layers;     ///< Layer -> nodes in that layer
};

/// Abstract interface for layer assignment algorithms
/// 
/// Implementations can provide different strategies:
/// - Longest Path (default, minimizes height)
/// - Network Simplex (optimal layer assignment)
/// - Coffman-Graham (bounded width)
/// 
/// Use this interface to swap layer assignment algorithms without
/// modifying SugiyamaLayout or other dependent code.
class ILayerAssignment {
public:
    virtual ~ILayerAssignment() = default;

    /// Assign layers using the implemented algorithm
    /// @param graph The input graph
    /// @param reversedEdges Edges that should be treated as reversed for layer calculation
    /// @param strategy Strategy hint (implementation may ignore if not supported)
    /// @return Result containing layer assignments
    virtual LayerAssignmentResult assignLayers(
        const Graph& graph,
        const std::unordered_set<EdgeId>& reversedEdges,
        arborvia::LayerAssignment strategy) const = 0;

    /// Assign layers with specified root nodes (they become layer 0)
    /// @param graph The input graph
    /// @param roots Nodes to place in layer 0
    /// @param reversedEdges Edges that should be treated as reversed
    /// @return Result containing layer assignments
    virtual LayerAssignmentResult assignLayersWithRoots(
        const Graph& graph,
        const std::vector<NodeId>& roots,
        const std::unordered_set<EdgeId>& reversedEdges) const = 0;

    /// Get algorithm name for debugging/logging
    virtual const char* algorithmName() const = 0;
};

}  // namespace algorithms
}  // namespace arborvia
