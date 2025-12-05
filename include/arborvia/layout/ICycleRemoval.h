#pragma once

#include "../core/Types.h"
#include "../core/Graph.h"

#include <vector>

namespace arborvia {

/// Result of cycle removal operation
struct CycleRemovalResult {
    std::vector<EdgeId> reversedEdges;  ///< Edges that were logically reversed
    bool isAcyclic = false;             ///< Whether the graph is now acyclic
};

/// Abstract interface for cycle removal algorithms
/// 
/// Implementations can provide different strategies:
/// - DFS-based (default, finds back edges)
/// - Greedy (removes edges with most cycles)
/// - Feedback Arc Set approximation
/// 
/// Use this interface to swap cycle removal algorithms without
/// modifying SugiyamaLayout or other dependent code.
class ICycleRemoval {
public:
    virtual ~ICycleRemoval() = default;

    /// Detect and mark edges to reverse for making graph acyclic
    /// Does not modify the graph, returns edges to reverse
    /// @param graph The input graph
    /// @return Result containing edges to reverse
    virtual CycleRemovalResult findEdgesToReverse(const Graph& graph) const = 0;

    /// Check if graph has cycles
    /// @param graph The input graph
    /// @return true if graph contains cycles
    virtual bool hasCycles(const Graph& graph) const = 0;

    /// Get algorithm name for debugging/logging
    virtual const char* algorithmName() const = 0;
};

}  // namespace arborvia
