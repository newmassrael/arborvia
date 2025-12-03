#pragma once

#include "arborvia/layout/ICycleRemoval.h"

#include <unordered_set>
#include <vector>

namespace arborvia {
namespace algorithms {

/// DFS-based cycle removal algorithm
/// 
/// This is the default implementation of ICycleRemoval using depth-first
/// search to detect back edges which indicate cycles.
class CycleRemoval : public ICycleRemoval {
public:
    /// Type alias for backward compatibility
    using Result = CycleRemovalResult;

    CycleRemoval() = default;

    /// Get algorithm name
    const char* algorithmName() const override { return "DFS"; }

    /// Detect and mark edges to reverse for making graph acyclic
    CycleRemovalResult findEdgesToReverse(const Graph& graph) const override;

    /// Check if graph has cycles
    bool hasCycles(const Graph& graph) const override;

private:
    enum class NodeState { White, Gray, Black };

    void dfs(NodeId node, const Graph& graph,
             const std::unordered_map<NodeId, size_t>& nodeIndex,
             std::vector<NodeState>& state,
             std::unordered_set<EdgeId>& backEdges) const;
};

}  // namespace algorithms
}  // namespace arborvia
