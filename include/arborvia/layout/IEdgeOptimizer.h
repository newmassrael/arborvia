#pragma once

#include "../core/Types.h"
#include "LayoutResult.h"

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace arborvia {

/// Interface for edge routing optimization algorithms
/// 
/// This is SEPARATE from the constraint system (IDragConstraint):
/// - Constraints: Boolean validation ("Can we do this?")
/// - Optimizer: Numeric optimization ("Which option is best?")
/// 
/// Execution flow:
/// 1. ConstraintManager validates position (early return on failure)
/// 2. EdgeOptimizer finds best edge combinations (runs only if valid)
/// 
/// Example usage:
/// @code
/// auto optimizer = std::make_shared<AStarEdgeOptimizer>();
/// auto optimized = optimizer->optimize(edges, edgeLayouts, nodeLayouts);
/// @endcode
class IEdgeOptimizer {
public:
    virtual ~IEdgeOptimizer() = default;

    /// Optimize edge routing by selecting best source/target edge combinations
    /// @param edges List of edge IDs to optimize
    /// @param currentLayouts Current edge layouts
    /// @param nodeLayouts Node positions for routing calculations
    /// @return Optimized edge layouts (modified copies, does not modify input)
    virtual std::unordered_map<EdgeId, EdgeLayout> optimize(
        const std::vector<EdgeId>& edges,
        const std::unordered_map<EdgeId, EdgeLayout>& currentLayouts,
        const std::unordered_map<NodeId, NodeLayout>& nodeLayouts) = 0;

    /// Get algorithm name for debugging/logging
    /// @return Algorithm identifier (e.g., "Greedy", "SimulatedAnnealing")
    virtual const char* algorithmName() const = 0;
};

}  // namespace arborvia
