#pragma once

#include "../core/Types.h"
#include "LayoutResult.h"
#include "EdgeConstraintManager.h"

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
/// Edge constraints (IEdgeConstraint) are integrated through EdgeConstraintManager:
/// - All optimizers share the same constraint manager
/// - Adding a constraint to the manager applies to ALL optimizers automatically
/// - Constraints filter invalid candidates before scoring
///
/// Execution flow:
/// 1. ConstraintManager validates position (early return on failure)
/// 2. EdgeOptimizer finds best edge combinations (runs only if valid)
/// 3. EdgeConstraintManager filters invalid candidates during optimization
///
/// Example usage:
/// @code
/// auto optimizer = std::make_shared<AStarEdgeOptimizer>();
/// optimizer->setConstraintManager(EdgeConstraintManager::createDefault());
/// auto optimized = optimizer->optimize(edges, edgeLayouts, nodeLayouts);
/// @endcode
class IEdgeOptimizer {
public:
    virtual ~IEdgeOptimizer() = default;

    /// Set the constraint manager for filtering invalid edge combinations
    /// @param manager Shared constraint manager (same instance across all optimizers)
    virtual void setConstraintManager(std::shared_ptr<EdgeConstraintManager> manager) {
        constraintManager_ = std::move(manager);
    }

    /// Enable direction preservation mode
    /// When true, optimizer keeps existing sourceEdge/targetEdge values
    /// and only optimizes path routing. Used during drag operations.
    /// @param preserve True to preserve directions, false to optimize them
    void setPreserveDirections(bool preserve) {
        preserveDirections_ = preserve;
    }

    /// Check if direction preservation is enabled
    bool preserveDirections() const {
        return preserveDirections_;
    }

    /// Get the current constraint manager
    /// @return Shared pointer to constraint manager, or nullptr if not set
    std::shared_ptr<EdgeConstraintManager> constraintManager() const {
        return constraintManager_;
    }

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

protected:
    /// Check if a candidate layout passes all constraints
    /// @param candidate The edge layout to validate
    /// @param assignedLayouts Already assigned edge layouts
    /// @return true if all constraints pass (or no manager set)
    bool passesConstraints(
        const EdgeLayout& candidate,
        const std::unordered_map<EdgeId, EdgeLayout>& assignedLayouts) const {
        if (!constraintManager_) {
            return true;  // No constraints = always valid
        }
        return constraintManager_->isValid(candidate, assignedLayouts);
    }

    std::shared_ptr<EdgeConstraintManager> constraintManager_;
    bool preserveDirections_ = false;
};

}  // namespace arborvia
