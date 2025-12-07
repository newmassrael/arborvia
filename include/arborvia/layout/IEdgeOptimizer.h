#pragma once

#include "../core/Types.h"
#include "LayoutResult.h"
#include "EdgePenaltySystem.h"

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
/// Edge constraints are now integrated through EdgePenaltySystem:
/// - All optimizers share the same penalty system
/// - Adding a penalty to the system applies to ALL optimizers automatically
/// - Hard constraint penalties (weight >= 200,000) filter invalid candidates
///
/// Execution flow:
/// 1. ConstraintManager validates position (early return on failure)
/// 2. EdgeOptimizer evaluates combinations using penalty system
/// 3. Hard constraints filter invalid candidates, soft penalties rank options
///
/// Example usage:
/// @code
/// auto optimizer = std::make_shared<AStarEdgeOptimizer>();
/// optimizer->setPenaltySystem(EdgePenaltySystem::createDefault());
/// auto optimized = optimizer->optimize(edges, edgeLayouts, nodeLayouts);
/// @endcode
class IEdgeOptimizer {
public:
    virtual ~IEdgeOptimizer() = default;

    /// Set the penalty system for scoring edge layouts
    /// @param system Shared penalty system (replaces EdgeScorer and EdgeConstraintManager)
    virtual void setPenaltySystem(std::shared_ptr<EdgePenaltySystem> system) {
        penaltySystem_ = std::move(system);
    }

    /// Get the current penalty system
    /// @return Shared pointer to penalty system, or nullptr if not set
    std::shared_ptr<EdgePenaltySystem> penaltySystem() const {
        return penaltySystem_;
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

    /// Regenerate bendPoints for edges while preserving sourceEdge/targetEdge
    ///
    /// This method creates new bendPoints using the optimizer's path generation
    /// algorithm, respecting the already-decided source/target edges.
    /// Used when snap positions change after optimization (e.g., redistribution).
    ///
    /// Each optimizer implements this with its own path generation strategy:
    /// - GeometricEdgeOptimizer: Uses geometric path with obstacle avoidance
    /// - AStarEdgeOptimizer: Uses A* pathfinding
    ///
    /// @param edges List of edge IDs to regenerate bendPoints for
    /// @param edgeLayouts Edge layouts to modify (in-place)
    /// @param nodeLayouts Node positions for path calculation
    virtual void regenerateBendPoints(
        const std::vector<EdgeId>& edges,
        std::unordered_map<EdgeId, EdgeLayout>& edgeLayouts,
        const std::unordered_map<NodeId, NodeLayout>& nodeLayouts) = 0;

protected:
    /// Check if a candidate layout passes all hard constraints via penalty system
    /// @param candidate The edge layout to validate
    /// @param context Penalty context with assigned layouts and forbidden zones
    /// @return true if all hard constraints pass (or no penalty system set)
    bool passesHardConstraints(
        const EdgeLayout& candidate,
        const PenaltyContext& context) const {
        if (!penaltySystem_) {
            return true;  // No penalty system = always valid
        }
        return penaltySystem_->passesHardConstraints(candidate, context);
    }

    /// Calculate total penalty score for a candidate layout
    /// @param candidate The edge layout to score
    /// @param context Penalty context with assigned layouts and forbidden zones
    /// @return Total penalty score (0 = perfect, higher = worse)
    int calculatePenalty(
        const EdgeLayout& candidate,
        const PenaltyContext& context) const {
        if (!penaltySystem_) {
            return 0;  // No penalty system = no penalty
        }
        return penaltySystem_->calculateTotalPenalty(candidate, context);
    }

    std::shared_ptr<EdgePenaltySystem> penaltySystem_;
    bool preserveDirections_ = false;
};

}  // namespace arborvia
