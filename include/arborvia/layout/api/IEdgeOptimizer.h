#pragma once

#include "../../core/Types.h"
#include "../config/LayoutResult.h"
#include "EdgePenaltySystem.h"

#include <memory>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace arborvia {

/// Interface for edge routing optimization algorithms
///
/// Edge constraints are integrated through EdgePenaltySystem:
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
    virtual void setPenaltySystem(std::shared_ptr<EdgePenaltySystem> system) {
        penaltySystem_ = std::move(system);
    }

    /// Get the current penalty system
    std::shared_ptr<EdgePenaltySystem> penaltySystem() const {
        return penaltySystem_;
    }

    /// Enable direction preservation mode
    /// When true, optimizer keeps existing sourceEdge/targetEdge values
    /// and only optimizes path routing. Used during drag operations.
    void setPreserveDirections(bool preserve) {
        preserveDirections_ = preserve;
    }

    /// Check if direction preservation is enabled
    bool preserveDirections() const {
        return preserveDirections_;
    }

    /// Optimize edge routing by selecting best source/target edge combinations
    /// @param edges List of edge IDs to optimize
    /// @param currentLayouts Current edge layouts (used as original for constraint checking)
    /// @param nodeLayouts Node positions for routing calculations
    /// @param movedNodes Nodes that were moved (endpoints on other nodes are fixed)
    ///                   Empty set means no constraints (all endpoints can change)
    virtual std::unordered_map<EdgeId, EdgeLayout> optimize(
        const std::vector<EdgeId>& edges,
        const std::unordered_map<EdgeId, EdgeLayout>& currentLayouts,
        const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
        const std::unordered_set<NodeId>& movedNodes = {}) = 0;

    /// Get algorithm name for debugging/logging
    virtual const char* algorithmName() const = 0;

    /// Regenerate bendPoints for edges while preserving sourceEdge/targetEdge
    virtual void regenerateBendPoints(
        const std::vector<EdgeId>& edges,
        std::unordered_map<EdgeId, EdgeLayout>& edgeLayouts,
        const std::unordered_map<NodeId, NodeLayout>& nodeLayouts) = 0;

protected:
    /// Check if a candidate layout passes all hard constraints via penalty system
    bool passesHardConstraints(
        const EdgeLayout& candidate,
        const PenaltyContext& context) const {
        if (!penaltySystem_) {
            return true;
        }
        return penaltySystem_->passesHardConstraints(candidate, context);
    }

    /// Calculate total penalty score for a candidate layout
    int calculatePenalty(
        const EdgeLayout& candidate,
        const PenaltyContext& context) const {
        if (!penaltySystem_) {
            return 0;
        }
        return penaltySystem_->calculateTotalPenalty(candidate, context);
    }

    /// Create PenaltyContext with all required fields populated
    /// This is the SINGLE POINT for PenaltyContext creation to ensure consistency
    PenaltyContext createPenaltyContext(
        const std::unordered_map<EdgeId, EdgeLayout>& assignedLayouts,
        const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
        const std::vector<ForbiddenZone>& forbiddenZones,
        float gridSize,
        const EdgeLayout& candidate) const {
        
        PenaltyContext ctx{assignedLayouts, nodeLayouts, forbiddenZones, gridSize};
        ctx.sourceNodeId = candidate.from;
        ctx.targetNodeId = candidate.to;
        ctx.originalLayouts = originalLayouts_;
        ctx.movedNodes = movedNodes_;
        return ctx;
    }
    
    /// Store constraint state at the start of optimize()
    /// This enables FixedEndpointPenalty to check endpoint preservation
    void setConstraintState(
        const std::unordered_map<EdgeId, EdgeLayout>& originalLayouts,
        const std::unordered_set<NodeId>& movedNodes) {
        originalLayouts_ = &originalLayouts;
        movedNodes_ = movedNodes;
    }
    
    /// Check if a node is fixed (not moved during drag)
    bool isNodeFixed(NodeId nodeId) const {
        if (movedNodes_.empty()) return false;
        return movedNodes_.count(nodeId) == 0;
    }

    std::shared_ptr<EdgePenaltySystem> penaltySystem_;
    bool preserveDirections_ = false;
    
    // Constraint state for FixedEndpointPenalty
    const std::unordered_map<EdgeId, EdgeLayout>* originalLayouts_ = nullptr;
    std::unordered_set<NodeId> movedNodes_;
};

}  // namespace arborvia
