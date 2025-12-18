#pragma once

#include "arborvia/layout/api/IPathFinder.h"
#include "arborvia/layout/config/LayoutTypes.h"
#include "arborvia/layout/api/EdgePenaltySystem.h"
#include "../../sugiyama/routing/EdgeRoutingUtils.h"

#include <memory>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace arborvia {

/// Context for edge routing operations
/// Bundles all dependencies needed by NodeEdgeSelector and OverlapResolver
/// This enables clean dependency injection without back-references to optimizer
struct EdgeRoutingContext {
    // Core dependencies
    IPathFinder* pathFinder = nullptr;
    std::shared_ptr<EdgePenaltySystem> penaltySystem;

    // Layout state (immutable during optimization)
    const std::unordered_map<NodeId, NodeLayout>* nodeLayouts = nullptr;
    const std::vector<ForbiddenZone>* forbiddenZones = nullptr;

    // Grid configuration
    float gridSize = 0.0f;

    // Constraint state for penalty calculations
    const std::unordered_map<EdgeId, EdgeLayout>* originalLayouts = nullptr;
    std::unordered_set<NodeId> movedNodes;

    // Behavior flags
    bool preserveDirections = false;

    /// Create PenaltyContext with all required fields populated
    /// @param assignedLayouts Currently assigned edge layouts for intersection checking
    /// @param candidate The candidate layout being evaluated
    /// @pre nodeLayouts and forbiddenZones must not be null
    PenaltyContext createPenaltyContext(
        const std::unordered_map<EdgeId, EdgeLayout>& assignedLayouts,
        const EdgeLayout& candidate) const {

        // Use static empty containers to avoid dangling references
        // when ternary operator would create temporaries
        static const std::unordered_map<NodeId, NodeLayout> emptyNodeLayouts;
        static const std::vector<ForbiddenZone> emptyForbiddenZones;

        PenaltyContext ctx{
            assignedLayouts,
            nodeLayouts ? *nodeLayouts : emptyNodeLayouts,
            forbiddenZones ? *forbiddenZones : emptyForbiddenZones,
            gridSize
        };
        ctx.sourceNodeId = candidate.from;
        ctx.targetNodeId = candidate.to;
        ctx.originalLayouts = originalLayouts;
        ctx.movedNodes = movedNodes;
        return ctx;
    }

    /// Calculate total penalty score for a candidate layout
    /// @return Penalty score (0 if no penalty system)
    int calculatePenalty(
        const EdgeLayout& candidate,
        const PenaltyContext& context) const {
        if (!penaltySystem) {
            return 0;
        }
        return penaltySystem->calculateTotalPenalty(candidate, context);
    }

    /// Check if a candidate layout passes all hard constraints
    /// @return true if passes (or no penalty system configured)
    bool passesHardConstraints(
        const EdgeLayout& candidate,
        const PenaltyContext& context) const {
        if (!penaltySystem) {
            return true;
        }
        return penaltySystem->passesHardConstraints(candidate, context);
    }

    /// Check if a node is fixed (not moved during drag)
    /// @return true if node is fixed (should not change endpoints)
    bool isNodeFixed(NodeId nodeId) const {
        if (movedNodes.empty()) return false;
        return movedNodes.count(nodeId) == 0;
    }

    /// Validate that all required fields are set
    /// @return true if context is valid for routing operations
    bool isValid() const {
        return pathFinder != nullptr &&
               nodeLayouts != nullptr &&
               forbiddenZones != nullptr &&
               gridSize > 0.0f;
    }
};

}  // namespace arborvia
