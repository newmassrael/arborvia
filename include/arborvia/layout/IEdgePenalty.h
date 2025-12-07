#pragma once

#include "../core/Types.h"
#include "LayoutResult.h"
#include "ValidRegionCalculator.h"

#include <string>
#include <unordered_map>
#include <vector>

namespace arborvia {

/// Context passed to penalty calculations
/// Contains all information needed to evaluate edge routing quality
struct PenaltyContext {
    /// Already assigned edge layouts (for overlap detection)
    const std::unordered_map<EdgeId, EdgeLayout>& assignedLayouts;

    /// Node positions and sizes (for collision detection)
    const std::unordered_map<NodeId, NodeLayout>& nodeLayouts;

    /// Pre-calculated forbidden zones (for constraint violation)
    const std::vector<ForbiddenZone>& forbiddenZones;

    /// Grid size in pixels (for distance calculations)
    float gridSize = 20.0f;

    /// Source and target node IDs (for exclusion in collision checks)
    NodeId sourceNodeId = INVALID_NODE;
    NodeId targetNodeId = INVALID_NODE;
};

/// Hard constraint penalty value (200,000)
/// Any penalty >= this value is considered a hard constraint violation
constexpr int HARD_CONSTRAINT_PENALTY = 200000;

/// Interface for edge routing penalties
///
/// Penalties are evaluated during edge optimization to select
/// the best source/target edge combination. Each penalty calculates
/// a score based on how well the candidate layout satisfies its constraint.
///
/// Penalty types:
/// - Hard constraints (200,000): Must be satisfied, violations are rejected
/// - Soft penalties (<200,000): Preferences, lower scores are better
///
/// Usage:
/// @code
/// auto penaltySystem = EdgePenaltySystem::createDefault();
/// penaltySystem->addPenalty(std::make_unique<CustomPenalty>());
/// @endcode
class IEdgePenalty {
public:
    virtual ~IEdgePenalty() = default;

    /// Calculate penalty score for a candidate edge layout
    /// @param candidate The edge layout to evaluate
    /// @param context Context with node layouts, other edges, etc.
    /// @return Penalty score (0 = perfect, higher = worse)
    ///         Returns 0 if constraint is satisfied
    ///         Returns defaultWeight() * violation_count if violated
    virtual int calculatePenalty(
        const EdgeLayout& candidate,
        const PenaltyContext& context) const = 0;

    /// Get the penalty name for debugging/logging
    /// @return Unique identifier for this penalty type
    virtual std::string name() const = 0;

    /// Get the default weight for this penalty
    /// @return Weight value (200,000 for hard constraints)
    virtual int defaultWeight() const = 0;

    /// Check if this is a hard constraint
    /// Hard constraints must be satisfied; violations are rejected
    /// @return true if defaultWeight() >= HARD_CONSTRAINT_PENALTY
    virtual bool isHardConstraint() const {
        return defaultWeight() >= HARD_CONSTRAINT_PENALTY;
    }
};

}  // namespace arborvia
