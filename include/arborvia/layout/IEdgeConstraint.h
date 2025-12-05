#pragma once

#include "arborvia/core/Types.h"
#include "arborvia/layout/LayoutResult.h"

#include <string>
#include <unordered_map>

namespace arborvia {

/// Interface for edge routing constraints
/// Constraints determine which edge routing candidates are valid
/// 
/// Unlike scoring (which ranks options), constraints are binary:
/// - Valid: candidate can be used
/// - Invalid: candidate must be rejected
///
/// Adding a constraint to EdgeConstraintManager automatically applies
/// it to ALL optimizers (AStarEdgeOptimizer, GeometricEdgeOptimizer, etc.)
class IEdgeConstraint {
public:
    virtual ~IEdgeConstraint() = default;

    /// Check if a candidate edge layout satisfies this constraint
    /// @param candidate The edge layout being evaluated
    /// @param assignedLayouts Other edges already assigned (for checking conflicts)
    /// @return true if constraint is satisfied, false if violated
    virtual bool isValid(
        const EdgeLayout& candidate,
        const std::unordered_map<EdgeId, EdgeLayout>& assignedLayouts) const = 0;

    /// Get human-readable name of this constraint
    /// @return Constraint name (e.g., "SegmentOverlap", "AdjacentEdge")
    virtual std::string name() const = 0;
};

}  // namespace arborvia
