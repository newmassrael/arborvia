#pragma once

#include "ConstraintViolation.h"
#include "../config/LayoutResult.h"
#include <string>
#include <unordered_map>
#include <vector>

namespace arborvia {

/// Context provided to constraint checking functions
struct EdgeConstraintContext {
    /// All edge layouts (for overlap/intersection checking)
    const std::unordered_map<EdgeId, EdgeLayout>& allEdgeLayouts;

    /// All node layouts (for penetration checking)
    const std::unordered_map<NodeId, NodeLayout>& nodeLayouts;

    /// Grid size used for pathfinding
    float gridSize = 20.0f;

    /// Tolerance for floating point comparisons
    float tolerance = 1.0f;
};

/// Interface for edge routing constraints
///
/// Each constraint checks a specific property that edge layouts should satisfy.
/// Constraints can be "hard" (must be satisfied) or "soft" (penalized but allowed).
class IEdgeConstraint {
public:
    virtual ~IEdgeConstraint() = default;

    /// Check the constraint for a single edge layout
    /// @param layout The edge layout to check
    /// @param ctx Context with other edges and nodes for cross-checking
    /// @return List of violations (empty if constraint is satisfied)
    virtual std::vector<ConstraintViolation> check(
        const EdgeLayout& layout,
        const EdgeConstraintContext& ctx) const = 0;

    /// Get the unique name of this constraint
    virtual std::string name() const = 0;

    /// Check if this is a hard constraint (must be satisfied)
    /// Hard constraints cause validation failures.
    /// Soft constraints are penalized but don't cause failure.
    virtual bool isHardConstraint() const = 0;
};

} // namespace arborvia
