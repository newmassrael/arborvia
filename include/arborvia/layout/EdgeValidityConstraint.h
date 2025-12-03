#pragma once

#include "IDragConstraint.h"

namespace arborvia {

/// Constraint that validates edge routing remains valid after node move
///
/// This is an expensive constraint that:
/// 1. Creates temporary layouts with the proposed position
/// 2. Updates edge positions for affected edges
/// 3. Validates each affected edge for orthogonality and node intersection
///
/// Example usage:
/// @code
/// ConstraintManager manager;
/// manager.addConstraint(std::make_unique<EdgeValidityConstraint>());
/// @endcode
class EdgeValidityConstraint : public IDragConstraint {
public:
    EdgeValidityConstraint() = default;

    ConstraintResult check(const ConstraintContext& ctx) const override;
    ConstraintTier tier() const override { return ConstraintTier::Expensive; }
    std::string name() const override { return "EdgeValidity"; }

    // No visualization for this constraint (too complex to show)
    bool hasVisualization() const override { return false; }
};

}  // namespace arborvia
