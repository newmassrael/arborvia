#pragma once

#include "arborvia/layout/api/IDragConstraint.h"

namespace arborvia {

/// Constraint that prevents node overlap
///
/// This is a simple AABB (Axis-Aligned Bounding Box) collision check
/// that ensures nodes don't overlap with each other. For direction-aware
/// margins that consider edge connections, use DirectionAwareMarginConstraint.
///
/// Example usage:
/// @code
/// ConstraintManager manager;
/// manager.addConstraint(std::make_unique<NoOverlapConstraint>());
/// @endcode
class NoOverlapConstraint : public IDragConstraint {
public:
    /// Create a no-overlap constraint
    /// @param margin Additional margin around nodes (default 0)
    explicit NoOverlapConstraint(float margin = 0.0f);

    ConstraintResult check(const ConstraintContext& ctx) const override;
    ConstraintTier tier() const override { return ConstraintTier::Fast; }
    std::string name() const override { return "NoOverlap"; }

    bool hasVisualization() const override { return true; }
    std::vector<Rect> getBlockedRegions(const ConstraintContext& ctx) const override;

    float margin() const { return margin_; }

private:
    float margin_;
};

}  // namespace arborvia
