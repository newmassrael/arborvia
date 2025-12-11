#pragma once

#include "arborvia/layout/api/IDragConstraint.h"

namespace arborvia {

/// Constraint that enforces direction-aware margins between nodes
///
/// This constraint considers the number of edge connections on each side
/// of nodes to calculate appropriate margins. Sides with more snap points
/// require larger margins to ensure proper edge routing.
///
/// The margin calculation follows:
///   margin(side) = baseMargin + max(0, snapPoints - threshold) * gridSize
///
/// This replaces the previous ValidRegionCalculator-based approach with
/// a proper IDragConstraint implementation for architectural consistency.
///
/// Example usage:
/// @code
/// ConstraintManager manager;
/// manager.addConstraint(std::make_unique<DirectionAwareMarginConstraint>());
/// @endcode
class DirectionAwareMarginConstraint : public IDragConstraint {
public:
    /// Create a direction-aware margin constraint
    /// @param baseGridDistance Base minimum distance in grid units (default 5.0)
    explicit DirectionAwareMarginConstraint(float baseGridDistance = 5.0f);

    ConstraintResult check(const ConstraintContext& ctx) const override;
    ConstraintTier tier() const override { return ConstraintTier::Fast; }
    std::string name() const override { return "DirectionAwareMargin"; }

    bool hasVisualization() const override { return true; }
    std::vector<Rect> getBlockedRegions(const ConstraintContext& ctx) const override;

    /// Get the base grid distance
    float baseGridDistance() const { return baseGridDistance_; }

private:
    float baseGridDistance_;
};

}  // namespace arborvia
