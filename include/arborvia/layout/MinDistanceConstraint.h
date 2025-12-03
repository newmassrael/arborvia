#pragma once

#include "IDragConstraint.h"

namespace arborvia {

/// Constraint that enforces minimum distance between nodes
///
/// Nodes must be at least minGridDistance grid units apart.
/// This constraint supports visualization by returning the forbidden
/// zones around each node.
///
/// Example usage:
/// @code
/// ConstraintManager manager;
/// manager.addConstraint(std::make_unique<MinDistanceConstraint>(5.0f));
/// @endcode
class MinDistanceConstraint : public IDragConstraint {
public:
    /// Create a minimum distance constraint
    /// @param minGridDistance Minimum distance in grid units (default 5.0)
    explicit MinDistanceConstraint(float minGridDistance = 5.0f);

    ConstraintResult check(const ConstraintContext& ctx) const override;
    ConstraintTier tier() const override { return ConstraintTier::Fast; }
    std::string name() const override { return "MinDistance"; }

    bool hasVisualization() const override { return true; }
    std::vector<Rect> getBlockedRegions(const ConstraintContext& ctx) const override;

    /// Get the minimum grid distance
    float minGridDistance() const { return minGridDistance_; }

private:
    float minGridDistance_;
};

}  // namespace arborvia
