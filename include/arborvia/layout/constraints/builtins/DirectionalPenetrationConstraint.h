#pragma once

#include "../IEdgeConstraint.h"

namespace arborvia {

/// Constraint that ensures intermediate segments don't penetrate source node
/// 
/// The first segment (sourcePoint -> first bendPoint) is allowed to touch/exit
/// the source node. All other segments must NOT penetrate the source node interior.
/// Uses geometry::segmentPenetratesNodeInterior for consistent checking.
class DirectionalSourcePenetrationConstraint : public IEdgeConstraint {
public:
    std::vector<ConstraintViolation> check(
        const EdgeLayout& layout,
        const EdgeConstraintContext& ctx) const override;

    std::string name() const override { return "DirectionalSourcePenetration"; }
    bool isHardConstraint() const override { return true; }
};

/// Constraint that ensures intermediate segments don't penetrate target node
/// 
/// The last segment (last bendPoint -> targetPoint) is allowed to touch/enter
/// the target node. All other segments must NOT penetrate the target node interior.
/// Uses geometry::segmentPenetratesNodeInterior for consistent checking.
class DirectionalTargetPenetrationConstraint : public IEdgeConstraint {
public:
    std::vector<ConstraintViolation> check(
        const EdgeLayout& layout,
        const EdgeConstraintContext& ctx) const override;

    std::string name() const override { return "DirectionalTargetPenetration"; }
    bool isHardConstraint() const override { return true; }
};

} // namespace arborvia
