#pragma once

#include "../IEdgeConstraint.h"

namespace arborvia {

/// Constraint that ensures all edge segments are orthogonal (horizontal or vertical)
class OrthogonalityConstraint : public IEdgeConstraint {
public:
    std::vector<ConstraintViolation> check(
        const EdgeLayout& layout,
        const EdgeConstraintContext& ctx) const override;

    std::string name() const override { return "Orthogonality"; }
    bool isHardConstraint() const override { return true; }
};

} // namespace arborvia
