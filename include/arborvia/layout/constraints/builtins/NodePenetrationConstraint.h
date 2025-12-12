#pragma once

#include "../IEdgeConstraint.h"

namespace arborvia {

/// Constraint that ensures edge segments don't pass through non-source/target nodes
class NodePenetrationConstraint : public IEdgeConstraint {
public:
    std::vector<ConstraintViolation> check(
        const EdgeLayout& layout,
        const EdgeConstraintContext& ctx) const override;

    std::string name() const override { return "NodePenetration"; }
    bool isHardConstraint() const override { return true; }

private:
    /// Check if a segment intersects with a node's AABB
    bool segmentIntersectsNode(
        const Point& p1, const Point& p2,
        const NodeLayout& node,
        float tolerance) const;
};

} // namespace arborvia
