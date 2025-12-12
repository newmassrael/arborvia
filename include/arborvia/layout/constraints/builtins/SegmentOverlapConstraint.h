#pragma once

#include "../IEdgeConstraint.h"

namespace arborvia {

/// Soft constraint that detects segment overlap between edges
/// 
/// This is a soft constraint - overlapping edges are penalized but allowed.
/// Useful for scoring and optimization but doesn't cause validation failure.
class SegmentOverlapConstraint : public IEdgeConstraint {
public:
    std::vector<ConstraintViolation> check(
        const EdgeLayout& layout,
        const EdgeConstraintContext& ctx) const override;

    std::string name() const override { return "SegmentOverlap"; }
    bool isHardConstraint() const override { return false; }  // Soft constraint

private:
    /// Check if two segments overlap (collinear and share length)
    bool segmentsOverlap(
        const Point& a1, const Point& a2,
        const Point& b1, const Point& b2,
        float tolerance) const;
};

} // namespace arborvia
