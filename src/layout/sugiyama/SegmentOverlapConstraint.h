#pragma once

#include "arborvia/layout/IEdgeConstraint.h"

namespace arborvia {

/// Constraint: Edge segments must not overlap (share the same path)
/// 
/// This prevents two edges from using the same line segment.
/// Even partial overlap is forbidden.
///
/// Example of violation:
///   Edge A: (100,200) → (300,200)  // horizontal at y=200
///   Edge B: (150,200) → (250,200)  // overlaps at y=200, x=150-250
class SegmentOverlapConstraint : public IEdgeConstraint {
public:
    bool isValid(
        const EdgeLayout& candidate,
        const std::unordered_map<EdgeId, EdgeLayout>& assignedLayouts) const override;

    std::string name() const override { return "SegmentOverlap"; }
};

}  // namespace arborvia
