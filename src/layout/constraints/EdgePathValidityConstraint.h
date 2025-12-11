#pragma once

#include "arborvia/layout/api/IDragConstraint.h"

namespace arborvia {

/// Constraint that validates A* paths exist for all connected edges
///
/// This is an expensive constraint that performs A* pathfinding to verify
/// that all edges connected to the moved node can have valid paths.
/// It should be placed in the Expensive tier to run after cheaper checks.
///
/// Example usage:
/// @code
/// ConstraintManager manager;
/// manager.addConstraint(std::make_unique<EdgePathValidityConstraint>(gridSize));
/// @endcode
class EdgePathValidityConstraint : public IDragConstraint {
public:
    /// Create an edge path validity constraint
    /// @param gridSize Grid size for A* pathfinding
    explicit EdgePathValidityConstraint(float gridSize = 10.0f);

    ConstraintResult check(const ConstraintContext& ctx) const override;
    ConstraintTier tier() const override { return ConstraintTier::Expensive; }
    std::string name() const override { return "EdgePathValidity"; }

    float gridSize() const { return gridSize_; }

private:
    /// Calculate snap point position on a node edge
    Point calculateSnapPointOnEdge(
        const NodeLayout& node,
        NodeEdge edge,
        int snapIndex) const;

    float gridSize_;
};

}  // namespace arborvia
