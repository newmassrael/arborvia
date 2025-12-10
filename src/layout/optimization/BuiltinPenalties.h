#pragma once

#include "arborvia/layout/api/IEdgePenalty.h"

namespace arborvia {

/// Penalty for segment overlap with other edges
/// Weight: 200,000 (hard constraint)
class SegmentOverlapPenalty : public IEdgePenalty {
public:
    int calculatePenalty(const EdgeLayout& candidate,
                         const PenaltyContext& context) const override;
    std::string name() const override { return "SegmentOverlap"; }
    int defaultWeight() const override { return HARD_CONSTRAINT_PENALTY; }
};

/// Penalty for direction constraint violations
/// Checks if first/last segments match expected entry/exit directions
/// Weight: 200,000 (hard constraint)
class DirectionPenalty : public IEdgePenalty {
public:
    int calculatePenalty(const EdgeLayout& candidate,
                         const PenaltyContext& context) const override;
    std::string name() const override { return "Direction"; }
    int defaultWeight() const override { return HARD_CONSTRAINT_PENALTY; }
};

/// Penalty for path segments passing through nodes
/// Weight: 200,000 (hard constraint)
class NodeCollisionPenalty : public IEdgePenalty {
public:
    int calculatePenalty(const EdgeLayout& candidate,
                         const PenaltyContext& context) const override;
    std::string name() const override { return "NodeCollision"; }
    int defaultWeight() const override { return HARD_CONSTRAINT_PENALTY; }
};

/// Penalty for snap points being too close together
/// Weight: 200,000 (hard constraint)
class TooCloseSnapPenalty : public IEdgePenalty {
public:
    explicit TooCloseSnapPenalty(float minDistance = 60.0f)
        : minDistance_(minDistance) {}

    int calculatePenalty(const EdgeLayout& candidate,
                         const PenaltyContext& context) const override;
    std::string name() const override { return "TooCloseSnap"; }
    int defaultWeight() const override { return HARD_CONSTRAINT_PENALTY; }

private:
    float minDistance_;
};

/// Penalty for snap point overlap with other edges on the same NodeEdge
/// Checks both source and target snap points against all other edges
/// Weight: 200,000 (hard constraint)
class SnapPointOverlapPenalty : public IEdgePenalty {
public:
    explicit SnapPointOverlapPenalty(float tolerance = 1.0f)
        : tolerance_(tolerance) {}

    int calculatePenalty(const EdgeLayout& candidate,
                         const PenaltyContext& context) const override;
    std::string name() const override { return "SnapPointOverlap"; }
    int defaultWeight() const override { return HARD_CONSTRAINT_PENALTY; }
    bool isHardConstraint() const override { return true; }

private:
    float tolerance_;
};

/// Penalty for self-overlapping paths (backtracking)
/// Weight: 200,000 (hard constraint)
class SelfOverlapPenalty : public IEdgePenalty {
public:
    explicit SelfOverlapPenalty(float minSegmentLength = 20.0f)
        : minSegmentLength_(minSegmentLength) {}

    int calculatePenalty(const EdgeLayout& candidate,
                         const PenaltyContext& context) const override;
    std::string name() const override { return "SelfOverlap"; }
    int defaultWeight() const override { return HARD_CONSTRAINT_PENALTY; }

private:
    float minSegmentLength_;
};

/// Penalty for paths entering forbidden zones
/// Weight: 200,000 (hard constraint)
class ForbiddenZonePenalty : public IEdgePenalty {
public:
    int calculatePenalty(const EdgeLayout& candidate,
                         const PenaltyContext& context) const override;
    std::string name() const override { return "ForbiddenZone"; }
    int defaultWeight() const override { return HARD_CONSTRAINT_PENALTY; }
};

/// Penalty for path intersections with other edges
/// Weight: 1,000 (soft penalty)
class PathIntersectionPenalty : public IEdgePenalty {
public:
    explicit PathIntersectionPenalty(int weight = 1000)
        : weight_(weight) {}

    int calculatePenalty(const EdgeLayout& candidate,
                         const PenaltyContext& context) const override;
    std::string name() const override { return "PathIntersection"; }
    int defaultWeight() const override { return weight_; }

private:
    int weight_;
};

/// Penalty for non-orthogonal segments (Hard Constraint)
/// Ensures all edge segments are either horizontal or vertical
/// Weight: 200,000 (hard constraint)
class OrthogonalityPenalty : public IEdgePenalty {
public:
    int calculatePenalty(const EdgeLayout& candidate,
                         const PenaltyContext& context) const override;
    std::string name() const override { return "Orthogonality"; }
    int defaultWeight() const override { return HARD_CONSTRAINT_PENALTY; }
    bool isHardConstraint() const override { return true; }
};

/// Penalty for changing endpoints on fixed (unmoved) nodes
/// When a node was not moved during drag, its edge endpoints should be preserved
/// Weight: 200,000 (hard constraint)
class FixedEndpointPenalty : public IEdgePenalty {
public:
    explicit FixedEndpointPenalty(float tolerance = 1.0f)
        : tolerance_(tolerance) {}

    int calculatePenalty(const EdgeLayout& candidate,
                         const PenaltyContext& context) const override;
    std::string name() const override { return "FixedEndpoint"; }
    int defaultWeight() const override { return HARD_CONSTRAINT_PENALTY; }
    bool isHardConstraint() const override { return true; }

private:
    float tolerance_;
};

}  // namespace arborvia
