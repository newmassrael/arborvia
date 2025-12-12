#pragma once

#include "../../core/Types.h"
#include <optional>
#include <string>
#include <vector>

namespace arborvia {

/// Types of constraint violations that can occur during edge routing
enum class ConstraintViolationType {
    /// Segment is not horizontal or vertical
    Orthogonality,

    /// Segment passes through a non-source/target node
    NodePenetration,

    /// Intermediate segment passes through the source node interior
    /// (First segment is allowed to touch source node)
    DirectionalSourcePenetration,

    /// Intermediate segment passes through the target node interior
    /// (Last segment is allowed to touch target node)
    DirectionalTargetPenetration,

    /// Segment overlaps with another edge's segment
    SegmentOverlap
};

/// Detailed information about a constraint violation
struct ConstraintViolation {
    ConstraintViolationType type;
    EdgeId edgeId = INVALID_EDGE;

    /// For penetration violations: which node was penetrated
    std::optional<NodeId> nodeId;

    /// For overlap violations: which other edge is involved
    std::optional<EdgeId> otherEdgeId;

    /// Which segment (0-based index) violated the constraint
    int segmentIndex = -1;

    /// Human-readable description of the violation
    std::string message;

    /// Convert to string for debugging
    std::string toString() const;
};

/// Convert violation type to string
const char* constraintViolationTypeToString(ConstraintViolationType type);

} // namespace arborvia
