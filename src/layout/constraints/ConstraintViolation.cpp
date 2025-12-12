#include "arborvia/layout/constraints/ConstraintViolation.h"
#include <sstream>

namespace arborvia {

const char* constraintViolationTypeToString(ConstraintViolationType type) {
    switch (type) {
        case ConstraintViolationType::Orthogonality:
            return "Orthogonality";
        case ConstraintViolationType::NodePenetration:
            return "NodePenetration";
        case ConstraintViolationType::DirectionalSourcePenetration:
            return "DirectionalSourcePenetration";
        case ConstraintViolationType::DirectionalTargetPenetration:
            return "DirectionalTargetPenetration";
        case ConstraintViolationType::SegmentOverlap:
            return "SegmentOverlap";
        default:
            return "Unknown";
    }
}

std::string ConstraintViolation::toString() const {
    std::ostringstream oss;
    oss << "[" << constraintViolationTypeToString(type) << "] ";
    oss << "Edge " << edgeId;
    if (nodeId.has_value()) {
        oss << " -> Node " << nodeId.value();
    }
    if (otherEdgeId.has_value()) {
        oss << " overlaps Edge " << otherEdgeId.value();
    }
    if (segmentIndex >= 0) {
        oss << " (segment " << segmentIndex << ")";
    }
    if (!message.empty()) {
        oss << ": " << message;
    }
    return oss.str();
}

} // namespace arborvia
