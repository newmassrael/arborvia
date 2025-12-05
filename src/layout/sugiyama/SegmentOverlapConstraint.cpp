#include "SegmentOverlapConstraint.h"
#include "PathIntersection.h"

namespace arborvia {

bool SegmentOverlapConstraint::isValid(
    const EdgeLayout& candidate,
    const std::unordered_map<EdgeId, EdgeLayout>& assignedLayouts) const {
    
    // Check if candidate has any segment overlap with assigned edges
    return !PathIntersection::hasOverlapWithOthers(candidate, assignedLayouts, candidate.id);
}

}  // namespace arborvia
