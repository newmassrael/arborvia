#include "arborvia/layout/constraints/SegmentObstacleProvider.h"

namespace arborvia {

std::set<NodeId> SegmentObstacleProvider::getExcludedNodes(
    NodeId sourceNode,
    NodeId targetNode,
    SegmentPosition position) {
    
    switch (position) {
        case SegmentPosition::First:
            // First segment exits from source node - exclude source from collision
            return {sourceNode};
        
        case SegmentPosition::Last:
            // Last segment enters target node - exclude target from collision
            return {targetNode};
        
        case SegmentPosition::Middle:
        default:
            // Middle segments must avoid ALL nodes including source/target
            // This prevents penetration through source/target interiors
            return {};
    }
}

bool SegmentObstacleProvider::shouldExcludeNode(
    NodeId nodeId,
    NodeId sourceNode,
    NodeId targetNode,
    SegmentPosition position) {
    
    switch (position) {
        case SegmentPosition::First:
            return nodeId == sourceNode;
        
        case SegmentPosition::Last:
            return nodeId == targetNode;
        
        case SegmentPosition::Middle:
        default:
            return false;
    }
}

SegmentPosition SegmentObstacleProvider::getSegmentPosition(
    size_t segmentIndex,
    size_t totalSegments) {
    
    if (totalSegments == 0) {
        return SegmentPosition::Middle;
    }
    
    if (totalSegments == 1) {
        // Single segment path: it's both first and last
        // For collision purposes, we should check against all intermediate nodes
        // but allow touching source at start and target at end
        // This is a special case - treat as First (source excluded)
        // Target exclusion is implicit since segment ends at target
        return SegmentPosition::First;
    }
    
    if (segmentIndex == 0) {
        return SegmentPosition::First;
    }
    
    if (segmentIndex == totalSegments - 1) {
        return SegmentPosition::Last;
    }
    
    return SegmentPosition::Middle;
}

}  // namespace arborvia
