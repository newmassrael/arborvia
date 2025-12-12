#pragma once

#include "arborvia/core/Types.h"
#include <set>

namespace arborvia {

/// Position of a segment within an edge path
/// Used to determine which nodes can be excluded from collision detection
enum class SegmentPosition {
    First,   ///< First segment (sourcePoint -> first bend) - source node excluded
    Middle,  ///< Middle segments - no exclusions, all nodes checked
    Last     ///< Last segment (last bend -> targetPoint) - target node excluded
};

/// Single Source of Truth for segment-based obstacle exclusion logic
/// 
/// This class centralizes the logic for determining which nodes should be
/// excluded from collision detection based on segment position. All algorithms
/// (A*, Geometric, future algorithms) should use this API to ensure consistent
/// constraint enforcement.
///
/// Exclusion Rules:
/// - First segment: source node excluded (edge exits from source)
/// - Middle segments: no exclusions (must avoid all nodes)
/// - Last segment: target node excluded (edge enters target)
///
/// This matches the DirectionalPenetrationConstraint logic and ensures that
/// intermediate segments cannot penetrate source/target nodes.
class SegmentObstacleProvider {
public:
    /// Get the set of nodes that should be excluded from collision detection
    /// for a segment at the given position
    ///
    /// @param sourceNode The source node of the edge
    /// @param targetNode The target node of the edge
    /// @param position The position of the segment in the path
    /// @return Set of node IDs to exclude from collision detection
    [[nodiscard]] static std::set<NodeId> getExcludedNodes(
        NodeId sourceNode,
        NodeId targetNode,
        SegmentPosition position
    );

    /// Check if a specific node should be excluded for a segment at the given position
    ///
    /// @param nodeId The node to check
    /// @param sourceNode The source node of the edge
    /// @param targetNode The target node of the edge
    /// @param position The position of the segment in the path
    /// @return true if the node should be excluded from collision detection
    [[nodiscard]] static bool shouldExcludeNode(
        NodeId nodeId,
        NodeId sourceNode,
        NodeId targetNode,
        SegmentPosition position
    );

    /// Determine the segment position based on segment index and total segment count
    ///
    /// @param segmentIndex 0-based index of the segment
    /// @param totalSegments Total number of segments in the path
    /// @return The position classification of the segment
    [[nodiscard]] static SegmentPosition getSegmentPosition(
        size_t segmentIndex,
        size_t totalSegments
    );
};

}  // namespace arborvia
