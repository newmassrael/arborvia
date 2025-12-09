#pragma once

#include "arborvia/core/Types.h"
#include "arborvia/layout/config/LayoutResult.h"
#include "arborvia/layout/config/LayoutOptions.h"

namespace arborvia {

/// Routes self-loop edges (edges where source == target)
/// 
/// CONSTRAINT: Self-loops must use adjacent edges only:
/// - Same edge forbidden: left→left, right→right, etc.
/// - Opposite edge forbidden: left→right, top→bottom, etc.
/// - Adjacent edges only: left→top, left→bottom, right→top, right→bottom, etc.
class SelfLoopRouter {
public:
    /// Default self-loop spacing when grid is disabled
    static constexpr float DEFAULT_SPACING = 10.0f;

    /// Default offset for self-loop routing (distance from node edge)
    static constexpr float DEFAULT_OFFSET = 20.0f;

    /// Adjacent edges for a given source edge
    struct AdjacentEdges {
        NodeEdge primary;    ///< Primary adjacent edge (used by default)
        NodeEdge secondary;  ///< Secondary adjacent edge (alternative)
    };

    /// Get adjacent edges for a source edge (constraint: only these are valid targets)
    /// @param sourceEdge The edge where self-loop starts
    /// @return Adjacent edges that are valid targets
    static AdjacentEdges getAdjacentEdges(NodeEdge sourceEdge);

    /// Check if a self-loop edge combination is valid (constraint check)
    /// @param source Source edge of self-loop
    /// @param target Target edge of self-loop
    /// @return true if combination is valid (adjacent edges), false otherwise
    static bool isValidSelfLoopCombination(NodeEdge source, NodeEdge target);

    /// Determine if a self-loop endpoint should be at first or last snap index
    /// Self-loops are placed at corners, so endpoints need to be at edge extremes
    /// @param thisEdge The edge where this endpoint is located
    /// @param otherEdge The edge where the other endpoint is located
    /// @return true if endpoint should be at last index (corner), false for first index
    static bool shouldBeAtLastIndex(NodeEdge thisEdge, NodeEdge otherEdge);

    /// Reorder connections to place self-loop endpoints at corner positions
    /// CONSTRAINT: Self-loops must be at corners with no other snap points between them
    /// @param connections Vector of (edgeId, isSource) pairs to reorder in-place
    /// @param edgeLayouts Edge layouts to check for self-loops
    /// @param nodeEdge The node edge being processed
    static void applySelfLoopCornerPositioning(
        std::vector<std::pair<EdgeId, bool>>& connections,
        const std::unordered_map<EdgeId, EdgeLayout>& edgeLayouts,
        NodeEdge nodeEdge);

    /// Route a self-loop edge around a node
    /// @param edgeId The edge identifier
    /// @param from Source/target node ID (same for self-loops)
    /// @param nodeLayout The node layout
    /// @param loopIndex Index for stacking multiple self-loops (0-based)
    /// @param options Layout options containing self-loop configuration
    /// @return The routed edge layout
    static EdgeLayout route(
        EdgeId edgeId,
        NodeId from,
        const NodeLayout& nodeLayout,
        int loopIndex,
        const LayoutOptions& options);

    /// Calculate spacing between source and target points on self-loop
    /// @param gridSize Grid cell size (0 = use default)
    /// @return Spacing distance
    static float calculateSpacing(float gridSize);

    /// Calculate loop index for consistent ordering of multiple self-loops
    /// Uses edge ID ordering to ensure deterministic results in parallel execution.
    /// @param edgeId Current edge being processed
    /// @param nodeId Node where self-loops are attached
    /// @param layouts All edge layouts to search for other self-loops
    /// @return Loop index (0 for first self-loop, 1 for second, etc.)
    static int calculateLoopIndex(
        EdgeId edgeId,
        NodeId nodeId,
        const std::unordered_map<EdgeId, EdgeLayout>& layouts);
};

}  // namespace arborvia
