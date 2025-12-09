#pragma once

#include "arborvia/core/Types.h"
#include "arborvia/layout/config/LayoutTypes.h"
#include "arborvia/layout/config/LayoutOptions.h"
#include "arborvia/layout/config/LayoutResult.h"

#include <unordered_map>
#include <vector>
#include <map>
#include <string>
#include <stdexcept>

namespace arborvia {


/// Error types for snap index operations
enum class SnapIndexError {
    None,
    InvalidIndex,
    InvalidCount,
    IndexOutOfRange,
    NodeNotFound,
    EdgeNotFound
};

/// Result of snap index validation
struct SnapIndexValidation {
    bool valid = true;
    SnapIndexError error = SnapIndexError::None;
    std::string message;
    int correctedIndex = 0;  // Suggested correction if invalid

    static SnapIndexValidation ok(int index) {
        return {true, SnapIndexError::None, "", index};
    }

    static SnapIndexValidation fail(SnapIndexError err, const std::string& msg, int corrected = 0) {
        return {false, err, msg, corrected};
    }
};

/// Connection info for a node edge
struct NodeEdgeConnections {
    std::vector<EdgeId> incoming;  // Edges where this node is target
    std::vector<EdgeId> outgoing;  // Edges where this node is source

    int incomingCount() const { return static_cast<int>(incoming.size()); }
    int outgoingCount() const { return static_cast<int>(outgoing.size()); }
    int totalCount() const { return incomingCount() + outgoingCount(); }
    bool hasOnlyIncoming() const { return incomingCount() > 0 && outgoingCount() == 0; }
    bool hasOnlyOutgoing() const { return outgoingCount() > 0 && incomingCount() == 0; }
    bool hasBoth() const { return incomingCount() > 0 && outgoingCount() > 0; }
};

/// Range for snap point distribution
struct SnapRange {
    float start = 0.0f;
    float end = 1.0f;

    float midpoint() const { return (start + end) / 2.0f; }
    float width() const { return end - start; }
    bool isFullRange() const { return start == 0.0f && end == 1.0f; }
};

/// Manages snap point indices for edge routing
///
/// Unified mode (default):
///   - All edges share a single index space [0, totalCount)
///   - All edges use the full range [0.0, 1.0)
class SnapIndexManager {
public:
    // === Index Conversion ===

    /// Convert unified snap index to local index within a group
    /// @param unifiedIdx The unified snap index from EdgeLayout
    /// @param offset The offset for this group (0 for incoming, inCount for outgoing)
    /// @param count The count of items in this group
    /// @return Local index clamped to [0, count)
    static int unifiedToLocal(int unifiedIdx, int offset, int count);

    /// Convert local index to unified index
    /// @param localIdx Local index within group [0, count)
    /// @param offset The offset for this group
    /// @return Unified index
    static int localToUnified(int localIdx, int offset);

    // === Validation ===

    /// Validate a snap index
    /// @param index The index to validate
    /// @param count Maximum valid count
    /// @param context Description for error messages
    /// @return Validation result with corrected index if needed
    static SnapIndexValidation validate(int index, int count, const char* context = "snap index");

    /// Validate and clamp index to valid range
    /// @param index The index to validate
    /// @param count Maximum valid count
    /// @return Clamped index
    static int clamp(int index, int count);

    // === Range Calculation ===

    /// Calculate relative position for a snap point
    /// @param localIdx Local index within the group [0, count)
    /// @param count Total count in this group
    /// @param range The range to distribute within
    /// @return Relative position (0.0 to 1.0)
    static float calculatePosition(int localIdx, int count, const SnapRange& range);

    /// Calculate grid-aligned positions for snap points with minimum spacing
    /// Ensures no two snap points occupy the same grid cell
    /// @param edgeLength Length of the node edge in pixels
    /// @param count Number of snap points needed
    /// @param gridSize Size of grid cell in pixels
    /// @param minSpacing Minimum spacing between snap points (default: gridSize)
    /// @return Vector of pixel positions along the edge (relative to edge start)
    static std::vector<float> calculateGridAlignedPositions(
        float edgeLength, int count, float gridSize, float minSpacing = 0.0f);

    /// Calculate required length for snap points with minimum spacing
    /// @param count Number of snap points
    /// @param gridSize Size of grid cell
    /// @return Minimum required length in pixels
    static float calculateRequiredLength(int count, float gridSize);

    // === Connection Analysis ===

    /// Build connection map for all node edges from edge layouts
    /// Key: (NodeId, NodeEdge) -> NodeEdgeConnections
    static std::map<std::pair<NodeId, NodeEdge>, NodeEdgeConnections> buildConnectionMap(
        const std::unordered_map<EdgeId, EdgeLayout>& edgeLayouts);

    /// Count connections for a specific node edge
    static NodeEdgeConnections getConnections(
        const std::unordered_map<EdgeId, EdgeLayout>& edgeLayouts,
        NodeId nodeId,
        NodeEdge nodeEdge);

    // === Debug Helpers ===

    /// Format snap index info for debugging
    static std::string formatIndexInfo(int unifiedIdx, int localIdx, int offset, int count);

    // === Snap Point Sorting ===

    /// Sort snap points by other node position to minimize edge crossings
    /// Uses the scxml-core-engine/visualizer algorithm:
    /// - Incoming edges sorted by source node position
    /// - Outgoing edges sorted by target node position
    /// - For horizontal edges (Top/Bottom): sort by X coordinate
    /// - For vertical edges (Left/Right): sort by Y coordinate
    /// - Incoming edges sorted in reverse for horizontal edges
    /// 
    /// @param nodeId Target node whose snap points are being sorted
    /// @param edge Which edge of the node
    /// @param edgeLayouts All edge layouts
    /// @param nodeLayouts All node layouts (for position lookup)
    /// @return Sorted pairs (EdgeId, isSource): incoming first (isSource=false), then outgoing (isSource=true)
    static std::vector<std::pair<EdgeId, bool>> sortSnapPointsByOtherNode(
        NodeId nodeId,
        NodeEdge edge,
        const std::unordered_map<EdgeId, EdgeLayout>& edgeLayouts,
        const std::unordered_map<NodeId, NodeLayout>& nodeLayouts);

private:
    /// Get sort key for a node based on edge orientation
    /// @param node Node to get key for
    /// @param edge Edge orientation (determines X vs Y sorting)
    /// @return Sort key (center X for horizontal edges, center Y for vertical edges)
    static float getSortKey(const NodeLayout& node, NodeEdge edge);
};


}  // namespace arborvia
