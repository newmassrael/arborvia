#include "SnapIndexManager.h"

#include <algorithm>
#include <sstream>

namespace arborvia {


// =============================================================================
// Index Conversion
// =============================================================================

int SnapIndexManager::unifiedToLocal(int unifiedIdx, int offset, int count) {
    if (count <= 0) {
        return 0;
    }

    int localIdx = unifiedIdx - offset;

    // Clamp to valid range
    if (localIdx < 0) {
        localIdx = 0;
    } else if (localIdx >= count) {
        localIdx = count - 1;
    }

    return localIdx;
}

int SnapIndexManager::localToUnified(int localIdx, int offset) {
    return localIdx + offset;
}

// =============================================================================
// Validation
// =============================================================================

SnapIndexValidation SnapIndexManager::validate(int index, int count, const char* context) {
    if (count <= 0) {
        return SnapIndexValidation::fail(
            SnapIndexError::InvalidCount,
            std::string(context) + ": count must be positive, got " + std::to_string(count),
            0
        );
    }

    if (index < 0) {
        return SnapIndexValidation::fail(
            SnapIndexError::InvalidIndex,
            std::string(context) + ": index must be non-negative, got " + std::to_string(index),
            0
        );
    }

    if (index >= count) {
        return SnapIndexValidation::fail(
            SnapIndexError::IndexOutOfRange,
            std::string(context) + ": index " + std::to_string(index) +
                " out of range [0, " + std::to_string(count) + ")",
            count - 1
        );
    }

    return SnapIndexValidation::ok(index);
}

int SnapIndexManager::clamp(int index, int count) {
    if (count <= 0) return 0;
    if (index < 0) return 0;
    if (index >= count) return count - 1;
    return index;
}

// =============================================================================
// Range Calculation
// =============================================================================

float SnapIndexManager::calculatePosition(int localIdx, int count, const SnapRange& range) {
    if (count <= 0) {
        return range.midpoint();
    }

    // Distribute evenly within range: position = start + (idx + 1) / (count + 1) * width
    float rangeWidth = range.width();
    float position = range.start +
        static_cast<float>(localIdx + 1) / static_cast<float>(count + 1) * rangeWidth;

    return position;
}

// =============================================================================
// Connection Analysis
// =============================================================================

std::map<std::pair<NodeId, NodeEdge>, NodeEdgeConnections>
SnapIndexManager::buildConnectionMap(
    const std::unordered_map<EdgeId, EdgeLayout>& edgeLayouts) {

    std::map<std::pair<NodeId, NodeEdge>, NodeEdgeConnections> result;

    for (const auto& [edgeId, layout] : edgeLayouts) {
        // Outgoing from source node
        result[{layout.from, layout.sourceEdge}].outgoing.push_back(edgeId);
        // Incoming to target node
        result[{layout.to, layout.targetEdge}].incoming.push_back(edgeId);
    }

    return result;
}

NodeEdgeConnections SnapIndexManager::getConnections(
    const std::unordered_map<EdgeId, EdgeLayout>& edgeLayouts,
    NodeId nodeId,
    NodeEdge nodeEdge) {

    NodeEdgeConnections result;

    for (const auto& [edgeId, layout] : edgeLayouts) {
        // Outgoing from this node on this edge
        if (layout.from == nodeId && layout.sourceEdge == nodeEdge) {
            result.outgoing.push_back(edgeId);
        }
        // Incoming to this node on this edge
        if (layout.to == nodeId && layout.targetEdge == nodeEdge) {
            result.incoming.push_back(edgeId);
        }
    }

    return result;
}

// =============================================================================
// Debug Helpers
// =============================================================================

std::string SnapIndexManager::formatIndexInfo(int unifiedIdx, int localIdx, int offset, int count) {
    std::ostringstream oss;
    oss << "unified=" << unifiedIdx
        << " local=" << localIdx
        << " offset=" << offset
        << " count=" << count;
    return oss.str();
}

// =============================================================================
// Snap Point Sorting
// =============================================================================

float SnapIndexManager::getSortKey(const NodeLayout& node, NodeEdge edge) {
    // For vertical edges (Left/Right): sort by Y coordinate
    // For horizontal edges (Top/Bottom): sort by X coordinate
    if (edge == NodeEdge::Left || edge == NodeEdge::Right) {
        return node.center().y;
    }
    return node.center().x;
}

std::vector<std::pair<EdgeId, bool>> SnapIndexManager::sortSnapPointsByOtherNode(
    NodeId nodeId,
    NodeEdge edge,
    const std::unordered_map<EdgeId, EdgeLayout>& edgeLayouts,
    const std::unordered_map<NodeId, NodeLayout>& nodeLayouts) {

    // Get all connections on this node edge
    auto connections = getConnections(edgeLayouts, nodeId, edge);

    // Build sortable vectors with sort keys
    std::vector<std::pair<EdgeId, float>> incomingSorted;
    std::vector<std::pair<EdgeId, float>> outgoingSorted;

    // Process incoming edges (target = nodeId, isSource = false)
    for (EdgeId edgeId : connections.incoming) {
        auto it = edgeLayouts.find(edgeId);
        if (it == edgeLayouts.end()) continue;

        NodeId otherNodeId = it->second.from;  // Source node
        auto nodeIt = nodeLayouts.find(otherNodeId);
        if (nodeIt == nodeLayouts.end()) continue;

        float sortKey = getSortKey(nodeIt->second, edge);
        incomingSorted.emplace_back(edgeId, sortKey);
    }

    // Process outgoing edges (source = nodeId, isSource = true)
    for (EdgeId edgeId : connections.outgoing) {
        auto it = edgeLayouts.find(edgeId);
        if (it == edgeLayouts.end()) continue;

        NodeId otherNodeId = it->second.to;  // Target node
        auto nodeIt = nodeLayouts.find(otherNodeId);
        if (nodeIt == nodeLayouts.end()) continue;

        float sortKey = getSortKey(nodeIt->second, edge);
        outgoingSorted.emplace_back(edgeId, sortKey);
    }

    // Determine sort direction
    // For horizontal edges (Top/Bottom): incoming sorted in reverse (right to left)
    // For vertical edges (Left/Right): incoming sorted ascending (top to bottom)
    bool reverseIncoming = (edge == NodeEdge::Top || edge == NodeEdge::Bottom);

    // Sort incoming
    if (reverseIncoming) {
        std::sort(incomingSorted.begin(), incomingSorted.end(),
                  [](const auto& a, const auto& b) { return a.second > b.second; });
    } else {
        std::sort(incomingSorted.begin(), incomingSorted.end(),
                  [](const auto& a, const auto& b) { return a.second < b.second; });
    }

    // Sort outgoing (always ascending)
    std::sort(outgoingSorted.begin(), outgoingSorted.end(),
              [](const auto& a, const auto& b) { return a.second < b.second; });

    // Build result: incoming first (isSource=false), then outgoing (isSource=true)
    std::vector<std::pair<EdgeId, bool>> result;
    result.reserve(incomingSorted.size() + outgoingSorted.size());

    for (const auto& [id, _] : incomingSorted) {
        result.emplace_back(id, false);  // incoming = target side, isSource=false
    }
    for (const auto& [id, _] : outgoingSorted) {
        result.emplace_back(id, true);  // outgoing = source side, isSource=true
    }

    return result;
}


// =============================================================================
// Grid-Aligned Position Calculation
// =============================================================================

std::vector<float> SnapIndexManager::calculateGridAlignedPositions(
    float edgeLength, int count, float gridSize, float minSpacing) {

    std::vector<float> positions;
    if (count <= 0 || gridSize <= 0.0f) {
        return positions;
    }

    positions.reserve(static_cast<size_t>(count));

    // Use gridSize as minimum spacing if not specified
    float spacing = minSpacing > 0.0f ? minSpacing : gridSize;

    // Calculate required length for count snap points with minimum spacing
    float requiredLength = calculateRequiredLength(count, spacing);

    // Single snap point: center of edge
    if (count == 1) {
        float center = edgeLength / 2.0f;
        // Snap to grid
        positions.push_back(std::round(center / gridSize) * gridSize);
        return positions;
    }

    // Multiple snap points: distribute evenly with minimum spacing
    float startOffset;
    float actualSpacing;

    if (edgeLength >= requiredLength) {
        // Enough space: distribute evenly within edge bounds
        // Leave margin at edges equal to half the spacing between points
        float totalSpan = static_cast<float>(count - 1) * spacing;
        startOffset = (edgeLength - totalSpan) / 2.0f;
        actualSpacing = spacing;
    } else {
        // Not enough space: extend beyond edge bounds (centered)
        // Snap points will extend past node edge but maintain minimum spacing
        float totalSpan = static_cast<float>(count - 1) * spacing;
        startOffset = (edgeLength - totalSpan) / 2.0f;  // Will be negative
        actualSpacing = spacing;
    }

    // Generate positions
    for (int i = 0; i < count; ++i) {
        float pos = startOffset + static_cast<float>(i) * actualSpacing;
        // Snap to grid
        pos = std::round(pos / gridSize) * gridSize;
        positions.push_back(pos);
    }

    return positions;
}

float SnapIndexManager::calculateRequiredLength(int count, float gridSize) {
    if (count <= 1 || gridSize <= 0.0f) {
        return gridSize;  // Single point needs minimal space
    }

    // For N snap points with minimum gridSize spacing:
    // Need (N-1) gaps of gridSize each, plus margin at each end
    // Total = (N-1) * gridSize + gridSize = N * gridSize
    return static_cast<float>(count) * gridSize;
}


}  // namespace arborvia
