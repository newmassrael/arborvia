#include "SnapIndexManager.h"

#include <algorithm>
#include <sstream>

namespace arborvia {
namespace algorithms {

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

}  // namespace algorithms
}  // namespace arborvia
