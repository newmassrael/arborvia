#include "SnapIndexManager.h"
#include "GridSnapCalculator.h"

#include <algorithm>
#include <sstream>
#include <cmath>
#include <limits>

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
// Optimal Snap Selection (Manhattan Distance)
// =============================================================================

std::vector<SnapIndexManager::SnapAssignment> SnapIndexManager::selectOptimalCandidates(
    const std::vector<std::pair<EdgeId, bool>>& connections,
    const NodeLayout& node,
    NodeEdge nodeEdge,
    const std::unordered_map<EdgeId, EdgeLayout>& edgeLayouts,
    const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
    float gridSize) {

    std::vector<SnapAssignment> result;
    if (connections.empty()) return result;

    result.reserve(connections.size());

    // Get candidate count and positions
    int candidateCount = GridSnapCalculator::getCandidateCount(node, nodeEdge, gridSize);
    if (candidateCount <= 0) {
        // Fallback: all at center
        Point center = node.center();
        for (const auto& [edgeId, isSource] : connections) {
            result.push_back({edgeId, isSource, 0, center});
        }
        return result;
    }

    // Pre-calculate all candidate positions
    std::vector<Point> candidatePositions;
    candidatePositions.reserve(candidateCount);
    for (int i = 0; i < candidateCount; ++i) {
        candidatePositions.push_back(
            GridSnapCalculator::getPositionFromCandidateIndex(node, nodeEdge, i, gridSize));
    }

    // Calculate target point for each connection (the "other" endpoint)
    struct ConnectionInfo {
        EdgeId edgeId;
        bool isSource;
        Point targetPoint;  // The other endpoint we want to get closer to
        size_t originalIndex;  // For maintaining input order
    };
    std::vector<ConnectionInfo> connectionInfos;
    connectionInfos.reserve(connections.size());

    for (size_t idx = 0; idx < connections.size(); ++idx) {
        const auto& [edgeId, isSource] = connections[idx];
        Point targetPoint = node.center();  // Default to node center

        auto layoutIt = edgeLayouts.find(edgeId);
        if (layoutIt != edgeLayouts.end()) {
            const EdgeLayout& layout = layoutIt->second;

            if (isSource) {
                // Source side: target is the target node
                auto nodeIt = nodeLayouts.find(layout.to);
                if (nodeIt != nodeLayouts.end()) {
                    targetPoint = nodeIt->second.center();
                } else {
                    targetPoint = layout.targetPoint;
                }
            } else {
                // Target side: target is the source node
                auto nodeIt = nodeLayouts.find(layout.from);
                if (nodeIt != nodeLayouts.end()) {
                    targetPoint = nodeIt->second.center();
                } else {
                    targetPoint = layout.sourcePoint;
                }
            }
        }

        // Always add to connectionInfos - never skip
        connectionInfos.push_back({edgeId, isSource, targetPoint, idx});
    }

    // NOTE: Do NOT re-sort here. Input order is already optimized by sortSnapPointsByOtherNode
    // (incoming first in reverse/ascending order, then outgoing in ascending order)
    // We optimize within exclusive ranges to minimize Manhattan distance while preventing duplicates.

    size_t totalConnections = connectionInfos.size();

    // Case 1: More connections than candidates (narrow node) - use sequential allocation
    if (totalConnections > static_cast<size_t>(candidateCount)) {
        std::vector<int> selectedIndices = GridSnapCalculator::selectCandidateIndices(
            candidateCount, static_cast<int>(totalConnections));

        for (size_t connIdx = 0; connIdx < connectionInfos.size(); ++connIdx) {
            const auto& info = connectionInfos[connIdx];
            int candidateIndex = (connIdx < selectedIndices.size()) 
                ? selectedIndices[connIdx] : 0;

            result.push_back({
                info.edgeId,
                info.isSource,
                candidateIndex,
                candidatePositions[candidateIndex]
            });
        }
        return result;
    }

    // Case 2: Enough candidates - use range-based Manhattan optimization
    // Each connection gets an exclusive candidate range, then picks the closest within that range
    for (size_t connIdx = 0; connIdx < connectionInfos.size(); ++connIdx) {
        const auto& info = connectionInfos[connIdx];

        // Calculate exclusive range for this connection
        int rangeStart = static_cast<int>(connIdx * candidateCount / totalConnections);
        int rangeEnd = static_cast<int>((connIdx + 1) * candidateCount / totalConnections) - 1;
        // Ensure at least one candidate in range
        if (rangeEnd < rangeStart) rangeEnd = rangeStart;

        // Find best candidate within exclusive range by Manhattan distance
        int bestCandidate = rangeStart;
        float bestDistance = std::numeric_limits<float>::max();

        for (int i = rangeStart; i <= rangeEnd; ++i) {
            float dist = std::abs(candidatePositions[i].x - info.targetPoint.x) +
                        std::abs(candidatePositions[i].y - info.targetPoint.y);

            if (dist < bestDistance) {
                bestDistance = dist;
                bestCandidate = i;
            }
        }

        result.push_back({
            info.edgeId,
            info.isSource,
            bestCandidate,
            candidatePositions[bestCandidate]
        });
    }

    return result;
}

}  // namespace arborvia
