#include "SnapDistributor.h"
#include "GridSnapCalculator.h"
#include "SnapIndexManager.h"
#include "../sugiyama/routing/SelfLoopRouter.h"
#include "../sugiyama/routing/PathCleanup.h"
#include "arborvia/layout/util/LayoutUtils.h"
#include "arborvia/core/GeometryUtils.h"
#include "arborvia/common/Logger.h"
#include <algorithm>
#include <map>

namespace arborvia {

SnapDistributor::SnapDistributor(RecalcBendPointsFunc recalcFunc)
    : recalcFunc_(std::move(recalcFunc)) {
}

void SnapDistributor::distribute(
    EdgeRouting::Result& result,
    const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
    float gridSize,
    bool sortSnapPoints) {

    float gridSizeToUse = constants::effectiveGridSize(gridSize);

    // Unified mode: all connections on same edge distributed together
    // Key: (nodeId, nodeEdge) -> list of (edgeId, isSource)
    std::map<std::pair<NodeId, NodeEdge>, std::vector<std::pair<EdgeId, bool>>> allConnections;

    for (auto& [edgeId, layout] : result.edgeLayouts) {
        allConnections[{layout.from, layout.sourceEdge}].push_back({edgeId, true});
        allConnections[{layout.to, layout.targetEdge}].push_back({edgeId, false});
    }

    for (auto& [key, connections] : allConnections) {
        auto [nodeId, nodeEdge] = key;

        // IMPORTANT: Sort by EdgeId first to ensure deterministic ordering
        // This prevents snap collision caused by non-deterministic unordered_map iteration
        std::sort(connections.begin(), connections.end(),
                  [](const auto& a, const auto& b) { return a.first < b.first; });

        // Re-sort by other node position to minimize edge crossings
        // This must happen before selectOptimalCandidates which preserves this order
        if (sortSnapPoints && connections.size() > 1) {
            auto sortedPairs = SnapIndexManager::sortSnapPointsByOtherNode(
                nodeId, nodeEdge, result.edgeLayouts, nodeLayouts);

            if (sortedPairs.size() == connections.size()) {
                connections = std::move(sortedPairs);
            }
        }

        // Move self-loop endpoints to corner positions
        SelfLoopRouter::applySelfLoopCornerPositioning(connections, result.edgeLayouts, nodeEdge);

        auto nodeIt = nodeLayouts.find(nodeId);
        if (nodeIt == nodeLayouts.end()) continue;

        const NodeLayout& node = nodeIt->second;

        // Point nodes: all edges connect at center, no distribution needed
        if (node.isPointNode()) {
            Point center = node.center();
            for (const auto& [edgeId, isSource] : connections) {
                EdgeLayout& layout = result.edgeLayouts[edgeId];
                if (isSource) {
                    layout.setSourceSnap(constants::SNAP_INDEX_POINT_NODE_CENTER, center);
                    LOG_DEBUG("[SNAP-TRACE] SnapDistributor edge={} SOURCE PointNode={} center=({},{})",
                              edgeId, nodeId, center.x, center.y);
                } else {
                    layout.setTargetSnap(constants::SNAP_INDEX_POINT_NODE_CENTER, center);
                    LOG_DEBUG("[SNAP-TRACE] SnapDistributor edge={} TARGET PointNode={} center=({},{})",
                              edgeId, nodeId, center.x, center.y);
                }
            }
            continue;
        }

        // Normal nodes: sequential distribution
        // Note: Manhattan optimization disabled due to complex interaction with other layout stages
        {
            int connectionCount = static_cast<int>(connections.size());

            for (int i = 0; i < connectionCount; ++i) {
                auto [edgeId, isSource] = connections[i];
                EdgeLayout& layout = result.edgeLayouts[edgeId];

                int candidateIndex = 0;
                Point snapPoint = GridSnapCalculator::calculateSnapPosition(
                    node, nodeEdge, i, connectionCount, gridSizeToUse, &candidateIndex);

                // SSOT: Use setter methods to ensure snapIndex and Point are synchronized
                if (isSource) {
                    layout.setSourceSnap(candidateIndex, snapPoint);
                    LOG_DEBUG("[SNAP-TRACE] SnapDistributor edge={} SOURCE nodeId={} edge={} idx={}/{} pos=({},{}) snapIdx={}",
                              edgeId, nodeId, static_cast<int>(nodeEdge), i, connectionCount, snapPoint.x, snapPoint.y, candidateIndex);
                } else {
                    layout.setTargetSnap(candidateIndex, snapPoint);
                    LOG_DEBUG("[SNAP-TRACE] SnapDistributor edge={} TARGET nodeId={} edge={} idx={}/{} pos=({},{}) snapIdx={}",
                              edgeId, nodeId, static_cast<int>(nodeEdge), i, connectionCount, snapPoint.x, snapPoint.y, candidateIndex);
                }
            }
        }
    }

    // Recalculate bend points
    for (auto& [edgeId, layout] : result.edgeLayouts) {
        recalcFunc_(layout, nodeLayouts, gridSizeToUse);

        // Grid mode: bend points are already orthogonal from quantized routing.
        // Remove spikes/duplicates for path cleanup.
        std::vector<Point> fullPath;
        fullPath.push_back(layout.sourcePoint);
        for (const auto& bp : layout.bendPoints) {
            fullPath.push_back(bp.position);
        }
        fullPath.push_back(layout.targetPoint);

        PathCleanup::removeSpikesAndDuplicates(fullPath);

        layout.bendPoints.clear();
        for (size_t i = 1; i + 1 < fullPath.size(); ++i) {
            layout.bendPoints.push_back({fullPath[i]});
        }

        // Ensure first bend has proper clearance from source
        ensureFirstBendClearance(layout, gridSizeToUse);

        layout.labelPosition = LayoutUtils::calculateEdgeLabelPosition(layout);
    }
}

void SnapDistributor::ensureFirstBendClearance(
    EdgeLayout& layout,
    float effectiveGridSize) {

    if (layout.bendPoints.empty()) return;

    Point& firstBend = layout.bendPoints[0].position;
    float minClearance = std::max(effectiveGridSize, constants::PATHFINDING_GRID_SIZE);

    switch (layout.sourceEdge) {
        case NodeEdge::Top:
            if (firstBend.y > layout.sourcePoint.y - minClearance) {
                float newY = layout.sourcePoint.y - minClearance;
                firstBend.y = newY;
                if (layout.bendPoints.size() >= 2) {
                    layout.bendPoints[1].position.y = newY;
                }
            }
            break;
        case NodeEdge::Bottom:
            if (firstBend.y < layout.sourcePoint.y + minClearance) {
                float newY = layout.sourcePoint.y + minClearance;
                firstBend.y = newY;
                if (layout.bendPoints.size() >= 2) {
                    layout.bendPoints[1].position.y = newY;
                }
            }
            break;
        case NodeEdge::Left:
            if (firstBend.x > layout.sourcePoint.x - minClearance) {
                float newX = layout.sourcePoint.x - minClearance;
                firstBend.x = newX;
                if (layout.bendPoints.size() >= 2) {
                    layout.bendPoints[1].position.x = newX;
                }
            }
            break;
        case NodeEdge::Right:
            if (firstBend.x < layout.sourcePoint.x + minClearance) {
                float newX = layout.sourcePoint.x + minClearance;
                firstBend.x = newX;
                if (layout.bendPoints.size() >= 2) {
                    layout.bendPoints[1].position.x = newX;
                }
            }
            break;
    }
}

} // namespace arborvia
