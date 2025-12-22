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
                    layout.sourceSnapIndex = constants::SNAP_INDEX_POINT_NODE_CENTER;
                    layout.sourcePoint = center;
                    LOG_DEBUG("[SNAP-TRACE] SnapDistributor edge={} SOURCE PointNode={} center=({},{})",
                              edgeId, nodeId, center.x, center.y);
                } else {
                    layout.targetSnapIndex = constants::SNAP_INDEX_POINT_NODE_CENTER;
                    layout.targetPoint = center;
                    LOG_DEBUG("[SNAP-TRACE] SnapDistributor edge={} TARGET PointNode={} center=({},{})",
                              edgeId, nodeId, center.x, center.y);
                }
            }
            continue;
        }

        // Normal nodes: Manhattan distance optimization
        // Select snap points that minimize distance to target nodes
        {
            auto assignments = SnapIndexManager::selectOptimalCandidates(
                connections, node, nodeEdge, result.edgeLayouts, nodeLayouts, gridSizeToUse);

            for (const auto& assignment : assignments) {
                EdgeLayout& layout = result.edgeLayouts[assignment.edgeId];

                // SSOT: Update snapIndex and Point together
                if (assignment.isSource) {
                    layout.sourceSnapIndex = assignment.candidateIndex;
                    layout.sourcePoint = assignment.snapPosition;
                    LOG_DEBUG("[SNAP-TRACE] SnapDistributor edge={} SOURCE nodeId={} Manhattan snapIdx={} pos=({},{})",
                              assignment.edgeId, nodeId, assignment.candidateIndex,
                              assignment.snapPosition.x, assignment.snapPosition.y);
                } else {
                    layout.targetSnapIndex = assignment.candidateIndex;
                    layout.targetPoint = assignment.snapPosition;
                    LOG_DEBUG("[SNAP-TRACE] SnapDistributor edge={} TARGET nodeId={} Manhattan snapIdx={} pos=({},{})",
                              assignment.edgeId, nodeId, assignment.candidateIndex,
                              assignment.snapPosition.x, assignment.snapPosition.y);
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
