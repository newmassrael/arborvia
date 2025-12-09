#include "SnapDistributor.h"
#include "GridSnapCalculator.h"
#include "SnapIndexManager.h"
#include "../sugiyama/routing/SelfLoopRouter.h"
#include "../sugiyama/routing/PathCleanup.h"
#include "arborvia/layout/util/LayoutUtils.h"
#include <map>

namespace arborvia {

namespace constants {
    constexpr float PATHFINDING_GRID_SIZE = 20.0f;
}

SnapDistributor::SnapDistributor(RecalcBendPointsFunc recalcFunc)
    : recalcFunc_(std::move(recalcFunc)) {
}

void SnapDistributor::distribute(
    EdgeRouting::Result& result,
    const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
    float gridSize,
    bool sortSnapPoints) {

    float effectiveGridSize = GridSnapCalculator::getEffectiveGridSize(gridSize);

    // Unified mode: all connections on same edge distributed together
    // Key: (nodeId, nodeEdge) -> list of (edgeId, isSource)
    std::map<std::pair<NodeId, NodeEdge>, std::vector<std::pair<EdgeId, bool>>> allConnections;

    for (auto& [edgeId, layout] : result.edgeLayouts) {
        allConnections[{layout.from, layout.sourceEdge}].push_back({edgeId, true});
        allConnections[{layout.to, layout.targetEdge}].push_back({edgeId, false});
    }

    for (auto& [key, connections] : allConnections) {
        auto [nodeId, nodeEdge] = key;

        // Sort snap points by other node position to minimize edge crossings
        if (sortSnapPoints && connections.size() > 1) {
            auto sortedPairs = SnapIndexManager::sortSnapPointsByOtherNode(
                nodeId, nodeEdge, result.edgeLayouts, nodeLayouts);

            // Replace original connections if sorting succeeded
            if (sortedPairs.size() == connections.size()) {
                connections = std::move(sortedPairs);
            }
        }

        // Move self-loop endpoints to corner positions
        SelfLoopRouter::applySelfLoopCornerPositioning(connections, result.edgeLayouts, nodeEdge);

        auto nodeIt = nodeLayouts.find(nodeId);
        if (nodeIt == nodeLayouts.end()) continue;

        const NodeLayout& node = nodeIt->second;
        int connectionCount = static_cast<int>(connections.size());

        // Use grid-based calculation for all snap point positions
        for (int i = 0; i < connectionCount; ++i) {
            auto [edgeId, isSource] = connections[i];
            EdgeLayout& layout = result.edgeLayouts[edgeId];

            // Calculate snap position and store the candidate index
            int candidateIndex = 0;
            Point snapPoint = GridSnapCalculator::calculateSnapPosition(
                node, nodeEdge, i, connectionCount, effectiveGridSize, &candidateIndex);

            if (isSource) {
                layout.sourcePoint = snapPoint;
                layout.sourceSnapIndex = candidateIndex;
            } else {
                layout.targetPoint = snapPoint;
                layout.targetSnapIndex = candidateIndex;
            }
        }
    }

    // Recalculate bend points
    for (auto& [edgeId, layout] : result.edgeLayouts) {
        recalcFunc_(layout, nodeLayouts, effectiveGridSize);

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
        ensureFirstBendClearance(layout, effectiveGridSize);

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
