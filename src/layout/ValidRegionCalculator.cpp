#include "arborvia/layout/ValidRegionCalculator.h"
#include "arborvia/core/GeometryUtils.h"  // for constants::MIN_NODE_GRID_DISTANCE

#include <algorithm>

namespace arborvia {

namespace {
    // Count edges connected to a node on each direction
    struct DirectionEdgeCounts {
        int top = 0;
        int bottom = 0;
        int left = 0;
        int right = 0;
    };

    DirectionEdgeCounts countEdgesPerDirection(
        NodeId nodeId,
        const std::unordered_map<EdgeId, EdgeLayout>& edgeLayouts) {

        DirectionEdgeCounts counts;
        for (const auto& [edgeId, layout] : edgeLayouts) {
            if (layout.from == nodeId) {
                switch (layout.sourceEdge) {
                    case NodeEdge::Top: counts.top++; break;
                    case NodeEdge::Bottom: counts.bottom++; break;
                    case NodeEdge::Left: counts.left++; break;
                    case NodeEdge::Right: counts.right++; break;
                }
            }
            if (layout.to == nodeId) {
                switch (layout.targetEdge) {
                    case NodeEdge::Top: counts.top++; break;
                    case NodeEdge::Bottom: counts.bottom++; break;
                    case NodeEdge::Left: counts.left++; break;
                    case NodeEdge::Right: counts.right++; break;
                }
            }
        }
        return counts;
    }
}

std::vector<ForbiddenZone> ValidRegionCalculator::calculate(
    NodeId draggedNode,
    const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
    const std::unordered_map<EdgeId, EdgeLayout>& edgeLayouts,
    float gridSize) {

    std::vector<ForbiddenZone> zones;

    // Verify dragged node exists
    if (nodeLayouts.find(draggedNode) == nodeLayouts.end()) {
        return zones;
    }
    auto draggedCounts = countEdgesPerDirection(draggedNode, edgeLayouts);

    // margin(side) = base + max(0, snapPoints - threshold)
    const float baseMargin = constants::MIN_NODE_GRID_DISTANCE * gridSize;
    const int threshold = static_cast<int>(constants::MIN_NODE_GRID_DISTANCE) - 1;

    auto calcMargin = [baseMargin, threshold, gridSize](int snapPoints) {
        return baseMargin + std::max(0, snapPoints - threshold) * gridSize;
    };

    for (const auto& [otherId, otherNode] : nodeLayouts) {
        if (otherId == draggedNode) {
            continue;
        }

        auto otherCounts = countEdgesPerDirection(otherId, edgeLayouts);

        float marginTop = calcMargin(std::max(draggedCounts.bottom, otherCounts.top));
        float marginBottom = calcMargin(std::max(draggedCounts.top, otherCounts.bottom));
        float marginLeft = calcMargin(std::max(draggedCounts.right, otherCounts.left));
        float marginRight = calcMargin(std::max(draggedCounts.left, otherCounts.right));

        ForbiddenZone zone;
        zone.blockedBy = otherId;
        // Option B: margin-only zone (no Minkowski expansion)
        // Rectangle overlap check in isValid() accounts for dragged node size
        zone.bounds = {
            otherNode.position.x - marginLeft,
            otherNode.position.y - marginTop,
            otherNode.size.width + marginLeft + marginRight,
            otherNode.size.height + marginTop + marginBottom
        };

        zones.push_back(zone);
    }

    return zones;
}

bool ValidRegionCalculator::isValid(
    const Point& position,
    const Size& nodeSize,
    const std::vector<ForbiddenZone>& zones) {

    // Option B: Rectangle-rectangle overlap check
    // Zone bounds represent margin area around blocker (no Minkowski expansion)
    // We check if dragged node rectangle overlaps with forbidden zone
    for (const auto& zone : zones) {
        // Rectangle overlap: both X and Y ranges must overlap
        bool overlapsX = position.x < zone.bounds.x + zone.bounds.width &&
                         position.x + nodeSize.width > zone.bounds.x;
        bool overlapsY = position.y < zone.bounds.y + zone.bounds.height &&
                         position.y + nodeSize.height > zone.bounds.y;
        if (overlapsX && overlapsY) {
            return false;
        }
    }
    return true;
}

}  // namespace arborvia
