#include "layout/constraints/DirectionAwareMarginConstraint.h"
#include "arborvia/core/GeometryUtils.h"

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

DirectionAwareMarginConstraint::DirectionAwareMarginConstraint(float baseGridDistance)
    : baseGridDistance_(baseGridDistance) {
}

ConstraintResult DirectionAwareMarginConstraint::check(const ConstraintContext& ctx) const {
    auto nodeIt = ctx.nodeLayouts.find(ctx.nodeId);
    if (nodeIt == ctx.nodeLayouts.end()) {
        return ConstraintResult::ok();
    }

    const auto& nodeSize = nodeIt->second.size;
    auto zones = getBlockedRegions(ctx);

    // Check if proposed position overlaps with any forbidden zone
    for (const auto& zone : zones) {
        bool overlapsX = ctx.newPosition.x < zone.x + zone.width &&
                         ctx.newPosition.x + nodeSize.width > zone.x;
        bool overlapsY = ctx.newPosition.y < zone.y + zone.height &&
                         ctx.newPosition.y + nodeSize.height > zone.y;
        if (overlapsX && overlapsY) {
            return ConstraintResult::fail("Too close to another node");
        }
    }

    return ConstraintResult::ok();
}

std::vector<Rect> DirectionAwareMarginConstraint::getBlockedRegions(const ConstraintContext& ctx) const {
    std::vector<Rect> zones;

    auto nodeIt = ctx.nodeLayouts.find(ctx.nodeId);
    if (nodeIt == ctx.nodeLayouts.end()) {
        return zones;
    }

    auto draggedCounts = countEdgesPerDirection(ctx.nodeId, ctx.edgeLayouts);

    // margin(side) = base + max(0, snapPoints - threshold)
    const float baseMargin = baseGridDistance_ * ctx.gridSize;
    const int threshold = static_cast<int>(baseGridDistance_) - 1;

    auto calcMargin = [baseMargin, threshold, &ctx](int snapPoints) {
        return baseMargin + std::max(0, snapPoints - threshold) * ctx.gridSize;
    };

    for (const auto& [otherId, otherNode] : ctx.nodeLayouts) {
        if (otherId == ctx.nodeId) {
            continue;
        }

        auto otherCounts = countEdgesPerDirection(otherId, ctx.edgeLayouts);

        float marginTop = calcMargin(std::max(draggedCounts.bottom, otherCounts.top));
        float marginBottom = calcMargin(std::max(draggedCounts.top, otherCounts.bottom));
        float marginLeft = calcMargin(std::max(draggedCounts.right, otherCounts.left));
        float marginRight = calcMargin(std::max(draggedCounts.left, otherCounts.right));

        Rect zone{
            otherNode.position.x - marginLeft,
            otherNode.position.y - marginTop,
            otherNode.size.width + marginLeft + marginRight,
            otherNode.size.height + marginTop + marginBottom
        };

        zones.push_back(zone);
    }

    return zones;
}

}  // namespace arborvia
