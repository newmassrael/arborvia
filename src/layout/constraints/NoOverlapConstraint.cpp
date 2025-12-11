#include "layout/constraints/NoOverlapConstraint.h"

namespace arborvia {

NoOverlapConstraint::NoOverlapConstraint(float margin)
    : margin_(margin) {
}

ConstraintResult NoOverlapConstraint::check(const ConstraintContext& ctx) const {
    auto nodeIt = ctx.nodeLayouts.find(ctx.nodeId);
    if (nodeIt == ctx.nodeLayouts.end()) {
        return ConstraintResult::ok();
    }

    const auto& nodeSize = nodeIt->second.size;
    float x1 = ctx.newPosition.x - margin_;
    float y1 = ctx.newPosition.y - margin_;
    float x2 = ctx.newPosition.x + nodeSize.width + margin_;
    float y2 = ctx.newPosition.y + nodeSize.height + margin_;

    for (const auto& [otherId, otherNode] : ctx.nodeLayouts) {
        if (otherId == ctx.nodeId) continue;

        float ox1 = otherNode.position.x;
        float oy1 = otherNode.position.y;
        float ox2 = ox1 + otherNode.size.width;
        float oy2 = oy1 + otherNode.size.height;

        // AABB intersection check
        if (x1 < ox2 && x2 > ox1 && y1 < oy2 && y2 > oy1) {
            return ConstraintResult::fail("Node would overlap with another node");
        }
    }

    return ConstraintResult::ok();
}

std::vector<Rect> NoOverlapConstraint::getBlockedRegions(const ConstraintContext& ctx) const {
    std::vector<Rect> regions;

    auto nodeIt = ctx.nodeLayouts.find(ctx.nodeId);
    if (nodeIt == ctx.nodeLayouts.end()) {
        return regions;
    }

    const auto& nodeSize = nodeIt->second.size;

    for (const auto& [otherId, otherNode] : ctx.nodeLayouts) {
        if (otherId == ctx.nodeId) continue;

        // Minkowski sum: expand other node by dragged node's size + margin
        Rect blockedRegion{
            otherNode.position.x - nodeSize.width - margin_,
            otherNode.position.y - nodeSize.height - margin_,
            otherNode.size.width + nodeSize.width + 2 * margin_,
            otherNode.size.height + nodeSize.height + 2 * margin_
        };

        regions.push_back(blockedRegion);
    }

    return regions;
}

}  // namespace arborvia
