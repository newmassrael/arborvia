#include "arborvia/layout/MinDistanceConstraint.h"

#include <algorithm>
#include <cmath>

namespace arborvia {

MinDistanceConstraint::MinDistanceConstraint(float minGridDistance)
    : minGridDistance_(minGridDistance) {}

ConstraintResult MinDistanceConstraint::check(const ConstraintContext& ctx) const {
    auto nodeIt = ctx.nodeLayouts.find(ctx.nodeId);
    if (nodeIt == ctx.nodeLayouts.end()) {
        return ConstraintResult::ok();  // Node not found, allow move
    }

    float minDist = minGridDistance_ * ctx.gridSize;
    const Size& nodeSize = nodeIt->second.size;

    for (const auto& [otherId, other] : ctx.nodeLayouts) {
        if (otherId == ctx.nodeId) continue;

        // Calculate AABB distance (gap between bounding boxes)
        float dx = std::max(0.0f, std::max(other.position.x - (ctx.newPosition.x + nodeSize.width),
                                           ctx.newPosition.x - (other.position.x + other.size.width)));
        float dy = std::max(0.0f, std::max(other.position.y - (ctx.newPosition.y + nodeSize.height),
                                           ctx.newPosition.y - (other.position.y + other.size.height)));

        if (dx < minDist && dy < minDist) {
            return ConstraintResult::fail("Too close to another node");
        }
    }

    return ConstraintResult::ok();
}

std::vector<Rect> MinDistanceConstraint::getBlockedRegions(const ConstraintContext& ctx) const {
    std::vector<Rect> regions;

    auto nodeIt = ctx.nodeLayouts.find(ctx.nodeId);
    if (nodeIt == ctx.nodeLayouts.end()) {
        return regions;
    }

    float margin = minGridDistance_ * ctx.gridSize;

    for (const auto& [otherId, other] : ctx.nodeLayouts) {
        if (otherId == ctx.nodeId) continue;

        // Create a margin zone around the node
        // This represents the area where the dragged node cannot go
        Rect blockedRegion{
            other.position.x - margin,
            other.position.y - margin,
            other.size.width + 2 * margin,
            other.size.height + 2 * margin
        };

        regions.push_back(blockedRegion);
    }

    return regions;
}

}  // namespace arborvia
