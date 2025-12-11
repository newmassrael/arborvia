#include "layout/constraints/DirectionAwareMarginConstraint.h"
#include "arborvia/core/GeometryUtils.h"
#include "arborvia/common/Logger.h"

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
        LOG_DEBUG("[DirectionAwareMargin::check] nodeId={} NOT FOUND in nodeLayouts", ctx.nodeId);
        return ConstraintResult::ok();
    }

    const auto& nodeSize = nodeIt->second.size;
    auto zones = getBlockedRegions(ctx);

    LOG_DEBUG("[DirectionAwareMargin::check] nodeId={} pos=({},{}) size=({},{}) zones={}",
              ctx.nodeId, ctx.newPosition.x, ctx.newPosition.y, nodeSize.width, nodeSize.height, zones.size());

    // Check if proposed position overlaps with any forbidden zone
    for (size_t i = 0; i < zones.size(); ++i) {
        const auto& zone = zones[i];

        // Node ranges
        float nodeMinX = ctx.newPosition.x;
        float nodeMaxX = ctx.newPosition.x + nodeSize.width;
        float nodeMinY = ctx.newPosition.y;
        float nodeMaxY = ctx.newPosition.y + nodeSize.height;

        // Zone ranges
        float zoneMinX = zone.x;
        float zoneMaxX = zone.x + zone.width;
        float zoneMinY = zone.y;
        float zoneMaxY = zone.y + zone.height;

        bool overlapsX = nodeMinX < zoneMaxX && nodeMaxX > zoneMinX;
        bool overlapsY = nodeMinY < zoneMaxY && nodeMaxY > zoneMinY;

        LOG_DEBUG("[DirectionAwareMargin::check] zone[{}]: node=[{},{})x[{},{}) zone=[{},{})x[{},{}) overlapsX={} overlapsY={}",
                  i, nodeMinX, nodeMaxX, nodeMinY, nodeMaxY,
                  zoneMinX, zoneMaxX, zoneMinY, zoneMaxY,
                  overlapsX, overlapsY);

        // Detect boundary cases (node exactly at zone edge - uses strict inequality so no overlap detected)
        bool atBoundaryX = (nodeMinX == zoneMaxX || nodeMaxX == zoneMinX);
        bool atBoundaryY = (nodeMinY == zoneMaxY || nodeMaxY == zoneMinY);
        if (atBoundaryX || atBoundaryY) {
            LOG_DEBUG("[DirectionAwareMargin::check] BOUNDARY: zone[{}] node at exact boundary (atBoundaryX={} atBoundaryY={})",
                      i, atBoundaryX, atBoundaryY);
        }

        if (overlapsX && overlapsY) {
            LOG_DEBUG("[DirectionAwareMargin::check] VIOLATION: node overlaps zone[{}]", i);
            return ConstraintResult::fail("Too close to another node");
        }
    }

    return ConstraintResult::ok();
}

std::vector<Rect> DirectionAwareMarginConstraint::getBlockedRegions(const ConstraintContext& ctx) const {
    std::vector<Rect> zones;

    LOG_DEBUG("[DirectionAwareMargin::getBlockedRegions] nodeId={} nodeLayouts.size={} edgeLayouts.size={}",
              ctx.nodeId, ctx.nodeLayouts.size(), ctx.edgeLayouts.size());

    auto nodeIt = ctx.nodeLayouts.find(ctx.nodeId);
    if (nodeIt == ctx.nodeLayouts.end()) {
        LOG_DEBUG("[DirectionAwareMargin::getBlockedRegions] nodeId={} NOT FOUND in nodeLayouts!", ctx.nodeId);
        return zones;
    }

    LOG_DEBUG("[DirectionAwareMargin::getBlockedRegions] draggedNode id={} pos=({},{}) size=({},{})",
              ctx.nodeId, nodeIt->second.position.x, nodeIt->second.position.y,
              nodeIt->second.size.width, nodeIt->second.size.height);

    auto draggedCounts = countEdgesPerDirection(ctx.nodeId, ctx.edgeLayouts);

    // margin(side) = base + max(0, snapPoints - threshold)
    const float baseMargin = baseGridDistance_ * ctx.gridSize;
    const int threshold = static_cast<int>(baseGridDistance_) - 1;

    LOG_DEBUG("[DirectionAwareMargin::getBlockedRegions] baseGridDistance_={} gridSize={} baseMargin={} threshold={}",
              baseGridDistance_, ctx.gridSize, baseMargin, threshold);

    auto calcMargin = [baseMargin, threshold, &ctx](int snapPoints) {
        return baseMargin + std::max(0, snapPoints - threshold) * ctx.gridSize;
    };

    for (const auto& [otherId, otherNode] : ctx.nodeLayouts) {
        if (otherId == ctx.nodeId) {
            LOG_DEBUG("[DirectionAwareMargin::getBlockedRegions] SKIPPING self nodeId={}", otherId);
            continue;
        }

        LOG_DEBUG("[DirectionAwareMargin::getBlockedRegions] otherNode id={} pos=({},{}) size=({},{})",
                  otherId, otherNode.position.x, otherNode.position.y,
                  otherNode.size.width, otherNode.size.height);

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

        LOG_DEBUG("[DirectionAwareMargin::getBlockedRegions] zone for node {} = ({},{},{},{}) margins=({},{},{},{})",
                  otherId, zone.x, zone.y, zone.width, zone.height,
                  marginLeft, marginTop, marginRight, marginBottom);

        zones.push_back(zone);
    }

    LOG_DEBUG("[DirectionAwareMargin::getBlockedRegions] total zones={}", zones.size());
    return zones;
}

}  // namespace arborvia
