#include "SelfLoopPathCalculator.h"
#include "../sugiyama/routing/SelfLoopRouter.h"

#include <cmath>

namespace arborvia {

bool SelfLoopPathCalculator::canHandle(const EdgeLayout& layout) const {
    return layout.from == layout.to;
}

EdgePathResult SelfLoopPathCalculator::calculatePath(
    const EdgeLayout& layout,
    const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
    const PathConfig& config) {

    EdgePathResult result;

    // Must be a self-loop
    if (layout.from != layout.to) {
        result.failureReason = "Not a self-loop edge";
        return result;
    }

    // Find the node
    auto nodeIt = nodeLayouts.find(layout.from);
    if (nodeIt == nodeLayouts.end()) {
        result.failureReason = "Node not found";
        return result;
    }
    const NodeLayout& node = nodeIt->second;

    // Validate edge combination (must be adjacent edges)
    if (!SelfLoopRouter::isValidSelfLoopCombination(layout.sourceEdge, layout.targetEdge)) {
        result.failureReason = "Invalid self-loop edge combination (not adjacent)";
        return result;
    }

    // Calculate extension points perpendicular to each edge (in grid cells)
    float offset = config.extensionOffset();
    Point srcExt = calculateExtensionPoint(layout.sourcePoint, layout.sourceEdge, offset);
    Point tgtExt = calculateExtensionPoint(layout.targetPoint, layout.targetEdge, offset);

    // Snap to grid if enabled
    if (config.snapToGrid && config.gridSize > 0) {
        srcExt.x = std::round(srcExt.x / config.gridSize) * config.gridSize;
        srcExt.y = std::round(srcExt.y / config.gridSize) * config.gridSize;
        tgtExt.x = std::round(tgtExt.x / config.gridSize) * config.gridSize;
        tgtExt.y = std::round(tgtExt.y / config.gridSize) * config.gridSize;
    }

    // Check if corner point is needed (srcExt and tgtExt not aligned)
    // After grid snap, coordinates are exact multiples - no epsilon needed
    bool needsCorner = (srcExt.x != tgtExt.x && srcExt.y != tgtExt.y);

    if (needsCorner) {
        Point corner = selectCornerOutsideNode(srcExt, tgtExt, node);
        result.bendPoints.push_back(BendPoint{srcExt});
        result.bendPoints.push_back(BendPoint{corner});
        result.bendPoints.push_back(BendPoint{tgtExt});
    } else {
        result.bendPoints.push_back(BendPoint{srcExt});
        result.bendPoints.push_back(BendPoint{tgtExt});
    }

    result.success = true;
    return result;
}

Point SelfLoopPathCalculator::calculateExtensionPoint(
    const Point& snapPoint,
    NodeEdge edge,
    float offset) {

    Point ext = snapPoint;
    switch (edge) {
        case NodeEdge::Top:
            ext.y -= offset;
            break;
        case NodeEdge::Bottom:
            ext.y += offset;
            break;
        case NodeEdge::Left:
            ext.x -= offset;
            break;
        case NodeEdge::Right:
            ext.x += offset;
            break;
    }
    return ext;
}

Point SelfLoopPathCalculator::selectCornerOutsideNode(
    const Point& srcExt,
    const Point& tgtExt,
    const NodeLayout& node) {

    // Two possible corners
    Point cornerA = {srcExt.x, tgtExt.y};
    Point cornerB = {tgtExt.x, srcExt.y};

    // Node bounds
    float nodeLeft = node.position.x;
    float nodeRight = node.position.x + node.size.width;
    float nodeTop = node.position.y;
    float nodeBottom = node.position.y + node.size.height;

    auto isInside = [&](const Point& p) {
        return p.x >= nodeLeft && p.x <= nodeRight &&
               p.y >= nodeTop && p.y <= nodeBottom;
    };

    bool cornerAInside = isInside(cornerA);
    bool cornerBInside = isInside(cornerB);

    // Prefer corner that is outside the node
    if (!cornerAInside && cornerBInside) {
        return cornerA;
    }
    if (cornerAInside && !cornerBInside) {
        return cornerB;
    }
    
    // Both outside or both inside: pick the one farther from node center
    Point nodeCenter = {
        node.position.x + node.size.width * 0.5f,
        node.position.y + node.size.height * 0.5f
    };
    
    float distA = std::abs(cornerA.x - nodeCenter.x) + std::abs(cornerA.y - nodeCenter.y);
    float distB = std::abs(cornerB.x - nodeCenter.x) + std::abs(cornerB.y - nodeCenter.y);
    
    return (distA >= distB) ? cornerA : cornerB;
}

}  // namespace arborvia
