#include "SelfLoopPathCalculator.h"
#include "../sugiyama/routing/SelfLoopRouter.h"
#include "arborvia/layout/config/LayoutOptions.h"

#include <cmath>

namespace arborvia {

bool SelfLoopPathCalculator::canHandle(const EdgeLayout& layout) const {
    return layout.from == layout.to;
}

void SelfLoopPathCalculator::setContext(const PathCalculatorContext& ctx) {
    context_ = ctx;
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

    // Validate edge combination (must be adjacent edges)
    if (!SelfLoopRouter::isValidSelfLoopCombination(layout.sourceEdge, layout.targetEdge)) {
        result.failureReason = "Invalid self-loop edge combination (not adjacent)";
        return result;
    }

    // DELEGATION: If context has edgeLayouts, use SelfLoopRouter for consistent stacking
    // (options is optional - will use default with context.gridSize if null)
    if (context_.edgeLayouts) {
        return calculatePathWithRouter(layout, nodeLayouts);
    }

    // FALLBACK: Simple geometric L-shape (no loopIndex stacking)
    // Only used when setContext was not called (e.g., standalone usage)
    const NodeLayout& node = nodeIt->second;

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

    // Check if corner point is needed
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

EdgePathResult SelfLoopPathCalculator::calculatePathWithRouter(
    const EdgeLayout& layout,
    const std::unordered_map<NodeId, NodeLayout>& nodeLayouts) const {

    EdgePathResult result;

    // Node existence already validated in calculatePath()
    const NodeLayout& node = nodeLayouts.at(layout.from);

    // Calculate loopIndex for proper stacking of multiple self-loops
    int loopIndex = SelfLoopRouter::calculateLoopIndex(
        layout.id, layout.from, *context_.edgeLayouts);

    // Use provided options or create default with context's gridSize
    LayoutOptions options;
    if (context_.options) {
        options = *context_.options;
    } else {
        options.gridConfig.cellSize = context_.gridSize;
    }

    // Delegate to SelfLoopRouter (Single Source of Truth)
    EdgeLayout routedLayout = SelfLoopRouter::route(
        layout.id, layout.from, node, loopIndex, options);

    // Extract bend points and NEW snap state from routed layout
    // SelfLoopRouter calculates snap positions based on loopIndex for proper stacking,
    // so we must also return these new snap indices and points to maintain orthogonality
    // IMPORTANT: snapIndex and Point must always be updated together (SSOT)
    result.bendPoints = std::move(routedLayout.bendPoints);
    result.newSourceSnapIndex = routedLayout.sourceSnapIndex;
    result.newSourcePoint = routedLayout.sourcePoint;
    result.newTargetSnapIndex = routedLayout.targetSnapIndex;
    result.newTargetPoint = routedLayout.targetPoint;
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
