#include "PathCalculator.h"
#include "../../pathfinding/ObstacleMap.h"
#include "../../routing/UnifiedRetryChain.h"
#include "EdgeRoutingUtils.h"
#include "arborvia/layout/config/LayoutOptions.h"
#include "arborvia/core/GeometryUtils.h"
#include <cmath>
#include "arborvia/common/Logger.h"


namespace arborvia {

PathCalculator::PathCalculator(IPathFinder& pathFinder)
    : pathFinder_(pathFinder) {
}

PathCalculator::~PathCalculator() = default;

UnifiedRetryChain& PathCalculator::getRetryChain(float gridSize) {
    if (!retryChain_ || lastGridSize_ != gridSize) {
        // Create shared_ptr with null deleter (we don't own pathFinder_)
        auto pathFinderPtr = std::shared_ptr<IPathFinder>(&pathFinder_, [](IPathFinder*){});
        retryChain_ = std::make_unique<UnifiedRetryChain>(pathFinderPtr, gridSize);
        lastGridSize_ = gridSize;
    }
    return *retryChain_;
}

bool PathCalculator::hasFreshBendPoints(const EdgeLayout& layout, float gridSize) {
    return EdgeRoutingUtils::hasFreshBendPoints(layout, gridSize);
}

void PathCalculator::handleSelfLoop(
    EdgeLayout& layout,
    const NodeLayout& srcNode,
    float effectiveGridSize) {
    
    constexpr float BASE_OFFSET = 30.0f;
    
    // Generate orthogonal path for self-loop
    Point srcExt, tgtExt;
    
    // Calculate source extension point (perpendicular to source edge)
    switch (layout.sourceEdge) {
        case NodeEdge::Top:
            srcExt = {layout.sourcePoint.x, layout.sourcePoint.y - BASE_OFFSET};
            break;
        case NodeEdge::Bottom:
            srcExt = {layout.sourcePoint.x, layout.sourcePoint.y + BASE_OFFSET};
            break;
        case NodeEdge::Left:
            srcExt = {layout.sourcePoint.x - BASE_OFFSET, layout.sourcePoint.y};
            break;
        case NodeEdge::Right:
            srcExt = {layout.sourcePoint.x + BASE_OFFSET, layout.sourcePoint.y};
            break;
    }
    
    // Calculate target extension point (perpendicular to target edge)
    switch (layout.targetEdge) {
        case NodeEdge::Top:
            tgtExt = {layout.targetPoint.x, layout.targetPoint.y - BASE_OFFSET};
            break;
        case NodeEdge::Bottom:
            tgtExt = {layout.targetPoint.x, layout.targetPoint.y + BASE_OFFSET};
            break;
        case NodeEdge::Left:
            tgtExt = {layout.targetPoint.x - BASE_OFFSET, layout.targetPoint.y};
            break;
        case NodeEdge::Right:
            tgtExt = {layout.targetPoint.x + BASE_OFFSET, layout.targetPoint.y};
            break;
    }
    
    // Snap to grid
    srcExt.x = std::round(srcExt.x / effectiveGridSize) * effectiveGridSize;
    srcExt.y = std::round(srcExt.y / effectiveGridSize) * effectiveGridSize;
    tgtExt.x = std::round(tgtExt.x / effectiveGridSize) * effectiveGridSize;
    tgtExt.y = std::round(tgtExt.y / effectiveGridSize) * effectiveGridSize;
    
    layout.bendPoints.clear();
    
    // Check if we need a corner point
    constexpr float EPSILON = 1.0f;
    if (std::abs(srcExt.x - tgtExt.x) > EPSILON && std::abs(srcExt.y - tgtExt.y) > EPSILON) {
        // Need corner - pick the one outside the node
        Point cornerA = {srcExt.x, tgtExt.y};
        Point cornerB = {tgtExt.x, srcExt.y};
        
        float nodeLeft = srcNode.position.x;
        float nodeRight = srcNode.position.x + srcNode.size.width;
        float nodeTop = srcNode.position.y;
        float nodeBottom = srcNode.position.y + srcNode.size.height;
        
        bool cornerAInside = (cornerA.x >= nodeLeft && cornerA.x <= nodeRight &&
                              cornerA.y >= nodeTop && cornerA.y <= nodeBottom);
        
        Point corner = cornerAInside ? cornerB : cornerA;
        
        layout.bendPoints.push_back({srcExt});
        layout.bendPoints.push_back({corner});
        layout.bendPoints.push_back({tgtExt});
    } else {
        // srcExt and tgtExt are aligned
        layout.bendPoints.push_back({srcExt});
        layout.bendPoints.push_back({tgtExt});
    }
    
}

bool PathCalculator::tryAStarPathfinding(
    EdgeLayout& layout,
    const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
    float effectiveGridSize,
    std::unordered_map<EdgeId, EdgeLayout>& otherEdges,
    const std::unordered_set<NodeId>* movedNodes) {

    // Use UnifiedRetryChain for full retry sequence
    auto& retryChain = getRetryChain(effectiveGridSize);

    UnifiedRetryChain::RetryConfig config;
    config.maxSnapRetries = 9;
    config.enableCooperativeReroute = true;
    config.gridSize = effectiveGridSize;
    config.movedNodes = movedNodes;

    auto result = retryChain.calculatePath(
        layout.id, layout, otherEdges, nodeLayouts, config);

    if (result.success) {
        layout = result.layout;

        // Update other edges if they were rerouted (within local copy)
        for (const auto& reroutedLayout : result.reroutedEdges) {
            otherEdges[reroutedLayout.id] = reroutedLayout;
        }

        return true;
    }

    // All attempts failed
    layout.bendPoints.clear();

    return false;
}

void PathCalculator::recalculateBendPoints(
    EdgeLayout& layout,
    const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
    float gridSize,
    const std::unordered_map<EdgeId, EdgeLayout>* otherEdges,
    const std::unordered_set<NodeId>* movedNodes) {

    float gridSizeToUse = constants::effectiveGridSize(gridSize);

    auto srcNodeIt = nodeLayouts.find(layout.from);
    if (srcNodeIt == nodeLayouts.end()) {
        return;
    }
    const NodeLayout& srcNode = srcNodeIt->second;

    // Handle self-loops specially
    if (layout.from == layout.to) {
        handleSelfLoop(layout, srcNode, gridSizeToUse);
        return;
    }

    // Regular edge - use A* pathfinding with UnifiedRetryChain
    // Note: otherEdges is passed as const, so we create a local copy for
    // the retry chain. Rerouted edges are not propagated back to caller.
    std::unordered_map<EdgeId, EdgeLayout> mutableOtherEdges;
    if (otherEdges) {
        mutableOtherEdges = *otherEdges;
    }
    tryAStarPathfinding(layout, nodeLayouts, gridSizeToUse, mutableOtherEdges, movedNodes);
}

} // namespace arborvia
