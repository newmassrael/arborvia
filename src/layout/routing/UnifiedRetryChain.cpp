#include "UnifiedRetryChain.h"
#include "../pathfinding/ObstacleMap.h"
#include "../snap/GridSnapCalculator.h"

#include <array>
#include <cmath>
#include <iostream>

namespace arborvia {

namespace {
    /// Snap ratios to try during retry sequence (ordered by priority)
    static constexpr std::array<float, 9> SNAP_RATIOS = {
        0.2f, 0.4f, 0.5f, 0.6f, 0.8f, 0.1f, 0.9f, 0.3f, 0.7f
    };
}  // anonymous namespace

UnifiedRetryChain::UnifiedRetryChain(
    std::shared_ptr<IPathFinder> pathFinder,
    float gridSize)
    : pathFinder_(std::move(pathFinder))
    , gridSize_(gridSize) {
}

UnifiedRetryChain::~UnifiedRetryChain() = default;

void UnifiedRetryChain::setGridSize(float gridSize) {
    gridSize_ = gridSize;
    if (cooperativeRerouter_) {
        cooperativeRerouter_->setGridSize(gridSize);
    }
}

float UnifiedRetryChain::effectiveGridSize() const {
    return GridSnapCalculator::getEffectiveGridSize(gridSize_);
}

UnifiedRetryChain::RetryResult UnifiedRetryChain::calculatePath(
    EdgeId edgeId,
    const EdgeLayout& layout,
    std::unordered_map<EdgeId, EdgeLayout>& otherEdges,
    const std::unordered_map<NodeId, NodeLayout>& nodeLayouts) {
    RetryConfig defaultConfig;
    return calculatePath(edgeId, layout, otherEdges, nodeLayouts, defaultConfig);
}

UnifiedRetryChain::RetryResult UnifiedRetryChain::calculatePath(
    EdgeId edgeId,
    const EdgeLayout& layout,
    std::unordered_map<EdgeId, EdgeLayout>& otherEdges,
    const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
    const RetryConfig& config) {

    RetryResult result;
    EdgeLayout workingLayout = layout;

#ifdef EDGE_ROUTING_DEBUG
    std::cout << "[UnifiedRetryChain] Edge " << edgeId 
              << " starting calculatePath" << std::endl;
#endif

    // Step 1: Basic A* attempt
    result.astarAttempts++;
    if (tryAStarPath(workingLayout, nodeLayouts, otherEdges)) {
        result.success = true;
        result.layout = workingLayout;
#ifdef EDGE_ROUTING_DEBUG
        std::cout << "[UnifiedRetryChain] Edge " << edgeId 
                  << " SUCCESS at Step 1 (basic A*)" << std::endl;
#endif
        return result;
    }

    // Step 2: CooperativeRerouter (immediately after A* failure)
    if (config.enableCooperativeReroute) {
        result.cooperativeAttempts++;
        auto coopResult = tryCooperativeReroute(edgeId, layout, otherEdges, nodeLayouts);
        if (coopResult.success) {
            result.success = true;
            result.layout = coopResult.layout;
            result.reroutedEdges = std::move(coopResult.reroutedEdges);
#ifdef EDGE_ROUTING_DEBUG
            std::cout << "[UnifiedRetryChain] Edge " << edgeId 
                      << " SUCCESS at Step 2 (CooperativeRerouter)" << std::endl;
#endif
            return result;
        }
    }

    // Step 3: Snap point variations + A* + rerouter
    auto snapResult = trySnapPointVariations(edgeId, layout, otherEdges, nodeLayouts, config);
    result.astarAttempts += snapResult.astarAttempts;
    result.cooperativeAttempts += snapResult.cooperativeAttempts;
    if (snapResult.success) {
#ifdef EDGE_ROUTING_DEBUG
        std::cout << "[UnifiedRetryChain] Edge " << edgeId 
                  << " SUCCESS at Step 3 (snap variations)" << std::endl;
#endif
        return snapResult;
    }

    // Step 4: NodeEdge switch with rerouter attempts
    auto edgeSwitchResult = tryNodeEdgeSwitch(edgeId, layout, otherEdges, nodeLayouts, config);
    result.astarAttempts += edgeSwitchResult.astarAttempts;
    result.cooperativeAttempts += edgeSwitchResult.cooperativeAttempts;
    if (edgeSwitchResult.success) {
#ifdef EDGE_ROUTING_DEBUG
        std::cout << "[UnifiedRetryChain] Edge " << edgeId 
                  << " SUCCESS at Step 4 (NodeEdge switch)" << std::endl;
#endif
        return edgeSwitchResult;
    }

    // All steps failed
    result.failureReason = "All retry attempts exhausted";
    result.layout = layout;  // Return original layout

#ifdef EDGE_ROUTING_DEBUG
    std::cout << "[UnifiedRetryChain] Edge " << edgeId 
              << " ALL STEPS FAILED! astarAttempts=" << result.astarAttempts
              << " cooperativeAttempts=" << result.cooperativeAttempts << std::endl;
#endif

    return result;
}

bool UnifiedRetryChain::tryAStarPath(
    EdgeLayout& layout,
    const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
    const std::unordered_map<EdgeId, EdgeLayout>& otherEdges) {

    float gridSize = effectiveGridSize();

    // Build obstacle map
    ObstacleMap obstacles;
    obstacles.buildFromNodes(nodeLayouts, gridSize, 0);

    // Add other edges as obstacles
    obstacles.addEdgeSegments(otherEdges, layout.id);

    GridPoint startGrid = obstacles.pixelToGrid(layout.sourcePoint);
    GridPoint goalGrid = obstacles.pixelToGrid(layout.targetPoint);

    // Find path
    PathResult pathResult = pathFinder_->findPath(
        startGrid, goalGrid, obstacles,
        layout.from, layout.to,
        layout.sourceEdge, layout.targetEdge,
        {}, {});

    if (pathResult.found && pathResult.path.size() >= 2) {
        layout.bendPoints.clear();
        for (size_t i = 1; i + 1 < pathResult.path.size(); ++i) {
            Point pixelPoint = obstacles.gridToPixel(pathResult.path[i].x, pathResult.path[i].y);
            layout.bendPoints.push_back({pixelPoint});
        }
        return true;
    }

    return false;
}

CooperativeRerouter::RerouteResult UnifiedRetryChain::tryCooperativeReroute(
    EdgeId edgeId,
    const EdgeLayout& layout,
    std::unordered_map<EdgeId, EdgeLayout>& otherEdges,
    const std::unordered_map<NodeId, NodeLayout>& nodeLayouts) {

    // Lazy initialize CooperativeRerouter
    if (!cooperativeRerouter_) {
        cooperativeRerouter_ = std::make_unique<CooperativeRerouter>(
            pathFinder_, effectiveGridSize());
    }

    return cooperativeRerouter_->rerouteWithCooperation(
        edgeId, layout, otherEdges, nodeLayouts);
}

UnifiedRetryChain::RetryResult UnifiedRetryChain::trySnapPointVariations(
    EdgeId edgeId,
    const EdgeLayout& originalLayout,
    std::unordered_map<EdgeId, EdgeLayout>& otherEdges,
    const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
    const RetryConfig& config) {

    RetryResult result;

    // Get source node
    auto srcNodeIt = nodeLayouts.find(originalLayout.from);
    if (srcNodeIt == nodeLayouts.end()) {
        result.failureReason = "Source node not found";
        return result;
    }
    const NodeLayout& srcNode = srcNodeIt->second;

    // Use snap ratios from constant, limited by config
    const size_t maxRatios = std::min(
        SNAP_RATIOS.size(),
        static_cast<size_t>(config.maxSnapRetries));

    for (size_t i = 0; i < maxRatios; ++i) {
        float ratio = SNAP_RATIOS[i];
        EdgeLayout workingLayout = originalLayout;

        // Calculate new source snap point
        int candidateIndex = 0;
        Point newSnapPoint = calculateSnapPointForRatio(
            srcNode, workingLayout.sourceEdge, ratio, &candidateIndex);

        // Skip if same as original
        float dx = std::abs(newSnapPoint.x - originalLayout.sourcePoint.x);
        float dy = std::abs(newSnapPoint.y - originalLayout.sourcePoint.y);
        if (dx < 1.0f && dy < 1.0f) {
            continue;
        }

        workingLayout.sourcePoint = newSnapPoint;
        workingLayout.sourceSnapIndex = candidateIndex;

        // Step 3a: Try A* with new snap point
        result.astarAttempts++;
        if (tryAStarPath(workingLayout, nodeLayouts, otherEdges)) {
            result.success = true;
            result.layout = workingLayout;
            return result;
        }

        // Step 3b: Try CooperativeRerouter
        if (config.enableCooperativeReroute) {
            result.cooperativeAttempts++;
            auto coopResult = tryCooperativeReroute(edgeId, workingLayout, otherEdges, nodeLayouts);
            if (coopResult.success) {
                result.success = true;
                result.layout = coopResult.layout;
                result.reroutedEdges = std::move(coopResult.reroutedEdges);
                return result;
            }
        }

        // Step 3c: Try A* again (rerouter may have moved other edges)
        result.astarAttempts++;
        if (tryAStarPath(workingLayout, nodeLayouts, otherEdges)) {
            result.success = true;
            result.layout = workingLayout;
            return result;
        }
    }

    result.failureReason = "All snap point variations failed";
    return result;
}

UnifiedRetryChain::RetryResult UnifiedRetryChain::tryNodeEdgeSwitch(
    EdgeId edgeId,
    const EdgeLayout& originalLayout,
    std::unordered_map<EdgeId, EdgeLayout>& otherEdges,
    const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
    const RetryConfig& config) {

    RetryResult result;

    auto srcNodeIt = nodeLayouts.find(originalLayout.from);
    auto tgtNodeIt = nodeLayouts.find(originalLayout.to);

    if (srcNodeIt == nodeLayouts.end() || tgtNodeIt == nodeLayouts.end()) {
        result.failureReason = "Source or target node not found";
        return result;
    }

    const NodeLayout& srcNode = srcNodeIt->second;
    const NodeLayout& tgtNode = tgtNodeIt->second;
    float gridSize = effectiveGridSize();

    constexpr std::array<NodeEdge, 4> allEdges = {
        NodeEdge::Top, NodeEdge::Bottom, NodeEdge::Left, NodeEdge::Right
    };

    int combinationsTried = 0;

    for (NodeEdge srcEdge : allEdges) {
        if (combinationsTried >= config.maxNodeEdgeCombinations) break;

        for (NodeEdge tgtEdge : allEdges) {
            if (combinationsTried >= config.maxNodeEdgeCombinations) break;
            combinationsTried++;

            // Skip original combination
            if (srcEdge == originalLayout.sourceEdge && tgtEdge == originalLayout.targetEdge) {
                continue;
            }

            int srcCandidateCount = GridSnapCalculator::getCandidateCount(srcNode, srcEdge, gridSize);
            int tgtCandidateCount = GridSnapCalculator::getCandidateCount(tgtNode, tgtEdge, gridSize);

            // Try center positions for this NodeEdge combination
            int srcCandidateIdx = 0;
            int tgtCandidateIdx = 0;
            Point newSrc = GridSnapCalculator::calculateSnapPosition(
                srcNode, srcEdge, 0, std::max(1, srcCandidateCount), gridSize, &srcCandidateIdx);
            Point newTgt = GridSnapCalculator::calculateSnapPosition(
                tgtNode, tgtEdge, 0, std::max(1, tgtCandidateCount), gridSize, &tgtCandidateIdx);

            EdgeLayout workingLayout = originalLayout;
            workingLayout.sourceEdge = srcEdge;
            workingLayout.targetEdge = tgtEdge;
            workingLayout.sourcePoint = newSrc;
            workingLayout.targetPoint = newTgt;
            workingLayout.sourceSnapIndex = srcCandidateIdx;
            workingLayout.targetSnapIndex = tgtCandidateIdx;

            // Step 4a: Try A*
            result.astarAttempts++;
            if (tryAStarPath(workingLayout, nodeLayouts, otherEdges)) {
                result.success = true;
                result.layout = workingLayout;
                return result;
            }

            // Step 4b: Try CooperativeRerouter
            if (config.enableCooperativeReroute) {
                result.cooperativeAttempts++;
                auto coopResult = tryCooperativeReroute(edgeId, workingLayout, otherEdges, nodeLayouts);
                if (coopResult.success) {
                    result.success = true;
                    result.layout = coopResult.layout;
                    result.reroutedEdges = std::move(coopResult.reroutedEdges);
                    return result;
                }
            }

            // Step 4c: Try A* again
            result.astarAttempts++;
            if (tryAStarPath(workingLayout, nodeLayouts, otherEdges)) {
                result.success = true;
                result.layout = workingLayout;
                return result;
            }
        }
    }

    result.failureReason = "All NodeEdge combinations failed";
    return result;
}

Point UnifiedRetryChain::calculateSnapPointForRatio(
    const NodeLayout& node,
    NodeEdge edge,
    float ratio,
    int* outCandidateIndex) const {

    float gridSize = effectiveGridSize();

    // Calculate edge length
    float edgeLength = 0.0f;
    if (edge == NodeEdge::Top || edge == NodeEdge::Bottom) {
        edgeLength = node.size.width;
    } else {
        edgeLength = node.size.height;
    }

    // Calculate position from ratio
    Point position;
    if (edge == NodeEdge::Top) {
        position = {node.position.x + edgeLength * ratio, node.position.y};
    } else if (edge == NodeEdge::Bottom) {
        position = {node.position.x + edgeLength * ratio, node.position.y + node.size.height};
    } else if (edge == NodeEdge::Left) {
        position = {node.position.x, node.position.y + edgeLength * ratio};
    } else {
        position = {node.position.x + node.size.width, node.position.y + edgeLength * ratio};
    }

    // Quantize to grid
    position.x = std::round(position.x / gridSize) * gridSize;
    position.y = std::round(position.y / gridSize) * gridSize;

    // Get candidate index
    if (outCandidateIndex) {
        *outCandidateIndex = GridSnapCalculator::getCandidateIndexFromPosition(
            node, edge, position, gridSize);
    }

    return position;
}

}  // namespace arborvia
