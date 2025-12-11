#include "UnifiedRetryChain.h"
#include "../pathfinding/ObstacleMap.h"
#include "../sugiyama/routing/PathIntersection.h"
#include "arborvia/core/GeometryUtils.h"
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

    /// Log constraint violation details
    void logValidationFailure(EdgeId edgeId, const UnifiedRetryChain::PathValidationResult& validation) {
        if (validation.valid) return;
        
        std::cout << "[UnifiedRetryChain] Edge " << edgeId 
                  << " CONSTRAINT VIOLATIONS:" << std::endl;
        if (validation.hasOverlap) {
            std::cout << "  - OVERLAP with edges:";
            for (EdgeId eid : validation.overlappingEdges) {
                std::cout << " " << eid;
            }
            std::cout << std::endl;
            // Log detailed overlap info
            for (const auto& detail : validation.overlapDetails) {
                std::cout << "    Edge" << edgeId << "[" << detail.thisSegmentIndex << "] "
                          << "(" << detail.thisSegStart.x << "," << detail.thisSegStart.y << ")->"
                          << "(" << detail.thisSegEnd.x << "," << detail.thisSegEnd.y << ")"
                          << " overlaps Edge" << detail.otherEdgeId << "[" << detail.otherSegmentIndex << "] "
                          << "(" << detail.otherSegStart.x << "," << detail.otherSegStart.y << ")->"
                          << "(" << detail.otherSegEnd.x << "," << detail.otherSegEnd.y << ")"
                          << std::endl;
            }
        }
        if (validation.hasDiagonal) {
            std::cout << "  - DIAGONAL segment (non-orthogonal)" << std::endl;
        }
        if (validation.hasNodePenetration) {
            std::cout << "  - NODE PENETRATION" << std::endl;
        }
    }
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
    return constants::effectiveGridSize(gridSize_);
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

    std::cout << "[UnifiedRetryChain] Edge " << edgeId 
              << " starting calculatePath"
              << " src=(" << layout.sourcePoint.x << "," << layout.sourcePoint.y << ")"
              << " tgt=(" << layout.targetPoint.x << "," << layout.targetPoint.y << ")"
              << " maxSnapRetries=" << config.maxSnapRetries
              << " maxNodeEdgeCombinations=" << config.maxNodeEdgeCombinations
              << " enableCooperativeReroute=" << config.enableCooperativeReroute
              << std::endl;

    // Step 1: Basic A* attempt
    result.astarAttempts++;
    if (tryAStarPath(workingLayout, nodeLayouts, otherEdges)) {
        result.success = true;
        result.layout = workingLayout;
        result.validation = validatePathResult(edgeId, workingLayout, otherEdges, nodeLayouts);
        std::cout << "[UnifiedRetryChain] Edge " << edgeId 
                  << " SUCCESS at Step 1 (basic A*)"
                  << " validation=" << (result.validation.valid ? "PASS" : "FAIL")
                  << std::endl;
        logValidationFailure(edgeId, result.validation);
        return result;
    }
    std::cout << "[UnifiedRetryChain] Edge " << edgeId 
              << " Step 1 FAILED (basic A*)" << std::endl;

    // Step 2: CooperativeRerouter (immediately after A* failure)
    if (config.enableCooperativeReroute) {
        result.cooperativeAttempts++;
        auto coopResult = tryCooperativeReroute(edgeId, layout, otherEdges, nodeLayouts);
        if (coopResult.success) {
            result.success = true;
            result.layout = coopResult.layout;
            result.reroutedEdges = std::move(coopResult.reroutedEdges);
            result.validation = validatePathResult(edgeId, coopResult.layout, otherEdges, nodeLayouts);
            std::cout << "[UnifiedRetryChain] Edge " << edgeId 
                      << " SUCCESS at Step 2 (CooperativeRerouter)"
                      << " validation=" << (result.validation.valid ? "PASS" : "FAIL")
                      << std::endl;
            logValidationFailure(edgeId, result.validation);
            return result;
        }
        std::cout << "[UnifiedRetryChain] Edge " << edgeId 
                  << " Step 2 FAILED (CooperativeRerouter)" << std::endl;
    } else {
        std::cout << "[UnifiedRetryChain] Edge " << edgeId 
                  << " Step 2 SKIPPED (enableCooperativeReroute=false)" << std::endl;
    }

    // Step 3: Snap point variations + A* + rerouter
    auto snapResult = trySnapPointVariations(edgeId, layout, otherEdges, nodeLayouts, config);
    result.astarAttempts += snapResult.astarAttempts;
    result.cooperativeAttempts += snapResult.cooperativeAttempts;
    if (snapResult.success) {
        snapResult.validation = validatePathResult(edgeId, snapResult.layout, otherEdges, nodeLayouts);
        std::cout << "[UnifiedRetryChain] Edge " << edgeId 
                  << " SUCCESS at Step 3 (snap variations)"
                  << " validation=" << (snapResult.validation.valid ? "PASS" : "FAIL")
                  << std::endl;
        logValidationFailure(edgeId, snapResult.validation);
        return snapResult;
    }
    std::cout << "[UnifiedRetryChain] Edge " << edgeId 
              << " Step 3 FAILED (snap variations)" << std::endl;

    // Step 4: NodeEdge switch with rerouter attempts
    auto edgeSwitchResult = tryNodeEdgeSwitch(edgeId, layout, otherEdges, nodeLayouts, config);
    result.astarAttempts += edgeSwitchResult.astarAttempts;
    result.cooperativeAttempts += edgeSwitchResult.cooperativeAttempts;
    if (edgeSwitchResult.success) {
        edgeSwitchResult.validation = validatePathResult(edgeId, edgeSwitchResult.layout, otherEdges, nodeLayouts);
        std::cout << "[UnifiedRetryChain] Edge " << edgeId 
                  << " SUCCESS at Step 4 (NodeEdge switch)"
                  << " validation=" << (edgeSwitchResult.validation.valid ? "PASS" : "FAIL")
                  << std::endl;
        logValidationFailure(edgeId, edgeSwitchResult.validation);
        return edgeSwitchResult;
    }
    std::cout << "[UnifiedRetryChain] Edge " << edgeId 
              << " Step 4 FAILED (NodeEdge switch), maxNodeEdgeCombinations=" 
              << config.maxNodeEdgeCombinations << std::endl;

    // All steps failed - validate the original layout before returning
    result.failureReason = "All retry attempts exhausted";
    result.layout = layout;  // Return original layout
    result.validation = validatePathResult(edgeId, layout, otherEdges, nodeLayouts);

    std::cout << "[UnifiedRetryChain] Edge " << edgeId 
              << " ALL STEPS FAILED! astarAttempts=" << result.astarAttempts
              << " cooperativeAttempts=" << result.cooperativeAttempts << std::endl;
    logValidationFailure(edgeId, result.validation);

    return result;
}

bool UnifiedRetryChain::tryAStarPath(
    EdgeLayout& layout,
    const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
    const std::unordered_map<EdgeId, EdgeLayout>& otherEdges) {

    float gridSize = effectiveGridSize();

    // Build obstacle map
    // Include edge layouts in bounds calculation to prevent out-of-bounds segments
    ObstacleMap obstacles;
    obstacles.buildFromNodes(nodeLayouts, gridSize, 0, &otherEdges);

    // Add other edges as obstacles
    std::cout << "[tryAStarPath] Edge " << layout.id << " adding " << otherEdges.size()
              << " other edges as obstacles" << std::endl;
    for (const auto& [eid, el] : otherEdges) {
        if (eid != layout.id) {
            std::cout << "  OtherEdge " << eid << " path: (" 
                      << el.sourcePoint.x << "," << el.sourcePoint.y << ")";
            for (const auto& bp : el.bendPoints) {
                std::cout << "->(" << bp.position.x << "," << bp.position.y << ")";
            }
            std::cout << "->(" << el.targetPoint.x << "," << el.targetPoint.y << ")" << std::endl;
        }
    }
    obstacles.addEdgeSegments(otherEdges, layout.id);

    GridPoint startGrid = obstacles.pixelToGrid(layout.sourcePoint);
    GridPoint goalGrid = obstacles.pixelToGrid(layout.targetPoint);

    bool startBlocked = obstacles.isBlocked(startGrid.x, startGrid.y);
    bool goalBlocked = obstacles.isBlocked(goalGrid.x, goalGrid.y);
    
    std::cout << "[tryAStarPath] Edge " << layout.id
              << " src=(" << layout.sourcePoint.x << "," << layout.sourcePoint.y << ")"
              << " tgt=(" << layout.targetPoint.x << "," << layout.targetPoint.y << ")"
              << " startGrid=(" << startGrid.x << "," << startGrid.y << ")"
              << " goalGrid=(" << goalGrid.x << "," << goalGrid.y << ")"
              << " gridSize=" << gridSize 
              << " startBlocked=" << startBlocked
              << " goalBlocked=" << goalBlocked << std::endl;

    // Find path
    PathResult pathResult = pathFinder_->findPath(
        startGrid, goalGrid, obstacles,
        layout.from, layout.to,
        layout.sourceEdge, layout.targetEdge,
        {}, {});

    std::cout << "[tryAStarPath] Edge " << layout.id
              << " pathResult.found=" << pathResult.found
              << " pathSize=" << pathResult.path.size() << std::endl;

    if (pathResult.found && pathResult.path.size() >= 2) {
        layout.bendPoints.clear();
        
        // Log full A* grid path
        std::cout << "[tryAStarPath] Edge " << layout.id << " A* grid path:";
        for (size_t i = 0; i < pathResult.path.size(); ++i) {
            std::cout << " (" << pathResult.path[i].x << "," << pathResult.path[i].y << ")";
        }
        std::cout << std::endl;
        
        // Convert grid path to pixel bend points
        for (size_t i = 1; i + 1 < pathResult.path.size(); ++i) {
            Point pixelPoint = obstacles.gridToPixel(pathResult.path[i].x, pathResult.path[i].y);
            layout.bendPoints.push_back({pixelPoint});
        }
        
        // Log generated bend points in pixel coordinates
        std::cout << "[tryAStarPath] Edge " << layout.id << " generated path:";
        std::cout << " src=(" << layout.sourcePoint.x << "," << layout.sourcePoint.y << ")";
        for (const auto& bp : layout.bendPoints) {
            std::cout << " -> (" << bp.position.x << "," << bp.position.y << ")";
        }
        std::cout << " -> tgt=(" << layout.targetPoint.x << "," << layout.targetPoint.y << ")";
        std::cout << std::endl;
        
        std::cout << "[tryAStarPath] Edge " << layout.id
                  << " SUCCESS, bendPoints=" << layout.bendPoints.size() << std::endl;
        return true;
    }

    std::cout << "[tryAStarPath] Edge " << layout.id << " FAILED" << std::endl;
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

    // Soft constraint: determine which endpoints can be modified
    bool canModifySource = !config.movedNodes || config.movedNodes->count(originalLayout.from) > 0;
    bool canModifyTarget = !config.movedNodes || config.movedNodes->count(originalLayout.to) > 0;

    // If neither endpoint can be modified, skip this step
    if (!canModifySource && !canModifyTarget) {
#ifdef EDGE_ROUTING_DEBUG
        std::cout << "[UnifiedRetryChain] Edge " << edgeId 
                  << " skipping snap variations (neither node moved)" << std::endl;
#endif
        result.failureReason = "Neither node moved, snap variations skipped";
        return result;
    }

    // Get nodes
    auto srcNodeIt = nodeLayouts.find(originalLayout.from);
    auto tgtNodeIt = nodeLayouts.find(originalLayout.to);
    if (srcNodeIt == nodeLayouts.end() || tgtNodeIt == nodeLayouts.end()) {
        result.failureReason = "Source or target node not found";
        return result;
    }
    const NodeLayout& srcNode = srcNodeIt->second;
    const NodeLayout& tgtNode = tgtNodeIt->second;

    // Use snap ratios from constant, limited by config
    const size_t maxRatios = std::min(
        SNAP_RATIOS.size(),
        static_cast<size_t>(config.maxSnapRetries));

    // Helper lambda to try A* and cooperative reroute
    auto tryWithReroute = [&](EdgeLayout& workingLayout) -> bool {
        // Try A*
        result.astarAttempts++;
        if (tryAStarPath(workingLayout, nodeLayouts, otherEdges)) {
            result.success = true;
            result.layout = workingLayout;
            return true;
        }

        // Try CooperativeRerouter
        if (config.enableCooperativeReroute) {
            result.cooperativeAttempts++;
            auto coopResult = tryCooperativeReroute(edgeId, workingLayout, otherEdges, nodeLayouts);
            if (coopResult.success) {
                result.success = true;
                result.layout = coopResult.layout;
                result.reroutedEdges = std::move(coopResult.reroutedEdges);
                return true;
            }
        }

        // Try A* again (rerouter may have moved other edges)
        result.astarAttempts++;
        if (tryAStarPath(workingLayout, nodeLayouts, otherEdges)) {
            result.success = true;
            result.layout = workingLayout;
            return true;
        }

        return false;
    };

    // Try source snap variations if allowed
    if (canModifySource) {
        for (size_t i = 0; i < maxRatios; ++i) {
            float ratio = SNAP_RATIOS[i];
            EdgeLayout workingLayout = originalLayout;

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

            if (tryWithReroute(workingLayout)) {
                return result;
            }
        }
    }

    // Try target snap variations if allowed
    if (canModifyTarget) {
        for (size_t i = 0; i < maxRatios; ++i) {
            float ratio = SNAP_RATIOS[i];
            EdgeLayout workingLayout = originalLayout;

            int candidateIndex = 0;
            Point newSnapPoint = calculateSnapPointForRatio(
                tgtNode, workingLayout.targetEdge, ratio, &candidateIndex);

            // Skip if same as original
            float dx = std::abs(newSnapPoint.x - originalLayout.targetPoint.x);
            float dy = std::abs(newSnapPoint.y - originalLayout.targetPoint.y);
            if (dx < 1.0f && dy < 1.0f) {
                continue;
            }

            workingLayout.targetPoint = newSnapPoint;
            workingLayout.targetSnapIndex = candidateIndex;

            if (tryWithReroute(workingLayout)) {
                return result;
            }
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

    // Soft constraint: determine which endpoints can be modified
    bool canModifySource = !config.movedNodes || config.movedNodes->count(originalLayout.from) > 0;
    bool canModifyTarget = !config.movedNodes || config.movedNodes->count(originalLayout.to) > 0;

    // If neither endpoint can be modified, skip this step
    if (!canModifySource && !canModifyTarget) {
#ifdef EDGE_ROUTING_DEBUG
        std::cout << "[UnifiedRetryChain] Edge " << edgeId 
                  << " skipping NodeEdge switch (neither node moved)" << std::endl;
#endif
        result.failureReason = "Neither node moved, NodeEdge switch skipped";
        return result;
    }

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

    // If source can't be modified, only iterate original sourceEdge
    // If target can't be modified, only iterate original targetEdge
    auto srcEdgesToTry = canModifySource ? std::vector<NodeEdge>(allEdges.begin(), allEdges.end())
                                          : std::vector<NodeEdge>{originalLayout.sourceEdge};
    auto tgtEdgesToTry = canModifyTarget ? std::vector<NodeEdge>(allEdges.begin(), allEdges.end())
                                          : std::vector<NodeEdge>{originalLayout.targetEdge};

    for (NodeEdge srcEdge : srcEdgesToTry) {
        if (combinationsTried >= config.maxNodeEdgeCombinations) break;

        for (NodeEdge tgtEdge : tgtEdgesToTry) {
            if (combinationsTried >= config.maxNodeEdgeCombinations) break;
            combinationsTried++;

            // Skip original combination
            if (srcEdge == originalLayout.sourceEdge && tgtEdge == originalLayout.targetEdge) {
                continue;
            }

            int srcCandidateCount = GridSnapCalculator::getCandidateCount(srcNode, srcEdge, gridSize);
            int tgtCandidateCount = GridSnapCalculator::getCandidateCount(tgtNode, tgtEdge, gridSize);

            // Calculate new positions only for modifiable endpoints
            int srcCandidateIdx = originalLayout.sourceSnapIndex;
            int tgtCandidateIdx = originalLayout.targetSnapIndex;
            Point newSrc = originalLayout.sourcePoint;
            Point newTgt = originalLayout.targetPoint;

            if (canModifySource) {
                newSrc = GridSnapCalculator::calculateSnapPosition(
                    srcNode, srcEdge, 0, std::max(1, srcCandidateCount), gridSize, &srcCandidateIdx);
            }
            if (canModifyTarget) {
                newTgt = GridSnapCalculator::calculateSnapPosition(
                    tgtNode, tgtEdge, 0, std::max(1, tgtCandidateCount), gridSize, &tgtCandidateIdx);
            }

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

UnifiedRetryChain::PathValidationResult UnifiedRetryChain::validatePathResult(
    EdgeId edgeId,
    const EdgeLayout& layout,
    const std::unordered_map<EdgeId, EdgeLayout>& otherEdges,
    const std::unordered_map<NodeId, NodeLayout>& nodeLayouts) const {

    PathValidationResult result;

    // Build full path: source -> bends -> target
    std::vector<Point> fullPath;
    fullPath.push_back(layout.sourcePoint);
    for (const auto& bp : layout.bendPoints) {
        fullPath.push_back(bp.position);
    }
    fullPath.push_back(layout.targetPoint);

    // Check 1: Segment overlaps with other edges (with detailed info)
    for (const auto& [otherId, otherLayout] : otherEdges) {
        if (otherId == edgeId) continue;
        
        // Build other edge's full path
        std::vector<Point> otherPath;
        otherPath.push_back(otherLayout.sourcePoint);
        for (const auto& bp : otherLayout.bendPoints) {
            otherPath.push_back(bp.position);
        }
        otherPath.push_back(otherLayout.targetPoint);
        
        // Check each segment pair for overlap
        bool foundOverlap = false;
        for (size_t i = 0; i + 1 < fullPath.size(); ++i) {
            for (size_t j = 0; j + 1 < otherPath.size(); ++j) {
                if (PathIntersection::segmentsOverlap(
                        fullPath[i], fullPath[i + 1],
                        otherPath[j], otherPath[j + 1])) {
                    if (!foundOverlap) {
                        result.hasOverlap = true;
                        result.overlappingEdges.push_back(otherId);
                        foundOverlap = true;
                    }
                    // Add detail
                    OverlapDetail detail;
                    detail.otherEdgeId = otherId;
                    detail.thisSegmentIndex = static_cast<int>(i);
                    detail.otherSegmentIndex = static_cast<int>(j);
                    detail.thisSegStart = fullPath[i];
                    detail.thisSegEnd = fullPath[i + 1];
                    detail.otherSegStart = otherPath[j];
                    detail.otherSegEnd = otherPath[j + 1];
                    result.overlapDetails.push_back(detail);
                }
            }
        }
    }

    // Check 2: Diagonal segments (non-orthogonal paths)
    auto checkDiagonal = [](const Point& p1, const Point& p2) -> bool {
        constexpr float EPSILON = 1.0f;
        float dx = std::abs(p1.x - p2.x);
        float dy = std::abs(p1.y - p2.y);
        // Diagonal if neither horizontal (dy < EPSILON) nor vertical (dx < EPSILON)
        return dx > EPSILON && dy > EPSILON;
    };

    for (size_t i = 0; i + 1 < fullPath.size(); ++i) {
        if (checkDiagonal(fullPath[i], fullPath[i + 1])) {
            result.hasDiagonal = true;
            break;
        }
    }

    // Check 3: Node penetration (segment crosses through a node)
    for (const auto& [nodeId, nodeLayout] : nodeLayouts) {
        // Skip source and target nodes
        if (nodeId == layout.from || nodeId == layout.to) continue;

        for (size_t i = 0; i + 1 < fullPath.size(); ++i) {
            if (geometry::segmentIntersectsAABB(
                fullPath[i], fullPath[i + 1],
                nodeLayout.position, nodeLayout.size)) {
                result.hasNodePenetration = true;
                break;
            }
        }
        if (result.hasNodePenetration) break;
    }

    // Set overall valid flag
    result.valid = !result.hasOverlap && !result.hasDiagonal && !result.hasNodePenetration;

    return result;
}

}  // namespace arborvia
