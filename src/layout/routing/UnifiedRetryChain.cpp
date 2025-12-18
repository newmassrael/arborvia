#include "UnifiedRetryChain.h"
#include "../pathfinding/ObstacleMap.h"
#include "../sugiyama/routing/PathIntersection.h"
#include "arborvia/core/GeometryUtils.h"
#include "../snap/GridSnapCalculator.h"
#include "../snap/SnapPointCalculator.h"
#include "arborvia/layout/constraints/ConstraintGateway.h"

#include "arborvia/common/Logger.h"
#include <array>
#include <cmath>
#include <set>

namespace arborvia {

namespace {
    /// Collect snap indices already used on a specific NodeEdge
    /// Returns set of snap indices that are occupied by other edges
    std::set<int> collectUsedSnapIndices(
        EdgeId currentEdgeId,
        NodeId nodeId,
        NodeEdge nodeEdge,
        bool isSource,
        const std::unordered_map<EdgeId, EdgeLayout>& otherEdges) {
        
        std::set<int> usedIndices;
        
        for (const auto& [edgeId, layout] : otherEdges) {
            if (edgeId == currentEdgeId) continue;
            
            // Check source side
            if (isSource && layout.from == nodeId && layout.sourceEdge == nodeEdge) {
                usedIndices.insert(layout.sourceSnapIndex);
            }
            // Check target side
            if (!isSource && layout.to == nodeId && layout.targetEdge == nodeEdge) {
                usedIndices.insert(layout.targetSnapIndex);
            }
        }
        
        return usedIndices;
    }
    
    /// Generate adjacent snap indices in priority order: +1, -1, +2, -2, ...
    /// Skips indices that are already used or out of bounds
    std::vector<int> generateAdjacentIndices(
        int currentIndex,
        int candidateCount,
        const std::set<int>& usedIndices) {
        
        std::vector<int> result;
        
        for (int offset = 1; offset < candidateCount; ++offset) {
            // Try +offset
            int plusIdx = currentIndex + offset;
            if (plusIdx < candidateCount && usedIndices.find(plusIdx) == usedIndices.end()) {
                result.push_back(plusIdx);
            }
            
            // Try -offset
            int minusIdx = currentIndex - offset;
            if (minusIdx >= 0 && usedIndices.find(minusIdx) == usedIndices.end()) {
                result.push_back(minusIdx);
            }
        }
        
        return result;
    }

    /// Log constraint violation details
    void logValidationFailure(EdgeId edgeId, const UnifiedRetryChain::PathValidationResult& validation) {
        if (validation.valid) return;
        
        LOG_DEBUG("[UnifiedRetryChain] Edge {} CONSTRAINT VIOLATIONS:", edgeId);
        if (validation.hasOverlap) {
            std::string overlapEdges;
            for (EdgeId eid : validation.overlappingEdges) {
                overlapEdges += " " + std::to_string(eid);
            }
            LOG_DEBUG("  - OVERLAP with edges:{}", overlapEdges);
            // Log detailed overlap info
            for (const auto& detail : validation.overlapDetails) {
                LOG_DEBUG("    Edge{}[{}] ({},{})->({},{}) overlaps Edge{}[{}] ({},{})->({},{})",
                          edgeId, detail.thisSegmentIndex,
                          detail.thisSegStart.x, detail.thisSegStart.y,
                          detail.thisSegEnd.x, detail.thisSegEnd.y,
                          detail.otherEdgeId, detail.otherSegmentIndex,
                          detail.otherSegStart.x, detail.otherSegStart.y,
                          detail.otherSegEnd.x, detail.otherSegEnd.y);
            }
        }
        if (validation.hasDiagonal) {
            LOG_DEBUG("  - DIAGONAL segment (non-orthogonal)");
        }
        if (validation.hasNodePenetration) {
            LOG_DEBUG("  - NODE PENETRATION (non-source/target)");
        }
        if (validation.hasSourcePenetration) {
            LOG_DEBUG("  - SOURCE PENETRATION (intermediate segment penetrates source node)");
        }
        if (validation.hasTargetPenetration) {
            LOG_DEBUG("  - TARGET PENETRATION (intermediate segment penetrates target node)");
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

    LOG_DEBUG("[UnifiedRetryChain] Edge {} starting calculatePath src=({},{}) tgt=({},{}) maxSnapRetries={} maxNodeEdgeCombinations={} enableCooperativeReroute={}",
              edgeId, layout.sourcePoint.x, layout.sourcePoint.y,
              layout.targetPoint.x, layout.targetPoint.y,
              config.maxSnapRetries, config.maxNodeEdgeCombinations, config.enableCooperativeReroute);

    // Step 1: Basic A* attempt
    result.astarAttempts++;
    if (tryAStarPath(workingLayout, nodeLayouts, otherEdges)) {
        result.success = true;
        result.layout = workingLayout;
        result.validation = validatePathResult(edgeId, workingLayout, otherEdges, nodeLayouts);
        LOG_DEBUG("[UnifiedRetryChain] Edge {} SUCCESS at Step 1 (basic A*) validation={}",
                  edgeId, result.validation.valid ? "PASS" : "FAIL");
        logValidationFailure(edgeId, result.validation);
        return result;
    }
    LOG_DEBUG("[UnifiedRetryChain] Edge {} Step 1 FAILED (basic A*)", edgeId);

    // Step 2: CooperativeRerouter (immediately after A* failure)
    if (config.enableCooperativeReroute) {
        result.cooperativeAttempts++;
        auto coopResult = tryCooperativeReroute(edgeId, layout, otherEdges, nodeLayouts);
        if (coopResult.success) {
            result.success = true;
            result.layout = coopResult.layout;
            result.reroutedEdges = std::move(coopResult.reroutedEdges);
            result.validation = validatePathResult(edgeId, coopResult.layout, otherEdges, nodeLayouts);
            LOG_DEBUG("[UnifiedRetryChain] Edge {} SUCCESS at Step 2 (CooperativeRerouter) validation={}",
                      edgeId, result.validation.valid ? "PASS" : "FAIL");
            logValidationFailure(edgeId, result.validation);
            return result;
        }
        LOG_DEBUG("[UnifiedRetryChain] Edge {} Step 2 FAILED (CooperativeRerouter)", edgeId);
    } else {
        LOG_DEBUG("[UnifiedRetryChain] Edge {} Step 2 SKIPPED (enableCooperativeReroute=false)", edgeId);
    }

    // Step 3: Snap point variations + A* + rerouter
    auto snapResult = trySnapPointVariations(edgeId, layout, otherEdges, nodeLayouts, config);
    result.astarAttempts += snapResult.astarAttempts;
    result.cooperativeAttempts += snapResult.cooperativeAttempts;
    if (snapResult.success) {
        snapResult.validation = validatePathResult(edgeId, snapResult.layout, otherEdges, nodeLayouts);
        LOG_DEBUG("[UnifiedRetryChain] Edge {} SUCCESS at Step 3 (snap variations) validation={}",
                  edgeId, snapResult.validation.valid ? "PASS" : "FAIL");
        logValidationFailure(edgeId, snapResult.validation);
        return snapResult;
    }
    LOG_DEBUG("[UnifiedRetryChain] Edge {} Step 3 FAILED (snap variations)", edgeId);

    // Step 4: NodeEdge switch with rerouter attempts
    auto edgeSwitchResult = tryNodeEdgeSwitch(edgeId, layout, otherEdges, nodeLayouts, config);
    result.astarAttempts += edgeSwitchResult.astarAttempts;
    result.cooperativeAttempts += edgeSwitchResult.cooperativeAttempts;
    if (edgeSwitchResult.success) {
        edgeSwitchResult.validation = validatePathResult(edgeId, edgeSwitchResult.layout, otherEdges, nodeLayouts);
        LOG_DEBUG("[UnifiedRetryChain] Edge {} SUCCESS at Step 4 (NodeEdge switch) validation={}",
                  edgeId, edgeSwitchResult.validation.valid ? "PASS" : "FAIL");
        logValidationFailure(edgeId, edgeSwitchResult.validation);
        return edgeSwitchResult;
    }
    LOG_DEBUG("[UnifiedRetryChain] Edge {} Step 4 FAILED (NodeEdge switch), maxNodeEdgeCombinations={}",
              edgeId, config.maxNodeEdgeCombinations);

    // All steps failed - validate the original layout before returning
    result.failureReason = "All retry attempts exhausted";
    result.layout = layout;  // Return original layout
    result.validation = validatePathResult(edgeId, layout, otherEdges, nodeLayouts);

    LOG_DEBUG("[UnifiedRetryChain] Edge {} ALL STEPS FAILED! astarAttempts={} cooperativeAttempts={}",
              edgeId, result.astarAttempts, result.cooperativeAttempts);
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

    // Add other edges as obstacles (automatically handles Point node awareness)
    LOG_DEBUG("[tryAStarPath] Edge {} adding {} other edges as obstacles", layout.id, otherEdges.size());
    for (const auto& [eid, el] : otherEdges) {
        if (eid != layout.id) {
            std::string pathStr = "(" + std::to_string(static_cast<int>(el.sourcePoint.x)) + "," 
                                + std::to_string(static_cast<int>(el.sourcePoint.y)) + ")";
            for (const auto& bp : el.bendPoints) {
                pathStr += "->(" + std::to_string(static_cast<int>(bp.position.x)) + "," 
                         + std::to_string(static_cast<int>(bp.position.y)) + ")";
            }
            pathStr += "->(" + std::to_string(static_cast<int>(el.targetPoint.x)) + "," 
                     + std::to_string(static_cast<int>(el.targetPoint.y)) + ")";
            LOG_DEBUG("  OtherEdge {} path: {}", eid, pathStr);
        }
    }
    obstacles.addEdgeSegmentsForLayout(layout, otherEdges, nodeLayouts);

    GridPoint startGrid = obstacles.pixelToGrid(layout.sourcePoint);
    GridPoint goalGrid = obstacles.pixelToGrid(layout.targetPoint);

    bool startBlocked = obstacles.isBlocked(startGrid.x, startGrid.y);
    bool goalBlocked = obstacles.isBlocked(goalGrid.x, goalGrid.y);
    
    LOG_DEBUG("[tryAStarPath] Edge {} src=({},{}) tgt=({},{}) startGrid=({},{}) goalGrid=({},{}) gridSize={} startBlocked={} goalBlocked={}",
              layout.id, layout.sourcePoint.x, layout.sourcePoint.y,
              layout.targetPoint.x, layout.targetPoint.y,
              startGrid.x, startGrid.y, goalGrid.x, goalGrid.y,
              gridSize, startBlocked, goalBlocked);

    // Find path
    LOG_DEBUG("[CALLER:UnifiedRetryChain.cpp] A* findPath called");
    PathResult pathResult = pathFinder_->findPath(
        startGrid, goalGrid, obstacles,
        layout.from, layout.to,
        layout.sourceEdge, layout.targetEdge,
        {}, {});

    LOG_DEBUG("[tryAStarPath] Edge {} pathResult.found={} pathSize={}",
              layout.id, pathResult.found, pathResult.path.size());

    if (pathResult.found && pathResult.path.size() >= 2) {
        layout.bendPoints.clear();
        
        // Log full A* grid path
        std::string gridPathStr;
        for (size_t i = 0; i < pathResult.path.size(); ++i) {
            gridPathStr += " (" + std::to_string(pathResult.path[i].x) + "," + std::to_string(pathResult.path[i].y) + ")";
        }
        LOG_DEBUG("[tryAStarPath] Edge {} A* grid path:{}", layout.id, gridPathStr);
        
        // Convert grid path to pixel bend points
        for (size_t i = 1; i + 1 < pathResult.path.size(); ++i) {
            Point pixelPoint = obstacles.gridToPixel(pathResult.path[i].x, pathResult.path[i].y);
            layout.bendPoints.push_back({pixelPoint});
        }

        // Ensure orthogonality at source and target
        // The grid path may not align perfectly with sourcePoint/targetPoint
        // (e.g., sourcePoint.y=50 while grid bend is at y=60 for gridSize=20)
        ensureEndpointOrthogonality(layout);

        // Log generated bend points in pixel coordinates
        std::string pixelPathStr = "src=(" + std::to_string(static_cast<int>(layout.sourcePoint.x)) + "," 
                                 + std::to_string(static_cast<int>(layout.sourcePoint.y)) + ")";
        for (const auto& bp : layout.bendPoints) {
            pixelPathStr += " -> (" + std::to_string(static_cast<int>(bp.position.x)) + "," 
                          + std::to_string(static_cast<int>(bp.position.y)) + ")";
        }
        pixelPathStr += " -> tgt=(" + std::to_string(static_cast<int>(layout.targetPoint.x)) + "," 
                      + std::to_string(static_cast<int>(layout.targetPoint.y)) + ")";
        LOG_DEBUG("[tryAStarPath] Edge {} generated path: {}", layout.id, pixelPathStr);
        
        LOG_DEBUG("[tryAStarPath] Edge {} SUCCESS, bendPoints={}", layout.id, layout.bendPoints.size());
        return true;
    }

    LOG_DEBUG("[tryAStarPath] Edge {} FAILED", layout.id);
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
    float gridSize = effectiveGridSize();

    // Soft constraint: determine which endpoints can be modified
    bool canModifySource = !config.movedNodes || config.movedNodes->count(originalLayout.from) > 0;
    bool canModifyTarget = !config.movedNodes || config.movedNodes->count(originalLayout.to) > 0;

    // If neither endpoint can be modified, skip this step
    if (!canModifySource && !canModifyTarget) {
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

    // Try source snap index variations if allowed
    if (canModifySource) {
        // Collect already used snap indices on this NodeEdge
        std::set<int> usedSourceIndices = collectUsedSnapIndices(
            edgeId, originalLayout.from, originalLayout.sourceEdge, true, otherEdges);
        
        // Get candidate count for this NodeEdge
        int srcCandidateCount = GridSnapCalculator::getCandidateCount(
            srcNode, originalLayout.sourceEdge, gridSize);
        
        // Generate adjacent indices (skipping used ones)
        auto adjacentIndices = generateAdjacentIndices(
            originalLayout.sourceSnapIndex, srcCandidateCount, usedSourceIndices);
        
        // Limit retries
        size_t maxRetries = std::min(adjacentIndices.size(), static_cast<size_t>(config.maxSnapRetries));
        
        for (size_t i = 0; i < maxRetries; ++i) {
            int newIndex = adjacentIndices[i];
            EdgeLayout workingLayout = originalLayout;
            
            Point newSnapPoint = GridSnapCalculator::getPositionFromCandidateIndex(
                srcNode, originalLayout.sourceEdge, newIndex, gridSize);
            
            workingLayout.setSourceSnap(newIndex, newSnapPoint);
            
            LOG_DEBUG("[UnifiedRetryChain] Edge {} trying source snapIndex {} (was {})",
                      edgeId, newIndex, originalLayout.sourceSnapIndex);

            if (tryWithReroute(workingLayout)) {
                return result;
            }
        }
    }

    // Try target snap index variations if allowed
    if (canModifyTarget) {
        // Collect already used snap indices on this NodeEdge
        std::set<int> usedTargetIndices = collectUsedSnapIndices(
            edgeId, originalLayout.to, originalLayout.targetEdge, false, otherEdges);
        
        // Get candidate count for this NodeEdge
        int tgtCandidateCount = GridSnapCalculator::getCandidateCount(
            tgtNode, originalLayout.targetEdge, gridSize);
        
        // Generate adjacent indices (skipping used ones)
        auto adjacentIndices = generateAdjacentIndices(
            originalLayout.targetSnapIndex, tgtCandidateCount, usedTargetIndices);
        
        // Limit retries
        size_t maxRetries = std::min(adjacentIndices.size(), static_cast<size_t>(config.maxSnapRetries));
        
        for (size_t i = 0; i < maxRetries; ++i) {
            int newIndex = adjacentIndices[i];
            EdgeLayout workingLayout = originalLayout;
            
            Point newSnapPoint = GridSnapCalculator::getPositionFromCandidateIndex(
                tgtNode, originalLayout.targetEdge, newIndex, gridSize);
            
            workingLayout.setTargetSnap(newIndex, newSnapPoint);
            
            LOG_DEBUG("[UnifiedRetryChain] Edge {} trying target snapIndex {} (was {})",
                      edgeId, newIndex, originalLayout.targetSnapIndex);

            if (tryWithReroute(workingLayout)) {
                return result;
            }
        }
    }

    result.failureReason = "All snap index variations failed";
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
            // Find first unused snap index to avoid collision
            int srcCandidateIdx = 0;
            int tgtCandidateIdx = 0;
            Point newSrc = originalLayout.sourcePoint;
            Point newTgt = originalLayout.targetPoint;

            if (canModifySource) {
                // Collect used indices on this new NodeEdge
                std::set<int> usedSrcIndices = collectUsedSnapIndices(
                    edgeId, originalLayout.from, srcEdge, true, otherEdges);
                
                // Find first unused index
                for (int i = 0; i < srcCandidateCount; ++i) {
                    if (usedSrcIndices.find(i) == usedSrcIndices.end()) {
                        srcCandidateIdx = i;
                        break;
                    }
                }
                newSrc = GridSnapCalculator::getPositionFromCandidateIndex(
                    srcNode, srcEdge, srcCandidateIdx, gridSize);
            }
            if (canModifyTarget) {
                // Collect used indices on this new NodeEdge
                std::set<int> usedTgtIndices = collectUsedSnapIndices(
                    edgeId, originalLayout.to, tgtEdge, false, otherEdges);
                
                // Find first unused index
                for (int i = 0; i < tgtCandidateCount; ++i) {
                    if (usedTgtIndices.find(i) == usedTgtIndices.end()) {
                        tgtCandidateIdx = i;
                        break;
                    }
                }
                newTgt = GridSnapCalculator::getPositionFromCandidateIndex(
                    tgtNode, tgtEdge, tgtCandidateIdx, gridSize);
            }

            EdgeLayout workingLayout = originalLayout;
            workingLayout.sourceEdge = srcEdge;
            workingLayout.targetEdge = tgtEdge;
            if (canModifySource) {
                workingLayout.setSourceSnap(srcCandidateIdx, newSrc);
            }
            if (canModifyTarget) {
                workingLayout.setTargetSnap(tgtCandidateIdx, newTgt);
            }

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

    // Use shared SnapPointCalculator for grid-aligned snap point calculation
    Point position = SnapPointCalculator::calculateFromRatio(node, edge, ratio, gridSize);

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

    // Use ConstraintGateway for centralized constraint checking
    ConstraintGateway gateway;
    auto gatewayResult = gateway.validatePathResult(edgeId, layout, otherEdges, nodeLayouts);

    // Map gateway result to our PathValidationResult
    result.valid = gatewayResult.valid;
    result.hasOverlap = gatewayResult.hasOverlap;
    result.hasDiagonal = gatewayResult.hasDiagonal;
    result.hasNodePenetration = gatewayResult.hasNodePenetration;
    result.hasSourcePenetration = gatewayResult.hasSourcePenetration;
    result.hasTargetPenetration = gatewayResult.hasTargetPenetration;
    result.overlappingEdges = gatewayResult.overlappingEdges;

    // Generate detailed overlap information (for logging purposes)
    // The gateway provides the edge IDs, but we generate details here for backward compatibility
    if (result.hasOverlap) {
        std::vector<Point> fullPath;
        fullPath.push_back(layout.sourcePoint);
        for (const auto& bp : layout.bendPoints) {
            fullPath.push_back(bp.position);
        }
        fullPath.push_back(layout.targetPoint);

        for (EdgeId otherId : result.overlappingEdges) {
            auto it = otherEdges.find(otherId);
            if (it == otherEdges.end()) continue;
            const auto& otherLayout = it->second;

            std::vector<Point> otherPath;
            otherPath.push_back(otherLayout.sourcePoint);
            for (const auto& bp : otherLayout.bendPoints) {
                otherPath.push_back(bp.position);
            }
            otherPath.push_back(otherLayout.targetPoint);

            for (size_t i = 0; i + 1 < fullPath.size(); ++i) {
                for (size_t j = 0; j + 1 < otherPath.size(); ++j) {
                    if (PathIntersection::segmentsOverlap(
                            fullPath[i], fullPath[i + 1],
                            otherPath[j], otherPath[j + 1])) {
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
    }

    return result;
}

void UnifiedRetryChain::ensureEndpointOrthogonality(EdgeLayout& layout) const {
    // Ensure the path from sourcePoint to first bend is orthogonal based on sourceEdge
    // and the path from last bend to targetPoint is orthogonal based on targetEdge
    //
    // The issue: A* works on a grid, so bend points are grid-aligned.
    // But sourcePoint/targetPoint may not be grid-aligned (e.g., Y=50 for gridSize=20).
    // This creates diagonal segments from source to first bend or from last bend to target.

    constexpr float TOLERANCE = 1.0f;  // Tolerance for coordinate comparison

    // --- Fix source side ---
    Point firstRef = layout.bendPoints.empty() ? layout.targetPoint : layout.bendPoints[0].position;
    bool isHorizontalSource = (layout.sourceEdge == NodeEdge::Left || layout.sourceEdge == NodeEdge::Right);

    if (isHorizontalSource) {
        // First segment should be horizontal (same Y)
        // If Y values differ, insert alignment point
        if (std::abs(firstRef.y - layout.sourcePoint.y) > TOLERANCE) {
            // Insert alignment point at (firstRef.x, sourcePoint.y)
            // This creates: source -> (firstRef.x, source.y) -> firstRef
            Point alignPoint{firstRef.x, layout.sourcePoint.y};
            layout.bendPoints.insert(layout.bendPoints.begin(), {alignPoint});
            LOG_DEBUG("[ensureEndpointOrthogonality] Source alignment: inserted ({},{}) before first bend",
                      alignPoint.x, alignPoint.y);
        }
    } else {
        // First segment should be vertical (same X)
        // If X values differ, insert alignment point
        if (std::abs(firstRef.x - layout.sourcePoint.x) > TOLERANCE) {
            // Insert alignment point at (sourcePoint.x, firstRef.y)
            // This creates: source -> (source.x, firstRef.y) -> firstRef
            Point alignPoint{layout.sourcePoint.x, firstRef.y};
            layout.bendPoints.insert(layout.bendPoints.begin(), {alignPoint});
            LOG_DEBUG("[ensureEndpointOrthogonality] Source alignment: inserted ({},{}) before first bend",
                      alignPoint.x, alignPoint.y);
        }
    }

    // --- Fix target side ---
    Point lastRef = layout.bendPoints.empty() ? layout.sourcePoint : layout.bendPoints.back().position;
    bool isHorizontalTarget = (layout.targetEdge == NodeEdge::Left || layout.targetEdge == NodeEdge::Right);

    if (isHorizontalTarget) {
        // Last segment should be horizontal (same Y)
        // If Y values differ, insert alignment point
        if (std::abs(lastRef.y - layout.targetPoint.y) > TOLERANCE) {
            // Insert alignment point at (lastRef.x, targetPoint.y)
            // This creates: lastRef -> (lastRef.x, target.y) -> target
            Point alignPoint{lastRef.x, layout.targetPoint.y};
            layout.bendPoints.push_back({alignPoint});
            LOG_DEBUG("[ensureEndpointOrthogonality] Target alignment: inserted ({},{}) after last bend",
                      alignPoint.x, alignPoint.y);
        }
    } else {
        // Last segment should be vertical (same X)
        // If X values differ, insert alignment point
        if (std::abs(lastRef.x - layout.targetPoint.x) > TOLERANCE) {
            // Insert alignment point at (targetPoint.x, lastRef.y)
            // This creates: lastRef -> (target.x, lastRef.y) -> target
            Point alignPoint{layout.targetPoint.x, lastRef.y};
            layout.bendPoints.push_back({alignPoint});
            LOG_DEBUG("[ensureEndpointOrthogonality] Target alignment: inserted ({},{}) after last bend",
                      alignPoint.x, alignPoint.y);
        }
    }
}

}  // namespace arborvia
