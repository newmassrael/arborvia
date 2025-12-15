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

namespace arborvia {

namespace {
    /// Snap ratios to try during retry sequence (ordered by priority)
    static constexpr std::array<float, 9> SNAP_RATIOS = {
        0.2f, 0.4f, 0.5f, 0.6f, 0.8f, 0.1f, 0.9f, 0.3f, 0.7f
    };

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
            // NOTE: snapIndex is no longer stored - computed from position as needed

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
            // NOTE: snapIndex is no longer stored - computed from position as needed

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
            // NOTE: snapIndex is no longer stored - computed from position as needed
            int srcCandidateIdx = 0;
            int tgtCandidateIdx = 0;
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
            // NOTE: snapIndex is no longer stored - computed from position as needed

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

}  // namespace arborvia
