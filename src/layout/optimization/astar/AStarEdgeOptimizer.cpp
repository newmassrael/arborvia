#include "AStarEdgeOptimizer.h"
#include "../../pathfinding/ObstacleMap.h"
#include "../../pathfinding/AStarPathFinder.h"
#include "../../snap/GridSnapCalculator.h"
#include "../../routing/CooperativeRerouter.h"
#include "../../routing/UnifiedRetryChain.h"
#include "../../sugiyama/routing/PathIntersection.h"
#include "../../sugiyama/routing/SelfLoopRouter.h"
#include "arborvia/core/GeometryUtils.h"
#include "arborvia/layout/api/IEdgePenalty.h"

#include "arborvia/common/Logger.h"
#include <algorithm>
#include <array>
#include <future>
#include <thread>
#include <unordered_set>


namespace arborvia {

AStarEdgeOptimizer::AStarEdgeOptimizer(
    std::shared_ptr<IPathFinder> pathFinder)
    : pathFinder_(std::move(pathFinder)) {

    // Create default pathfinder if not provided
    if (!pathFinder_) {
        pathFinder_ = std::make_shared<AStarPathFinder>();
    }

    // Ensure penalty system is set
    if (!penaltySystem()) {
        setPenaltySystem(EdgePenaltySystem::createDefault());
    }
}

AStarEdgeOptimizer::~AStarEdgeOptimizer() = default;

UnifiedRetryChain& AStarEdgeOptimizer::getRetryChain(float gridSize) {
    if (!retryChain_ || lastRetryChainGridSize_ != gridSize) {
        retryChain_ = std::make_unique<UnifiedRetryChain>(pathFinder_, gridSize);
        lastRetryChainGridSize_ = gridSize;
    }
    return *retryChain_;
}

void AStarEdgeOptimizer::setPathFinder(std::shared_ptr<IPathFinder> pathFinder) {
    pathFinder_ = std::move(pathFinder);
    if (!pathFinder_) {
        pathFinder_ = std::make_shared<AStarPathFinder>();
    }
}

float AStarEdgeOptimizer::effectiveGridSize() const {
    return constants::effectiveGridSize(gridSize_);
}

std::unordered_map<EdgeId, EdgeLayout> AStarEdgeOptimizer::optimize(
    const std::vector<EdgeId>& edges,
    const std::unordered_map<EdgeId, EdgeLayout>& currentLayouts,
    const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
    float gridSizeParam,
    const std::unordered_set<NodeId>& movedNodes) {

    // Store gridSize for use in helper methods (effectiveGridSize() will return this)
    gridSize_ = gridSizeParam;
    
    // Store constraint state in base class for FixedEndpointPenalty
    setConstraintState(currentLayouts, movedNodes);

    // Calculate forbidden zones once for all edges
    float gridSize = effectiveGridSize();
    auto forbiddenZones = calculateForbiddenZones(nodeLayouts, gridSize);

    std::unordered_map<EdgeId, EdgeLayout> result;
    std::unordered_map<EdgeId, EdgeLayout> assignedLayouts = currentLayouts;

    // Determine thread count (limit to avoid overhead for small edge counts)
    const size_t numEdges = edges.size();
    const size_t maxThreads = std::min(
        static_cast<size_t>(std::thread::hardware_concurrency()),
        numEdges);
    const bool useParallel = parallelEnabled_ && numEdges >= 3 && maxThreads >= 2;

    if (useParallel) {
        // ===== PARALLEL MULTI-PASS OPTIMIZATION =====
        constexpr int MAX_PASSES = 3;

        // Track edges that had overlap resolution applied - don't re-evaluate them
        std::unordered_set<EdgeId> resolvedEdges;

        for (int pass = 0; pass < MAX_PASSES; ++pass) {
            // ===== PHASE 1: Parallel independent evaluation =====
            // Note: nodeLayouts and forbiddenZones are read-only during optimize(),
            // so reference capture is safe and avoids expensive copying
            std::vector<std::future<ParallelEdgeResult>> futures;
            futures.reserve(numEdges);

            for (EdgeId edgeId : edges) {
                // Skip edges that had successful overlap resolution - keep their resolved layout
                if (resolvedEdges.count(edgeId)) continue;

                auto it = currentLayouts.find(edgeId);
                if (it == currentLayouts.end()) continue;

                // Use result from previous pass if available
                const EdgeLayout baseLayout = (pass > 0 && result.count(edgeId))
                    ? result[edgeId] : it->second;

                // Launch async evaluation - reference capture is safe for read-only data
                futures.push_back(std::async(std::launch::async,
                    [this, edgeId, baseLayout, &nodeLayouts, &forbiddenZones, &currentLayouts]() {
                        // Create thread-local pathfinder for thread safety
                        AStarPathFinder localPathFinder;
                        return evaluateEdgeIndependent(
                            edgeId, baseLayout, nodeLayouts, forbiddenZones, localPathFinder, currentLayouts);
                    }));
            }

            // Collect results
            bool anyOverlapFound = false;

            for (auto& future : futures) {
                ParallelEdgeResult presult = future.get();
                if (presult.hasValidResult) {
                    if (presult.best.score > 0) anyOverlapFound = true;
                    result[presult.edgeId] = presult.best.layout;
                    assignedLayouts[presult.edgeId] = presult.best.layout;
                    if (presult.isSelfLoop) {
                        LOG_DEBUG("[SNAP-TRACE] PARALLEL SELFLOOP SAVED edge={} srcPos=({},{}) tgtPos=({},{})",
                                  presult.edgeId, presult.best.layout.sourcePoint.x, presult.best.layout.sourcePoint.y, 
                                  presult.best.layout.targetPoint.x, presult.best.layout.targetPoint.y);
                    }
                } else {
                    // Mark as failed - will be processed in Phase 1.5
                    auto it = currentLayouts.find(presult.edgeId);
                    if (it != currentLayouts.end()) {
                        result[presult.edgeId] = it->second;
                        assignedLayouts[presult.edgeId] = it->second;
                    }
                }
            }

            // ===== PHASE 1.5: Sequential retry chain for failed edges =====
            // Process failed edges (those with diagonal paths) using UnifiedRetryChain
            // This must be done sequentially because CooperativeRerouter modifies other edges
            std::vector<EdgeId> failedEdgesInParallel;
            for (EdgeId edgeId : edges) {
                if (resolvedEdges.count(edgeId)) continue;

                auto it = result.find(edgeId);
                if (it == result.end()) continue;

                const EdgeLayout& layout = it->second;
                if (layout.from == layout.to) continue;  // Skip self-loops

                // Check if edge has diagonal path
                bool hasDiagonal = false;
                if (layout.bendPoints.empty()) {
                    float dx = std::abs(layout.sourcePoint.x - layout.targetPoint.x);
                    float dy = std::abs(layout.sourcePoint.y - layout.targetPoint.y);
                    hasDiagonal = (dx > 1.0f && dy > 1.0f);
                } else {
                    float dx_src = std::abs(layout.sourcePoint.x - layout.bendPoints[0].position.x);
                    float dy_src = std::abs(layout.sourcePoint.y - layout.bendPoints[0].position.y);
                    const auto& lastBend = layout.bendPoints.back();
                    float dx_tgt = std::abs(lastBend.position.x - layout.targetPoint.x);
                    float dy_tgt = std::abs(lastBend.position.y - layout.targetPoint.y);
                    hasDiagonal = (dx_src > 1.0f && dy_src > 1.0f) || (dx_tgt > 1.0f && dy_tgt > 1.0f);
                }

                if (hasDiagonal) {
                    failedEdgesInParallel.push_back(edgeId);
                }
            }

            // Process failed edges sequentially with UnifiedRetryChain
            for (EdgeId edgeId : failedEdgesInParallel) {
                auto it = result.find(edgeId);
                if (it == result.end()) continue;

                auto& retryChain = getRetryChain(gridSize);

                UnifiedRetryChain::RetryConfig config;
                config.maxSnapRetries = 9;
                config.maxNodeEdgeCombinations = 16;
                config.enableCooperativeReroute = true;
                config.gridSize = gridSize;
                config.movedNodes = movedNodes_.empty() ? nullptr : &movedNodes_;

                auto retryResult = retryChain.calculatePath(
                    edgeId, it->second, assignedLayouts, nodeLayouts, config);

                if (retryResult.success) {
                    result[edgeId] = retryResult.layout;
                    assignedLayouts[edgeId] = retryResult.layout;

                    // Update rerouted edges
                    for (const auto& reroutedLayout : retryResult.reroutedEdges) {
                        if (result.count(reroutedLayout.id)) {
                            result[reroutedLayout.id] = reroutedLayout;
                        }
                        assignedLayouts[reroutedLayout.id] = reroutedLayout;
                    }
                }
            }

            // ===== PHASE 2: Detect overlaps =====
            auto overlappingPairs = detectOverlaps(result);
            LOG_DEBUG("[AStarOpt] PHASE 2: detectOverlaps found {} pairs (pass={}, parallel=true)",
                      overlappingPairs.size(), pass);
            for (const auto& [a, b] : overlappingPairs) {
                LOG_DEBUG("[AStarOpt]   Overlap: Edge {} & Edge {}", a, b);
            }
            if (!overlappingPairs.empty()) anyOverlapFound = true;

            // ===== PHASE 3: Resolve overlaps (serial - cooperative rerouting) =====
            resolveAllOverlaps(overlappingPairs, result, assignedLayouts,
                               nodeLayouts, forbiddenZones, &resolvedEdges);

            // Exit early if no overlaps found
            if (!anyOverlapFound) break;
        }
    } else {
        // ===== SEQUENTIAL FALLBACK (original algorithm) =====
        constexpr int MAX_PASSES = 3;
        std::vector<EdgeId> failedEdges;

        for (int pass = 0; pass < MAX_PASSES; ++pass) {
            bool anyOverlapFound = false;
            failedEdges.clear();

            for (EdgeId edgeId : edges) {
                auto it = currentLayouts.find(edgeId);
                if (it == currentLayouts.end()) continue;

                const EdgeLayout& baseLayout = (pass > 0 && result.count(edgeId))
                    ? result[edgeId] : it->second;

                // Handle self-loops
                if (baseLayout.from == baseLayout.to) {
                    auto selfLoopCombinations = evaluateSelfLoopCombinations(
                        edgeId, baseLayout, assignedLayouts, nodeLayouts, forbiddenZones);
                    if (!selfLoopCombinations.empty()) {
                        const auto& best = selfLoopCombinations.front();
                        if (best.score > 0) anyOverlapFound = true;
                        result[edgeId] = best.layout;
                        assignedLayouts[edgeId] = best.layout;
                        LOG_DEBUG("[SNAP-TRACE] AStarOptimizer SELFLOOP SAVED edge={} srcPos=({},{}) tgtPos=({},{})",
                                  edgeId, best.layout.sourcePoint.x, best.layout.sourcePoint.y,
                                  best.layout.targetPoint.x, best.layout.targetPoint.y);
                    } else {
                        result[edgeId] = baseLayout;
                        assignedLayouts[edgeId] = baseLayout;
                        LOG_DEBUG("[SNAP-TRACE] AStarOptimizer SELFLOOP FALLBACK edge={}", edgeId);
                    }
                    continue;
                }

                // Evaluate edge combinations
                auto combinations = evaluateCombinations(
                    edgeId, baseLayout, assignedLayouts, nodeLayouts, forbiddenZones);

                if (combinations.empty()) {
                    // All 16 combinations failed - use UnifiedRetryChain as fallback
                    auto& retryChain = getRetryChain(gridSize);

                    UnifiedRetryChain::RetryConfig config;
                    config.maxSnapRetries = 9;
                    config.maxNodeEdgeCombinations = 16;
                    config.enableCooperativeReroute = true;
                    config.gridSize = gridSize;
                    config.movedNodes = movedNodes_.empty() ? nullptr : &movedNodes_;

                    auto retryResult = retryChain.calculatePath(
                        edgeId, baseLayout, assignedLayouts, nodeLayouts, config);

                    if (retryResult.success) {
                        result[edgeId] = retryResult.layout;
                        assignedLayouts[edgeId] = retryResult.layout;
                        // Update rerouted edges
                        for (const auto& reroutedLayout : retryResult.reroutedEdges) {
                            if (result.count(reroutedLayout.id)) {
                                result[reroutedLayout.id] = reroutedLayout;
                            }
                            assignedLayouts[reroutedLayout.id] = reroutedLayout;
                        }
                    } else {
                        failedEdges.push_back(edgeId);
                        result[edgeId] = baseLayout;
                        assignedLayouts[edgeId] = baseLayout;
                    }
                    continue;
                }

                const auto& best = combinations.front();
                if (best.score > 0) anyOverlapFound = true;
                result[edgeId] = best.layout;
                assignedLayouts[edgeId] = best.layout;
            }

            // Detect and resolve overlaps (not just for failed edges)
            auto overlappingPairs = detectOverlaps(result);
            LOG_DEBUG("[AStarOpt] PHASE 2: detectOverlaps found {} pairs (pass={}, sequential=true)",
                      overlappingPairs.size(), pass);
            for (const auto& [a, b] : overlappingPairs) {
                LOG_DEBUG("[AStarOpt]   Overlap: Edge {} & Edge {}", a, b);
            }
            if (!overlappingPairs.empty()) {
                anyOverlapFound = true;
                resolveAllOverlaps(overlappingPairs, result, assignedLayouts,
                                   nodeLayouts, forbiddenZones, nullptr);
            }

            if (!anyOverlapFound) break;
        }
    }

    return result;
}

std::vector<AStarEdgeOptimizer::CombinationResult>
AStarEdgeOptimizer::evaluateCombinations(
    EdgeId /*edgeId*/,
    const EdgeLayout& baseLayout,
    const std::unordered_map<EdgeId, EdgeLayout>& assignedLayouts,
    const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
    const std::vector<ForbiddenZone>& forbiddenZones) {

    std::vector<CombinationResult> results;
    results.reserve(16);

    // Build obstacle map once for all combinations (more efficient)
    // Include edge layouts in bounds calculation to prevent out-of-bounds segments
    float gridSize = effectiveGridSize();
    ObstacleMap obstacles;
    obstacles.buildFromNodes(nodeLayouts, gridSize, 0, &assignedLayouts);

    // Add OTHER edge segments as obstacles so A* finds paths that avoid them
    obstacles.addEdgeSegments(assignedLayouts, baseLayout.id);

    // Lambda to evaluate a single edge combination
    auto evaluateEdge = [&](NodeEdge srcEdge, NodeEdge tgtEdge) {
        // First, try keeping original bendPoints if same source/target edges
        // AND snap points are at the same location (within tolerance)
        constexpr float SNAP_TOLERANCE = 5.0f;
        bool sameSnapPoints = false;
        
        if (srcEdge == baseLayout.sourceEdge && tgtEdge == baseLayout.targetEdge &&
            !baseLayout.bendPoints.empty()) {
            
            // Check if snap points match the original layout
            // (They may have changed due to redistribution)
            auto srcNodeIt = nodeLayouts.find(baseLayout.from);
            auto tgtNodeIt = nodeLayouts.find(baseLayout.to);
            if (srcNodeIt != nodeLayouts.end() && tgtNodeIt != nodeLayouts.end()) {
                // Snap points should be close to the original source/target points
                // If they've moved significantly, the original path won't work
                Point currentSrc = baseLayout.sourcePoint;
                Point currentTgt = baseLayout.targetPoint;
                
                // Check if first bendpoint is close to source (path starts correctly)
                // and last bendpoint is close to target (path ends correctly)
                if (baseLayout.bendPoints.size() >= 1) {
                    const Point& firstBend = baseLayout.bendPoints.front().position;
                    const Point& lastBend = baseLayout.bendPoints.back().position;
                    
                    // For orthogonal paths, check axis alignment
                    bool srcAligned = false;
                    bool tgtAligned = false;
                    
                    // Source to first bend should be orthogonal from source edge
                    if (srcEdge == NodeEdge::Top || srcEdge == NodeEdge::Bottom) {
                        srcAligned = std::abs(currentSrc.x - firstBend.x) < SNAP_TOLERANCE;
                    } else {
                        srcAligned = std::abs(currentSrc.y - firstBend.y) < SNAP_TOLERANCE;
                    }
                    
                    // Last bend to target should be orthogonal to target edge
                    if (tgtEdge == NodeEdge::Top || tgtEdge == NodeEdge::Bottom) {
                        tgtAligned = std::abs(currentTgt.x - lastBend.x) < SNAP_TOLERANCE;
                    } else {
                        tgtAligned = std::abs(currentTgt.y - lastBend.y) < SNAP_TOLERANCE;
                    }
                    
                    sameSnapPoints = srcAligned && tgtAligned;
                }
            }
        }
        
        if (sameSnapPoints) {
            PenaltyContext ctx{assignedLayouts, nodeLayouts, forbiddenZones, effectiveGridSize()};
            ctx.sourceNodeId = baseLayout.from;
            ctx.targetNodeId = baseLayout.to;
            ctx.originalLayouts = originalLayouts_;
            ctx.movedNodes = movedNodes_;
            
            if (passesHardConstraints(baseLayout, ctx)) {
                int score = calculatePenalty(baseLayout, ctx);
                results.push_back({srcEdge, tgtEdge, score, baseLayout, true});
                return;  // Use original path, no need to generate new one
            }
        }

        // Generate new path with A*
        bool pathFound = false;
        EdgeLayout candidate = createCandidateLayout(
            baseLayout, srcEdge, tgtEdge, nodeLayouts, obstacles, pathFound);

        // Skip invalid candidates (node not found or no valid path)
        if (!pathFound) {
            return;
        }

        // Calculate score using unified penalty system
        PenaltyContext ctx{assignedLayouts, nodeLayouts, forbiddenZones, effectiveGridSize()};
        ctx.sourceNodeId = candidate.from;
        ctx.targetNodeId = candidate.to;
        ctx.originalLayouts = originalLayouts_;
        ctx.movedNodes = movedNodes_;

        // Check hard constraints first
        if (!passesHardConstraints(candidate, ctx)) {
            return;
        }

        int score = calculatePenalty(candidate, ctx);
        results.push_back({srcEdge, tgtEdge, score, std::move(candidate), true});
    };

    if (preserveDirections()) {
        // Preserve existing direction - only evaluate current combination
        evaluateEdge(baseLayout.sourceEdge, baseLayout.targetEdge);
    } else {
        // Evaluate all combinations - FixedEndpointPenalty will handle constraint scoring
        // Optimize directions with smart ordering based on node positions
        // Threshold for early return: score below this means acceptable path found
        constexpr int EARLY_RETURN_SCORE_THRESHOLD = 100;
        
        // Get node centers to determine relative positions
        auto srcNodeIt = nodeLayouts.find(baseLayout.from);
        auto tgtNodeIt = nodeLayouts.find(baseLayout.to);
        
        // Use a bitmask for O(1) duplicate checking (4 src * 4 tgt = 16 bits)
        // Bit index = srcEdge * 4 + tgtEdge
        auto edgeToIndex = [](NodeEdge e) -> int {
            switch (e) {
                case NodeEdge::Top: return 0;
                case NodeEdge::Bottom: return 1;
                case NodeEdge::Left: return 2;
                case NodeEdge::Right: return 3;
                default: return 0;
            }
        };
        uint16_t addedMask = 0;
        
        auto tryAddCombo = [&](NodeEdge src, NodeEdge tgt) {
            int bit = edgeToIndex(src) * 4 + edgeToIndex(tgt);
            if (!(addedMask & (1 << bit))) {
                addedMask |= (1 << bit);
                evaluateEdge(src, tgt);
                return !results.empty() && results.back().score < EARLY_RETURN_SCORE_THRESHOLD;
            }
            return false;
        };
        
        // 1. Always try current direction first (might already be optimal)
        if (tryAddCombo(baseLayout.sourceEdge, baseLayout.targetEdge)) {
            goto done_evaluating;
        }
        
        // 2. Try position-based optimal directions
        if (srcNodeIt != nodeLayouts.end() && tgtNodeIt != nodeLayouts.end()) {
            Point srcCenter = srcNodeIt->second.center();
            Point tgtCenter = tgtNodeIt->second.center();
            
            float dx = tgtCenter.x - srcCenter.x;
            float dy = tgtCenter.y - srcCenter.y;
            
            // Default directions for edge case where dx == dy == 0
            NodeEdge primarySrc = NodeEdge::Bottom;
            NodeEdge primaryTgt = NodeEdge::Top;
            NodeEdge secondarySrc = NodeEdge::Right;
            NodeEdge secondaryTgt = NodeEdge::Left;
            
            // Determine primary and secondary directions based on relative position
            if (std::abs(dx) > std::abs(dy)) {
                // Horizontal dominant
                primarySrc = (dx > 0) ? NodeEdge::Right : NodeEdge::Left;
                primaryTgt = (dx > 0) ? NodeEdge::Left : NodeEdge::Right;
                secondarySrc = (dy > 0) ? NodeEdge::Bottom : NodeEdge::Top;
                secondaryTgt = (dy > 0) ? NodeEdge::Top : NodeEdge::Bottom;
            } else if (std::abs(dy) > 0.1f) {
                // Vertical dominant (with tolerance for near-zero)
                primarySrc = (dy > 0) ? NodeEdge::Bottom : NodeEdge::Top;
                primaryTgt = (dy > 0) ? NodeEdge::Top : NodeEdge::Bottom;
                secondarySrc = (dx > 0) ? NodeEdge::Right : NodeEdge::Left;
                secondaryTgt = (dx > 0) ? NodeEdge::Left : NodeEdge::Right;
            }
            // else: use defaults for overlapping nodes
            
            // Try high-priority combinations based on relative position
            if (tryAddCombo(primarySrc, primaryTgt)) goto done_evaluating;
            if (tryAddCombo(secondarySrc, secondaryTgt)) goto done_evaluating;
            if (tryAddCombo(primarySrc, secondaryTgt)) goto done_evaluating;
            if (tryAddCombo(secondarySrc, primaryTgt)) goto done_evaluating;
        }
        
        // 3. Try remaining combinations
        {
            constexpr std::array<NodeEdge, 4> allEdges = {
                NodeEdge::Top, NodeEdge::Bottom, NodeEdge::Left, NodeEdge::Right
            };
            for (NodeEdge srcEdge : allEdges) {
                for (NodeEdge tgtEdge : allEdges) {
                    if (tryAddCombo(srcEdge, tgtEdge)) {
                        goto done_evaluating;
                    }
                }
            }
        }
        done_evaluating:;
    }

    // Sort by score (ascending - lower is better)
    // Use stable_sort with deterministic tie-breaker to prevent flickering
    std::stable_sort(results.begin(), results.end(),
              [](const CombinationResult& a, const CombinationResult& b) {
                  if (a.score != b.score) return a.score < b.score;
                  // Tie-breaker: deterministic order by edge combination
                  if (a.sourceEdge != b.sourceEdge) return a.sourceEdge < b.sourceEdge;
                  return a.targetEdge < b.targetEdge;
              });

    return results;
}

// Thread-safe version with explicit pathfinder
std::vector<AStarEdgeOptimizer::CombinationResult>
AStarEdgeOptimizer::evaluateCombinations(
    EdgeId /*edgeId*/,
    const EdgeLayout& baseLayout,
    const std::unordered_map<EdgeId, EdgeLayout>& assignedLayouts,
    const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
    const std::vector<ForbiddenZone>& forbiddenZones,
    IPathFinder& pathFinder) {

    std::vector<CombinationResult> results;
    results.reserve(16);

    // Build obstacle map once for all combinations (more efficient)
    // Include edge layouts in bounds calculation to prevent out-of-bounds segments
    float gridSize = effectiveGridSize();
    ObstacleMap obstacles;
    obstacles.buildFromNodes(nodeLayouts, gridSize, 0, &assignedLayouts);

    // Add OTHER edge segments as obstacles so A* finds paths that avoid them
    obstacles.addEdgeSegments(assignedLayouts, baseLayout.id);

    // Lambda to evaluate a single edge combination
    auto evaluateEdge = [&](NodeEdge srcEdge, NodeEdge tgtEdge) {
        // First, try keeping original bendPoints if same source/target edges
        // AND snap points are at the same location (within tolerance)
        constexpr float SNAP_TOLERANCE = 5.0f;
        bool sameSnapPoints = false;
        
        if (srcEdge == baseLayout.sourceEdge && tgtEdge == baseLayout.targetEdge &&
            !baseLayout.bendPoints.empty()) {
            
            // Check if snap points match the original layout
            auto srcNodeIt = nodeLayouts.find(baseLayout.from);
            auto tgtNodeIt = nodeLayouts.find(baseLayout.to);
            if (srcNodeIt != nodeLayouts.end() && tgtNodeIt != nodeLayouts.end()) {
                Point currentSrc = baseLayout.sourcePoint;
                Point currentTgt = baseLayout.targetPoint;
                
                if (baseLayout.bendPoints.size() >= 1) {
                    const Point& firstBend = baseLayout.bendPoints.front().position;
                    const Point& lastBend = baseLayout.bendPoints.back().position;
                    
                    bool srcAligned = false;
                    bool tgtAligned = false;
                    
                    if (srcEdge == NodeEdge::Top || srcEdge == NodeEdge::Bottom) {
                        srcAligned = std::abs(currentSrc.x - firstBend.x) < SNAP_TOLERANCE;
                    } else {
                        srcAligned = std::abs(currentSrc.y - firstBend.y) < SNAP_TOLERANCE;
                    }
                    
                    if (tgtEdge == NodeEdge::Top || tgtEdge == NodeEdge::Bottom) {
                        tgtAligned = std::abs(currentTgt.x - lastBend.x) < SNAP_TOLERANCE;
                    } else {
                        tgtAligned = std::abs(currentTgt.y - lastBend.y) < SNAP_TOLERANCE;
                    }
                    
                    sameSnapPoints = srcAligned && tgtAligned;
                }
            }
        }
        
        if (sameSnapPoints) {
            PenaltyContext ctx{assignedLayouts, nodeLayouts, forbiddenZones, effectiveGridSize()};
            ctx.sourceNodeId = baseLayout.from;
            ctx.targetNodeId = baseLayout.to;
            ctx.originalLayouts = originalLayouts_;
            ctx.movedNodes = movedNodes_;
            
            if (passesHardConstraints(baseLayout, ctx)) {
                int score = calculatePenalty(baseLayout, ctx);
                results.push_back({srcEdge, tgtEdge, score, baseLayout, true});
                return;  // Use original path
            }
        }

        // Generate new path with A* using provided pathfinder
        bool pathFound = false;
        EdgeLayout candidate = createCandidateLayout(
            baseLayout, srcEdge, tgtEdge, nodeLayouts, obstacles, pathFound, pathFinder);

        if (!pathFound) {
            return;
        }

        // Calculate score using unified penalty system
        PenaltyContext ctx{assignedLayouts, nodeLayouts, forbiddenZones, effectiveGridSize()};
        ctx.sourceNodeId = candidate.from;
        ctx.targetNodeId = candidate.to;
        ctx.originalLayouts = originalLayouts_;
        ctx.movedNodes = movedNodes_;

        if (!passesHardConstraints(candidate, ctx)) {
            return;
        }

        int score = calculatePenalty(candidate, ctx);
        results.push_back({srcEdge, tgtEdge, score, std::move(candidate), true});
    };

    if (preserveDirections()) {
        // Preserve existing direction - only evaluate current combination
        evaluateEdge(baseLayout.sourceEdge, baseLayout.targetEdge);
    } else {
        // Evaluate all combinations - FixedEndpointPenalty will handle constraint scoring
        constexpr int EARLY_RETURN_SCORE_THRESHOLD = 100;
        
        auto srcNodeIt = nodeLayouts.find(baseLayout.from);
        auto tgtNodeIt = nodeLayouts.find(baseLayout.to);
        
        auto edgeToIndex = [](NodeEdge e) -> int {
            switch (e) {
                case NodeEdge::Top: return 0;
                case NodeEdge::Bottom: return 1;
                case NodeEdge::Left: return 2;
                case NodeEdge::Right: return 3;
                default: return 0;
            }
        };
        uint16_t addedMask = 0;
        
        auto tryAddCombo = [&](NodeEdge src, NodeEdge tgt) {
            int bit = edgeToIndex(src) * 4 + edgeToIndex(tgt);
            if (!(addedMask & (1 << bit))) {
                addedMask |= (1 << bit);
                evaluateEdge(src, tgt);
                return !results.empty() && results.back().score < EARLY_RETURN_SCORE_THRESHOLD;
            }
            return false;
        };
        
        if (tryAddCombo(baseLayout.sourceEdge, baseLayout.targetEdge)) {
            goto done_evaluating;
        }
        
        if (srcNodeIt != nodeLayouts.end() && tgtNodeIt != nodeLayouts.end()) {
            Point srcCenter = srcNodeIt->second.center();
            Point tgtCenter = tgtNodeIt->second.center();
            
            float dx = tgtCenter.x - srcCenter.x;
            float dy = tgtCenter.y - srcCenter.y;
            
            NodeEdge primarySrc = NodeEdge::Bottom;
            NodeEdge primaryTgt = NodeEdge::Top;
            NodeEdge secondarySrc = NodeEdge::Right;
            NodeEdge secondaryTgt = NodeEdge::Left;
            
            if (std::abs(dx) > std::abs(dy)) {
                primarySrc = (dx > 0) ? NodeEdge::Right : NodeEdge::Left;
                primaryTgt = (dx > 0) ? NodeEdge::Left : NodeEdge::Right;
                secondarySrc = (dy > 0) ? NodeEdge::Bottom : NodeEdge::Top;
                secondaryTgt = (dy > 0) ? NodeEdge::Top : NodeEdge::Bottom;
            } else if (std::abs(dy) > 0.1f) {
                primarySrc = (dy > 0) ? NodeEdge::Bottom : NodeEdge::Top;
                primaryTgt = (dy > 0) ? NodeEdge::Top : NodeEdge::Bottom;
                secondarySrc = (dx > 0) ? NodeEdge::Right : NodeEdge::Left;
                secondaryTgt = (dx > 0) ? NodeEdge::Left : NodeEdge::Right;
            }
            
            if (tryAddCombo(primarySrc, primaryTgt)) goto done_evaluating;
            if (tryAddCombo(secondarySrc, secondaryTgt)) goto done_evaluating;
            if (tryAddCombo(primarySrc, secondaryTgt)) goto done_evaluating;
            if (tryAddCombo(secondarySrc, primaryTgt)) goto done_evaluating;
        }
        
        {
            constexpr std::array<NodeEdge, 4> allEdges = {
                NodeEdge::Top, NodeEdge::Bottom, NodeEdge::Left, NodeEdge::Right
            };
            for (NodeEdge srcEdge : allEdges) {
                for (NodeEdge tgtEdge : allEdges) {
                    if (tryAddCombo(srcEdge, tgtEdge)) {
                        goto done_evaluating;
                    }
                }
            }
        }
        done_evaluating:;
    }

    std::stable_sort(results.begin(), results.end(),
              [](const CombinationResult& a, const CombinationResult& b) {
                  if (a.score != b.score) return a.score < b.score;
                  if (a.sourceEdge != b.sourceEdge) return a.sourceEdge < b.sourceEdge;
                  return a.targetEdge < b.targetEdge;
              });

    return results;
}

EdgeLayout AStarEdgeOptimizer::createCandidateLayout(
    const EdgeLayout& base,
    NodeEdge sourceEdge,
    NodeEdge targetEdge,
    const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
    ObstacleMap& obstacles,
    bool& pathFound) {
    // Delegate to thread-safe version with member pathfinder
    return createCandidateLayout(base, sourceEdge, targetEdge, nodeLayouts, obstacles, pathFound, *pathFinder_);
}

EdgeLayout AStarEdgeOptimizer::createCandidateLayout(
    const EdgeLayout& base,
    NodeEdge sourceEdge,
    NodeEdge targetEdge,
    const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
    ObstacleMap& obstacles,
    bool& pathFound,
    IPathFinder& pathFinder) {

    pathFound = false;

    EdgeLayout candidate = base;
    candidate.sourceEdge = sourceEdge;
    candidate.targetEdge = targetEdge;
    candidate.bendPoints.clear();

    // NOTE: snapIndex is no longer stored - computed from position as needed

    // Get node layouts
    auto srcIt = nodeLayouts.find(base.from);
    if (srcIt == nodeLayouts.end()) {
        return candidate;
    }

    auto tgtIt = nodeLayouts.find(base.to);
    if (tgtIt == nodeLayouts.end()) {
        return candidate;
    }

    float gridSize = effectiveGridSize();

    // === Fixed Endpoint Optimization ===
    // This is a PERFORMANCE OPTIMIZATION that works with FixedEndpointPenalty:
    // - Here: Preserve position/snapIndex early to avoid generating invalid candidates
    // - FixedEndpointPenalty: Final enforcement that rejects any violations
    // Both check: (1) node is fixed, (2) edge matches original
    // If edge differs, new position is generated and FixedEndpointPenalty will reject it
    bool srcFixed = isNodeFixed(base.from);
    bool tgtFixed = isNodeFixed(base.to);

    GridPoint startGrid, goalGrid;
    
    // === SOURCE SNAP POSITION ===
    // Rule: Same NodeEdge → preserve SnapDistributor's position (respect distributed candidateIdx)
    //       Different NodeEdge → compute center position for new edge
    if (sourceEdge == base.sourceEdge) {
        // Same NodeEdge - preserve existing position (SnapDistributor already distributed)
        if (srcFixed) {
            // Node is fixed - position unchanged
            candidate.sourcePoint = base.sourcePoint;
        } else {
            // Node moved - recalculate position preserving candidateIdx
            int candidateIdx = GridSnapCalculator::getCandidateIndexFromPosition(
                srcIt->second, base.sourceEdge, base.sourcePoint, gridSize);
            // Recompute position at same candidateIdx (accounts for node movement)
            candidate.sourcePoint = GridSnapCalculator::getPositionFromCandidateIndex(
                srcIt->second, sourceEdge, candidateIdx, gridSize);
        }
        startGrid = {
            static_cast<int>(std::round(candidate.sourcePoint.x / gridSize)),
            static_cast<int>(std::round(candidate.sourcePoint.y / gridSize))
        };
    } else {
        // Different NodeEdge - compute center position (grid-aligned)
        // Use center candidateIdx for new edge
        int candidateCount = GridSnapCalculator::getCandidateCount(srcIt->second, sourceEdge, gridSize);
        int centerIdx = candidateCount > 0 ? candidateCount / 2 : 0;
        candidate.sourcePoint = GridSnapCalculator::getPositionFromCandidateIndex(
            srcIt->second, sourceEdge, centerIdx, gridSize);
        startGrid = obstacles.pixelToGrid(candidate.sourcePoint);
    }
    
    // === TARGET SNAP POSITION ===
    if (targetEdge == base.targetEdge) {
        // Same NodeEdge - preserve existing position
        if (tgtFixed) {
            candidate.targetPoint = base.targetPoint;
        } else {
            int candidateIdx = GridSnapCalculator::getCandidateIndexFromPosition(
                tgtIt->second, base.targetEdge, base.targetPoint, gridSize);
            candidate.targetPoint = GridSnapCalculator::getPositionFromCandidateIndex(
                tgtIt->second, targetEdge, candidateIdx, gridSize);
        }
        goalGrid = {
            static_cast<int>(std::round(candidate.targetPoint.x / gridSize)),
            static_cast<int>(std::round(candidate.targetPoint.y / gridSize))
        };
    } else {
        // Different NodeEdge - compute center position (grid-aligned)
        int candidateCount = GridSnapCalculator::getCandidateCount(tgtIt->second, targetEdge, gridSize);
        int centerIdx = candidateCount > 0 ? candidateCount / 2 : 0;
        candidate.targetPoint = GridSnapCalculator::getPositionFromCandidateIndex(
            tgtIt->second, targetEdge, centerIdx, gridSize);
        goalGrid = obstacles.pixelToGrid(candidate.targetPoint);
    }

    // Use provided pathfinder (thread-safe)
    LOG_DEBUG("[CALLER:AStarEdgeOptimizer.cpp] A* findPath called");
    PathResult pathResult = pathFinder.findPath(
        startGrid, goalGrid, obstacles,
        base.from, base.to,
        sourceEdge, targetEdge,
        {}, {});

    if (!pathResult.found || pathResult.path.size() < 2) {
        return candidate;
    }

    for (size_t i = 1; i + 1 < pathResult.path.size(); ++i) {
        Point pixelPoint = obstacles.gridToPixel(
            pathResult.path[i].x, pathResult.path[i].y);
        candidate.bendPoints.push_back({pixelPoint});
    }

    pathFound = true;
    return candidate;
}

Point AStarEdgeOptimizer::calculateSnapPosition(
    const NodeLayout& node,
    NodeEdge edge,
    float position) {

    switch (edge) {
        case NodeEdge::Top:
            return {
                node.position.x + node.size.width * position,
                node.position.y
            };
        case NodeEdge::Bottom:
            return {
                node.position.x + node.size.width * position,
                node.position.y + node.size.height
            };
        case NodeEdge::Left:
            return {
                node.position.x,
                node.position.y + node.size.height * position
            };
        case NodeEdge::Right:
            return {
                node.position.x + node.size.width,
                node.position.y + node.size.height * position
            };
    }

    return node.center();
}

GridPoint AStarEdgeOptimizer::calculateGridPosition(
    const NodeLayout& node,
    NodeEdge edge,
    float gridSize,
    float position) {

    // Calculate pixel position at edge center
    float px, py;
    
    switch (edge) {
        case NodeEdge::Top:
            px = node.position.x + node.size.width * position;
            py = node.position.y;
            break;
        case NodeEdge::Bottom:
            px = node.position.x + node.size.width * position;
            py = node.position.y + node.size.height;
            break;
        case NodeEdge::Left:
            px = node.position.x;
            py = node.position.y + node.size.height * position;
            break;
        case NodeEdge::Right:
            px = node.position.x + node.size.width;
            py = node.position.y + node.size.height * position;
            break;
        default:
            px = node.position.x + node.size.width * 0.5f;
            py = node.position.y + node.size.height * 0.5f;
            break;
    }

    // Convert to grid coordinates (round to nearest grid vertex)
    return {
        static_cast<int>(std::round(px / gridSize)),
        static_cast<int>(std::round(py / gridSize))
    };
}

std::vector<ForbiddenZone> AStarEdgeOptimizer::calculateForbiddenZones(
    const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
    float gridSize) const {

    const float baseMargin = constants::MIN_NODE_GRID_DISTANCE * gridSize;
    std::vector<ForbiddenZone> zones;
    zones.reserve(nodeLayouts.size());

    for (const auto& [nodeId, node] : nodeLayouts) {
        ForbiddenZone zone;
        zone.blockedBy = nodeId;
        zone.bounds = {
            node.position.x - baseMargin,
            node.position.y - baseMargin,
            node.size.width + 2 * baseMargin,
            node.size.height + 2 * baseMargin
        };
        zones.push_back(zone);
    }

    return zones;
}

// =============================================================================
// BendPoints Regeneration (preserves sourceEdge/targetEdge)
// =============================================================================

void AStarEdgeOptimizer::regenerateBendPoints(
    const std::vector<EdgeId>& edges,
    std::unordered_map<EdgeId, EdgeLayout>& edgeLayouts,
    const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
    float gridSizeParam) {

    if (edges.empty()) {
        return;
    }

    // Store gridSize for use in helper methods
    gridSize_ = gridSizeParam;
    float gridSize = effectiveGridSize();

    // Track already-routed edges for blocking
    std::unordered_map<EdgeId, EdgeLayout> routedEdges;

    // Build base obstacle map once (from nodes only) - PERFORMANCE OPTIMIZATION
    // This avoids rebuilding the expensive node-based grid for each edge
    ObstacleMap baseObstacles;
    baseObstacles.buildFromNodes(nodeLayouts, gridSize, 0, &edgeLayouts);

    // Phase 1: Route edges sequentially, each sees previously-routed edges as blocking
    for (EdgeId edgeId : edges) {
        auto it = edgeLayouts.find(edgeId);
        if (it == edgeLayouts.end()) {
            continue;
        }

        EdgeLayout& layout = it->second;

        // Handle self-loops: use geometric routing for valid adjacent edge combinations,
        // fall through to A* for invalid combinations (opposite edges, same edge)
        if (layout.from == layout.to && 
            SelfLoopRouter::isValidSelfLoopCombination(layout.sourceEdge, layout.targetEdge)) {
            auto nodeIt = nodeLayouts.find(layout.from);
            if (nodeIt == nodeLayouts.end()) {
                continue;
            }
            const NodeLayout& selfNode = nodeIt->second;
            
            constexpr float BASE_OFFSET = 30.0f;
            
            // Calculate extension points perpendicular to each edge
            Point srcExt, tgtExt;
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
            srcExt.x = std::round(srcExt.x / gridSize) * gridSize;
            srcExt.y = std::round(srcExt.y / gridSize) * gridSize;
            tgtExt.x = std::round(tgtExt.x / gridSize) * gridSize;
            tgtExt.y = std::round(tgtExt.y / gridSize) * gridSize;
            
            layout.bendPoints.clear();
            
            // Check if corner point is needed (srcExt and tgtExt not aligned)
            constexpr float EPSILON = 1.0f;
            if (std::abs(srcExt.x - tgtExt.x) > EPSILON && std::abs(srcExt.y - tgtExt.y) > EPSILON) {
                // Need corner - pick the one outside the node
                Point cornerA = {srcExt.x, tgtExt.y};
                Point cornerB = {tgtExt.x, srcExt.y};
                
                float nodeLeft = selfNode.position.x;
                float nodeRight = selfNode.position.x + selfNode.size.width;
                float nodeTop = selfNode.position.y;
                float nodeBottom = selfNode.position.y + selfNode.size.height;
                
                bool cornerAInside = (cornerA.x >= nodeLeft && cornerA.x <= nodeRight &&
                                      cornerA.y >= nodeTop && cornerA.y <= nodeBottom);
                
                Point corner = cornerAInside ? cornerB : cornerA;
                
                layout.bendPoints.push_back({srcExt});
                layout.bendPoints.push_back({corner});
                layout.bendPoints.push_back({tgtExt});
            } else {
                layout.bendPoints.push_back({srcExt});
                layout.bendPoints.push_back({tgtExt});
            }
            
            // Add to routed edges and continue
            routedEdges[edgeId] = layout;
            continue;
        }

        // Copy base obstacles and add edge segments - PERFORMANCE OPTIMIZATION
        // This reuses the expensive node-based grid built once before the loop
        ObstacleMap obstacles = baseObstacles;

        // Add all edges from edgeLayouts as obstacles (excluding current edge)
        // This ensures we avoid overlapping with edges that are NOT being re-routed
        obstacles.addEdgeSegments(edgeLayouts, edgeId);

        // Also add already-routed edges from current call (they may have updated paths)
        if (!routedEdges.empty()) {
            obstacles.addEdgeSegments(routedEdges, edgeId);
        }

        // Use UnifiedRetryChain for retry sequence
        // IMPORTANT: regenerateBendPoints must preserve sourceEdge/targetEdge/sourcePoint/targetPoint
        // Only bendPoints should be recalculated
        auto& retryChain = getRetryChain(gridSize);

        UnifiedRetryChain::RetryConfig config;
        config.maxSnapRetries = 9;
        config.maxNodeEdgeCombinations = 0;  // Disable NodeEdge switching for regenerate
        config.enableCooperativeReroute = true;
        config.gridSize = gridSize;
        config.movedNodes = movedNodes_.empty() ? nullptr : &movedNodes_;

        // Create mutable copy of edgeLayouts for retry chain
        std::unordered_map<EdgeId, EdgeLayout> otherEdges = edgeLayouts;
        // Merge routedEdges into otherEdges
        for (const auto& [id, routedLayout] : routedEdges) {
            otherEdges[id] = routedLayout;
        }

        LOG_DEBUG("[AStarOpt::regenerateBendPoints] Edge {} BEFORE retryChain: src=({},{}) tgt=({},{}) srcEdge={} tgtEdge={}",
                  edgeId, layout.sourcePoint.x, layout.sourcePoint.y,
                  layout.targetPoint.x, layout.targetPoint.y,
                  static_cast<int>(layout.sourceEdge), static_cast<int>(layout.targetEdge));

        auto result = retryChain.calculatePath(
            edgeId, layout, otherEdges, nodeLayouts, config);

        LOG_DEBUG("[AStarOpt::regenerateBendPoints] Edge {} retryChain result: success={} bendPoints={}",
                  edgeId, result.success, result.layout.bendPoints.size());
        if (result.success) {
            for (size_t i = 0; i < result.layout.bendPoints.size(); ++i) {
                LOG_DEBUG("  result.bend[{}]=({},{})", i,
                          result.layout.bendPoints[i].position.x,
                          result.layout.bendPoints[i].position.y);
            }
        }

        if (result.success) {
            // Update bendPoints and snap positions (may be changed by retry chain)
            layout.bendPoints = result.layout.bendPoints;
            layout.sourcePoint = result.layout.sourcePoint;
            layout.targetPoint = result.layout.targetPoint;
            // NOTE: snapIndex is no longer stored - computed from position as needed
            layout.usedGridSize = effectiveGridSize();  // Single Source of Truth

            // Update rerouted edges back to edgeLayouts (only bendPoints)
            for (const auto& reroutedLayout : result.reroutedEdges) {
                auto it = edgeLayouts.find(reroutedLayout.id);
                if (it != edgeLayouts.end()) {
                    it->second.bendPoints = reroutedLayout.bendPoints;
                }
                auto rit = routedEdges.find(reroutedLayout.id);
                if (rit != routedEdges.end()) {
                    rit->second.bendPoints = reroutedLayout.bendPoints;
                }
            }
        } else {
            LOG_DEBUG("[AStarOpt::regenerateBendPoints] Edge {} retryChain FAILED, keeping existing bendPoints={}",
                      edgeId, layout.bendPoints.size());
            for (size_t i = 0; i < layout.bendPoints.size(); ++i) {
                LOG_DEBUG("  existing.bend[{}]=({},{})", i,
                          layout.bendPoints[i].position.x,
                          layout.bendPoints[i].position.y);
            }
        }
        // If retry chain fails, keep existing bendPoints

        // Add this edge to routed edges for blocking subsequent edges
        routedEdges[edgeId] = layout;
    }

    // Phase 2: Rip-up and Reroute for overlapping edges
    constexpr int MAX_RIP_UP_ITERATIONS = 3;

    for (int iteration = 0; iteration < MAX_RIP_UP_ITERATIONS; ++iteration) {
        // Find edges that overlap with others
        // IMPORTANT: Check ALL edges in edgeLayouts, not just input edges
        // This catches overlaps when snap sync changes a path to overlap with
        // an edge that wasn't in the original reroute list
        std::vector<EdgeId> dirtyEdges;
        std::unordered_set<EdgeId> dirtySet;  // For deduplication
        
        for (const auto& [edgeId, layout] : edgeLayouts) {
            if (layout.from == layout.to) {
                continue;  // Skip self-loops
            }

            // Check if this edge overlaps with any other assigned edge
            if (PathIntersection::hasOverlapWithOthers(layout, edgeLayouts, edgeId)) {
                if (dirtySet.insert(edgeId).second) {
                    dirtyEdges.push_back(edgeId);
                }
            }
        }

        if (dirtyEdges.empty()) {
            LOG_DEBUG("[AStarOpt::regenerateBendPoints] Phase 2 iteration {}: No overlaps found", iteration);
            break;  // No overlaps, we're done
        }

        LOG_DEBUG("[AStarOpt::regenerateBendPoints] Phase 2 iteration {}: Found {} dirty edges", iteration, dirtyEdges.size());
        for (EdgeId dId : dirtyEdges) {
            LOG_DEBUG("[AStarOpt::regenerateBendPoints] Phase 2: dirty edge {}", dId);
        }

        // Re-route dirty edges using UnifiedRetryChain
        for (EdgeId dirtyId : dirtyEdges) {
            auto it = edgeLayouts.find(dirtyId);
            if (it == edgeLayouts.end()) {
                continue;
            }

            EdgeLayout& layout = it->second;

            // Use UnifiedRetryChain for full retry sequence
            auto& retryChain = getRetryChain(gridSize);

            UnifiedRetryChain::RetryConfig config;
            config.maxSnapRetries = 9;
            config.maxNodeEdgeCombinations = 16;
            config.enableCooperativeReroute = true;
            config.gridSize = gridSize;
            config.movedNodes = movedNodes_.empty() ? nullptr : &movedNodes_;

            auto result = retryChain.calculatePath(
                dirtyId, layout, edgeLayouts, nodeLayouts, config);

            if (result.success) {
                layout = result.layout;

                // Update rerouted edges
                for (const auto& reroutedLayout : result.reroutedEdges) {
                    edgeLayouts[reroutedLayout.id] = reroutedLayout;
                }

            }
        }
    }
}

AStarEdgeOptimizer::EdgePairResult AStarEdgeOptimizer::tryPathAdjustmentFallback(
    EdgeId edgeIdA, EdgeId edgeIdB,
    const EdgeLayout& layoutA, const EdgeLayout& layoutB,
    const std::unordered_map<EdgeId, EdgeLayout>& otherLayouts,
    const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
    const std::vector<ForbiddenZone>& forbiddenZones,
    float gridSize) {

    EdgePairResult result;
    result.valid = false;

    // Strategy 1: Try adjusting layoutB's path to avoid overlap with layoutA
    {
        std::unordered_map<EdgeId, EdgeLayout> withA;
        withA[edgeIdA] = layoutA;
        auto adjustedBendsB = PathIntersection::adjustPathToAvoidOverlap(layoutB, withA, gridSize);

        EdgeLayout adjustedB = layoutB;
        adjustedB.bendPoints = adjustedBendsB;

        if (!PathIntersection::hasSegmentOverlap(layoutA, adjustedB)) {
            PenaltyContext ctxA{otherLayouts, nodeLayouts, forbiddenZones, gridSize};
            ctxA.sourceNodeId = layoutA.from;
            ctxA.targetNodeId = layoutA.to;
            ctxA.originalLayouts = originalLayouts_;
            ctxA.movedNodes = movedNodes_;
            int scoreA = calculatePenalty(layoutA, ctxA);

            std::unordered_map<EdgeId, EdgeLayout> withALayout = otherLayouts;
            withALayout[edgeIdA] = layoutA;
            PenaltyContext ctxB{withALayout, nodeLayouts, forbiddenZones, gridSize};
            ctxB.sourceNodeId = adjustedB.from;
            ctxB.targetNodeId = adjustedB.to;
            ctxB.originalLayouts = originalLayouts_;
            ctxB.movedNodes = movedNodes_;
            int scoreB = calculatePenalty(adjustedB, ctxB);

            result.layoutA = layoutA;
            result.layoutB = adjustedB;
            result.combinedScore = scoreA + scoreB;
            result.valid = true;
            return result;
        }
    }

    // Strategy 2: Try adjusting layoutA's path instead
    {
        std::unordered_map<EdgeId, EdgeLayout> withB;
        withB[edgeIdB] = layoutB;
        auto adjustedBendsA = PathIntersection::adjustPathToAvoidOverlap(layoutA, withB, gridSize);

        EdgeLayout adjustedA = layoutA;
        adjustedA.bendPoints = adjustedBendsA;

        if (!PathIntersection::hasSegmentOverlap(adjustedA, layoutB)) {
            std::unordered_map<EdgeId, EdgeLayout> withAdjA = otherLayouts;
            withAdjA[edgeIdA] = adjustedA;
            PenaltyContext ctxA{otherLayouts, nodeLayouts, forbiddenZones, gridSize};
            ctxA.sourceNodeId = adjustedA.from;
            ctxA.targetNodeId = adjustedA.to;
            ctxA.originalLayouts = originalLayouts_;
            ctxA.movedNodes = movedNodes_;
            int scoreA = calculatePenalty(adjustedA, ctxA);

            PenaltyContext ctxB{withAdjA, nodeLayouts, forbiddenZones, gridSize};
            ctxB.sourceNodeId = layoutB.from;
            ctxB.targetNodeId = layoutB.to;
            ctxB.originalLayouts = originalLayouts_;
            ctxB.movedNodes = movedNodes_;
            int scoreB = calculatePenalty(layoutB, ctxB);

            result.layoutA = adjustedA;
            result.layoutB = layoutB;
            result.combinedScore = scoreA + scoreB;
            result.valid = true;
            return result;
        }
    }

    // Strategy 3: Handle first-segment vertical overlap
    if (!layoutB.bendPoints.empty() && !layoutA.bendPoints.empty()) {
        const Point& b0 = layoutB.sourcePoint;
        const Point& b1 = layoutB.bendPoints[0].position;
        const Point& a0 = layoutA.sourcePoint;
        const Point& a1 = layoutA.bendPoints[0].position;

        bool bFirstVertical = std::abs(b0.x - b1.x) < 1.0f;
        bool aFirstVertical = std::abs(a0.x - a1.x) < 1.0f;

        if (bFirstVertical && aFirstVertical && std::abs(b0.x - a0.x) < 1.0f) {
            float bMinY = std::min(b0.y, b1.y), bMaxY = std::max(b0.y, b1.y);
            float aMinY = std::min(a0.y, a1.y), aMaxY = std::max(a0.y, a1.y);

            if (std::max(bMinY, aMinY) <= std::min(bMaxY, aMaxY) + 1.0f) {
                float sharedX = b0.x;

                for (int dir = -1; dir <= 1; dir += 2) {
                    float tryX = sharedX + dir * gridSize;
                    float yOffset = dir * gridSize;

                    std::vector<BendPoint> newBends;
                    newBends.push_back({{tryX, layoutB.sourcePoint.y}});
                    bool firstHorizontalAdjusted = false;

                    for (size_t i = 0; i < layoutB.bendPoints.size(); ++i) {
                        const auto& bp = layoutB.bendPoints[i];
                        if (std::abs(bp.position.x - sharedX) < 1.0f) {
                            float adjustedY = bp.position.y;
                            if (!firstHorizontalAdjusted && i + 1 < layoutB.bendPoints.size()) {
                                const auto& nextBp = layoutB.bendPoints[i + 1];
                                if (std::abs(bp.position.y - nextBp.position.y) < 1.0f) {
                                    adjustedY = bp.position.y + yOffset;
                                    firstHorizontalAdjusted = true;
                                }
                            }
                            newBends.push_back({{tryX, adjustedY}});
                        } else if (firstHorizontalAdjusted && i > 0 &&
                                   std::abs(layoutB.bendPoints[i-1].position.y - bp.position.y) < 1.0f) {
                            newBends.push_back({{bp.position.x, bp.position.y + yOffset}});
                            firstHorizontalAdjusted = false;
                        } else {
                            newBends.push_back(bp);
                        }
                    }

                    EdgeLayout adjustedB = layoutB;
                    adjustedB.bendPoints = newBends;

                    if (!PathIntersection::hasSegmentOverlapExcludingLast(layoutA, adjustedB, 2)) {
                        PenaltyContext ctxA{otherLayouts, nodeLayouts, forbiddenZones, gridSize};
                        ctxA.sourceNodeId = layoutA.from;
                        ctxA.targetNodeId = layoutA.to;
                        ctxA.originalLayouts = originalLayouts_;
                        ctxA.movedNodes = movedNodes_;
                        int scoreA = calculatePenalty(layoutA, ctxA);

                        std::unordered_map<EdgeId, EdgeLayout> withALayout = otherLayouts;
                        withALayout[edgeIdA] = layoutA;
                        PenaltyContext ctxB{withALayout, nodeLayouts, forbiddenZones, gridSize};
                        ctxB.sourceNodeId = adjustedB.from;
                        ctxB.targetNodeId = adjustedB.to;
                        ctxB.originalLayouts = originalLayouts_;
                        ctxB.movedNodes = movedNodes_;
                        int scoreB = calculatePenalty(adjustedB, ctxB);

                        result.layoutA = layoutA;
                        result.layoutB = adjustedB;
                        result.combinedScore = scoreA + scoreB;
                        result.valid = true;
                        return result;
                    }
                }
            }
        }
    }

    // Strategy 4: General overlap detection and fix
    {
        std::unordered_map<EdgeId, EdgeLayout> withA;
        withA[edgeIdA] = layoutA;
        auto overlapInfo = PathIntersection::findOverlapInfo(layoutB, withA, layoutB.id, gridSize);

        if (overlapInfo.found) {
            // Convert grid coordinate to pixel
            float sharedPixel = overlapInfo.sharedGridCoord * gridSize;
            
            for (int dir = -1; dir <= 1; dir += 2) {
                // Use grid-based offset
                int tryGridCoord = overlapInfo.sharedGridCoord + dir;
                std::vector<BendPoint> newBends;

                if (overlapInfo.isVertical) {
                    float tryX = tryGridCoord * gridSize;
                    newBends.push_back({{tryX, layoutB.sourcePoint.y}});
                    for (const auto& bp : layoutB.bendPoints) {
                        if (std::abs(bp.position.x - sharedPixel) < 1.0f) {
                            newBends.push_back({{tryX, bp.position.y}});
                        } else {
                            newBends.push_back(bp);
                        }
                    }
                } else {
                    float tryY = tryGridCoord * gridSize;
                    newBends.push_back({{layoutB.sourcePoint.x, tryY}});
                    for (const auto& bp : layoutB.bendPoints) {
                        if (std::abs(bp.position.y - sharedPixel) < 1.0f) {
                            newBends.push_back({{bp.position.x, tryY}});
                        } else {
                            newBends.push_back(bp);
                        }
                    }
                }

                EdgeLayout adjustedB = layoutB;
                adjustedB.bendPoints = newBends;

                if (!PathIntersection::hasSegmentOverlap(layoutA, adjustedB)) {
                    PenaltyContext ctxA{otherLayouts, nodeLayouts, forbiddenZones, gridSize};
                    ctxA.sourceNodeId = layoutA.from;
                    ctxA.targetNodeId = layoutA.to;
                    ctxA.originalLayouts = originalLayouts_;
                    ctxA.movedNodes = movedNodes_;
                    int scoreA = calculatePenalty(layoutA, ctxA);

                    std::unordered_map<EdgeId, EdgeLayout> withALayout = otherLayouts;
                    withALayout[edgeIdA] = layoutA;
                    PenaltyContext ctxB{withALayout, nodeLayouts, forbiddenZones, gridSize};
                    ctxB.sourceNodeId = adjustedB.from;
                    ctxB.targetNodeId = adjustedB.to;
                    ctxB.originalLayouts = originalLayouts_;
                    ctxB.movedNodes = movedNodes_;
                    int scoreB = calculatePenalty(adjustedB, ctxB);

                    result.layoutA = layoutA;
                    result.layoutB = adjustedB;
                    result.combinedScore = scoreA + scoreB;
                    result.valid = true;
                    return result;
                }
            }
        }
    }

    return result;
}

std::vector<AStarEdgeOptimizer::CombinationResult>
AStarEdgeOptimizer::evaluateSelfLoopCombinations(
    EdgeId edgeId,
    const EdgeLayout& baseLayout,
    const std::unordered_map<EdgeId, EdgeLayout>& assignedLayouts,
    const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
    const std::vector<ForbiddenZone>& forbiddenZones) {

    std::vector<CombinationResult> results;

    // Find the node for this self-loop
    auto nodeIt = nodeLayouts.find(baseLayout.from);
    if (nodeIt == nodeLayouts.end()) {
        return results;  // Node not found
    }

    const NodeLayout& nodeLayout = nodeIt->second;

    // Calculate loop index for unique snap points using shared helper
    int loopIndex = SelfLoopRouter::calculateLoopIndex(edgeId, baseLayout.from, assignedLayouts);

    // Create default layout options for SelfLoopRouter
    LayoutOptions options;
    options.gridConfig.cellSize = effectiveGridSize();

    // Try all 4 self-loop directions
    std::array<SelfLoopDirection, 4> directions = {
        SelfLoopDirection::Right,
        SelfLoopDirection::Left,
        SelfLoopDirection::Top,
        SelfLoopDirection::Bottom
    };

    for (SelfLoopDirection dir : directions) {
        options.channelRouting.selfLoop.preferredDirection = dir;

        // Generate self-loop layout using SelfLoopRouter
        EdgeLayout candidate = SelfLoopRouter::route(
            edgeId, baseLayout.from, nodeLayout, loopIndex, options);

        // Calculate penalty score using unified penalty system
        PenaltyContext ctx{assignedLayouts, nodeLayouts, forbiddenZones, effectiveGridSize()};
        ctx.sourceNodeId = candidate.from;
        ctx.targetNodeId = candidate.to;
        ctx.originalLayouts = originalLayouts_;
        ctx.movedNodes = movedNodes_;

        // Check hard constraints (segment overlap, etc.)
        if (!passesHardConstraints(candidate, ctx)) {
            LOG_DEBUG("[SNAP-TRACE] SelfLoop edge={} REJECTED by hardConstraints srcPos=({},{})", 
                      edgeId, candidate.sourcePoint.x, candidate.sourcePoint.y);
            continue;  // Skip combinations that violate hard constraints
        }

        int score = calculatePenalty(candidate, ctx);
        LOG_DEBUG("[SNAP-TRACE] SelfLoop edge={} ACCEPTED srcPos=({},{}) score={}",
                  edgeId, candidate.sourcePoint.x, candidate.sourcePoint.y, score);

        // Map direction to source/target edges for CombinationResult
        NodeEdge srcEdge = candidate.sourceEdge;
        NodeEdge tgtEdge = candidate.targetEdge;

        results.push_back({srcEdge, tgtEdge, score, std::move(candidate), true});
    }

    // Sort by score (lowest first = best)
    std::sort(results.begin(), results.end(),
        [](const CombinationResult& a, const CombinationResult& b) {
            return a.score < b.score;
        });

    return results;
}

AStarEdgeOptimizer::EdgePairResult AStarEdgeOptimizer::resolveOverlappingPair(
    EdgeId edgeIdA, EdgeId edgeIdB,
    const EdgeLayout& layoutA, const EdgeLayout& layoutB,
    const std::unordered_map<EdgeId, EdgeLayout>& assignedLayouts,
    const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
    const std::vector<ForbiddenZone>& forbiddenZones) {

    EdgePairResult bestResult;
    bestResult.valid = false;
    bestResult.combinedScore = std::numeric_limits<int>::max();

    float gridSize = effectiveGridSize();

    // Build obstacle map WITHOUT edges A and B
    // This allows A and B to find paths without blocking each other
    std::unordered_map<EdgeId, EdgeLayout> otherLayouts;
    for (const auto& [id, layout] : assignedLayouts) {
        if (id != edgeIdA && id != edgeIdB) {
            otherLayouts[id] = layout;
        }
    }

    // Include all edge layouts in bounds calculation to prevent out-of-bounds segments
    ObstacleMap baseObstacles;
    baseObstacles.buildFromNodes(nodeLayouts, gridSize, 0, &assignedLayouts);
    baseObstacles.addEdgeSegments(otherLayouts, INVALID_EDGE);

    // All 4 node edges
    constexpr std::array<NodeEdge, 4> allEdges = {
        NodeEdge::Top, NodeEdge::Bottom, NodeEdge::Left, NodeEdge::Right
    };

    int totalCombos = 0;
    int pathFoundACount = 0;
    int passedHardA = 0;
    int pathFoundBCount = 0;
    int passedHardB = 0;
    int noOverlapCount = 0;

    // Evaluate all 16×16 = 256 combinations for the pair
    for (NodeEdge srcA : allEdges) {
        for (NodeEdge tgtA : allEdges) {
            ++totalCombos;
            // Create candidate A
            ObstacleMap obstaclesA = baseObstacles;  // Copy base obstacles
            bool pathFoundA = false;
            EdgeLayout candidateA = createCandidateLayout(
                layoutA, srcA, tgtA, nodeLayouts, obstaclesA, pathFoundA);

            if (!pathFoundA) continue;
            ++pathFoundACount;

            // Check hard constraints for A (SKIP overlap check with other edges for now)
            // We only care about self-overlap and forbidden zones
            PenaltyContext ctxA{otherLayouts, nodeLayouts, forbiddenZones, gridSize};
            ctxA.sourceNodeId = candidateA.from;
            ctxA.targetNodeId = candidateA.to;
            ctxA.originalLayouts = originalLayouts_;
            ctxA.movedNodes = movedNodes_;
            if (!passesHardConstraints(candidateA, ctxA)) continue;
            ++passedHardA;

            int scoreA = calculatePenalty(candidateA, ctxA);

            // Now try all combinations for B, with A as additional obstacle
            for (NodeEdge srcB : allEdges) {
                for (NodeEdge tgtB : allEdges) {
                    // Build obstacles for B: base + candidateA
                    ObstacleMap obstaclesB = baseObstacles;
                    std::unordered_map<EdgeId, EdgeLayout> withA = otherLayouts;
                    withA[edgeIdA] = candidateA;
                    obstaclesB.addEdgeSegments(withA, edgeIdB);

                    bool pathFoundB = false;
                    EdgeLayout candidateB = createCandidateLayout(
                        layoutB, srcB, tgtB, nodeLayouts, obstaclesB, pathFoundB);

                    if (!pathFoundB) continue;
                    ++pathFoundBCount;

                    // Check hard constraints for B (with A in the context)
                    PenaltyContext ctxB{withA, nodeLayouts, forbiddenZones, gridSize};
                    ctxB.sourceNodeId = candidateB.from;
                    ctxB.targetNodeId = candidateB.to;
                    ctxB.originalLayouts = originalLayouts_;
                    ctxB.movedNodes = movedNodes_;
                    if (!passesHardConstraints(candidateB, ctxB)) continue;
                    ++passedHardB;

                    int scoreB = calculatePenalty(candidateB, ctxB);

                    // Check for overlap between A and B
                    std::unordered_map<EdgeId, EdgeLayout> pairLayouts;
                    pairLayouts[edgeIdA] = candidateA;
                    pairLayouts[edgeIdB] = candidateB;
                    
                    if (PathIntersection::hasOverlapWithOthers(candidateA, pairLayouts, edgeIdA)) {
                        continue;  // A and B still overlap
                    }
                    ++noOverlapCount;

                    int combinedScore = scoreA + scoreB;
                    if (combinedScore < bestResult.combinedScore) {
                        bestResult.layoutA = candidateA;
                        bestResult.layoutB = candidateB;
                        bestResult.combinedScore = combinedScore;
                        bestResult.valid = true;
                    }
                }
            }
        }
    }

    // Fallback: If no valid combination found, try path adjustment
    if (!bestResult.valid) {
        bestResult = tryPathAdjustmentFallback(
            edgeIdA, edgeIdB, layoutA, layoutB,
            otherLayouts, nodeLayouts, forbiddenZones, gridSize);
    }


    return bestResult;
}

AStarEdgeOptimizer::ParallelEdgeResult AStarEdgeOptimizer::evaluateEdgeIndependent(
    EdgeId edgeId,
    const EdgeLayout& baseLayout,
    const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
    const std::vector<ForbiddenZone>& forbiddenZones,
    IPathFinder& pathFinder,
    const std::unordered_map<EdgeId, EdgeLayout>& currentLayouts) {

    ParallelEdgeResult presult;
    presult.edgeId = edgeId;
    presult.hasValidResult = false;

    // Handle self-loops (don't use pathfinder for self-loops)
    if (baseLayout.from == baseLayout.to) {
        presult.isSelfLoop = true;
        // Pass currentLayouts to calculate correct loopIndex for unique snap points
        auto combinations = evaluateSelfLoopCombinations(
            edgeId, baseLayout, currentLayouts, nodeLayouts, forbiddenZones);

        if (!combinations.empty()) {
            presult.best = combinations.front();
            presult.hasValidResult = true;
        }
        return presult;
    }

    // For regular edges, use thread-local pathfinder
    std::unordered_map<EdgeId, EdgeLayout> emptyLayouts;
    auto combinations = evaluateCombinations(
        edgeId, baseLayout, emptyLayouts, nodeLayouts, forbiddenZones, pathFinder);

    if (!combinations.empty()) {
        presult.best = combinations.front();
        presult.hasValidResult = true;
    }

    return presult;
}

std::vector<std::pair<EdgeId, EdgeId>> AStarEdgeOptimizer::detectOverlaps(
    const std::unordered_map<EdgeId, EdgeLayout>& layouts) {
    // Delegate to PathIntersection (Single Source of Truth)
    return PathIntersection::findAllOverlappingPairs(layouts);
}

bool AStarEdgeOptimizer::resolveAllOverlaps(
    const std::vector<std::pair<EdgeId, EdgeId>>& overlappingPairs,
    std::unordered_map<EdgeId, EdgeLayout>& result,
    std::unordered_map<EdgeId, EdgeLayout>& assignedLayouts,
    const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
    const std::vector<ForbiddenZone>& forbiddenZones,
    std::unordered_set<EdgeId>* resolvedEdges) {

    bool anyResolved = false;

    for (const auto& [idA, idB] : overlappingPairs) {
        auto itA = result.find(idA);
        auto itB = result.find(idB);
        if (itA == result.end() || itB == result.end()) continue;

        LOG_DEBUG("[AStarOpt] PHASE 3: Resolving overlap Edge {} & {}", idA, idB);

        EdgePairResult pairResult = resolveOverlappingPair(
            idA, idB, itA->second, itB->second,
            assignedLayouts, nodeLayouts, forbiddenZones);

        LOG_DEBUG("[AStarOpt]   resolveOverlappingPair: valid={}", pairResult.valid);

        if (pairResult.valid) {
            // Debug: log the resolved paths
            LOG_DEBUG("[AStarOpt]   RESOLVED Edge {} bends:", idA);
            for (const auto& bp : pairResult.layoutA.bendPoints) {
                LOG_DEBUG("[AStarOpt]     ({}, {})", bp.position.x, bp.position.y);
            }
            LOG_DEBUG("[AStarOpt]   RESOLVED Edge {} bends:", idB);
            for (const auto& bp : pairResult.layoutB.bendPoints) {
                LOG_DEBUG("[AStarOpt]     ({}, {})", bp.position.x, bp.position.y);
            }
            
            result[idA] = pairResult.layoutA;
            result[idB] = pairResult.layoutB;
            assignedLayouts[idA] = pairResult.layoutA;
            assignedLayouts[idB] = pairResult.layoutB;
            if (resolvedEdges) {
                resolvedEdges->insert(idA);
                resolvedEdges->insert(idB);
            }
            anyResolved = true;
            LOG_DEBUG("[AStarOpt]   SUCCESS via resolveOverlappingPair");
        } else {
            // Fallback: Use CooperativeRerouter when pair resolution fails
            LOG_DEBUG("[AStarOpt]   Trying CooperativeRerouter fallback...");
            CooperativeRerouter rerouter(pathFinder_, effectiveGridSize());

            // Build otherLayouts excluding both A and B
            std::unordered_map<EdgeId, EdgeLayout> otherLayouts;
            for (const auto& [eid, layout] : assignedLayouts) {
                if (eid != idA && eid != idB) {
                    otherLayouts[eid] = layout;
                }
            }
            // Add B to otherLayouts so A can route around it
            otherLayouts[idB] = itB->second;

            auto coopResult = rerouter.rerouteWithCooperation(
                idA, itA->second, otherLayouts, nodeLayouts);

            LOG_DEBUG("[AStarOpt]   CooperativeRerouter: success={} reason={}",
                      coopResult.success, coopResult.failureReason);

            if (coopResult.success) {
                result[idA] = coopResult.layout;
                assignedLayouts[idA] = coopResult.layout;
                for (const auto& reroutedLayout : coopResult.reroutedEdges) {
                    result[reroutedLayout.id] = reroutedLayout;
                    assignedLayouts[reroutedLayout.id] = reroutedLayout;
                }
                if (resolvedEdges) {
                    resolvedEdges->insert(idA);
                    resolvedEdges->insert(idB);
                }
                anyResolved = true;
                LOG_DEBUG("[AStarOpt]   SUCCESS via CooperativeRerouter");
            } else {
                LOG_DEBUG("[AStarOpt]   FAILED - overlap remains!");
            }
        }
    }

    return anyResolved;
}

}  // namespace arborvia
