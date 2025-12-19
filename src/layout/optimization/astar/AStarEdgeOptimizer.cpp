#include "AStarEdgeOptimizer.h"
#include "EdgeRoutingContext.h"
#include "NodeEdgeSelector.h"
#include "OverlapResolver.h"
#include "../../pathfinding/ObstacleMap.h"
#include "../../pathfinding/AStarPathFinder.h"
#include "../../pathfinding/SelfLoopPathCalculator.h"
#include "../../routing/UnifiedRetryChain.h"
#include "../../sugiyama/routing/PathIntersection.h"
#include "arborvia/core/GeometryUtils.h"

#include "arborvia/common/Logger.h"
#include <algorithm>
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

    // Store gridSize for use in helper methods
    gridSize_ = gridSizeParam;

    // Store constraint state in base class for FixedEndpointPenalty
    setConstraintState(currentLayouts, movedNodes);

    // Calculate forbidden zones once for all edges
    float gridSize = effectiveGridSize();
    auto forbiddenZones = calculateForbiddenZones(nodeLayouts, gridSize);

    // Build EdgeRoutingContext for new components
    EdgeRoutingContext context;
    context.pathFinder = pathFinder_.get();
    context.penaltySystem = penaltySystem_;
    context.nodeLayouts = &nodeLayouts;
    context.forbiddenZones = &forbiddenZones;
    context.gridSize = gridSize;
    context.originalLayouts = originalLayouts_;
    context.movedNodes = movedNodes_;
    context.preserveDirections = preserveDirections();

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
            std::vector<std::future<NodeEdgeSelector::ParallelEdgeResult>> futures;
            futures.reserve(numEdges);

            for (EdgeId edgeId : edges) {
                // Skip edges that had successful overlap resolution
                if (resolvedEdges.count(edgeId)) continue;

                auto it = currentLayouts.find(edgeId);
                if (it == currentLayouts.end()) continue;

                // Use result from previous pass if available
                const EdgeLayout baseLayout = (pass > 0 && result.count(edgeId))
                    ? result[edgeId] : it->second;

                // Launch async evaluation with thread-local pathfinder
                futures.push_back(std::async(std::launch::async,
                    [&context, edgeId, baseLayout, &currentLayouts]() {
                        AStarPathFinder localPathFinder;
                        return NodeEdgeSelector::evaluateEdgeIndependent(
                            context, edgeId, baseLayout, currentLayouts, localPathFinder);
                    }));
            }

            // Collect results
            bool anyOverlapFound = false;

            for (auto& future : futures) {
                NodeEdgeSelector::ParallelEdgeResult presult = future.get();
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
                    auto it = currentLayouts.find(presult.edgeId);
                    if (it != currentLayouts.end()) {
                        result[presult.edgeId] = it->second;
                        assignedLayouts[presult.edgeId] = it->second;
                    }
                }
            }

            // ===== PHASE 1.5: Sequential retry chain for failed edges =====
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
            auto overlappingPairs = OverlapResolver::detectOverlaps(result);
            LOG_DEBUG("[AStarOpt] PHASE 2: detectOverlaps found {} pairs (pass={}, parallel=true)",
                      overlappingPairs.size(), pass);
            for (const auto& [a, b] : overlappingPairs) {
                LOG_DEBUG("[AStarOpt]   Overlap: Edge {} & Edge {}", a, b);
            }
            if (!overlappingPairs.empty()) anyOverlapFound = true;

            // ===== PHASE 3: Resolve overlaps (serial - cooperative rerouting) =====
            OverlapResolver::resolveAllOverlaps(context, overlappingPairs, result, assignedLayouts, &resolvedEdges);

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
                    auto selfLoopCombinations = NodeEdgeSelector::evaluateSelfLoopCombinations(
                        context, edgeId, baseLayout, assignedLayouts);
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
                auto combinations = NodeEdgeSelector::evaluateCombinations(
                    context, edgeId, baseLayout, assignedLayouts);

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

            // Detect and resolve overlaps
            auto overlappingPairs = OverlapResolver::detectOverlaps(result);
            LOG_DEBUG("[AStarOpt] PHASE 2: detectOverlaps found {} pairs (pass={}, sequential=true)",
                      overlappingPairs.size(), pass);
            for (const auto& [a, b] : overlappingPairs) {
                LOG_DEBUG("[AStarOpt]   Overlap: Edge {} & Edge {}", a, b);
            }
            if (!overlappingPairs.empty()) {
                anyOverlapFound = true;
                OverlapResolver::resolveAllOverlaps(context, overlappingPairs, result, assignedLayouts, nullptr);
            }

            if (!anyOverlapFound) break;
        }
    }

    return result;
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

    // Build base obstacle map once (from nodes only)
    ObstacleMap baseObstacles;
    baseObstacles.buildFromNodes(nodeLayouts, gridSize, 0, &edgeLayouts);

    // Pre-create self-loop calculator and config
    SelfLoopPathCalculator selfLoopCalc;
    PathConfig selfLoopConfig;
    selfLoopConfig.gridSize = gridSize;
    selfLoopConfig.extensionCells = 2;
    selfLoopConfig.snapToGrid = true;

    // Set context for self-loop stacking (loopIndex calculation)
    PathCalculatorContext selfLoopContext;
    selfLoopContext.edgeLayouts = &edgeLayouts;
    selfLoopContext.gridSize = gridSize;
    selfLoopCalc.setContext(selfLoopContext);

    // Phase 1: Route edges sequentially
    for (EdgeId edgeId : edges) {
        auto it = edgeLayouts.find(edgeId);
        if (it == edgeLayouts.end()) {
            continue;
        }

        EdgeLayout& layout = it->second;

        // Handle self-loops
        if (layout.from == layout.to) {
            auto result = selfLoopCalc.calculatePath(layout, nodeLayouts, selfLoopConfig);
            if (result.success) {
                layout.bendPoints = std::move(result.bendPoints);
                // Apply new snap state if provided (for loopIndex-based stacking)
                // This maintains orthogonality between snap points and bend points
                // IMPORTANT: snapIndex and Point must always be updated together (SSOT)
                if (result.newSourceSnapIndex && result.newSourcePoint) {
                    layout.setSourceSnap(*result.newSourceSnapIndex, *result.newSourcePoint);
                }
                if (result.newTargetSnapIndex && result.newTargetPoint) {
                    layout.setTargetSnap(*result.newTargetSnapIndex, *result.newTargetPoint);
                }
                routedEdges[edgeId] = layout;
                continue;
            }
        }

        ObstacleMap obstacles = baseObstacles;
        obstacles.addEdgeSegmentsForLayout(layout, edgeLayouts, nodeLayouts);

        if (!routedEdges.empty()) {
            obstacles.addEdgeSegmentsForLayout(layout, routedEdges, nodeLayouts);
        }

        auto& retryChain = getRetryChain(gridSize);

        UnifiedRetryChain::RetryConfig config;
        config.maxSnapRetries = 9;
        config.maxNodeEdgeCombinations = 0;  // Disable NodeEdge switching for regenerate
        config.enableCooperativeReroute = true;
        config.gridSize = gridSize;
        config.movedNodes = movedNodes_.empty() ? nullptr : &movedNodes_;

        std::unordered_map<EdgeId, EdgeLayout> otherEdges = edgeLayouts;
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
            layout.bendPoints = result.layout.bendPoints;
            layout.copySnapStateFrom(result.layout);
            layout.usedGridSize = effectiveGridSize();

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

        routedEdges[edgeId] = layout;
    }

    // Phase 2: Rip-up and Reroute for overlapping edges
    constexpr int MAX_RIP_UP_ITERATIONS = 3;

    for (int iteration = 0; iteration < MAX_RIP_UP_ITERATIONS; ++iteration) {
        std::vector<EdgeId> dirtyEdges;
        std::unordered_set<EdgeId> dirtySet;

        for (const auto& [edgeId, layout] : edgeLayouts) {
            if (layout.from == layout.to) {
                continue;
            }

            if (PathIntersection::hasOverlapWithOthers(layout, edgeLayouts, edgeId)) {
                if (dirtySet.insert(edgeId).second) {
                    dirtyEdges.push_back(edgeId);
                }
            }
        }

        if (dirtyEdges.empty()) {
            LOG_DEBUG("[AStarOpt::regenerateBendPoints] Phase 2 iteration {}: No overlaps found", iteration);
            break;
        }

        LOG_DEBUG("[AStarOpt::regenerateBendPoints] Phase 2 iteration {}: Found {} dirty edges", iteration, dirtyEdges.size());
        for (EdgeId dId : dirtyEdges) {
            LOG_DEBUG("[AStarOpt::regenerateBendPoints] Phase 2: dirty edge {}", dId);
        }

        for (EdgeId dirtyId : dirtyEdges) {
            auto it = edgeLayouts.find(dirtyId);
            if (it == edgeLayouts.end()) {
                continue;
            }

            EdgeLayout& layout = it->second;

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

                for (const auto& reroutedLayout : result.reroutedEdges) {
                    edgeLayouts[reroutedLayout.id] = reroutedLayout;
                }
            }
        }
    }
}

}  // namespace arborvia
