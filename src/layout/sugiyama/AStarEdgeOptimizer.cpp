#include "AStarEdgeOptimizer.h"
#include "AStarEdgeOptimizer.h"
#include "ObstacleMap.h"
#include "AStarPathFinder.h"
#include "PathIntersection.h"
#include "SelfLoopRouter.h"
#include "arborvia/core/GeometryUtils.h"
#include "arborvia/layout/IEdgePenalty.h"

#include <algorithm>
#include <array>
#include <iostream>

#ifndef EDGE_ROUTING_DEBUG
#define EDGE_ROUTING_DEBUG 1
#endif

namespace arborvia {

AStarEdgeOptimizer::AStarEdgeOptimizer(
    std::shared_ptr<IPathFinder> pathFinder,
    float gridSize)
    : pathFinder_(std::move(pathFinder))
    , gridSize_(gridSize) {

    // Create default pathfinder if not provided
    if (!pathFinder_) {
        pathFinder_ = std::make_shared<AStarPathFinder>();
    }

    // Ensure penalty system is set
    if (!penaltySystem()) {
        setPenaltySystem(EdgePenaltySystem::createDefault());
    }
}

void AStarEdgeOptimizer::setPathFinder(std::shared_ptr<IPathFinder> pathFinder) {
    pathFinder_ = std::move(pathFinder);
    if (!pathFinder_) {
        pathFinder_ = std::make_shared<AStarPathFinder>();
    }
}

void AStarEdgeOptimizer::setGridSize(float gridSize) {
    gridSize_ = gridSize;
}

float AStarEdgeOptimizer::effectiveGridSize() const {
    return gridSize_ > 0.0f ? gridSize_ : constants::PATHFINDING_GRID_SIZE;
}

std::unordered_map<EdgeId, EdgeLayout> AStarEdgeOptimizer::optimize(
    const std::vector<EdgeId>& edges,
    const std::unordered_map<EdgeId, EdgeLayout>& currentLayouts,
    const std::unordered_map<NodeId, NodeLayout>& nodeLayouts) {

    // Calculate forbidden zones once for all edges
    float gridSize = effectiveGridSize();
    auto forbiddenZones = calculateForbiddenZones(nodeLayouts, gridSize);

    // Multi-pass optimization to resolve overlaps
    constexpr int MAX_PASSES = 3;
    std::unordered_map<EdgeId, EdgeLayout> result;
    std::unordered_map<EdgeId, EdgeLayout> assignedLayouts = currentLayouts;
    std::vector<EdgeId> failedEdges;  // Edges with no valid combinations

    for (int pass = 0; pass < MAX_PASSES; ++pass) {
#if EDGE_ROUTING_DEBUG
        std::cout << "[AStarOptimizer] Pass " << (pass + 1) << "/" << MAX_PASSES << std::endl;
#endif
        bool anyOverlapFound = false;
        failedEdges.clear();

        for (EdgeId edgeId : edges) {
            auto it = currentLayouts.find(edgeId);
            if (it == currentLayouts.end()) {
                continue;
            }

            // Use result from previous pass if available, otherwise use currentLayouts
            const EdgeLayout& baseLayout = (pass > 0 && result.count(edgeId))
                ? result[edgeId] : it->second;

            // Handle self-loops with penalty-based direction selection
            if (baseLayout.from == baseLayout.to) {
                auto selfLoopCombinations = evaluateSelfLoopCombinations(
                    edgeId, baseLayout, assignedLayouts, nodeLayouts, forbiddenZones);

                if (!selfLoopCombinations.empty()) {
                    const auto& best = selfLoopCombinations.front();
                    if (best.score > 0) anyOverlapFound = true;
                    result[edgeId] = best.layout;
                    assignedLayouts[edgeId] = best.layout;
                } else {
                    result[edgeId] = baseLayout;
                    assignedLayouts[edgeId] = baseLayout;
                }
                continue;
            }

            // Evaluate edge combinations
            auto combinations = evaluateCombinations(
                edgeId, baseLayout, assignedLayouts, nodeLayouts, forbiddenZones);

#if EDGE_ROUTING_DEBUG
            std::cout << "[AStarOptimizer] Edge " << edgeId << ": "
                      << combinations.size() << " valid combinations found" << std::endl;
#endif

            if (combinations.empty()) {
#if EDGE_ROUTING_DEBUG
                std::cout << "[AStarOptimizer] Edge " << edgeId
                          << ": ALL combinations rejected by hard constraints!" << std::endl;
#endif
                failedEdges.push_back(edgeId);
                // Keep base layout for now, may be resolved by cooperative rerouting
                result[edgeId] = baseLayout;
                assignedLayouts[edgeId] = baseLayout;
                continue;
            }

            const auto& best = combinations.front();
#if EDGE_ROUTING_DEBUG
            std::cout << "[AStarOptimizer] Edge " << edgeId << ": best score=" << best.score
                      << " srcEdge=" << static_cast<int>(best.sourceEdge)
                      << " tgtEdge=" << static_cast<int>(best.targetEdge) << std::endl;
#endif
            if (best.score > 0) anyOverlapFound = true;
            result[edgeId] = best.layout;
            assignedLayouts[edgeId] = best.layout;
        }

        // Cooperative rerouting for failed edges
        if (failedEdges.size() >= 2) {
#if EDGE_ROUTING_DEBUG
            std::cout << "[AStarOptimizer] " << failedEdges.size()
                      << " failed edges: ";
            for (EdgeId id : failedEdges) std::cout << id << " ";
            std::cout << std::endl;
#endif
            // Find overlapping pairs among failed edges
            std::vector<std::pair<EdgeId, EdgeId>> overlappingPairs;
            for (size_t i = 0; i < failedEdges.size(); ++i) {
                for (size_t j = i + 1; j < failedEdges.size(); ++j) {
                    EdgeId idA = failedEdges[i];
                    EdgeId idB = failedEdges[j];
                    auto itA = result.find(idA);
                    auto itB = result.find(idB);
                    if (itA == result.end() || itB == result.end()) continue;

                    // Check if they overlap
                    std::unordered_map<EdgeId, EdgeLayout> pairCheck;
                    pairCheck[idA] = itA->second;
                    pairCheck[idB] = itB->second;
                    if (PathIntersection::hasOverlapWithOthers(itA->second, pairCheck, idA)) {
                        overlappingPairs.emplace_back(idA, idB);
#if EDGE_ROUTING_DEBUG
                        std::cout << "[CoopReroute] Edges " << idA << " & " << idB << " OVERLAP, adding to pairs" << std::endl;
#endif
                    }
#if EDGE_ROUTING_DEBUG
                    else {
                        std::cout << "[CoopReroute] Edges " << idA << " & " << idB << " do NOT overlap" << std::endl;
                    }
#endif
                }
            }
#if EDGE_ROUTING_DEBUG
            if (overlappingPairs.empty()) {
                std::cout << "[CoopReroute] No overlapping pairs among failed edges" << std::endl;
            }
#endif

            // Try cooperative rerouting for each overlapping pair
            for (const auto& [idA, idB] : overlappingPairs) {
                auto itA = result.find(idA);
                auto itB = result.find(idB);
                if (itA == result.end() || itB == result.end()) continue;

                EdgePairResult pairResult = resolveOverlappingPair(
                    idA, idB,
                    itA->second, itB->second,
                    assignedLayouts, nodeLayouts, forbiddenZones);

                if (pairResult.valid) {
                    result[idA] = pairResult.layoutA;
                    result[idB] = pairResult.layoutB;
                    assignedLayouts[idA] = pairResult.layoutA;
                    assignedLayouts[idB] = pairResult.layoutB;
                    anyOverlapFound = true;
#if EDGE_ROUTING_DEBUG
                    std::cout << "[AStarOptimizer] Cooperative rerouting succeeded for edges "
                              << idA << " & " << idB << std::endl;
#endif
                }
            }
        }

        // If no overlaps found in this pass, we're done
        if (!anyOverlapFound) {
#if EDGE_ROUTING_DEBUG
            std::cout << "[AStarOptimizer] No overlaps found, optimization complete" << std::endl;
#endif
            break;
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
    float gridSize = effectiveGridSize();
    ObstacleMap obstacles;
    obstacles.buildFromNodes(nodeLayouts, gridSize, 0);  // margin = 0

    // Add OTHER edge segments as obstacles so A* finds paths that avoid them
    // This is KEY for finding non-overlapping paths!
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
            
            if (passesHardConstraints(baseLayout, ctx)) {
                int score = calculatePenalty(baseLayout, ctx);
                results.push_back({srcEdge, tgtEdge, score, baseLayout, true});
#if EDGE_ROUTING_DEBUG
                std::cout << "[Preserve] Edge " << baseLayout.id << " original path ACCEPTED, score=" << score << std::endl;
#endif
                return;  // Use original path, no need to generate new one
            }
#if EDGE_ROUTING_DEBUG
            else {
                // Print why original path failed
                std::cout << "[Preserve] Edge " << baseLayout.id << " original path FAILED hard constraints:" << std::endl;
                auto breakdown = penaltySystem_->calculatePenaltyBreakdown(baseLayout, ctx);
                for (const auto& [name, penalty] : breakdown) {
                    if (penalty > 0) {
                        std::cout << "  - " << name << ": " << penalty << std::endl;
                    }
                }
            }
#endif
        }
#if EDGE_ROUTING_DEBUG
        else if (srcEdge == baseLayout.sourceEdge && tgtEdge == baseLayout.targetEdge &&
                 !baseLayout.bendPoints.empty()) {
            std::cout << "[Preserve] Edge " << baseLayout.id << " snap points changed, cannot preserve original path" << std::endl;
        }
#endif

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
        // Optimize directions - evaluate all 16 combinations (4 source × 4 target)
        constexpr std::array<NodeEdge, 4> allEdges = {
            NodeEdge::Top, NodeEdge::Bottom, NodeEdge::Left, NodeEdge::Right
        };
        for (NodeEdge srcEdge : allEdges) {
            for (NodeEdge tgtEdge : allEdges) {
                evaluateEdge(srcEdge, tgtEdge);
            }
        }
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

EdgeLayout AStarEdgeOptimizer::createCandidateLayout(
    const EdgeLayout& base,
    NodeEdge sourceEdge,
    NodeEdge targetEdge,
    const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
    ObstacleMap& obstacles,
    bool& pathFound) {

    pathFound = false;

    EdgeLayout candidate = base;
    candidate.sourceEdge = sourceEdge;
    candidate.targetEdge = targetEdge;
    candidate.bendPoints.clear();

    // When edge routing changes, mark snap indices for redistribution
    // This prevents duplicate snap indices on the same node edge
    if (sourceEdge != base.sourceEdge) {
        candidate.sourceSnapIndex = constants::SNAP_INDEX_UNASSIGNED;
    }
    if (targetEdge != base.targetEdge) {
        candidate.targetSnapIndex = constants::SNAP_INDEX_UNASSIGNED;
    }

    // Get node layouts
    auto srcIt = nodeLayouts.find(base.from);
    if (srcIt == nodeLayouts.end()) {
        return candidate;  // Invalid: source node not found
    }
    
    auto tgtIt = nodeLayouts.find(base.to);
    if (tgtIt == nodeLayouts.end()) {
        return candidate;  // Invalid: target node not found
    }

    // Use pre-built obstacle map (includes node and edge obstacles)
    float gridSize = effectiveGridSize();

    // Calculate source/target as grid coordinates first, then convert to pixel
    // This ensures all coordinates are exactly on grid vertices
    GridPoint startGrid = calculateGridPosition(srcIt->second, sourceEdge, gridSize);
    GridPoint goalGrid = calculateGridPosition(tgtIt->second, targetEdge, gridSize);
    
    candidate.sourcePoint = obstacles.gridToPixel(startGrid.x, startGrid.y);
    candidate.targetPoint = obstacles.gridToPixel(goalGrid.x, goalGrid.y);

    // Use PathFinder to find actual orthogonal path
    PathResult pathResult = pathFinder_->findPath(
        startGrid, goalGrid, obstacles,
        base.from, base.to,
        sourceEdge, targetEdge,
        {}, {});  // No extra excludes

    if (!pathResult.found || pathResult.path.size() < 2) {
        // No valid path found for this combination
        return candidate;
    }

    // Convert grid path to pixel bend points
    // Skip first (source) and last (target) points - they're the endpoints
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
    const std::unordered_map<NodeId, NodeLayout>& nodeLayouts) {

    if (edges.empty()) {
        return;
    }

    float gridSize = effectiveGridSize();

    // Track already-routed edges for blocking
    std::unordered_map<EdgeId, EdgeLayout> routedEdges;

    // Phase 1: Route edges sequentially, each sees previously-routed edges as blocking
    for (EdgeId edgeId : edges) {
        auto it = edgeLayouts.find(edgeId);
        if (it == edgeLayouts.end()) {
            continue;
        }

        EdgeLayout& layout = it->second;

        // Handle self-loops: generate orthogonal path wrapping around node corner
        if (layout.from == layout.to) {
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

        // Build obstacle map with nodes + already-routed edges as blocking
        ObstacleMap obstacles;
        obstacles.buildFromNodes(nodeLayouts, gridSize, 0);
        if (!routedEdges.empty()) {
            obstacles.addEdgeSegments(routedEdges, edgeId);
        }

        // Find path using existing sourceEdge/targetEdge
        GridPoint startGrid = obstacles.pixelToGrid(layout.sourcePoint);
        GridPoint goalGrid = obstacles.pixelToGrid(layout.targetPoint);

        PathResult pathResult = pathFinder_->findPath(
            startGrid, goalGrid, obstacles,
            layout.from, layout.to,
            layout.sourceEdge, layout.targetEdge,
            {}, {});

        if (pathResult.found && pathResult.path.size() >= 2) {
            // Convert grid path to pixel bend points
            layout.bendPoints.clear();
            for (size_t i = 1; i + 1 < pathResult.path.size(); ++i) {
                Point pixelPoint = obstacles.gridToPixel(
                    pathResult.path[i].x, pathResult.path[i].y);
                layout.bendPoints.push_back({pixelPoint});
            }
        } else {
            // A* failed - generate simple orthogonal fallback path
            // This ensures we have valid orthogonal bendPoints even if A* can't find a path
            // Don't keep old bendPoints as they may be for different snap positions
            layout.bendPoints.clear();
            
            // Generate L-shape or Z-shape based on source/target positions
            float srcX = layout.sourcePoint.x;
            float srcY = layout.sourcePoint.y;
            float tgtX = layout.targetPoint.x;
            float tgtY = layout.targetPoint.y;
            
            // Check if direct connection is possible (aligned already)
            bool xAligned = std::abs(srcX - tgtX) < 1.0f;
            bool yAligned = std::abs(srcY - tgtY) < 1.0f;
            
            if (!xAligned && !yAligned) {
                // Need at least one bend point for orthogonal path
                // Choose based on source/target edge directions
                bool preferHorizontalFirst = 
                    (layout.sourceEdge == NodeEdge::Left || layout.sourceEdge == NodeEdge::Right);
                
                if (preferHorizontalFirst) {
                    // Horizontal first, then vertical: bend at (tgtX, srcY)
                    Point bend = {tgtX, srcY};
                    bend.x = std::round(bend.x / gridSize) * gridSize;
                    bend.y = std::round(bend.y / gridSize) * gridSize;
                    layout.bendPoints.push_back({bend});
                } else {
                    // Vertical first, then horizontal: bend at (srcX, tgtY)
                    Point bend = {srcX, tgtY};
                    bend.x = std::round(bend.x / gridSize) * gridSize;
                    bend.y = std::round(bend.y / gridSize) * gridSize;
                    layout.bendPoints.push_back({bend});
                }
            }
            // else: already aligned, no bendPoints needed
#if EDGE_ROUTING_DEBUG
            std::cout << "[regenerateBendPoints] Edge " << edgeId 
                      << " A* failed, using fallback orthogonal path with "
                      << layout.bendPoints.size() << " bends" << std::endl;
#endif
        }

        // Add this edge to routed edges for blocking subsequent edges
        routedEdges[edgeId] = layout;
    }

    // Phase 2: Rip-up and Reroute for overlapping edges
    constexpr int MAX_RIP_UP_ITERATIONS = 3;

    for (int iteration = 0; iteration < MAX_RIP_UP_ITERATIONS; ++iteration) {
        // Find edges that overlap with others
        std::vector<EdgeId> dirtyEdges;
        
        for (EdgeId edgeId : edges) {
            auto it = edgeLayouts.find(edgeId);
            if (it == edgeLayouts.end() || it->second.from == it->second.to) {
                continue;
            }

            // Check if this edge overlaps with any other assigned edge
            if (PathIntersection::hasOverlapWithOthers(it->second, edgeLayouts, edgeId)) {
                dirtyEdges.push_back(edgeId);
            }
        }

        if (dirtyEdges.empty()) {
#if EDGE_ROUTING_DEBUG
            std::cout << "[RipUp] Iteration " << iteration << ": No overlaps, done" << std::endl;
#endif
            break;  // No overlaps, we're done
        }

#if EDGE_ROUTING_DEBUG
        std::cout << "[RipUp] Iteration " << iteration << ": " << dirtyEdges.size() 
                  << " edges with overlaps" << std::endl;
#endif

        // Re-route dirty edges with all other edges as blocking
        for (EdgeId dirtyId : dirtyEdges) {
            auto it = edgeLayouts.find(dirtyId);
            if (it == edgeLayouts.end()) {
                continue;
            }

            EdgeLayout& layout = it->second;

            // Build obstacle map with nodes + all other edges as blocking
            ObstacleMap obstacles;
            obstacles.buildFromNodes(nodeLayouts, gridSize, 0);
            obstacles.addEdgeSegments(edgeLayouts, dirtyId);

            GridPoint startGrid = obstacles.pixelToGrid(layout.sourcePoint);
            GridPoint goalGrid = obstacles.pixelToGrid(layout.targetPoint);

            PathResult pathResult = pathFinder_->findPath(
                startGrid, goalGrid, obstacles,
                layout.from, layout.to,
                layout.sourceEdge, layout.targetEdge,
                {}, {});

            if (pathResult.found && pathResult.path.size() >= 2) {
                // Update bend points
                layout.bendPoints.clear();
                for (size_t i = 1; i + 1 < pathResult.path.size(); ++i) {
                    Point pixelPoint = obstacles.gridToPixel(
                        pathResult.path[i].x, pathResult.path[i].y);
                    layout.bendPoints.push_back({pixelPoint});
                }

#if EDGE_ROUTING_DEBUG
                std::cout << "[RipUp] Edge " << dirtyId << " re-routed successfully" << std::endl;
#endif
            }
#if EDGE_ROUTING_DEBUG
            else {
                std::cout << "[RipUp] Edge " << dirtyId << " re-route failed, keeping original" << std::endl;
            }
#endif
        }
    }
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
            edgeId, baseLayout.from, nodeLayout, 0, options);

        // Calculate penalty score using unified penalty system
        PenaltyContext ctx{assignedLayouts, nodeLayouts, forbiddenZones, effectiveGridSize()};
        ctx.sourceNodeId = candidate.from;
        ctx.targetNodeId = candidate.to;

        // Check hard constraints (segment overlap, etc.)
        if (!passesHardConstraints(candidate, ctx)) {
            continue;  // Skip combinations that violate hard constraints
        }

        int score = calculatePenalty(candidate, ctx);

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

    ObstacleMap baseObstacles;
    baseObstacles.buildFromNodes(nodeLayouts, gridSize, 0);
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

#if EDGE_ROUTING_DEBUG
    std::cout << "[CoopReroute] Edges " << edgeIdA << " & " << edgeIdB << " stats:\n"
              << "  totalCombos=" << totalCombos << " pathFoundA=" << pathFoundACount 
              << " passedHardA=" << passedHardA << "\n"
              << "  pathFoundB=" << pathFoundBCount << " passedHardB=" << passedHardB 
              << " noOverlap=" << noOverlapCount << std::endl;
    if (bestResult.valid) {
        std::cout << "[CoopReroute] Edges " << edgeIdA << " & " << edgeIdB
                  << ": found valid pair with score " << bestResult.combinedScore << std::endl;
    } else {
        std::cout << "[CoopReroute] Edges " << edgeIdA << " & " << edgeIdB
                  << ": no valid non-overlapping pair found" << std::endl;
    }
#endif

    return bestResult;
}

}  // namespace arborvia
