#include "OverlapResolver.h"
#include "NodeEdgeSelector.h"
#include "../../pathfinding/ObstacleMap.h"
#include "../../pathfinding/AStarPathFinder.h"
#include "../../routing/CooperativeRerouter.h"
#include "../../sugiyama/routing/PathIntersection.h"
#include "arborvia/common/Logger.h"

#include <algorithm>
#include <array>
#include <limits>

namespace arborvia {

// =============================================================================
// detectOverlaps
// =============================================================================

std::vector<std::pair<EdgeId, EdgeId>> OverlapResolver::detectOverlaps(
    const std::unordered_map<EdgeId, EdgeLayout>& layouts) {
    // Delegate to PathIntersection (Single Source of Truth)
    return PathIntersection::findAllOverlappingPairs(layouts);
}

// =============================================================================
// resolveAllOverlaps
// =============================================================================

bool OverlapResolver::resolveAllOverlaps(
    const EdgeRoutingContext& context,
    const std::vector<std::pair<EdgeId, EdgeId>>& overlappingPairs,
    std::unordered_map<EdgeId, EdgeLayout>& result,
    std::unordered_map<EdgeId, EdgeLayout>& assignedLayouts,
    std::unordered_set<EdgeId>* resolvedEdges) {

    bool anyResolved = false;

    for (const auto& [idA, idB] : overlappingPairs) {
        auto itA = result.find(idA);
        auto itB = result.find(idB);
        if (itA == result.end() || itB == result.end()) continue;

        LOG_DEBUG("[OverlapResolver] Resolving overlap Edge {} & {}", idA, idB);

        EdgePairResult pairResult = resolveOverlappingPair(
            context, idA, idB, itA->second, itB->second, assignedLayouts);

        LOG_DEBUG("[OverlapResolver]   resolveOverlappingPair: valid={}", pairResult.valid);

        if (pairResult.valid) {
            LOG_DEBUG("[OverlapResolver]   RESOLVED Edge {} bends:", idA);
            for (const auto& bp : pairResult.layoutA.bendPoints) {
                LOG_DEBUG("[OverlapResolver]     ({}, {})", bp.position.x, bp.position.y);
            }
            LOG_DEBUG("[OverlapResolver]   RESOLVED Edge {} bends:", idB);
            for (const auto& bp : pairResult.layoutB.bendPoints) {
                LOG_DEBUG("[OverlapResolver]     ({}, {})", bp.position.x, bp.position.y);
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
            LOG_DEBUG("[OverlapResolver]   SUCCESS via resolveOverlappingPair");
        } else {
            // Fallback: Use CooperativeRerouter when pair resolution fails
            LOG_DEBUG("[OverlapResolver]   Trying CooperativeRerouter fallback...");
            
            if (!context.pathFinder) {
                LOG_DEBUG("[OverlapResolver]   FAILED - no pathFinder available");
                continue;
            }
            
            // Create shared_ptr wrapper for pathfinder
            // Note: This is a temporary wrapper; CooperativeRerouter doesn't take ownership
            auto pathFinderShared = std::shared_ptr<IPathFinder>(
                context.pathFinder, [](IPathFinder*) {});
            
            CooperativeRerouter rerouter(pathFinderShared, context.gridSize);

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
                idA, itA->second, otherLayouts, *context.nodeLayouts);

            LOG_DEBUG("[OverlapResolver]   CooperativeRerouter: success={} reason={}",
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
                LOG_DEBUG("[OverlapResolver]   SUCCESS via CooperativeRerouter");
            } else {
                LOG_DEBUG("[OverlapResolver]   FAILED - overlap remains!");
            }
        }
    }

    return anyResolved;
}

// =============================================================================
// resolveOverlappingPair
// =============================================================================

OverlapResolver::EdgePairResult OverlapResolver::resolveOverlappingPair(
    const EdgeRoutingContext& context,
    EdgeId edgeIdA, EdgeId edgeIdB,
    const EdgeLayout& layoutA, const EdgeLayout& layoutB,
    const std::unordered_map<EdgeId, EdgeLayout>& assignedLayouts) {

    EdgePairResult bestResult;
    bestResult.valid = false;
    bestResult.combinedScore = std::numeric_limits<int>::max();

    if (!context.nodeLayouts || !context.forbiddenZones || !context.pathFinder) {
        return bestResult;
    }

    const auto& nodeLayouts = *context.nodeLayouts;
    float gridSize = context.gridSize;

    // Build obstacle map WITHOUT edges A and B
    std::unordered_map<EdgeId, EdgeLayout> otherLayouts;
    for (const auto& [id, layout] : assignedLayouts) {
        if (id != edgeIdA && id != edgeIdB) {
            otherLayouts[id] = layout;
        }
    }

    // Check if targets are Point nodes
    NodeId targetA = layoutA.to;
    NodeId targetB = layoutB.to;
    auto targetAIt = nodeLayouts.find(targetA);
    auto targetBIt = nodeLayouts.find(targetB);
    bool targetAIsPoint = (targetAIt != nodeLayouts.end() && targetAIt->second.isPointNode());
    bool targetBIsPoint = (targetBIt != nodeLayouts.end() && targetBIt->second.isPointNode());

    ObstacleMap baseObstacles;
    baseObstacles.buildFromNodes(nodeLayouts, gridSize, 0, &assignedLayouts);
    
    if (targetAIsPoint && targetBIsPoint && targetA != targetB) {
        baseObstacles.addEdgeSegmentsWithPointNodeAwareness(
            otherLayouts, nodeLayouts, INVALID_EDGE, targetA);
    } else if (targetAIsPoint || targetBIsPoint) {
        NodeId pointTarget = targetAIsPoint ? targetA : targetB;
        baseObstacles.addEdgeSegmentsWithPointNodeAwareness(
            otherLayouts, nodeLayouts, INVALID_EDGE, pointTarget);
    } else {
        baseObstacles.addEdgeSegments(otherLayouts, INVALID_EDGE);
    }

    constexpr std::array<NodeEdge, 4> allEdges = {
        NodeEdge::Top, NodeEdge::Bottom, NodeEdge::Left, NodeEdge::Right
    };

    // Evaluate all 16×16 = 256 combinations for the pair
    for (NodeEdge srcA : allEdges) {
        for (NodeEdge tgtA : allEdges) {
            ObstacleMap obstaclesA = baseObstacles;
            bool pathFoundA = false;
            EdgeLayout candidateA = NodeEdgeSelector::createCandidateLayout(
                context, layoutA, srcA, tgtA, otherLayouts, obstaclesA, pathFoundA, *context.pathFinder);

            if (!pathFoundA) continue;

            auto ctxA = context.createPenaltyContext(otherLayouts, candidateA);
            if (!context.passesHardConstraints(candidateA, ctxA)) continue;

            int scoreA = context.calculatePenalty(candidateA, ctxA);

            for (NodeEdge srcB : allEdges) {
                for (NodeEdge tgtB : allEdges) {
                    ObstacleMap obstaclesB = baseObstacles;
                    std::unordered_map<EdgeId, EdgeLayout> withA = otherLayouts;
                    withA[edgeIdA] = candidateA;
                    
                    obstaclesB.addEdgeSegmentsForLayout(layoutB, withA, nodeLayouts);

                    bool pathFoundB = false;
                    EdgeLayout candidateB = NodeEdgeSelector::createCandidateLayout(
                        context, layoutB, srcB, tgtB, withA, obstaclesB, pathFoundB, *context.pathFinder);

                    if (!pathFoundB) continue;

                    auto ctxB = context.createPenaltyContext(withA, candidateB);
                    if (!context.passesHardConstraints(candidateB, ctxB)) continue;

                    int scoreB = context.calculatePenalty(candidateB, ctxB);

                    // Check for overlap between A and B
                    std::unordered_map<EdgeId, EdgeLayout> pairLayouts;
                    pairLayouts[edgeIdA] = candidateA;
                    pairLayouts[edgeIdB] = candidateB;
                    
                    if (PathIntersection::hasOverlapWithOthers(candidateA, pairLayouts, edgeIdA)) {
                        continue;
                    }

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
            context, edgeIdA, edgeIdB, layoutA, layoutB, otherLayouts);
    }

    return bestResult;
}

// =============================================================================
// tryPathAdjustmentFallback
// =============================================================================

OverlapResolver::EdgePairResult OverlapResolver::tryPathAdjustmentFallback(
    const EdgeRoutingContext& context,
    EdgeId edgeIdA, EdgeId edgeIdB,
    const EdgeLayout& layoutA, const EdgeLayout& layoutB,
    const std::unordered_map<EdgeId, EdgeLayout>& otherLayouts) {

    EdgePairResult result;
    result.valid = false;

    if (!context.nodeLayouts || !context.forbiddenZones) {
        return result;
    }

    float gridSize = context.gridSize;

    // Strategy 1: Try adjusting layoutB's path to avoid overlap with layoutA
    {
        std::unordered_map<EdgeId, EdgeLayout> withA;
        withA[edgeIdA] = layoutA;
        auto adjustedBendsB = PathIntersection::adjustPathToAvoidOverlap(layoutB, withA, gridSize);

        EdgeLayout adjustedB = layoutB;
        adjustedB.bendPoints = adjustedBendsB;

        if (!PathIntersection::hasSegmentOverlap(layoutA, adjustedB)) {
            auto ctxA = context.createPenaltyContext(otherLayouts, layoutA);
            int scoreA = context.calculatePenalty(layoutA, ctxA);

            std::unordered_map<EdgeId, EdgeLayout> withALayout = otherLayouts;
            withALayout[edgeIdA] = layoutA;
            auto ctxB = context.createPenaltyContext(withALayout, adjustedB);
            int scoreB = context.calculatePenalty(adjustedB, ctxB);

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
            auto ctxA = context.createPenaltyContext(otherLayouts, adjustedA);
            int scoreA = context.calculatePenalty(adjustedA, ctxA);

            auto ctxB = context.createPenaltyContext(withAdjA, layoutB);
            int scoreB = context.calculatePenalty(layoutB, ctxB);

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
                        auto ctxA = context.createPenaltyContext(otherLayouts, layoutA);
                        int scoreA = context.calculatePenalty(layoutA, ctxA);

                        std::unordered_map<EdgeId, EdgeLayout> withALayout = otherLayouts;
                        withALayout[edgeIdA] = layoutA;
                        auto ctxB = context.createPenaltyContext(withALayout, adjustedB);
                        int scoreB = context.calculatePenalty(adjustedB, ctxB);

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
            float sharedPixel = overlapInfo.sharedGridCoord * gridSize;
            
            for (int dir = -1; dir <= 1; dir += 2) {
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
                    auto ctxA = context.createPenaltyContext(otherLayouts, layoutA);
                    int scoreA = context.calculatePenalty(layoutA, ctxA);

                    std::unordered_map<EdgeId, EdgeLayout> withALayout = otherLayouts;
                    withALayout[edgeIdA] = layoutA;
                    auto ctxB = context.createPenaltyContext(withALayout, adjustedB);
                    int scoreB = context.calculatePenalty(adjustedB, ctxB);

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

}  // namespace arborvia
