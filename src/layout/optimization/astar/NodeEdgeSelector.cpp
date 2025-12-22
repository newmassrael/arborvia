#include "NodeEdgeSelector.h"
#include "../../pathfinding/ObstacleMap.h"
#include "../../pathfinding/AStarPathFinder.h"
#include "../../snap/GridSnapCalculator.h"
#include "../../snap/SnapIndexManager.h"
#include "../../sugiyama/routing/SelfLoopRouter.h"
#include "arborvia/common/Logger.h"

#include <algorithm>
#include <array>
#include <cmath>

namespace arborvia {

namespace {

// Constants for evaluation
constexpr float SNAP_TOLERANCE = 5.0f;
constexpr int EARLY_RETURN_SCORE_THRESHOLD = 100;

// Helper to convert NodeEdge to index for bitmask
int edgeToIndex(NodeEdge e) {
    switch (e) {
        case NodeEdge::Top: return 0;
        case NodeEdge::Bottom: return 1;
        case NodeEdge::Left: return 2;
        case NodeEdge::Right: return 3;
        default: return 0;
    }
}

}  // namespace

// =============================================================================
// evaluateCombinations (context.pathFinder version)
// =============================================================================

std::vector<NodeEdgeSelector::CombinationResult>
NodeEdgeSelector::evaluateCombinations(
    const EdgeRoutingContext& context,
    EdgeId edgeId,
    const EdgeLayout& baseLayout,
    const std::unordered_map<EdgeId, EdgeLayout>& assignedLayouts) {

    if (!context.pathFinder) {
        return {};
    }
    return evaluateCombinations(context, edgeId, baseLayout, assignedLayouts, *context.pathFinder);
}

// =============================================================================
// evaluateCombinations (explicit pathfinder version - thread-safe)
// =============================================================================

std::vector<NodeEdgeSelector::CombinationResult>
NodeEdgeSelector::evaluateCombinations(
    const EdgeRoutingContext& context,
    EdgeId /*edgeId*/,
    const EdgeLayout& baseLayout,
    const std::unordered_map<EdgeId, EdgeLayout>& assignedLayouts,
    IPathFinder& pathFinder) {

    if (!context.nodeLayouts || !context.forbiddenZones) {
        return {};
    }

    const auto& nodeLayouts = *context.nodeLayouts;
    float gridSize = context.gridSize;

    std::vector<CombinationResult> results;
    results.reserve(16);

    // Build obstacle map once for all combinations
    ObstacleMap obstacles;
    obstacles.buildFromNodes(nodeLayouts, gridSize, 0, &assignedLayouts);
    obstacles.addEdgeSegmentsForLayout(baseLayout, assignedLayouts, nodeLayouts);

    // Lambda to evaluate a single edge combination
    auto evaluateEdge = [&](NodeEdge srcEdge, NodeEdge tgtEdge) {
        // Try keeping original bendPoints if same source/target edges
        bool sameSnapPoints = false;
        
        if (srcEdge == baseLayout.sourceEdge && tgtEdge == baseLayout.targetEdge &&
            !baseLayout.bendPoints.empty()) {
            
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
            auto ctx = context.createPenaltyContext(assignedLayouts, baseLayout);
            
            if (context.passesHardConstraints(baseLayout, ctx)) {
                int score = context.calculatePenalty(baseLayout, ctx);
                results.push_back({srcEdge, tgtEdge, score, baseLayout, true});
                return;
            }
        }

        // Generate new path with A*
        bool pathFound = false;
        EdgeLayout candidate = createCandidateLayout(
            context, baseLayout, srcEdge, tgtEdge, assignedLayouts, obstacles, pathFound, pathFinder);

        if (!pathFound) {
            return;
        }

        auto ctx = context.createPenaltyContext(assignedLayouts, candidate);

        if (!context.passesHardConstraints(candidate, ctx)) {
            return;
        }

        int score = context.calculatePenalty(candidate, ctx);
        results.push_back({srcEdge, tgtEdge, score, std::move(candidate), true});
    };

    if (context.preserveDirections) {
        evaluateEdge(baseLayout.sourceEdge, baseLayout.targetEdge);
    } else {
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
        
        // 1. Try current direction first
        if (tryAddCombo(baseLayout.sourceEdge, baseLayout.targetEdge)) {
            goto done_evaluating;
        }
        
        // 2. Try position-based optimal directions
        {
            auto srcNodeIt = nodeLayouts.find(baseLayout.from);
            auto tgtNodeIt = nodeLayouts.find(baseLayout.to);
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
    std::stable_sort(results.begin(), results.end(),
              [](const CombinationResult& a, const CombinationResult& b) {
                  if (a.score != b.score) return a.score < b.score;
                  if (a.sourceEdge != b.sourceEdge) return a.sourceEdge < b.sourceEdge;
                  return a.targetEdge < b.targetEdge;
              });

    return results;
}

// =============================================================================
// evaluateEdgeIndependent (thread-safe parallel evaluation)
// =============================================================================

NodeEdgeSelector::ParallelEdgeResult
NodeEdgeSelector::evaluateEdgeIndependent(
    const EdgeRoutingContext& context,
    EdgeId edgeId,
    const EdgeLayout& baseLayout,
    const std::unordered_map<EdgeId, EdgeLayout>& currentLayouts,
    IPathFinder& pathFinder) {

    ParallelEdgeResult presult;
    presult.edgeId = edgeId;
    presult.hasValidResult = false;

    // Handle self-loops
    if (baseLayout.from == baseLayout.to) {
        presult.isSelfLoop = true;
        auto combinations = evaluateSelfLoopCombinations(
            context, edgeId, baseLayout, currentLayouts);

        if (!combinations.empty()) {
            presult.best = combinations.front();
            presult.hasValidResult = true;
        }
        return presult;
    }

    // For regular edges, use provided pathfinder
    auto combinations = evaluateCombinations(
        context, edgeId, baseLayout, currentLayouts, pathFinder);

    if (!combinations.empty()) {
        presult.best = combinations.front();
        presult.hasValidResult = true;
    }

    return presult;
}

// =============================================================================
// evaluateSelfLoopCombinations
// =============================================================================

std::vector<NodeEdgeSelector::CombinationResult>
NodeEdgeSelector::evaluateSelfLoopCombinations(
    const EdgeRoutingContext& context,
    EdgeId edgeId,
    const EdgeLayout& baseLayout,
    const std::unordered_map<EdgeId, EdgeLayout>& assignedLayouts) {

    std::vector<CombinationResult> results;

    if (!context.nodeLayouts || !context.forbiddenZones) {
        return results;
    }

    const auto& nodeLayouts = *context.nodeLayouts;

    auto nodeIt = nodeLayouts.find(baseLayout.from);
    if (nodeIt == nodeLayouts.end()) {
        return results;
    }

    const NodeLayout& nodeLayout = nodeIt->second;

    // Calculate loop index for unique snap points
    int loopIndex = SelfLoopRouter::calculateLoopIndex(edgeId, baseLayout.from, assignedLayouts);

    // Create layout options
    LayoutOptions options;
    options.gridConfig.cellSize = context.gridSize;

    // Try all 4 self-loop directions
    std::array<SelfLoopDirection, 4> directions = {
        SelfLoopDirection::Right,
        SelfLoopDirection::Left,
        SelfLoopDirection::Top,
        SelfLoopDirection::Bottom
    };

    for (SelfLoopDirection dir : directions) {
        options.channelRouting.selfLoop.preferredDirection = dir;

        EdgeLayout candidate = SelfLoopRouter::route(
            edgeId, baseLayout.from, nodeLayout, loopIndex, options);

        auto ctx = context.createPenaltyContext(assignedLayouts, candidate);

        if (!context.passesHardConstraints(candidate, ctx)) {
            LOG_DEBUG("[SNAP-TRACE] SelfLoop edge={} REJECTED by hardConstraints srcPos=({},{})", 
                      edgeId, candidate.sourcePoint.x, candidate.sourcePoint.y);
            continue;
        }

        int score = context.calculatePenalty(candidate, ctx);
        LOG_DEBUG("[SNAP-TRACE] SelfLoop edge={} ACCEPTED srcPos=({},{}) score={}",
                  edgeId, candidate.sourcePoint.x, candidate.sourcePoint.y, score);

        NodeEdge srcEdge = candidate.sourceEdge;
        NodeEdge tgtEdge = candidate.targetEdge;

        results.push_back({srcEdge, tgtEdge, score, std::move(candidate), true});
    }

    std::sort(results.begin(), results.end(),
        [](const CombinationResult& a, const CombinationResult& b) {
            return a.score < b.score;
        });

    return results;
}

// =============================================================================
// createCandidateLayout (context.pathFinder version)
// =============================================================================

EdgeLayout NodeEdgeSelector::createCandidateLayout(
    const EdgeRoutingContext& context,
    const EdgeLayout& base,
    NodeEdge sourceEdge,
    NodeEdge targetEdge,
    const std::unordered_map<EdgeId, EdgeLayout>& assignedLayouts,
    ObstacleMap& obstacles,
    bool& pathFound) {

    if (!context.pathFinder) {
        pathFound = false;
        return base;
    }
    return createCandidateLayout(context, base, sourceEdge, targetEdge, 
                                  assignedLayouts, obstacles, pathFound, *context.pathFinder);
}

// =============================================================================
// createCandidateLayout (explicit pathfinder - thread-safe)
// =============================================================================

EdgeLayout NodeEdgeSelector::createCandidateLayout(
    const EdgeRoutingContext& context,
    const EdgeLayout& base,
    NodeEdge sourceEdge,
    NodeEdge targetEdge,
    const std::unordered_map<EdgeId, EdgeLayout>& assignedLayouts,
    ObstacleMap& obstacles,
    bool& pathFound,
    IPathFinder& pathFinder) {

    pathFound = false;

    if (!context.nodeLayouts) {
        return base;
    }

    const auto& nodeLayouts = *context.nodeLayouts;
    float gridSize = context.gridSize;

    EdgeLayout candidate = base;
    candidate.sourceEdge = sourceEdge;
    candidate.targetEdge = targetEdge;
    candidate.bendPoints.clear();

    auto srcIt = nodeLayouts.find(base.from);
    if (srcIt == nodeLayouts.end()) {
        return candidate;
    }

    auto tgtIt = nodeLayouts.find(base.to);
    if (tgtIt == nodeLayouts.end()) {
        return candidate;
    }

    // Fixed Endpoint Optimization
    bool srcFixed = context.isNodeFixed(base.from);
    bool tgtFixed = context.isNodeFixed(base.to);

    GridPoint startGrid, goalGrid;
    
    // === SOURCE SNAP POSITION ===
    if (sourceEdge == base.sourceEdge) {
        if (!srcFixed) {
            int candidateIdx = GridSnapCalculator::getCandidateIndexFromPosition(
                srcIt->second, base.sourceEdge, base.sourcePoint, gridSize);
            Point derivedPoint = GridSnapCalculator::getPositionFromCandidateIndex(
                srcIt->second, sourceEdge, candidateIdx, gridSize);
            candidate.sourceSnapIndex = candidateIdx;
            candidate.sourcePoint = derivedPoint;
        }
        startGrid = {
            static_cast<int>(std::round(candidate.sourcePoint.x / gridSize)),
            static_cast<int>(std::round(candidate.sourcePoint.y / gridSize))
        };
    } else {
        int candidateCount = GridSnapCalculator::getCandidateCount(srcIt->second, sourceEdge, gridSize);
        std::set<int> usedIndices = SnapIndexManager::collectUsedIndices(
            base.from, sourceEdge, true, assignedLayouts);
        int selectedIdx = SnapIndexManager::findFirstUnusedIndex(candidateCount, usedIndices);
        Point snapPoint = GridSnapCalculator::getPositionFromCandidateIndex(
            srcIt->second, sourceEdge, selectedIdx, gridSize);
        candidate.sourceSnapIndex = selectedIdx;
        candidate.sourcePoint = snapPoint;
        startGrid = obstacles.pixelToGrid(candidate.sourcePoint);
    }
    
    // === TARGET SNAP POSITION ===
    if (targetEdge == base.targetEdge) {
        if (!tgtFixed) {
            int candidateIdx = GridSnapCalculator::getCandidateIndexFromPosition(
                tgtIt->second, base.targetEdge, base.targetPoint, gridSize);
            Point derivedPoint = GridSnapCalculator::getPositionFromCandidateIndex(
                tgtIt->second, targetEdge, candidateIdx, gridSize);
            candidate.targetSnapIndex = candidateIdx;
            candidate.targetPoint = derivedPoint;
        }
        goalGrid = {
            static_cast<int>(std::round(candidate.targetPoint.x / gridSize)),
            static_cast<int>(std::round(candidate.targetPoint.y / gridSize))
        };
    } else {
        int candidateCount = GridSnapCalculator::getCandidateCount(tgtIt->second, targetEdge, gridSize);
        std::set<int> usedIndices = SnapIndexManager::collectUsedIndices(
            base.to, targetEdge, false, assignedLayouts);
        int selectedIdx = SnapIndexManager::findFirstUnusedIndex(candidateCount, usedIndices);
        Point snapPoint = GridSnapCalculator::getPositionFromCandidateIndex(
            tgtIt->second, targetEdge, selectedIdx, gridSize);
        candidate.targetSnapIndex = selectedIdx;
        candidate.targetPoint = snapPoint;
        goalGrid = obstacles.pixelToGrid(candidate.targetPoint);
    }

    LOG_DEBUG("[CALLER:NodeEdgeSelector.cpp] A* findPath called for edge {}->{} srcEdge={} tgtEdge={}",
              base.from, base.to, static_cast<int>(sourceEdge), static_cast<int>(targetEdge));
    PathResult pathResult = pathFinder.findPath(
        startGrid, goalGrid, obstacles,
        base.from, base.to,
        sourceEdge, targetEdge,
        {}, {});

    if (!pathResult.found || pathResult.path.size() < 2) {
        LOG_DEBUG("[ROOT-CAUSE] A* path NOT found for edge {}->{}", base.from, base.to);
        return candidate;
    }

    LOG_DEBUG("[ROOT-CAUSE] A* path found for edge {}->{}. pathLen={} startGrid=({},{}) goalGrid=({},{})",
              base.from, base.to, pathResult.path.size(),
              startGrid.x, startGrid.y, goalGrid.x, goalGrid.y);

    for (size_t i = 1; i + 1 < pathResult.path.size(); ++i) {
        Point pixelPoint = obstacles.gridToPixel(
            pathResult.path[i].x, pathResult.path[i].y);
        candidate.bendPoints.push_back({pixelPoint});
        LOG_DEBUG("[ROOT-CAUSE]   bendPoint[{}] = grid({},{}) -> pixel({},{})",
                  i - 1, pathResult.path[i].x, pathResult.path[i].y,
                  pixelPoint.x, pixelPoint.y);
    }

    // Log the path before returning
    LOG_DEBUG("[ROOT-CAUSE] Final candidate: srcPt=({},{}) bendPts={} tgtPt=({},{})",
              candidate.sourcePoint.x, candidate.sourcePoint.y,
              candidate.bendPoints.size(),
              candidate.targetPoint.x, candidate.targetPoint.y);

    // Check first segment for diagonal
    if (!candidate.bendPoints.empty()) {
        float dx = candidate.bendPoints[0].position.x - candidate.sourcePoint.x;
        float dy = candidate.bendPoints[0].position.y - candidate.sourcePoint.y;
        if (std::abs(dx) > 1.0f && std::abs(dy) > 1.0f) {
            LOG_WARN("[ROOT-CAUSE] DIAGONAL detected in A* result! edge {}->{} srcEdge={} dx={} dy={}",
                      base.from, base.to, static_cast<int>(sourceEdge), dx, dy);
        }
    } else {
        float dx = candidate.targetPoint.x - candidate.sourcePoint.x;
        float dy = candidate.targetPoint.y - candidate.sourcePoint.y;
        if (std::abs(dx) > 1.0f && std::abs(dy) > 1.0f) {
            LOG_WARN("[ROOT-CAUSE] DIAGONAL (no bends)! edge {}->{} srcEdge={} dx={} dy={}",
                      base.from, base.to, static_cast<int>(sourceEdge), dx, dy);
        }
    }

    pathFound = true;
    return candidate;
}

// =============================================================================
// Static helper methods
// =============================================================================

Point NodeEdgeSelector::calculateSnapPosition(
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

GridPoint NodeEdgeSelector::calculateGridPosition(
    const NodeLayout& node,
    NodeEdge edge,
    float gridSize,
    float position) {

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

    return {
        static_cast<int>(std::round(px / gridSize)),
        static_cast<int>(std::round(py / gridSize))
    };
}

}  // namespace arborvia
