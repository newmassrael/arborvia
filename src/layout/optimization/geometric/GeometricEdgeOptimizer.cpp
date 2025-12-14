#include "GeometricEdgeOptimizer.h"
#include "GeometricEdgeOptimizer.h"
#include "../../sugiyama/routing/EdgeRoutingUtils.h"
#include "../../sugiyama/routing/EdgeValidator.h"
#include "../../sugiyama/routing/SelfLoopRouter.h"
#include "../../pathfinding/SelfLoopPathCalculator.h"
#include "../../snap/SnapPointCalculator.h"
#include "../../snap/GridSnapCalculator.h"
#include "arborvia/core/GeometryUtils.h"
#include "arborvia/layout/api/IEdgePenalty.h"
#include "arborvia/layout/constraints/SegmentObstacleProvider.h"

#include <algorithm>
#include "arborvia/common/Logger.h"

// Debug flag for edge routing
#include <array>
#include <cmath>
#include <limits>

namespace arborvia {

using constants::EPSILON;

namespace {
    constexpr float DEFAULT_OBSTACLE_MARGIN = 15.0f;
}

// Default constructor - penalty system is set lazily in optimize() if needed

std::unordered_map<EdgeId, EdgeLayout> GeometricEdgeOptimizer::optimize(
    const std::vector<EdgeId>& edges,
    const std::unordered_map<EdgeId, EdgeLayout>& currentLayouts,
    const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
    float gridSize,
    const std::unordered_set<NodeId>& movedNodes) {

    // Store gridSize for use in helper methods
    gridSize_ = constants::effectiveGridSize(gridSize);
    
    // Ensure penalty system is set
    if (!penaltySystem()) {
        setPenaltySystem(EdgePenaltySystem::createDefault());
    }

    // Store constraint state in base class for FixedEndpointPenalty
    setConstraintState(currentLayouts, movedNodes);

    // Initialize with edges NOT being optimized (so constraints check against them)
    std::unordered_map<EdgeId, EdgeLayout> assignedLayouts;
    for (const auto& [edgeId, layout] : currentLayouts) {
        if (std::find(edges.begin(), edges.end(), edgeId) == edges.end()) {
            assignedLayouts[edgeId] = layout;
        }
    }
    std::unordered_map<EdgeId, EdgeLayout> result;

    for (EdgeId edgeId : edges) {
        auto it = currentLayouts.find(edgeId);
        if (it == currentLayouts.end()) {
            continue;
        }

        const EdgeLayout& baseLayout = it->second;

        // Skip self-loops
        // Handle self-loops with penalty-based direction selection
        if (baseLayout.from == baseLayout.to) {
            auto selfLoopCombinations = evaluateSelfLoopCombinations(
                edgeId, baseLayout, assignedLayouts, nodeLayouts);

            if (selfLoopCombinations.empty()) {
                LOG_DEBUG("[GeometricEdgeOptimizer] Edge {} - selfLoopCombinations empty, skipping", edgeId);
                continue;
            }
            const auto& best = selfLoopCombinations.front();
            result[edgeId] = best.layout;
            assignedLayouts[edgeId] = best.layout;
            continue;
        }

        // === Greedy Path Generation ===
        // When preserveDirections() is true (drag mode), we directly create the path
        // without fallback logic. The greedy algorithm always produces valid orthogonal paths.
        
        if (preserveDirections()) {
            // Direct greedy path with existing sourceEdge/targetEdge
            EdgeLayout finalLayout = createCandidateLayout(
                baseLayout, baseLayout.sourceEdge, baseLayout.targetEdge,
                nodeLayouts, assignedLayouts);
            
            result[edgeId] = finalLayout;
            assignedLayouts[edgeId] = finalLayout;
            continue;
        }

        // === Full Optimization Mode (not during drag) ===
        // Evaluate all 16 combinations and pick the best
        auto combinations = evaluateCombinations(
            edgeId, baseLayout, assignedLayouts, nodeLayouts);

        if (combinations.empty()) {
            LOG_DEBUG("[GeometricEdgeOptimizer] Edge {} - combinations empty, skipping", edgeId);
            continue;
        }
        const auto& best = combinations.front();

        // Use the pre-computed layout from evaluation (includes bendPoints)
        EdgeLayout finalLayout = baseLayout;
        finalLayout.sourceEdge = best.layout.sourceEdge;
        finalLayout.targetEdge = best.layout.targetEdge;
        finalLayout.sourcePoint = best.layout.sourcePoint;
        finalLayout.targetPoint = best.layout.targetPoint;
        finalLayout.bendPoints = best.layout.bendPoints;

        result[edgeId] = finalLayout;
        assignedLayouts[edgeId] = finalLayout;
    }

    return result;
}

std::vector<GeometricEdgeOptimizer::CombinationResult>
GeometricEdgeOptimizer::evaluateCombinations(
    EdgeId /*edgeId*/,
    const EdgeLayout& baseLayout,
    const std::unordered_map<EdgeId, EdgeLayout>& assignedLayouts,
    const std::unordered_map<NodeId, NodeLayout>& nodeLayouts) {

    std::vector<CombinationResult> results;
    results.reserve(16);

    // Lambda to evaluate a single edge combination
    auto evaluateEdge = [&](NodeEdge srcEdge, NodeEdge tgtEdge) {
        EdgeLayout candidate = createCandidateLayout(
            baseLayout, srcEdge, tgtEdge, nodeLayouts, assignedLayouts);

        // Skip if nodes not found
        auto srcIt = nodeLayouts.find(baseLayout.from);
        auto tgtIt = nodeLayouts.find(baseLayout.to);
        if (srcIt == nodeLayouts.end() || tgtIt == nodeLayouts.end()) {
            return;
        }

        int score = scoreGeometricPath(candidate, assignedLayouts, nodeLayouts);

        // Always include all combinations - sort by score to pick best
        results.push_back({srcEdge, tgtEdge, score, std::move(candidate)});
    };

    if (preserveDirections()) {
        // Preserve existing direction - only evaluate current combination
        evaluateEdge(baseLayout.sourceEdge, baseLayout.targetEdge);
    } else {
        // Pre-filter combinations based on fixed endpoints
        // If a node is fixed, only its original edge direction is valid
        // This prevents FixedEndpointPenalty from rejecting 15/16 combinations
        bool srcFixed = isNodeFixed(baseLayout.from);
        bool tgtFixed = isNodeFixed(baseLayout.to);
        
        constexpr std::array<NodeEdge, 4> allEdges = {
            NodeEdge::Top, NodeEdge::Bottom, NodeEdge::Left, NodeEdge::Right
        };
        
        // Determine which edges to try for source and target
        std::vector<NodeEdge> srcEdgesToTry;
        std::vector<NodeEdge> tgtEdgesToTry;
        
        if (srcFixed) {
            srcEdgesToTry.push_back(baseLayout.sourceEdge);
        } else {
            srcEdgesToTry = {allEdges.begin(), allEdges.end()};
        }
        
        if (tgtFixed) {
            tgtEdgesToTry.push_back(baseLayout.targetEdge);
        } else {
            tgtEdgesToTry = {allEdges.begin(), allEdges.end()};
        }
        
        for (NodeEdge srcEdge : srcEdgesToTry) {
            for (NodeEdge tgtEdge : tgtEdgesToTry) {
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

EdgeLayout GeometricEdgeOptimizer::createCandidateLayout(
    const EdgeLayout& base,
    NodeEdge sourceEdge,
    NodeEdge targetEdge,
    const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
    const std::unordered_map<EdgeId, EdgeLayout>& assignedLayouts) {

    EdgeLayout candidate = base;
    candidate.sourceEdge = sourceEdge;
    candidate.targetEdge = targetEdge;

    // Check if endpoints should be preserved (on fixed nodes)
    bool srcFixed = isNodeFixed(base.from);
    bool tgtFixed = isNodeFixed(base.to);

    // NOTE: snapIndex is no longer stored - computed from position as needed

    auto srcIt = nodeLayouts.find(base.from);
    auto tgtIt = nodeLayouts.find(base.to);

    if (srcIt == nodeLayouts.end() || tgtIt == nodeLayouts.end()) {
        return candidate;
    }

    // Calculate source/target points
    // Preserve existing position for fixed endpoints
    // Use GridSnapCalculator (SSOT) for proper distribution based on existing connections
    Point sourcePoint, targetPoint;
    
    if (srcFixed && sourceEdge == base.sourceEdge) {
        // Source is fixed - preserve existing position
        sourcePoint = base.sourcePoint;
    } else {
        // Calculate position considering existing connections on this node edge
        sourcePoint = calculateSnapPositionWithContext(
            srcIt->second, sourceEdge, base, assignedLayouts, true /*isSource*/);
    }
    
    if (tgtFixed && targetEdge == base.targetEdge) {
        // Target is fixed - preserve existing position
        targetPoint = base.targetPoint;
    } else {
        // Calculate position considering existing connections on this node edge
        targetPoint = calculateSnapPositionWithContext(
            tgtIt->second, targetEdge, base, assignedLayouts, false /*isSource*/);
    }

    // === Step 1: Create greedy orthogonal path with node avoidance ===
    auto bendPoints = createPathWithObstacleAvoidance(
        sourcePoint, targetPoint, sourceEdge, targetEdge,
        nodeLayouts, base.from, base.to);

    candidate.sourcePoint = sourcePoint;
    candidate.targetPoint = targetPoint;
    candidate.bendPoints = std::move(bendPoints);

    // === Step 2: Iteratively avoid overlap AND re-check node collisions ===
    // After adjusting for overlap, the new path might penetrate nodes.
    // We need to re-run obstacle avoidance if that happens.
    if (!assignedLayouts.empty()) {
        constexpr int MAX_ITERATIONS = 5;
        
        for (int iter = 0; iter < MAX_ITERATIONS; ++iter) {
            // Check for edge overlap
            auto overlapInfo = PathIntersection::findOverlapInfo(candidate, assignedLayouts, candidate.id);
            if (!overlapInfo.found) {
                break;  // No overlap, we're done
            }
            
            // Adjust path to avoid overlap
            auto adjustedBends = adjustPathToAvoidOverlap(candidate, assignedLayouts);
            if (adjustedBends.empty() && !candidate.bendPoints.empty()) {
                break;  // Adjustment failed
            }
            candidate.bendPoints = std::move(adjustedBends);

            // === Re-check for node collisions after overlap adjustment ===
            // The adjusted path might now penetrate nodes
            bool hasNodeCollision = false;
            
            std::vector<Point> pathPoints;
            pathPoints.push_back(candidate.sourcePoint);
            for (const auto& bp : candidate.bendPoints) {
                pathPoints.push_back(bp.position);
            }
            pathPoints.push_back(candidate.targetPoint);

            LOG_DEBUG("[evaluateCombinations] Checking {} segments for collision (edge from={} to={})",
                      pathPoints.size() - 1, base.from, base.to);

            for (size_t i = 0; i + 1 < pathPoints.size(); ++i) {
                auto collisions = findCollidingNodes(
                    pathPoints[i], pathPoints[i + 1], 
                    nodeLayouts, base.from, base.to);
                if (!collisions.empty()) {
                    LOG_DEBUG("[evaluateCombinations] COLLISION at segment {}: ({},{})->({},{}) hits {} nodes",
                              i, pathPoints[i].x, pathPoints[i].y, 
                              pathPoints[i + 1].x, pathPoints[i + 1].y, collisions.size());
                    hasNodeCollision = true;
                    break;
                }
            }

            if (hasNodeCollision) {
                LOG_DEBUG("[evaluateCombinations] Re-running createPathWithObstacleAvoidance for collision fix");
                // Re-run obstacle avoidance with the current source/target points
                candidate.bendPoints = createPathWithObstacleAvoidance(
                    candidate.sourcePoint, candidate.targetPoint, 
                    sourceEdge, targetEdge,
                    nodeLayouts, base.from, base.to);
                LOG_DEBUG("[evaluateCombinations] After obstacle avoidance: {} bends", candidate.bendPoints.size());
            }
        }
    }

    return candidate;
}

Point GeometricEdgeOptimizer::calculateEdgeCenter(
    const NodeLayout& node,
    NodeEdge edge) const {
    // Use SnapPointCalculator for grid-aligned snap points (A* standard)
    float effectiveGridSize = constants::effectiveGridSize(gridSize_);
    return SnapPointCalculator::calculateFromRatio(node, edge, 0.5f, effectiveGridSize);
}

Point GeometricEdgeOptimizer::calculateSnapPositionWithContext(
    const NodeLayout& node,
    NodeEdge nodeEdge,
    const EdgeLayout& currentEdge,
    const std::unordered_map<EdgeId, EdgeLayout>& assignedLayouts,
    bool isSource) const {
    
    float effectiveGridSize = constants::effectiveGridSize(gridSize_);
    NodeId nodeId = isSource ? currentEdge.from : currentEdge.to;
    
    // Count existing connections on this node edge from already assigned layouts
    int existingConnections = 0;
    for (const auto& [edgeId, layout] : assignedLayouts) {
        // Check source side connections
        if (layout.from == nodeId && layout.sourceEdge == nodeEdge) {
            ++existingConnections;
        }
        // Check target side connections
        if (layout.to == nodeId && layout.targetEdge == nodeEdge) {
            ++existingConnections;
        }
    }
    
    // The current edge is the next connection (connectionIndex = existingConnections)
    // totalConnections = existingConnections + 1 (including current edge)
    int connectionIndex = existingConnections;
    int totalConnections = existingConnections + 1;
    
    return GridSnapCalculator::calculateSnapPosition(
        node, nodeEdge, connectionIndex, totalConnections, effectiveGridSize);
}

std::vector<BendPoint> GeometricEdgeOptimizer::predictOrthogonalPath(
    const Point& source,
    const Point& target,
    NodeEdge sourceEdge,
    NodeEdge targetEdge) {

    std::vector<BendPoint> bends;

    float dx = target.x - source.x;
    float dy = target.y - source.y;

    // Determine if horizontal or vertical edges
    bool srcHorizontal = (sourceEdge == NodeEdge::Top || sourceEdge == NodeEdge::Bottom);
    bool tgtHorizontal = (targetEdge == NodeEdge::Top || targetEdge == NodeEdge::Bottom);

    // Same orientation - need intermediate bend point
    if (srcHorizontal == tgtHorizontal) {
        if (srcHorizontal) {
            // Both horizontal: go vertical first, then horizontal
            float midY = source.y + dy * 0.5f;
            bends.push_back({{source.x, midY}});
            bends.push_back({{target.x, midY}});
        } else {
            // Both vertical: go horizontal first, then vertical
            float midX = source.x + dx * 0.5f;
            bends.push_back({{midX, source.y}});
            bends.push_back({{midX, target.y}});
        }
    } else {
        // Different orientation - single L-bend
        if (srcHorizontal) {
            // Start horizontal (from top/bottom), end vertical (from left/right)
            bends.push_back({{source.x, target.y}});
        } else {
            // Start vertical (from left/right), end horizontal (from top/bottom)
            bends.push_back({{target.x, source.y}});
        }
    }

    return bends;
}

// ============================================================================
// Obstacle Avoidance Implementation
// ============================================================================

bool GeometricEdgeOptimizer::segmentIntersectsNode(
    const Point& p1,
    const Point& p2,
    const NodeLayout& node,
    float margin) {
    return EdgeValidator::segmentIntersectsNode(p1, p2, node, margin);
}

std::vector<std::pair<NodeId, NodeLayout>> GeometricEdgeOptimizer::findCollidingNodes(
    const Point& p1,
    const Point& p2,
    const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
    NodeId sourceNode,
    NodeId targetNode,
    SegmentPosition segmentPos) {

    std::vector<std::pair<NodeId, NodeLayout>> colliding;

    // Skip zero-length segments (p1 == p2) - they can't collide with anything
    if (std::abs(p1.x - p2.x) < EPSILON && std::abs(p1.y - p2.y) < EPSILON) {
        return colliding;
    }

    // Helper to check if point is inside node (including boundary)
    auto isPointInsideNode = [](const Point& p, const NodeLayout& n) -> bool {
        return p.x >= n.position.x && p.x <= n.position.x + n.size.width &&
               p.y >= n.position.y && p.y <= n.position.y + n.size.height;
    };

    for (const auto& [nodeId, node] : nodeLayouts) {
        // Use SegmentObstacleProvider for consistent exclusion logic (Single Source of Truth)
        // - First segment: only source node excluded
        // - Middle segment: no exclusions (all nodes checked including src/tgt)
        // - Last segment: only target node excluded
        if (SegmentObstacleProvider::shouldExcludeNode(
                nodeId, sourceNode, targetNode, segmentPos)) {
            continue;
        }

        // ONLY skip when BOTH endpoints are inside the same node (node overlap scenario)
        // This is unavoidable at path level - the nodes themselves are overlapping
        if (isPointInsideNode(p1, node) && isPointInsideNode(p2, node)) {
            LOG_DEBUG("[findCollidingNodes] SKIP node {} - BOTH endpoints inside (overlap)! p1=({},{}) p2=({},{})",
                      nodeId, p1.x, p1.y, p2.x, p2.y);
            continue;
        }

        if (segmentIntersectsNode(p1, p2, node, 0.0f)) {
            LOG_DEBUG("[findCollidingNodes] COLLISION: segment ({},{})->({},{}) intersects node {} at ({},{} {}x{})",
                      p1.x, p1.y, p2.x, p2.y, nodeId,
                      node.position.x, node.position.y, node.size.width, node.size.height);
            colliding.emplace_back(nodeId, node);
        }
    }

    return colliding;
}

std::vector<Point> GeometricEdgeOptimizer::createDetourAroundNode(
    const Point& source,
    const Point& target,
    const NodeLayout& obstacle,
    float margin) {

    std::vector<Point> detourPoints;

    // Calculate obstacle bounds with margin
    float obsLeft = obstacle.position.x - margin;
    float obsRight = obstacle.position.x + obstacle.size.width + margin;
    float obsTop = obstacle.position.y - margin;
    float obsBottom = obstacle.position.y + obstacle.size.height + margin;

    // Determine if segment is primarily horizontal or vertical
    bool isHorizontal = std::abs(source.y - target.y) < std::abs(source.x - target.x);

    if (isHorizontal) {
        // Horizontal segment blocked by node - go above or below
        float centerY = obstacle.position.y + obstacle.size.height * 0.5f;
        float avgY = (source.y + target.y) * 0.5f;

        // Choose direction: go above if source/target are above center, else below
        bool goAbove = (avgY < centerY);
        float detourY = goAbove ? obsTop : obsBottom;

        // Create detour: source → up/down → around → target
        // For horizontal movement with vertical detour
        detourPoints.push_back({source.x, detourY});
        detourPoints.push_back({target.x, detourY});
    } else {
        // Vertical segment blocked by node - go left or right
        float centerX = obstacle.position.x + obstacle.size.width * 0.5f;
        float avgX = (source.x + target.x) * 0.5f;

        // Choose direction: go left if source/target are left of center, else right
        bool goLeft = (avgX < centerX);
        float detourX = goLeft ? obsLeft : obsRight;

        // Create detour: source → left/right → around → target
        detourPoints.push_back({detourX, source.y});
        detourPoints.push_back({detourX, target.y});
    }

    return detourPoints;
}

std::vector<BendPoint> GeometricEdgeOptimizer::createPathWithObstacleAvoidance(
    const Point& source,
    const Point& target,
    NodeEdge sourceEdge,
    NodeEdge targetEdge,
    const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
    NodeId sourceNodeId,
    NodeId targetNodeId) {

    LOG_DEBUG("[createPathWithObstacleAvoidance] ENTRY: src=({},{}) tgt=({},{}) srcEdge={} tgtEdge={} from={} to={} nodes={}",
              source.x, source.y, target.x, target.y,
              static_cast<int>(sourceEdge), static_cast<int>(targetEdge),
              sourceNodeId, targetNodeId, nodeLayouts.size());

    // Build full path as points: source → bends → target
    std::vector<Point> path;
    path.push_back(source);
    
    // Helper: check if a horizontal line at Y would intersect any obstacle
    // when the vertical segments from source/target to that Y are considered
    // Check if horizontal segment at midY would intersect any obstacle
    // This is for MIDDLE segments of Z-path, so use SegmentPosition::Middle
    // which means NO nodes are excluded - source/target must also be avoided
    auto yIntersectsObstacle = [&](float midY) -> const NodeLayout* {
        for (const auto& [nodeId, node] : nodeLayouts) {
            // Use SegmentObstacleProvider for consistent exclusion logic
            // Middle segments must check ALL nodes including source/target
            if (SegmentObstacleProvider::shouldExcludeNode(
                    nodeId, sourceNodeId, targetNodeId, SegmentPosition::Middle)) {
                continue;
            }
            
            float nodeYmin = node.position.y;
            float nodeYmax = node.position.y + node.size.height;
            float nodeXmin = node.position.x;
            float nodeXmax = node.position.x + node.size.width;
            
            // Check if the horizontal segment at midY passes through this node
            if (midY >= nodeYmin && midY <= nodeYmax) {
                // midY is inside or on boundary of node's Y range
                // Check if either source.x or target.x would pass through node's X range
                if ((source.x >= nodeXmin && source.x <= nodeXmax) ||
                    (target.x >= nodeXmin && target.x <= nodeXmax)) {
                    return &node;
                }
            }
            
            // Check if vertical segments from source/target to midY would pass through node
            // Segment from (source.x, source.y) to (source.x, midY)
            float srcYmin = std::min(source.y, midY);
            float srcYmax = std::max(source.y, midY);
            if (source.x >= nodeXmin && source.x <= nodeXmax) {
                // source.x is inside node's X range
                if (srcYmax >= nodeYmin && srcYmin <= nodeYmax) {
                    // Y ranges overlap - segment passes through node
                    return &node;
                }
            }
            
            // Segment from (target.x, midY) to (target.x, target.y)
            float tgtYmin = std::min(target.y, midY);
            float tgtYmax = std::max(target.y, midY);
            if (target.x >= nodeXmin && target.x <= nodeXmax) {
                // target.x is inside node's X range
                if (tgtYmax >= nodeYmin && tgtYmin <= nodeYmax) {
                    // Y ranges overlap - segment passes through node
                    return &node;
                }
            }
        }
        return nullptr;
    };
    
    // Compute obstacle-aware orthogonal path using grid-based calculations
    bool srcHorizontal = (sourceEdge == NodeEdge::Top || sourceEdge == NodeEdge::Bottom);
    bool tgtHorizontal = (targetEdge == NodeEdge::Top || targetEdge == NodeEdge::Bottom);
    
    if (srcHorizontal == tgtHorizontal) {
        if (srcHorizontal) {
            // Both horizontal: need to go vertical first
            // Find a midY that doesn't intersect obstacles (grid-based calculation)
            auto srcGrid = GridPoint::fromPixel(source, gridSize_);
            auto tgtGrid = GridPoint::fromPixel(target, gridSize_);
            int midYGrid = (srcGrid.y + tgtGrid.y) / 2;

            // Check if midY intersects any obstacle
            float midY = midYGrid * gridSize_;
            if (const NodeLayout* obstacle = yIntersectsObstacle(midY)) {
                // Try above or below the obstacle (grid-aligned)
                int marginCells = grid::pixelToCells(DEFAULT_OBSTACLE_MARGIN, gridSize_);
                auto obsGrid = GridPoint::fromPixel(obstacle->position, gridSize_);
                int obsHeightCells = grid::pixelToCells(obstacle->size.height, gridSize_);
                int aboveYGrid = obsGrid.y - marginCells;
                int belowYGrid = obsGrid.y + obsHeightCells + marginCells;

                // Pick the one closer to midY (in grid units)
                if (std::abs(midYGrid - aboveYGrid) < std::abs(midYGrid - belowYGrid)) {
                    midYGrid = aboveYGrid;
                } else {
                    midYGrid = belowYGrid;
                }
            }

            // Convert to pixel at final output
            float finalMidY = midYGrid * gridSize_;
            // Only add intermediate points if midY differs from source.y
            // Otherwise we'd create duplicate points (source == midPoint)
            if (std::abs(finalMidY - source.y) > EPSILON) {
                path.push_back({source.x, finalMidY});
                path.push_back({target.x, finalMidY});
            }
            // else: direct horizontal path from source to target (no bends needed)
        } else {
            // Both vertical: need to go horizontal first (grid-based calculation)
            auto srcGrid = GridPoint::fromPixel(source, gridSize_);
            auto tgtGrid = GridPoint::fromPixel(target, gridSize_);
            int midXGrid = (srcGrid.x + tgtGrid.x) / 2;
            float midX = midXGrid * gridSize_;

            // Check if midX or the horizontal segments would intersect any obstacle
            for (const auto& [nodeId, node] : nodeLayouts) {
                if (nodeId == sourceNodeId || nodeId == targetNodeId) continue;

                float nodeXmin = node.position.x;
                float nodeXmax = node.position.x + node.size.width;
                float nodeYmin = node.position.y;
                float nodeYmax = node.position.y + node.size.height;

                // Check if the vertical line at midX passes through node's X range
                bool midXInNode = (midX >= nodeXmin && midX <= nodeXmax);

                // Check if horizontal segments from source/target to midX would hit node
                float srcXmin = std::min(source.x, midX);
                float srcXmax = std::max(source.x, midX);
                bool srcSegmentHitsX = (srcXmax >= nodeXmin && srcXmin <= nodeXmax);
                bool srcSegmentHitsY = (source.y >= nodeYmin && source.y <= nodeYmax);

                float tgtXmin = std::min(target.x, midX);
                float tgtXmax = std::max(target.x, midX);
                bool tgtSegmentHitsX = (tgtXmax >= nodeXmin && tgtXmin <= nodeXmax);
                bool tgtSegmentHitsY = (target.y >= nodeYmin && target.y <= nodeYmax);

                bool wouldIntersect = false;

                // Vertical segment at midX from source.y to target.y
                if (midXInNode) {
                    float yMin = std::min(source.y, target.y);
                    float yMax = std::max(source.y, target.y);
                    if (yMax >= nodeYmin && yMin <= nodeYmax) {
                        wouldIntersect = true;
                    }
                }

                // Horizontal segments
                if (srcSegmentHitsX && srcSegmentHitsY) {
                    wouldIntersect = true;
                }
                if (tgtSegmentHitsX && tgtSegmentHitsY) {
                    wouldIntersect = true;
                }

                if (wouldIntersect) {
                    // Path would intersect, adjust midX (grid-aligned)
                    int marginCells = grid::pixelToCells(DEFAULT_OBSTACLE_MARGIN, gridSize_);
                    auto nodeGrid = GridPoint::fromPixel(node.position, gridSize_);
                    int nodeWidthCells = grid::pixelToCells(node.size.width, gridSize_);
                    int leftXGrid = nodeGrid.x - marginCells;
                    int rightXGrid = nodeGrid.x + nodeWidthCells + marginCells;

                    if (std::abs(midXGrid - leftXGrid) < std::abs(midXGrid - rightXGrid)) {
                        midXGrid = leftXGrid;
                    } else {
                        midXGrid = rightXGrid;
                    }
                    midX = midXGrid * gridSize_;
                    break;
                }
            }

            LOG_DEBUG("[createPath] Both vertical: midX={} (grid={}) src=({},{}) tgt=({},{})",
                      midX, midXGrid, source.x, source.y, target.x, target.y);
            // Only add intermediate points if midX differs from source.x
            // Otherwise we'd create duplicate points (source == midPoint)
            if (std::abs(midX - source.x) > EPSILON) {
                path.push_back({midX, source.y});
                path.push_back({midX, target.y});
            }
            // else: direct vertical path from source to target (no bends needed)
        }
    } else {
        // Different orientation - single L-bend
        if (srcHorizontal) {
            path.push_back({source.x, target.y});
        } else {
            path.push_back({target.x, source.y});
        }
    }
    
    path.push_back(target);

    // Helper lambda: check if a detour creates any collision
    auto detourHasCollision = [&](const std::vector<Point>& detourPath) -> bool {
        for (size_t j = 0; j + 1 < detourPath.size(); ++j) {
            auto cols = findCollidingNodes(detourPath[j], detourPath[j + 1], 
                                           nodeLayouts, sourceNodeId, targetNodeId);
            if (!cols.empty()) {
                return true;
            }
        }
        return false;
    };

    // === Iterative Greedy Collision Resolution ===
    constexpr int MAX_ITERATIONS = 25;
    constexpr int MAX_PATH_LENGTH = 150;

    for (int iter = 0; iter < MAX_ITERATIONS && path.size() < MAX_PATH_LENGTH; ++iter) {
        bool anyCollision = false;

        for (size_t i = 0; i + 1 < path.size(); ++i) {
            const Point& p1 = path[i];
            const Point& p2 = path[i + 1];

            // Use Middle position - the "BOTH endpoints inside" check in findCollidingNodes
            // will handle source/target exclusion for endpoints that touch their own nodes
            auto collisions = findCollidingNodes(p1, p2, nodeLayouts, sourceNodeId, targetNodeId);
            if (collisions.empty()) {
                continue;
            }

            anyCollision = true;

            // Find closest colliding node
            const NodeLayout* closestObstacle = nullptr;
            float closestDist = std::numeric_limits<float>::max();

            for (const auto& [nodeId, node] : collisions) {
                float dist = p1.distanceTo(node.center());
                if (dist < closestDist) {
                    closestDist = dist;
                    closestObstacle = &node;
                }
            }

            if (!closestObstacle) {
                continue;
            }

            // === Create Orthogonal Detour - Try All 4 Directions (grid-aligned) ===
            int marginCells = grid::pixelToCells(DEFAULT_OBSTACLE_MARGIN, gridSize_);
            auto obsGrid = GridPoint::fromPixel(closestObstacle->position, gridSize_);
            int obsWidthCells = grid::pixelToCells(closestObstacle->size.width, gridSize_);
            int obsHeightCells = grid::pixelToCells(closestObstacle->size.height, gridSize_);
            float obsLeft = (obsGrid.x - marginCells) * gridSize_;
            float obsRight = (obsGrid.x + obsWidthCells + marginCells) * gridSize_;
            float obsTop = (obsGrid.y - marginCells) * gridSize_;
            float obsBottom = (obsGrid.y + obsHeightCells + marginCells) * gridSize_;
            LOG_DEBUG("[createPath] Detour: marginCells={} obsGrid=({},{}) obsLeft={} obsRight={} obsTop={} obsBottom={}",
                      marginCells, obsGrid.x, obsGrid.y, obsLeft, obsRight, obsTop, obsBottom);

            // Determine segment type
            bool isHorizontal = std::abs(p1.y - p2.y) < EPSILON;
            bool isVertical = std::abs(p1.x - p2.x) < EPSILON;

            std::vector<Point> bestDetour;

            if (isHorizontal) {
                // Horizontal segment: try going above or below the obstacle
                // Try above
                std::vector<Point> detourAbove = {p1, {p1.x, obsTop}, {p2.x, obsTop}, p2};
                // Try below
                std::vector<Point> detourBelow = {p1, {p1.x, obsBottom}, {p2.x, obsBottom}, p2};

                if (!detourHasCollision(detourAbove)) {
                    bestDetour = {{p1.x, obsTop}, {p2.x, obsTop}};
                } else if (!detourHasCollision(detourBelow)) {
                    bestDetour = {{p1.x, obsBottom}, {p2.x, obsBottom}};
                } else {
                    // Both have collision - pick shorter route and let next iteration handle it
                    float distAbove = std::abs(p1.y - obsTop);
                    float distBelow = std::abs(p1.y - obsBottom);
                    if (distAbove < distBelow) {
                        bestDetour = {{p1.x, obsTop}, {p2.x, obsTop}};
                    } else {
                        bestDetour = {{p1.x, obsBottom}, {p2.x, obsBottom}};
                    }
                }
            } else if (isVertical) {
                // For vertical segment: check if p2.y is inside obstacle's Y bounds
                // If so, the return segment from detour will re-enter the obstacle
                bool p2InsideY = (p2.y > closestObstacle->position.y - EPSILON && 
                                  p2.y < closestObstacle->position.y + closestObstacle->size.height + EPSILON);
                
                if (p2InsideY) {
                    // Need to go completely around the obstacle in both X and Y
                    // Determine which Y direction to exit (above or below obstacle)
                    float distToTop = std::abs(p2.y - obsTop);
                    float distToBottom = std::abs(p2.y - obsBottom);
                    float exitY = (distToTop < distToBottom) ? obsTop : obsBottom;
                    
                    // Try left path with full bypass
                    std::vector<Point> detourLeft = {
                        p1, 
                        {obsLeft, p1.y},     // Go left
                        {obsLeft, exitY},    // Go to exit Y (above or below obstacle)
                        {p2.x, exitY},       // Go right to target X
                        p2                   // Continue to target
                    };
                    
                    // Try right path with full bypass
                    std::vector<Point> detourRight = {
                        p1,
                        {obsRight, p1.y},    // Go right
                        {obsRight, exitY},   // Go to exit Y
                        {p2.x, exitY},       // Go left to target X
                        p2                   // Continue to target
                    };
                    
                    if (!detourHasCollision(detourLeft)) {
                        bestDetour = {{obsLeft, p1.y}, {obsLeft, exitY}, {p2.x, exitY}};
                    } else if (!detourHasCollision(detourRight)) {
                        bestDetour = {{obsRight, p1.y}, {obsRight, exitY}, {p2.x, exitY}};
                    } else {
                        // Both have collision - try the other exit Y
                        exitY = (distToTop < distToBottom) ? obsBottom : obsTop;
                        detourLeft = {p1, {obsLeft, p1.y}, {obsLeft, exitY}, {p2.x, exitY}, p2};
                        detourRight = {p1, {obsRight, p1.y}, {obsRight, exitY}, {p2.x, exitY}, p2};
                        
                        if (!detourHasCollision(detourLeft)) {
                            bestDetour = {{obsLeft, p1.y}, {obsLeft, exitY}, {p2.x, exitY}};
                        } else if (!detourHasCollision(detourRight)) {
                            bestDetour = {{obsRight, p1.y}, {obsRight, exitY}, {p2.x, exitY}};
                        }
                        // No fallback - if both detours collide, bestDetour stays empty
                    }
                } else {
                    // p2 is outside obstacle's Y range - simple detour works
                    // Try left
                    std::vector<Point> detourLeft = {p1, {obsLeft, p1.y}, {obsLeft, p2.y}, p2};
                    // Try right
                    std::vector<Point> detourRight = {p1, {obsRight, p1.y}, {obsRight, p2.y}, p2};

                    if (!detourHasCollision(detourLeft)) {
                        bestDetour = {{obsLeft, p1.y}, {obsLeft, p2.y}};
                    } else if (!detourHasCollision(detourRight)) {
                        bestDetour = {{obsRight, p1.y}, {obsRight, p2.y}};
                    } else {
                        float distLeft = std::abs(p1.x - obsLeft);
                        float distRight = std::abs(p1.x - obsRight);
                        if (distLeft < distRight) {
                            bestDetour = {{obsLeft, p1.y}, {obsLeft, p2.y}};
                        } else {
                            bestDetour = {{obsRight, p1.y}, {obsRight, p2.y}};
                        }
                    }
                }
            } else {
                // Non-orthogonal segment detected: try both L-bend options
                // Option A: horizontal first (p1 -> (p2.x, p1.y) -> p2)
                // Option B: vertical first (p1 -> (p1.x, p2.y) -> p2)
                std::vector<Point> optionA = {p1, {p2.x, p1.y}, p2};
                std::vector<Point> optionB = {p1, {p1.x, p2.y}, p2};
                
                bool aHasCollision = detourHasCollision(optionA);
                bool bHasCollision = detourHasCollision(optionB);
                
                if (!aHasCollision) {
                    bestDetour = {{p2.x, p1.y}};  // horizontal first
                } else if (!bHasCollision) {
                    bestDetour = {{p1.x, p2.y}};  // vertical first
                } else {
                    // Both have collision - default to vertical first
                    bestDetour = {{p1.x, p2.y}};
                }
            }

            if (!bestDetour.empty()) {
                path.insert(path.begin() + static_cast<long>(i) + 1, 
                           bestDetour.begin(), bestDetour.end());
            }
            break;
        }

        if (!anyCollision) {
            break;
        }
    }

    // === Path Cleanup: Remove redundant collinear points ===
    std::vector<Point> cleanPath;
    cleanPath.push_back(path[0]);

    for (size_t i = 1; i + 1 < path.size(); ++i) {
        const Point& prev = cleanPath.back();
        const Point& curr = path[i];
        const Point& next = path[i + 1];

        bool collinearX = std::abs(prev.x - curr.x) < EPSILON && std::abs(curr.x - next.x) < EPSILON;
        bool collinearY = std::abs(prev.y - curr.y) < EPSILON && std::abs(curr.y - next.y) < EPSILON;

        if (!collinearX && !collinearY) {
            cleanPath.push_back(curr);
        }
    }
    cleanPath.push_back(path.back());

    // === Convert to BendPoints ===
    std::vector<BendPoint> result;
    for (size_t i = 1; i + 1 < cleanPath.size(); ++i) {
        result.push_back({cleanPath[i]});
    }

    // No L-bend/Z-path fallback - path should be generated by main logic above

    // No post-validation L-bend/Z-path regeneration - trust main path generation

    // Log final path for debugging
    LOG_DEBUG("[createPathWithObstacleAvoidance] EXIT: {} bends", result.size());
    for (size_t i = 0; i < result.size(); ++i) {
        LOG_DEBUG("[createPathWithObstacleAvoidance]   bend[{}] = ({},{})", 
                  i, result[i].position.x, result[i].position.y);
    }

    return result;
}

int GeometricEdgeOptimizer::scoreGeometricPath(
    const EdgeLayout& candidate,
    const std::unordered_map<EdgeId, EdgeLayout>& assignedLayouts,
    const std::unordered_map<NodeId, NodeLayout>& nodeLayouts) {

    // Use unified penalty system
    // Note: Geometric optimizer doesn't use forbiddenZones for speed
    // ForbiddenZonePenalty will return 0 with empty zones
    static const std::vector<ForbiddenZone> emptyZones;
    PenaltyContext ctx{assignedLayouts, nodeLayouts, emptyZones, gridSize_};
    ctx.sourceNodeId = candidate.from;
    ctx.targetNodeId = candidate.to;
    ctx.originalLayouts = originalLayouts_;
    ctx.movedNodes = movedNodes_;
    return penaltySystem()->calculateTotalPenalty(candidate, ctx);
}

// =============================================================================
// Overlap Avoidance Implementation
// =============================================================================

std::vector<BendPoint> GeometricEdgeOptimizer::adjustPathToAvoidOverlap(
    const EdgeLayout& candidate,
    const std::unordered_map<EdgeId, EdgeLayout>& assignedLayouts) {
    return PathIntersection::adjustPathToAvoidOverlap(candidate, assignedLayouts, gridSize_);
}

float GeometricEdgeOptimizer::findAlternativeX(
    float originalX,
    float yMin,
    float yMax,
    const std::unordered_map<EdgeId, EdgeLayout>& assignedLayouts,
    float gridSpacing) {
    return PathIntersection::findAlternativeX(originalX, yMin, yMax, assignedLayouts, gridSpacing);
}

float GeometricEdgeOptimizer::findAlternativeY(
    float originalY,
    float xMin,
    float xMax,
    const std::unordered_map<EdgeId, EdgeLayout>& assignedLayouts,
    float gridSpacing) {
    return PathIntersection::findAlternativeY(originalY, xMin, xMax, assignedLayouts, gridSpacing);
}

// =============================================================================
// BendPoints Regeneration (preserves sourceEdge/targetEdge)
// =============================================================================

void GeometricEdgeOptimizer::regenerateBendPoints(
    const std::vector<EdgeId>& edges,
    std::unordered_map<EdgeId, EdgeLayout>& edgeLayouts,
    const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
    float gridSize) {
    
    // Store gridSize for use in helper methods
    gridSize_ = constants::effectiveGridSize(gridSize);

    for (EdgeId edgeId : edges) {
        auto it = edgeLayouts.find(edgeId);
        if (it == edgeLayouts.end()) {
            continue;
        }

        EdgeLayout& layout = it->second;

        // Handle self-loops: use collision-aware path generation
        if (layout.from == layout.to) {
            auto nodeIt = nodeLayouts.find(layout.from);
            if (nodeIt == nodeLayouts.end()) continue;

            layout.bendPoints = createSelfLoopPath(layout, nodeIt->second, nodeLayouts);

            continue;
        }


        // Use current sourcePoint/targetPoint (already updated by redistribution)
        // and existing sourceEdge/targetEdge (preserve the optimizer's decision)
        auto newBends = createPathWithObstacleAvoidance(
            layout.sourcePoint,
            layout.targetPoint,
            layout.sourceEdge,
            layout.targetEdge,
            nodeLayouts,
            layout.from,
            layout.to);

        layout.bendPoints = std::move(newBends);


        // === Verify no node penetration ===
        // The greedy algorithm should have avoided all obstacles, but double-check
        bool hasCollision = false;
        std::vector<Point> pathPoints;
        pathPoints.push_back(layout.sourcePoint);
        for (const auto& bp : layout.bendPoints) {
            pathPoints.push_back(bp.position);
        }
        pathPoints.push_back(layout.targetPoint);


        for (size_t i = 0; i + 1 < pathPoints.size(); ++i) {
            auto collisions = findCollidingNodes(
                pathPoints[i], pathPoints[i + 1],
                nodeLayouts, layout.from, layout.to);
            if (!collisions.empty()) {
                hasCollision = true;
                break;
            }
        }

        // If collision still exists, try alternative mid-points with penalty scoring
        if (hasCollision) {
            // Create edgeLayouts map for penalty scoring (empty for now - could be enhanced)
            std::unordered_map<EdgeId, EdgeLayout> emptyLayouts;
            layout.bendPoints = tryAlternativeMidPoints(
                layout.sourcePoint,
                layout.targetPoint,
                layout.sourceEdge,
                layout.targetEdge,
                nodeLayouts,
                emptyLayouts,
                layout.from,
                layout.to);
        }
        
        // No L-bend/Z-path fallback in regenerateBendPoints - trust main path generation
    }
}

// =============================================================================
// Collision-Aware Path Generation
// =============================================================================

bool GeometricEdgeOptimizer::selfLoopHasCollision(
    const std::vector<BendPoint>& bends,
    const Point& source,
    const Point& target,
    NodeId selfNodeId,
    const std::unordered_map<NodeId, NodeLayout>& nodeLayouts) {

    // Build full path
    std::vector<Point> path;
    path.push_back(source);
    for (const auto& bp : bends) {
        path.push_back(bp.position);
    }
    path.push_back(target);

    // Check each segment against all OTHER nodes
    for (size_t i = 0; i + 1 < path.size(); ++i) {
        auto collisions = findCollidingNodes(path[i], path[i + 1], nodeLayouts, selfNodeId, selfNodeId);
        if (!collisions.empty()) {
            return true;
        }
    }
    return false;
}

std::vector<BendPoint> GeometricEdgeOptimizer::createSelfLoopPath(
    const EdgeLayout& layout,
    const NodeLayout& selfNode,
    const std::unordered_map<NodeId, NodeLayout>& nodeLayouts) {

    // Use grid cells for consistent extension point calculation
    constexpr int BASE_OFFSET_CELLS = 2;      // 2 grid cells from node edge
    constexpr int MAX_OFFSET_TRIES = 4;
    constexpr int OFFSET_INCREMENT_CELLS = 1; // Increment by 1 grid cell each try
    
    float baseOffset = BASE_OFFSET_CELLS * gridSize_;
    float offsetIncrement = OFFSET_INCREMENT_CELLS * gridSize_;

    auto generatePath = [&](float offset) -> std::vector<BendPoint> {
        std::vector<BendPoint> bends;

        Point srcExt, tgtExt;
        switch (layout.sourceEdge) {
            case NodeEdge::Top:
                srcExt = {layout.sourcePoint.x, layout.sourcePoint.y - offset};
                break;
            case NodeEdge::Bottom:
                srcExt = {layout.sourcePoint.x, layout.sourcePoint.y + offset};
                break;
            case NodeEdge::Left:
                srcExt = {layout.sourcePoint.x - offset, layout.sourcePoint.y};
                break;
            case NodeEdge::Right:
                srcExt = {layout.sourcePoint.x + offset, layout.sourcePoint.y};
                break;
        }
        switch (layout.targetEdge) {
            case NodeEdge::Top:
                tgtExt = {layout.targetPoint.x, layout.targetPoint.y - offset};
                break;
            case NodeEdge::Bottom:
                tgtExt = {layout.targetPoint.x, layout.targetPoint.y + offset};
                break;
            case NodeEdge::Left:
                tgtExt = {layout.targetPoint.x - offset, layout.targetPoint.y};
                break;
            case NodeEdge::Right:
                tgtExt = {layout.targetPoint.x + offset, layout.targetPoint.y};
                break;
        }

        // Need corner if srcExt and tgtExt are not aligned
        if (std::abs(srcExt.x - tgtExt.x) > EPSILON && std::abs(srcExt.y - tgtExt.y) > EPSILON) {
            bends.push_back({srcExt});

            // Try both corner options and pick the one outside node
            Point cornerA = {srcExt.x, tgtExt.y};
            Point cornerB = {tgtExt.x, srcExt.y};

            float nodeLeft = selfNode.position.x;
            float nodeRight = selfNode.position.x + selfNode.size.width;
            float nodeTop = selfNode.position.y;
            float nodeBottom = selfNode.position.y + selfNode.size.height;

            bool cornerAInside = (cornerA.x >= nodeLeft && cornerA.x <= nodeRight &&
                                   cornerA.y >= nodeTop && cornerA.y <= nodeBottom);

            // Choose corner that's outside the node (prefer A if both outside)
            Point corner = cornerAInside ? cornerB : cornerA;
            bends.push_back({corner});
            bends.push_back({tgtExt});
        } else {
            bends.push_back({srcExt});
            bends.push_back({tgtExt});
        }

        return bends;
    };

    // Try increasing offsets until no collision
    for (int i = 0; i < MAX_OFFSET_TRIES; ++i) {
        float currentOffset = baseOffset + i * offsetIncrement;
        auto bends = generatePath(currentOffset);

        if (!selfLoopHasCollision(bends, layout.sourcePoint, layout.targetPoint, layout.from, nodeLayouts)) {
            return bends;
        }
    }

    // Return best effort (largest offset)
    return generatePath(baseOffset + (MAX_OFFSET_TRIES - 1) * offsetIncrement);
}

std::vector<BendPoint> GeometricEdgeOptimizer::tryAlternativeMidPoints(
    const Point& source,
    const Point& target,
    NodeEdge sourceEdge,
    NodeEdge targetEdge,
    const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
    const std::unordered_map<EdgeId, EdgeLayout>& edgeLayouts,
    NodeId sourceNodeId,
    NodeId targetNodeId) {

    struct PathCandidate {
        std::vector<BendPoint> bends;
        int score;
        bool hasCollision;
    };
    std::vector<PathCandidate> candidates;

    // Helper to check if path has collision and find colliding nodes
    auto getCollidingNodes = [&](const std::vector<BendPoint>& bends) 
        -> std::vector<std::pair<NodeId, NodeLayout>> {
        std::vector<std::pair<NodeId, NodeLayout>> result;
        std::vector<Point> path;
        path.push_back(source);
        for (const auto& bp : bends) path.push_back(bp.position);
        path.push_back(target);

        for (size_t i = 0; i + 1 < path.size(); ++i) {
            auto cols = findCollidingNodes(path[i], path[i + 1], nodeLayouts, sourceNodeId, targetNodeId);
            for (const auto& c : cols) {
                bool exists = false;
                for (const auto& r : result) {
                    if (r.first == c.first) { exists = true; break; }
                }
                if (!exists) result.push_back(c);
            }
        }
        return result;
    };

    auto pathHasCollision = [&](const std::vector<BendPoint>& bends) -> bool {
        return !getCollidingNodes(bends).empty();
    };

    auto scorePath = [&](const std::vector<BendPoint>& bends) -> int {
        EdgeLayout candidate;
        candidate.sourcePoint = source;
        candidate.targetPoint = target;
        candidate.sourceEdge = sourceEdge;
        candidate.targetEdge = targetEdge;
        candidate.from = sourceNodeId;
        candidate.to = targetNodeId;
        candidate.bendPoints = bends;
        return scoreGeometricPath(candidate, edgeLayouts, nodeLayouts);
    };

    // Get base path and find which nodes are blocking
    auto basePath = createPathWithObstacleAvoidance(
        source, target, sourceEdge, targetEdge, nodeLayouts, sourceNodeId, targetNodeId);
    auto blockingNodes = getCollidingNodes(basePath);

    // Find bounds of all blocking nodes
    float minBlockX = std::numeric_limits<float>::max();
    float maxBlockX = std::numeric_limits<float>::lowest();
    float minBlockY = std::numeric_limits<float>::max();
    float maxBlockY = std::numeric_limits<float>::lowest();

    constexpr float MARGIN = 15.0f;

    // Filter out blocking nodes that contain source or target point
    // (these are caused by overlapping nodes and can't be avoided at path level)
    auto isPointInsideNode = [](const Point& p, const NodeLayout& n) -> bool {
        return p.x >= n.position.x && p.x <= n.position.x + n.size.width &&
               p.y >= n.position.y && p.y <= n.position.y + n.size.height;
    };

    std::vector<std::pair<NodeId, NodeLayout>> realBlockingNodes;
    for (const auto& [nodeId, node] : blockingNodes) {
        // Skip nodes that contain source or target - can't route around them
        if (isPointInsideNode(source, node) || isPointInsideNode(target, node)) {
            continue;
        }
        realBlockingNodes.push_back({nodeId, node});
    }

    // If all blocking nodes contain source/target, return empty - no alternative available
    if (realBlockingNodes.empty()) {
        return {};
    }

    for (const auto& [nodeId, node] : realBlockingNodes) {
        minBlockX = std::min(minBlockX, node.position.x - MARGIN);
        maxBlockX = std::max(maxBlockX, node.position.x + node.size.width + MARGIN);
        minBlockY = std::min(minBlockY, node.position.y - MARGIN);
        maxBlockY = std::max(maxBlockY, node.position.y + node.size.height + MARGIN);
    }

    // If no blocking nodes, return empty - no alternative needed
    if (blockingNodes.empty()) {
        return {};
    }

    // Check if source/target X/Y are inside blocking bounds
    bool srcXInside = (source.x >= minBlockX && source.x <= maxBlockX);
    bool srcYInside = (source.y >= minBlockY && source.y <= maxBlockY);
    bool tgtXInside = (target.x >= minBlockX && target.x <= maxBlockX);
    bool tgtYInside = (target.y >= minBlockY && target.y <= maxBlockY);

    // Generate comprehensive detour paths
    auto addCandidate = [&](const std::vector<BendPoint>& path) {
        candidates.push_back({path, scorePath(path), pathHasCollision(path)});
    };

    // Strategy: 3-bend paths that go completely around blocking nodes
    // These ensure we exit the blocking area before crossing it
    for (float extraMargin : {0.0f, 20.0f, 40.0f, 60.0f}) {
        float safeMinX = minBlockX - extraMargin;
        float safeMaxX = maxBlockX + extraMargin;
        float safeMinY = minBlockY - extraMargin;
        float safeMaxY = maxBlockY + extraMargin;

        // Go LEFT then route
        if (srcXInside || tgtXInside) {
            // src -> (safeMinX, src.y) -> (safeMinX, tgt.y) -> tgt
            addCandidate({{{safeMinX, source.y}}, {{safeMinX, target.y}}});
            
            // 3-bend: src -> (safeMinX, src.y) -> (safeMinX, safeMinY) -> (tgt.x, safeMinY) -> tgt
            if (srcYInside || tgtYInside) {
                addCandidate({{{safeMinX, source.y}}, {{safeMinX, safeMinY}}, {{target.x, safeMinY}}});
                addCandidate({{{safeMinX, source.y}}, {{safeMinX, safeMaxY}}, {{target.x, safeMaxY}}});
            }
        }

        // Go RIGHT then route
        if (srcXInside || tgtXInside) {
            addCandidate({{{safeMaxX, source.y}}, {{safeMaxX, target.y}}});
            
            if (srcYInside || tgtYInside) {
                addCandidate({{{safeMaxX, source.y}}, {{safeMaxX, safeMinY}}, {{target.x, safeMinY}}});
                addCandidate({{{safeMaxX, source.y}}, {{safeMaxX, safeMaxY}}, {{target.x, safeMaxY}}});
            }
        }

        // Go UP then route
        if (srcYInside || tgtYInside) {
            addCandidate({{{source.x, safeMinY}}, {{target.x, safeMinY}}});
            
            if (srcXInside || tgtXInside) {
                addCandidate({{{source.x, safeMinY}}, {{safeMinX, safeMinY}}, {{safeMinX, target.y}}});
                addCandidate({{{source.x, safeMinY}}, {{safeMaxX, safeMinY}}, {{safeMaxX, target.y}}});
            }
        }

        // Go DOWN then route
        if (srcYInside || tgtYInside) {
            addCandidate({{{source.x, safeMaxY}}, {{target.x, safeMaxY}}});
            
            if (srcXInside || tgtXInside) {
                addCandidate({{{source.x, safeMaxY}}, {{safeMinX, safeMaxY}}, {{safeMinX, target.y}}});
                addCandidate({{{source.x, safeMaxY}}, {{safeMaxX, safeMaxY}}, {{safeMaxX, target.y}}});
            }
        }

        // If neither is inside, try simple Z-paths outside blocking area
        if (!srcXInside && !srcYInside && !tgtXInside && !tgtYInside) {
            addCandidate({{{source.x, safeMinY}}, {{target.x, safeMinY}}});
            addCandidate({{{source.x, safeMaxY}}, {{target.x, safeMaxY}}});
            addCandidate({{{safeMinX, source.y}}, {{safeMinX, target.y}}});
            addCandidate({{{safeMaxX, source.y}}, {{safeMaxX, target.y}}});
        }
    }

    // Find collision-free path with lowest score
    PathCandidate* bestNoCollision = nullptr;
    for (auto& c : candidates) {
        if (!c.hasCollision) {
            if (!bestNoCollision || c.score < bestNoCollision->score) {
                bestNoCollision = &c;
            }
        }
    }

    if (bestNoCollision) {
        return bestNoCollision->bends;
    }


    // No fallback - return empty if all candidates collide
    return {};
}

// =============================================================================
// Self-Loop Combination Evaluation (penalty-based direction selection)
// =============================================================================

std::vector<GeometricEdgeOptimizer::CombinationResult>
GeometricEdgeOptimizer::evaluateSelfLoopCombinations(
    EdgeId edgeId,
    const EdgeLayout& baseLayout,
    const std::unordered_map<EdgeId, EdgeLayout>& assignedLayouts,
    const std::unordered_map<NodeId, NodeLayout>& nodeLayouts) {

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
    options.gridConfig.cellSize = gridSize_;

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
        // Note: Geometric optimizer doesn't use forbiddenZones for speed
        static const std::vector<ForbiddenZone> emptyZones;
        PenaltyContext ctx{assignedLayouts, nodeLayouts, emptyZones, gridSize_};
        ctx.sourceNodeId = candidate.from;
        ctx.targetNodeId = candidate.to;
        ctx.originalLayouts = originalLayouts_;
        ctx.movedNodes = movedNodes_;

        // Check hard constraints (segment overlap, etc.)
        // For GeometricEdgeOptimizer, we check if score exceeds hard constraint threshold
        int score = penaltySystem()->calculateTotalPenalty(candidate, ctx);
        if (score >= HARD_CONSTRAINT_PENALTY) {
            continue;  // Skip combinations that violate hard constraints
        }

        // Map direction to source/target edges for CombinationResult
        NodeEdge srcEdge = candidate.sourceEdge;
        NodeEdge tgtEdge = candidate.targetEdge;

        results.push_back({srcEdge, tgtEdge, score, std::move(candidate)});
    }

    // Sort by score (lowest first = best)
    std::sort(results.begin(), results.end(),
        [](const CombinationResult& a, const CombinationResult& b) {
            return a.score < b.score;
        });

    return results;
}

}  // namespace arborvia
