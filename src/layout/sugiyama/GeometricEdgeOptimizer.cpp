#include "GeometricEdgeOptimizer.h"
#include "arborvia/core/GeometryUtils.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

namespace arborvia {

using constants::EPSILON;

namespace {
    constexpr float DEFAULT_OBSTACLE_MARGIN = 15.0f;
}

GeometricEdgeOptimizer::GeometricEdgeOptimizer(const ScoringWeights& weights)
    : scorer_(weights) {}

void GeometricEdgeOptimizer::setWeights(const ScoringWeights& weights) {
    scorer_.setWeights(weights);
}

const ScoringWeights& GeometricEdgeOptimizer::weights() const {
    return scorer_.weights();
}

std::unordered_map<EdgeId, EdgeLayout> GeometricEdgeOptimizer::optimize(
    const std::vector<EdgeId>& edges,
    const std::unordered_map<EdgeId, EdgeLayout>& currentLayouts,
    const std::unordered_map<NodeId, NodeLayout>& nodeLayouts) {

    std::unordered_map<EdgeId, EdgeLayout> assignedLayouts;
    std::unordered_map<EdgeId, EdgeLayout> result;

    for (EdgeId edgeId : edges) {
        auto it = currentLayouts.find(edgeId);
        if (it == currentLayouts.end()) {
            continue;
        }

        const EdgeLayout& baseLayout = it->second;

        // Skip self-loops
        if (baseLayout.from == baseLayout.to) {
            result[edgeId] = baseLayout;
            assignedLayouts[edgeId] = baseLayout;
            continue;
        }

        // Evaluate all 16 combinations geometrically
        auto combinations = evaluateAllCombinations(
            edgeId, baseLayout, assignedLayouts, nodeLayouts);

        if (combinations.empty()) {
            result[edgeId] = baseLayout;
            assignedLayouts[edgeId] = baseLayout;
            continue;
        }

        // Select best combination (lowest score)
        const auto& best = combinations.front();

        // Return sourceEdge/targetEdge AND bendPoints with obstacle avoidance
        EdgeLayout finalLayout = baseLayout;
        finalLayout.sourceEdge = best.layout.sourceEdge;
        finalLayout.targetEdge = best.layout.targetEdge;
        // Keep obstacle-avoidance bendPoints from scoring
        finalLayout.bendPoints = best.layout.bendPoints;

        result[edgeId] = finalLayout;
        assignedLayouts[edgeId] = finalLayout;
    }

    return result;
}

std::vector<GeometricEdgeOptimizer::CombinationResult>
GeometricEdgeOptimizer::evaluateAllCombinations(
    EdgeId /*edgeId*/,
    const EdgeLayout& baseLayout,
    const std::unordered_map<EdgeId, EdgeLayout>& assignedLayouts,
    const std::unordered_map<NodeId, NodeLayout>& nodeLayouts) {

    std::vector<CombinationResult> results;
    results.reserve(16);

    constexpr std::array<NodeEdge, 4> allEdges = {
        NodeEdge::Top, NodeEdge::Bottom, NodeEdge::Left, NodeEdge::Right
    };

    for (NodeEdge srcEdge : allEdges) {
        for (NodeEdge tgtEdge : allEdges) {
            EdgeLayout candidate = createCandidateLayout(
                baseLayout, srcEdge, tgtEdge, nodeLayouts);

            // Skip if nodes not found
            auto srcIt = nodeLayouts.find(baseLayout.from);
            auto tgtIt = nodeLayouts.find(baseLayout.to);
            if (srcIt == nodeLayouts.end() || tgtIt == nodeLayouts.end()) {
                continue;
            }

            int score = scoreGeometricPath(candidate, assignedLayouts, nodeLayouts);
            results.push_back({srcEdge, tgtEdge, score, std::move(candidate)});
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
    const std::unordered_map<NodeId, NodeLayout>& nodeLayouts) {

    EdgeLayout candidate = base;
    candidate.sourceEdge = sourceEdge;
    candidate.targetEdge = targetEdge;

    // When edge routing changes, mark snap indices for redistribution
    // This prevents duplicate snap indices on the same node edge
    if (sourceEdge != base.sourceEdge) {
        candidate.sourceSnapIndex = constants::SNAP_INDEX_UNASSIGNED;
    }
    if (targetEdge != base.targetEdge) {
        candidate.targetSnapIndex = constants::SNAP_INDEX_UNASSIGNED;
    }

    auto srcIt = nodeLayouts.find(base.from);
    auto tgtIt = nodeLayouts.find(base.to);

    if (srcIt == nodeLayouts.end() || tgtIt == nodeLayouts.end()) {
        return candidate;
    }

    // Calculate temporary source/target points for scoring only
    Point tempSource = calculateEdgeCenter(srcIt->second, sourceEdge);
    Point tempTarget = calculateEdgeCenter(tgtIt->second, targetEdge);

    // Create path WITH obstacle avoidance
    auto tempBends = createPathWithObstacleAvoidance(
        tempSource, tempTarget, sourceEdge, targetEdge,
        nodeLayouts, base.from, base.to);

    // Store temporary values for scoring
    candidate.sourcePoint = tempSource;
    candidate.targetPoint = tempTarget;
    candidate.bendPoints = std::move(tempBends);

    return candidate;
}

Point GeometricEdgeOptimizer::calculateEdgeCenter(
    const NodeLayout& node,
    NodeEdge edge) {

    switch (edge) {
        case NodeEdge::Top:
            return {
                node.position.x + node.size.width * 0.5f,
                node.position.y
            };
        case NodeEdge::Bottom:
            return {
                node.position.x + node.size.width * 0.5f,
                node.position.y + node.size.height
            };
        case NodeEdge::Left:
            return {
                node.position.x,
                node.position.y + node.size.height * 0.5f
            };
        case NodeEdge::Right:
            return {
                node.position.x + node.size.width,
                node.position.y + node.size.height * 0.5f
            };
    }
    return node.center();
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

    // Expand node bounds by margin
    float nodeXmin = node.position.x - margin;
    float nodeXmax = node.position.x + node.size.width + margin;
    float nodeYmin = node.position.y - margin;
    float nodeYmax = node.position.y + node.size.height + margin;

    bool isHorizontal = (std::abs(p1.y - p2.y) < EPSILON);
    bool isVertical = (std::abs(p1.x - p2.x) < EPSILON);

    if (isHorizontal) {
        float y = p1.y;
        float xMin = std::min(p1.x, p2.x);
        float xMax = std::max(p1.x, p2.x);
        // Check if y is strictly inside node and x ranges overlap
        return (y > nodeYmin && y < nodeYmax && xMin < nodeXmax && xMax > nodeXmin);
    }
    else if (isVertical) {
        float x = p1.x;
        float yMin = std::min(p1.y, p2.y);
        float yMax = std::max(p1.y, p2.y);
        // Check if x is strictly inside node and y ranges overlap
        return (x > nodeXmin && x < nodeXmax && yMin < nodeYmax && yMax > nodeYmin);
    }

    // Non-orthogonal segment: use general AABB intersection
    float xMin = std::min(p1.x, p2.x);
    float xMax = std::max(p1.x, p2.x);
    float yMin = std::min(p1.y, p2.y);
    float yMax = std::max(p1.y, p2.y);

    return !(xMax < nodeXmin || xMin > nodeXmax || yMax < nodeYmin || yMin > nodeYmax);
}

std::vector<std::pair<NodeId, NodeLayout>> GeometricEdgeOptimizer::findCollidingNodes(
    const Point& p1,
    const Point& p2,
    const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
    NodeId excludeSource,
    NodeId excludeTarget) {

    std::vector<std::pair<NodeId, NodeLayout>> colliding;

    for (const auto& [nodeId, node] : nodeLayouts) {
        // Skip source and target nodes
        if (nodeId == excludeSource || nodeId == excludeTarget) {
            continue;
        }

        if (segmentIntersectsNode(p1, p2, node, 0.0f)) {
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

    // Start with simple path
    auto simplePath = predictOrthogonalPath(source, target, sourceEdge, targetEdge);

    // Build full path for collision checking
    std::vector<Point> pathPoints;
    pathPoints.push_back(source);
    for (const auto& bp : simplePath) {
        pathPoints.push_back(bp.position);
    }
    pathPoints.push_back(target);

    // Check each segment for collisions and build detour path
    std::vector<Point> finalPathPoints;
    finalPathPoints.push_back(source);

    for (size_t i = 0; i + 1 < pathPoints.size(); ++i) {
        const Point& p1 = (i == 0) ? source : finalPathPoints.back();
        const Point& p2 = pathPoints[i + 1];

        // Find colliding nodes for this segment
        auto collisions = findCollidingNodes(p1, p2, nodeLayouts, sourceNodeId, targetNodeId);

        if (collisions.empty()) {
            // No collision - add endpoint directly
            if (i + 1 < pathPoints.size() - 1) {
                finalPathPoints.push_back(p2);
            }
        } else {
            // Collision detected - find the closest colliding node
            const NodeLayout* closestObstacle = nullptr;
            float closestDist = std::numeric_limits<float>::max();

            for (const auto& [nodeId, node] : collisions) {
                float dist = p1.distanceTo(node.center());
                if (dist < closestDist) {
                    closestDist = dist;
                    closestObstacle = &node;
                }
            }

            if (closestObstacle) {
                // Create detour around the closest obstacle
                auto detourPoints = createDetourAroundNode(p1, p2, *closestObstacle, DEFAULT_OBSTACLE_MARGIN);

                // Add detour points
                for (const auto& dp : detourPoints) {
                    finalPathPoints.push_back(dp);
                }
            }
        }
    }

    // Convert to BendPoints (exclude source and target)
    std::vector<BendPoint> result;
    for (size_t i = 1; i < finalPathPoints.size(); ++i) {
        // Skip if this is essentially the target point
        if (finalPathPoints[i].distanceTo(target) < EPSILON) {
            continue;
        }
        result.push_back({finalPathPoints[i]});
    }

    // If no bend points but path needs turning, fall back to simple path
    if (result.empty() && !simplePath.empty()) {
        return simplePath;
    }

    return result;
}

int GeometricEdgeOptimizer::scoreGeometricPath(
    const EdgeLayout& candidate,
    const std::unordered_map<EdgeId, EdgeLayout>& assignedLayouts,
    const std::unordered_map<NodeId, NodeLayout>& nodeLayouts) {

    // Use the same scoring as AStarEdgeOptimizer for consistency
    return scorer_.calculateScore(candidate, assignedLayouts, nodeLayouts);
}

}  // namespace arborvia
