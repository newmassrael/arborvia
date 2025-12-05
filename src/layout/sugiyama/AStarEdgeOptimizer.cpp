#include "AStarEdgeOptimizer.h"
#include "ObstacleMap.h"
#include "AStarPathFinder.h"
#include "arborvia/core/GeometryUtils.h"

#include <algorithm>
#include <array>

namespace arborvia {

AStarEdgeOptimizer::AStarEdgeOptimizer(
    std::shared_ptr<IPathFinder> pathFinder,
    const ScoringWeights& weights,
    float gridSize)
    : scorer_(weights)
    , pathFinder_(std::move(pathFinder))
    , gridSize_(gridSize) {

    // Create default pathfinder if not provided
    if (!pathFinder_) {
        pathFinder_ = std::make_shared<AStarPathFinder>();
    }
}

AStarEdgeOptimizer::AStarEdgeOptimizer(const ScoringWeights& weights)
    : AStarEdgeOptimizer(nullptr, weights, 0.0f) {}

void AStarEdgeOptimizer::setWeights(const ScoringWeights& weights) {
    scorer_.setWeights(weights);
}

const ScoringWeights& AStarEdgeOptimizer::weights() const {
    return scorer_.weights();
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

    // Start with assigned layouts (will accumulate as we process)
    std::unordered_map<EdgeId, EdgeLayout> assignedLayouts;
    std::unordered_map<EdgeId, EdgeLayout> result;

    for (EdgeId edgeId : edges) {
        auto it = currentLayouts.find(edgeId);
        if (it == currentLayouts.end()) {
            continue;  // Edge not found in current layouts
        }

        const EdgeLayout& baseLayout = it->second;

        // Skip self-loops (handled separately)
        if (baseLayout.from == baseLayout.to) {
            result[edgeId] = baseLayout;
            assignedLayouts[edgeId] = baseLayout;
            continue;
        }

        // Evaluate all 16 combinations
        auto combinations = evaluateAllCombinations(
            edgeId, baseLayout, assignedLayouts, nodeLayouts);

        if (combinations.empty()) {
            // No valid combinations, keep original
            result[edgeId] = baseLayout;
            assignedLayouts[edgeId] = baseLayout;
            continue;
        }

        // Select best combination (lowest score)
        const auto& best = combinations.front();
        result[edgeId] = best.layout;
        assignedLayouts[edgeId] = best.layout;
    }

    return result;
}

std::vector<AStarEdgeOptimizer::CombinationResult>
AStarEdgeOptimizer::evaluateAllCombinations(
    EdgeId /*edgeId*/,
    const EdgeLayout& baseLayout,
    const std::unordered_map<EdgeId, EdgeLayout>& assignedLayouts,
    const std::unordered_map<NodeId, NodeLayout>& nodeLayouts) {

    std::vector<CombinationResult> results;
    results.reserve(16);

    // All 4 node edges
    constexpr std::array<NodeEdge, 4> allEdges = {
        NodeEdge::Top, NodeEdge::Bottom, NodeEdge::Left, NodeEdge::Right
    };

    // Evaluate all 16 combinations
    for (NodeEdge srcEdge : allEdges) {
        for (NodeEdge tgtEdge : allEdges) {
            bool pathFound = false;
            EdgeLayout candidate = createCandidateLayout(
                baseLayout, srcEdge, tgtEdge, nodeLayouts, pathFound);

            // Skip invalid candidates (node not found or no valid path)
            if (!pathFound) {
                continue;
            }

            // Score this combination (now with actual bend points)
            int score = scorer_.calculateScore(candidate, assignedLayouts, nodeLayouts);

            results.push_back({srcEdge, tgtEdge, score, std::move(candidate), true});
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
    bool& pathFound) {

    pathFound = false;

    EdgeLayout candidate = base;
    candidate.sourceEdge = sourceEdge;
    candidate.targetEdge = targetEdge;
    candidate.bendPoints.clear();

    // Get node layouts
    auto srcIt = nodeLayouts.find(base.from);
    if (srcIt == nodeLayouts.end()) {
        return candidate;  // Invalid: source node not found
    }
    
    auto tgtIt = nodeLayouts.find(base.to);
    if (tgtIt == nodeLayouts.end()) {
        return candidate;  // Invalid: target node not found
    }

    // Build obstacle map from node layouts
    float gridSize = effectiveGridSize();
    ObstacleMap obstacles;
    obstacles.buildFromNodes(nodeLayouts, gridSize, 0);  // margin = 0

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


}  // namespace arborvia
