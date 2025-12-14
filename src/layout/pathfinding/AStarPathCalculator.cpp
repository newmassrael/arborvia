#include "AStarPathCalculator.h"
#include "ObstacleMap.h"
#include "arborvia/common/Logger.h"

namespace arborvia {

AStarPathCalculator::AStarPathCalculator(std::shared_ptr<IPathFinder> pathFinder)
    : pathFinder_(std::move(pathFinder)) {
}

bool AStarPathCalculator::canHandle(const EdgeLayout& layout) const {
    return layout.from != layout.to;
}

EdgePathResult AStarPathCalculator::calculatePath(
    const EdgeLayout& layout,
    const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
    const PathConfig& config) {

    EdgePathResult result;

    // Must not be a self-loop
    if (layout.from == layout.to) {
        result.failureReason = "Self-loop edges should use SelfLoopPathCalculator";
        return result;
    }

    // PathFinder is required
    if (!pathFinder_) {
        result.failureReason = "No pathfinder available";
        return result;
    }

    // Validate grid size
    float gridSize = config.gridSize;
    if (gridSize <= 0) {
        gridSize = 20.0f;  // Default fallback
    }

    // Build obstacle map from nodes
    ObstacleMap obstacles;
    obstacles.buildFromNodes(nodeLayouts, gridSize, 0);

    // Convert to grid coordinates
    GridPoint startGrid = GridPoint::fromPixel(layout.sourcePoint, gridSize);
    GridPoint goalGrid = GridPoint::fromPixel(layout.targetPoint, gridSize);

    // Find path using A*
    LOG_DEBUG("[CALLER:AStarPathCalculator.cpp:calculatePath] A* findPath called");
    auto pathResult = pathFinder_->findPath(
        startGrid, goalGrid, obstacles,
        layout.from, layout.to,
        layout.sourceEdge, layout.targetEdge);

    if (!pathResult.found) {
        result.failureReason = "A* pathfinding failed";
        return result;
    }

    if (pathResult.path.size() < 2) {
        result.failureReason = "Path too short";
        return result;
    }

    // Convert grid path to pixel bend points
    // Skip first and last points (source and target positions)
    for (size_t i = 1; i + 1 < pathResult.path.size(); ++i) {
        Point pixelPos = pathResult.path[i].toPixel(gridSize);
        result.bendPoints.push_back(BendPoint{pixelPos});
    }

    result.success = true;
    return result;
}

}  // namespace arborvia
