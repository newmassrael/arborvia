#include "CooperativeRerouter.h"
#include "../pathfinding/ObstacleMap.h"
#include "../pathfinding/AStarPathFinder.h"
#include "../sugiyama/routing/PathIntersection.h"

#ifdef EDGE_ROUTING_DEBUG
#include <iostream>
#endif

namespace arborvia {

namespace {
    constexpr float DEFAULT_GRID_SIZE = 20.0f;
}

CooperativeRerouter::CooperativeRerouter(
    std::shared_ptr<IPathFinder> pathFinder,
    float gridSize)
    : pathFinder_(pathFinder ? pathFinder : std::make_shared<AStarPathFinder>())
    , gridSize_(gridSize) {
}

CooperativeRerouter::~CooperativeRerouter() = default;

void CooperativeRerouter::setPathFinder(std::shared_ptr<IPathFinder> pathFinder) {
    pathFinder_ = pathFinder ? pathFinder : std::make_shared<AStarPathFinder>();
}

void CooperativeRerouter::setGridSize(float gridSize) {
    gridSize_ = gridSize;
}

float CooperativeRerouter::effectiveGridSize() const {
    return gridSize_ > 0 ? gridSize_ : DEFAULT_GRID_SIZE;
}

// =============================================================================
// Main entry point: Make room for B by rerouting blocking edges (A)
// =============================================================================

CooperativeRerouter::RerouteResult CooperativeRerouter::rerouteWithCooperation(
    EdgeId edgeId,
    const EdgeLayout& currentLayout,
    std::unordered_map<EdgeId, EdgeLayout>& otherLayouts,
    const std::unordered_map<NodeId, NodeLayout>& nodeLayouts) {

    RerouteResult result;
    result.success = false;
    result.attempts = 0;

    // Validate nodes exist
    if (nodeLayouts.find(currentLayout.from) == nodeLayouts.end() ||
        nodeLayouts.find(currentLayout.to) == nodeLayouts.end()) {
        result.failureReason = "Source or target node not found";
        return result;
    }

    float gridSize = effectiveGridSize();

    // Step 1: Build shared obstacle map (nodes only)
    ObstacleMap obstacles;
    obstacles.buildFromNodes(nodeLayouts, gridSize, 0);

    // Step 2: Calculate B's expected path (simple geometric path)
    std::vector<GridPoint> myExpectedPath = calculateExpectedPath(currentLayout, obstacles);

    if (myExpectedPath.empty()) {
        result.failureReason = "Failed to calculate expected path";
        return result;
    }

#ifdef EDGE_ROUTING_DEBUG
    std::cout << "[CoopReroute] B's expected path: " << myExpectedPath.size() 
              << " grid points" << std::endl;
#endif

    // Step 3: Reserve B's expected path in obstacle map
    reservePathInObstacles(obstacles, edgeId, myExpectedPath);

    // Step 4: Find blocking edges (A) that overlap with B's expected path
    auto blockingEdges = findBlockingEdges(currentLayout, myExpectedPath, otherLayouts, obstacles);

#ifdef EDGE_ROUTING_DEBUG
    std::cout << "[CoopReroute] Found " << blockingEdges.size() 
              << " blocking edges" << std::endl;
#endif

    // Step 5: Reroute blocking edges (A) to avoid B's reserved area
    for (EdgeId blockingId : blockingEdges) {
        if (result.attempts >= MAX_REROUTE_ATTEMPTS) {
            result.failureReason = "Max rerouting attempts reached";
            break;
        }

        ++result.attempts;

        const EdgeLayout& blockingLayout = otherLayouts[blockingId];
        bool pathFound = false;

        // Reroute A using obstacle map (B's path is reserved, A must avoid it)
        EdgeLayout newBlockingLayout = calculatePath(blockingLayout, obstacles, pathFound);

        if (pathFound) {
            // A successfully rerouted around B's reserved area
            otherLayouts[blockingId] = newBlockingLayout;
            result.reroutedEdges.push_back(newBlockingLayout);

            // Add A's new path to obstacle map (prevents other blocking edges from overlapping)
            std::vector<GridPoint> aPath;
            aPath.push_back(obstacles.pixelToGrid(newBlockingLayout.sourcePoint));
            for (const auto& bp : newBlockingLayout.bendPoints) {
                aPath.push_back(obstacles.pixelToGrid(bp.position));
            }
            aPath.push_back(obstacles.pixelToGrid(newBlockingLayout.targetPoint));
            obstacles.markEdgePath(blockingId, aPath);

#ifdef EDGE_ROUTING_DEBUG
            std::cout << "[CoopReroute] Edge " << blockingId 
                      << " rerouted to avoid edge " << edgeId << std::endl;
#endif
        } else {
            // Track failed reroute
            result.failedReroutes.push_back(blockingId);
#ifdef EDGE_ROUTING_DEBUG
            std::cout << "[CoopReroute] Edge " << blockingId 
                      << " could not find alternative route" << std::endl;
#endif
        }
    }

    // Step 6: Calculate B's actual path (blocking edges should be cleared now)
    // Clear B's reserved path first so B can use it
    obstacles.clearEdgePath(edgeId);

    bool bPathFound = false;
    EdgeLayout bLayout = calculatePath(currentLayout, obstacles, bPathFound);

    if (bPathFound) {
        result.success = true;
        result.layout = bLayout;
#ifdef EDGE_ROUTING_DEBUG
        std::cout << "[CoopReroute] B's path calculated: " << bLayout.bendPoints.size() 
                  << " bendPoints" << std::endl;
#endif
    } else {
        result.failureReason = "B failed to find path after rerouting blocking edges";
        result.layout = currentLayout;
    }

    return result;
}

// =============================================================================
// Expected path calculation (simple geometric path for reservation)
// =============================================================================

std::vector<GridPoint> CooperativeRerouter::calculateExpectedPath(
    const EdgeLayout& layout,
    ObstacleMap& obstacles) const {

    std::vector<GridPoint> path;

    GridPoint start = obstacles.pixelToGrid(layout.sourcePoint);
    GridPoint end = obstacles.pixelToGrid(layout.targetPoint);

    // Simple L-shaped path based on source/target edge directions
    // This reserves the area B needs to use

    path.push_back(start);

    // Determine path shape based on NodeEdge directions
    bool horizontalFirst = (layout.sourceEdge == NodeEdge::Left || 
                           layout.sourceEdge == NodeEdge::Right);

    if (start.x != end.x && start.y != end.y) {
        // Need a bend point for L-shape
        if (horizontalFirst) {
            // Go horizontal first, then vertical
            path.push_back({end.x, start.y});
        } else {
            // Go vertical first, then horizontal
            path.push_back({start.x, end.y});
        }
    }

    path.push_back(end);

    // Interpolate cells between path points
    std::vector<GridPoint> fullPath;
    for (size_t i = 0; i + 1 < path.size(); ++i) {
        GridPoint from = path[i];
        GridPoint to = path[i + 1];

        // Check if diagonal (should not happen in expected path)
        if (from.x != to.x && from.y != to.y) {
            // Diagonal - just add endpoints
            fullPath.push_back(from);
        } else {
            // Orthogonal segment - add all cells safely
            int dx = (to.x > from.x) ? 1 : (to.x < from.x) ? -1 : 0;
            int dy = (to.y > from.y) ? 1 : (to.y < from.y) ? -1 : 0;

            GridPoint current = from;
            while (current.x != to.x || current.y != to.y) {
                fullPath.push_back(current);
                current.x += dx;
                current.y += dy;
            }
        }
    }
    fullPath.push_back(path.back());

    return fullPath;
}

// =============================================================================
// Find blocking edges
// =============================================================================

std::vector<EdgeId> CooperativeRerouter::findBlockingEdges(
    const EdgeLayout& myLayout,
    const std::vector<GridPoint>& myExpectedPath,
    const std::unordered_map<EdgeId, EdgeLayout>& otherLayouts,
    const ObstacleMap& obstacles) const {

    std::vector<EdgeId> blocking;

    // Convert my expected path to a set for fast lookup
    std::set<std::pair<int, int>> myPathCells;
    for (const auto& gp : myExpectedPath) {
        myPathCells.insert({gp.x, gp.y});
    }

    // Check each other edge
    for (const auto& [edgeId, otherLayout] : otherLayouts) {
        if (edgeId == myLayout.id) {
            continue;
        }

        // Get other edge's path cells
        std::vector<Point> otherPoints;
        otherPoints.push_back(otherLayout.sourcePoint);
        for (const auto& bp : otherLayout.bendPoints) {
            otherPoints.push_back(bp.position);
        }
        otherPoints.push_back(otherLayout.targetPoint);

        // Check if any segment of other edge overlaps with my expected path
        bool overlaps = false;
        for (size_t i = 0; i + 1 < otherPoints.size() && !overlaps; ++i) {
            GridPoint from = obstacles.pixelToGrid(otherPoints[i]);
            GridPoint to = obstacles.pixelToGrid(otherPoints[i + 1]);

            // Check if diagonal (invalid for orthogonal routing)
            if (from.x != to.x && from.y != to.y) {
                // Diagonal segment - just check endpoints
                if (myPathCells.count({from.x, from.y})) overlaps = true;
                if (myPathCells.count({to.x, to.y})) overlaps = true;
            } else {
                // Orthogonal segment - iterate all cells safely
                int dx = (to.x > from.x) ? 1 : (to.x < from.x) ? -1 : 0;
                int dy = (to.y > from.y) ? 1 : (to.y < from.y) ? -1 : 0;

                GridPoint current = from;
                while (current.x != to.x || current.y != to.y) {
                    if (myPathCells.count({current.x, current.y})) {
                        overlaps = true;
                        break;
                    }
                    current.x += dx;
                    current.y += dy;
                }
                if (myPathCells.count({to.x, to.y})) {
                    overlaps = true;
                }
            }
        }

        if (overlaps) {
            blocking.push_back(edgeId);
        }
    }

    return blocking;
}

// =============================================================================
// Path calculation
// =============================================================================

EdgeLayout CooperativeRerouter::calculatePath(
    const EdgeLayout& layout,
    ObstacleMap& obstacles,
    bool& pathFound) {

    pathFound = false;

    EdgeLayout result = layout;
    result.bendPoints.clear();

    GridPoint startGrid = obstacles.pixelToGrid(layout.sourcePoint);
    GridPoint goalGrid = obstacles.pixelToGrid(layout.targetPoint);

    PathResult pathResult = pathFinder_->findPath(
        startGrid, goalGrid, obstacles,
        layout.from, layout.to,
        layout.sourceEdge, layout.targetEdge,
        {}, {});

    if (!pathResult.found || pathResult.path.size() < 2) {
        return result;
    }

    // Convert grid path to bendPoints (excluding endpoints)
    for (size_t i = 1; i + 1 < pathResult.path.size(); ++i) {
        Point pixelPoint = obstacles.gridToPixel(
            pathResult.path[i].x, pathResult.path[i].y);
        result.bendPoints.push_back({pixelPoint});
    }

    // Snap points preserved (sourcePoint, targetPoint unchanged)
    pathFound = true;
    return result;
}

// =============================================================================
// Reserve path in obstacle map
// =============================================================================

void CooperativeRerouter::reservePathInObstacles(
    ObstacleMap& obstacles,
    EdgeId edgeId,
    const std::vector<GridPoint>& path) const {

    // Mark B's expected path as reserved (high cost)
    // This forces blocking edges (A) to find alternative routes
    obstacles.markEdgePath(edgeId, path);
}

// =============================================================================
// Overlap detection
// =============================================================================

bool CooperativeRerouter::hasSegmentOverlap(const EdgeLayout& a, const EdgeLayout& b) {
    return PathIntersection::hasSegmentOverlap(a, b);
}

}  // namespace arborvia
