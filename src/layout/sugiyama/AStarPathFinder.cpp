#include "AStarPathFinder.h"
#include <queue>
#include <unordered_map>
#include <cmath>
#include <algorithm>
#include <climits>

namespace arborvia {
namespace algorithms {

// Hash function for GridPoint in unordered_map
struct GridPointHash {
    std::size_t operator()(const GridPoint& p) const {
        return std::hash<int>()(p.x) ^ (std::hash<int>()(p.y) << 16);
    }
};

// Combined key for position + direction (for A* closed set)
struct SearchKey {
    GridPoint pos;
    MoveDirection dir;

    bool operator==(const SearchKey& other) const {
        return pos == other.pos && dir == other.dir;
    }
};

struct SearchKeyHash {
    std::size_t operator()(const SearchKey& k) const {
        return std::hash<int>()(k.pos.x) ^
               (std::hash<int>()(k.pos.y) << 16) ^
               (std::hash<int>()(static_cast<int>(k.dir)) << 24);
    }
};

MoveDirection AStarPathFinder::getRequiredSourceDirection(NodeEdge sourceEdge) {
    switch (sourceEdge) {
        case NodeEdge::Top:    return MoveDirection::Up;
        case NodeEdge::Bottom: return MoveDirection::Down;
        case NodeEdge::Left:   return MoveDirection::Left;
        case NodeEdge::Right:  return MoveDirection::Right;
        default:               return MoveDirection::None;  // No constraint
    }
}

MoveDirection AStarPathFinder::getRequiredTargetArrivalDirection(NodeEdge targetEdge) {
    // Arrival direction is opposite to the edge - we arrive "from" outside
    // e.g., entering Top edge means we come from above, so last move is Down
    switch (targetEdge) {
        case NodeEdge::Top:    return MoveDirection::Down;   // Enter from above
        case NodeEdge::Bottom: return MoveDirection::Up;     // Enter from below
        case NodeEdge::Left:   return MoveDirection::Right;  // Enter from left
        case NodeEdge::Right:  return MoveDirection::Left;   // Enter from right
        default:               return MoveDirection::None;   // No constraint
    }
}

PathResult AStarPathFinder::findPath(
    const GridPoint& start,
    const GridPoint& goal,
    const IObstacleProvider& obstacles,
    NodeId sourceNode,
    NodeId targetNode,
    NodeEdge sourceEdge,
    NodeEdge targetEdge,
    const std::unordered_set<NodeId>& extraStartExcludes,
    const std::unordered_set<NodeId>& extraGoalExcludes) const {

    PathResult result;

    // Get required directions
    MoveDirection requiredSourceDir = getRequiredSourceDirection(sourceEdge);
    MoveDirection requiredTargetDir = getRequiredTargetArrivalDirection(targetEdge);

    // Quick check: if start == goal, return trivial path
    if (start == goal) {
        result.found = true;
        result.path = {start};
        result.bendCount = 0;
        result.totalCost = 0;
        return result;
    }

    // Build combined exclude sets
    std::unordered_set<NodeId> startExcludes = extraStartExcludes;
    startExcludes.insert(sourceNode);

    std::unordered_set<NodeId> goalExcludes = extraGoalExcludes;
    goalExcludes.insert(targetNode);

    // Check if start cell is accessible (blocked only by excluded nodes is OK)
    if (obstacles.isBlocked(start.x, start.y, startExcludes)) {
        // Start is blocked by a non-excluded node - can't route from here
        return findPathViaSafeZone(start, goal, obstacles, sourceNode, targetNode,
                                   sourceEdge, targetEdge, extraStartExcludes, extraGoalExcludes);
    }

    // Check if goal cell is accessible
    if (obstacles.isBlocked(goal.x, goal.y, goalExcludes)) {
        // Goal is blocked by a non-excluded node - can't route to here
        return findPathViaSafeZone(start, goal, obstacles, sourceNode, targetNode,
                                   sourceEdge, targetEdge, extraStartExcludes, extraGoalExcludes);
    }

    // Full A* search with direction constraints
    std::priority_queue<SearchNode, std::vector<SearchNode>, std::greater<SearchNode>> openSet;
    std::unordered_map<SearchKey, int, SearchKeyHash> bestCost;
    std::unordered_map<SearchKey, SearchKey, SearchKeyHash> cameFrom;

    // Initialize with start node
    SearchNode startNode;
    startNode.pos = start;
    startNode.lastDir = MoveDirection::None;
    startNode.g = 0;
    startNode.h = manhattanDistance(start, goal);
    openSet.push(startNode);
    bestCost[{start, MoveDirection::None}] = 0;

    int iterations = 0;

    while (!openSet.empty() && iterations < MAX_ITERATIONS) {
        ++iterations;

        SearchNode current = openSet.top();
        openSet.pop();

        // Goal reached with correct arrival direction?
        if (current.pos == goal) {
            // Check if we arrived in the correct direction
            if (requiredTargetDir != MoveDirection::None && current.lastDir != requiredTargetDir) {
                // Wrong arrival direction, skip this path
                continue;
            }

            // Reconstruct path
            std::vector<GridPoint> path;
            SearchKey key = {current.pos, current.lastDir};

            while (true) {
                path.push_back(key.pos);
                auto it = cameFrom.find(key);
                if (it == cameFrom.end()) {
                    break;
                }
                key = it->second;
            }

            std::reverse(path.begin(), path.end());
            path = simplifyPath(path);

            result.found = true;
            result.path = path;
            result.bendCount = countBends(path);
            result.totalCost = current.g;
            return result;
        }

        // Check if this state was already processed with better cost
        SearchKey currentKey = {current.pos, current.lastDir};
        auto bestIt = bestCost.find(currentKey);
        if (bestIt != bestCost.end() && bestIt->second < current.g) {
            continue;
        }

        // Explore neighbors
        for (const auto& [neighborPos, moveDir] : getNeighbors(current.pos)) {
            // Direction constraint: if at start and have required source direction, enforce it
            if (current.pos == start && current.lastDir == MoveDirection::None) {
                if (requiredSourceDir != MoveDirection::None && moveDir != requiredSourceDir) {
                    continue;  // Skip moves that don't match required source direction
                }
            }

            // Per-cell exclusion logic:
            // - Start cell: exclude sourceNode + extraStartExcludes
            // - Goal cell: exclude targetNode + extraGoalExcludes
            // - All other cells: no exclusions
            std::unordered_set<NodeId> cellExclude;
            if (neighborPos == start) {
                cellExclude = startExcludes;
            } else if (neighborPos == goal) {
                cellExclude = goalExcludes;
            }
            // else: empty exclude set for intermediate cells

            if (obstacles.isBlocked(neighborPos.x, neighborPos.y, cellExclude)) {
                continue;
            }

            // Calculate cost
            int bendCost = (current.lastDir != MoveDirection::None &&
                           current.lastDir != moveDir) ? BEND_COST : 0;
            int newG = current.g + MOVE_COST + bendCost;

            SearchKey neighborKey = {neighborPos, moveDir};

            auto it = bestCost.find(neighborKey);
            if (it == bestCost.end() || newG < it->second) {
                bestCost[neighborKey] = newG;
                cameFrom[neighborKey] = currentKey;

                SearchNode neighborNode;
                neighborNode.pos = neighborPos;
                neighborNode.lastDir = moveDir;
                neighborNode.g = newG;
                neighborNode.h = manhattanDistance(neighborPos, goal);
                openSet.push(neighborNode);
            }
        }
    }

    // A* failed, try safe zone fallback
    return findPathViaSafeZone(start, goal, obstacles, sourceNode, targetNode,
                               sourceEdge, targetEdge, extraStartExcludes, extraGoalExcludes);
}

PathResult AStarPathFinder::findPathViaSafeZone(
    const GridPoint& start,
    const GridPoint& goal,
    const IObstacleProvider& obstacles,
    NodeId sourceNode,
    NodeId targetNode,
    NodeEdge sourceEdge,
    NodeEdge targetEdge,
    const std::unordered_set<NodeId>& extraStartExcludes,
    const std::unordered_set<NodeId>& extraGoalExcludes) const {

    PathResult result;
    const auto& safeZones = obstacles.safeZones();

    // Get required directions
    MoveDirection requiredSourceDir = getRequiredSourceDirection(sourceEdge);
    MoveDirection requiredTargetDir = getRequiredTargetArrivalDirection(targetEdge);

    // Helper to check if path respects direction constraints
    auto respectsDirections = [&](const std::vector<GridPoint>& path) -> bool {
        if (path.size() < 2) return true;

        // Check source direction (first move)
        if (requiredSourceDir != MoveDirection::None) {
            int dx = path[1].x - path[0].x;
            int dy = path[1].y - path[0].y;
            MoveDirection firstDir = MoveDirection::None;
            if (dy < 0) firstDir = MoveDirection::Up;
            else if (dy > 0) firstDir = MoveDirection::Down;
            else if (dx < 0) firstDir = MoveDirection::Left;
            else if (dx > 0) firstDir = MoveDirection::Right;

            if (firstDir != requiredSourceDir) return false;
        }

        // Check target direction (last move)
        if (requiredTargetDir != MoveDirection::None) {
            size_t n = path.size();
            int dx = path[n-1].x - path[n-2].x;
            int dy = path[n-1].y - path[n-2].y;
            MoveDirection lastDir = MoveDirection::None;
            if (dy < 0) lastDir = MoveDirection::Up;
            else if (dy > 0) lastDir = MoveDirection::Down;
            else if (dx < 0) lastDir = MoveDirection::Left;
            else if (dx > 0) lastDir = MoveDirection::Right;

            if (lastDir != requiredTargetDir) return false;
        }

        return true;
    };

    // Helper to validate path with per-segment logic
    auto validatePath = [&](const std::vector<GridPoint>& path) -> bool {
        if (path.size() < 2) return false;

        // Check direction constraints first
        if (!respectsDirections(path)) return false;

        for (size_t i = 0; i + 1 < path.size(); ++i) {
            bool isFirst = (i == 0);
            bool isLast = (i == path.size() - 2);
            if (!validateSegmentWithEndpoints(path[i], path[i+1], obstacles,
                                              sourceNode, targetNode, isFirst, isLast,
                                              extraStartExcludes, extraGoalExcludes)) {
                return false;
            }
        }
        return true;
    };

    // Path length calculator
    auto pathLength = [](const std::vector<GridPoint>& path) -> int {
        if (path.size() < 2) return INT_MAX;
        int len = 0;
        for (size_t i = 0; i + 1 < path.size(); ++i) {
            len += std::abs(path[i+1].x - path[i].x) + std::abs(path[i+1].y - path[i].y);
        }
        return len;
    };

    std::vector<std::pair<std::vector<GridPoint>, bool>> options;

    // Generate safe zone paths based on required directions
    // These paths go through safe zones (outside all nodes)

    // If sourceEdge is Top (need Up first): start -> (start.x, yAbove) -> ...
    // If sourceEdge is Bottom (need Down first): start -> (start.x, yBelow) -> ...
    // If sourceEdge is Left (need Left first): start -> (xLeft, start.y) -> ...
    // If sourceEdge is Right (need Right first): start -> (xRight, start.y) -> ...

    // Similarly for target arrival

    // Generate comprehensive options covering all direction combinations

    // Paths going UP first from source
    if (requiredSourceDir == MoveDirection::None || requiredSourceDir == MoveDirection::Up) {
        // Up then to target
        if (requiredTargetDir == MoveDirection::None || requiredTargetDir == MoveDirection::Down) {
            // Simple: up -> horizontal -> down
            options.push_back({{start, {start.x, safeZones.yAbove}, {goal.x, safeZones.yAbove}, goal}, false});
        }
        if (requiredTargetDir == MoveDirection::None || requiredTargetDir == MoveDirection::Up) {
            // Up -> horizontal -> down to below -> up to goal
            options.push_back({{start, {start.x, safeZones.yAbove}, {safeZones.xLeft, safeZones.yAbove},
                                {safeZones.xLeft, safeZones.yBelow}, {goal.x, safeZones.yBelow}, goal}, false});
            options.push_back({{start, {start.x, safeZones.yAbove}, {safeZones.xRight, safeZones.yAbove},
                                {safeZones.xRight, safeZones.yBelow}, {goal.x, safeZones.yBelow}, goal}, false});
        }
        if (requiredTargetDir == MoveDirection::None || requiredTargetDir == MoveDirection::Left) {
            // Up -> to right of goal -> left to goal
            options.push_back({{start, {start.x, safeZones.yAbove}, {safeZones.xRight, safeZones.yAbove},
                                {safeZones.xRight, goal.y}, goal}, false});
        }
        if (requiredTargetDir == MoveDirection::None || requiredTargetDir == MoveDirection::Right) {
            // Up -> to left of goal -> right to goal
            options.push_back({{start, {start.x, safeZones.yAbove}, {safeZones.xLeft, safeZones.yAbove},
                                {safeZones.xLeft, goal.y}, goal}, false});
        }
    }

    // Paths going DOWN first from source
    if (requiredSourceDir == MoveDirection::None || requiredSourceDir == MoveDirection::Down) {
        if (requiredTargetDir == MoveDirection::None || requiredTargetDir == MoveDirection::Up) {
            // Simple: down -> horizontal -> up
            options.push_back({{start, {start.x, safeZones.yBelow}, {goal.x, safeZones.yBelow}, goal}, false});
        }
        if (requiredTargetDir == MoveDirection::None || requiredTargetDir == MoveDirection::Down) {
            // Down -> horizontal -> up to above -> down to goal
            options.push_back({{start, {start.x, safeZones.yBelow}, {safeZones.xLeft, safeZones.yBelow},
                                {safeZones.xLeft, safeZones.yAbove}, {goal.x, safeZones.yAbove}, goal}, false});
            options.push_back({{start, {start.x, safeZones.yBelow}, {safeZones.xRight, safeZones.yBelow},
                                {safeZones.xRight, safeZones.yAbove}, {goal.x, safeZones.yAbove}, goal}, false});
        }
        if (requiredTargetDir == MoveDirection::None || requiredTargetDir == MoveDirection::Left) {
            // Down -> to right of goal -> left to goal
            options.push_back({{start, {start.x, safeZones.yBelow}, {safeZones.xRight, safeZones.yBelow},
                                {safeZones.xRight, goal.y}, goal}, false});
        }
        if (requiredTargetDir == MoveDirection::None || requiredTargetDir == MoveDirection::Right) {
            // Down -> to left of goal -> right to goal
            options.push_back({{start, {start.x, safeZones.yBelow}, {safeZones.xLeft, safeZones.yBelow},
                                {safeZones.xLeft, goal.y}, goal}, false});
        }
    }

    // Paths going LEFT first from source
    if (requiredSourceDir == MoveDirection::None || requiredSourceDir == MoveDirection::Left) {
        if (requiredTargetDir == MoveDirection::None || requiredTargetDir == MoveDirection::Right) {
            // Simple: left -> vertical -> right
            options.push_back({{start, {safeZones.xLeft, start.y}, {safeZones.xLeft, goal.y}, goal}, false});
        }
        if (requiredTargetDir == MoveDirection::None || requiredTargetDir == MoveDirection::Down) {
            // Left -> up to above goal -> down
            options.push_back({{start, {safeZones.xLeft, start.y}, {safeZones.xLeft, safeZones.yAbove},
                                {goal.x, safeZones.yAbove}, goal}, false});
        }
        if (requiredTargetDir == MoveDirection::None || requiredTargetDir == MoveDirection::Up) {
            // Left -> down to below goal -> up
            options.push_back({{start, {safeZones.xLeft, start.y}, {safeZones.xLeft, safeZones.yBelow},
                                {goal.x, safeZones.yBelow}, goal}, false});
        }
        if (requiredTargetDir == MoveDirection::None || requiredTargetDir == MoveDirection::Left) {
            // Complex: left -> around -> left to goal
            options.push_back({{start, {safeZones.xLeft, start.y}, {safeZones.xLeft, safeZones.yAbove},
                                {safeZones.xRight, safeZones.yAbove}, {safeZones.xRight, goal.y}, goal}, false});
        }
    }

    // Paths going RIGHT first from source
    if (requiredSourceDir == MoveDirection::None || requiredSourceDir == MoveDirection::Right) {
        if (requiredTargetDir == MoveDirection::None || requiredTargetDir == MoveDirection::Left) {
            // Simple: right -> vertical -> left
            options.push_back({{start, {safeZones.xRight, start.y}, {safeZones.xRight, goal.y}, goal}, false});
        }
        if (requiredTargetDir == MoveDirection::None || requiredTargetDir == MoveDirection::Down) {
            // Right -> up to above goal -> down
            options.push_back({{start, {safeZones.xRight, start.y}, {safeZones.xRight, safeZones.yAbove},
                                {goal.x, safeZones.yAbove}, goal}, false});
        }
        if (requiredTargetDir == MoveDirection::None || requiredTargetDir == MoveDirection::Up) {
            // Right -> down to below goal -> up
            options.push_back({{start, {safeZones.xRight, start.y}, {safeZones.xRight, safeZones.yBelow},
                                {goal.x, safeZones.yBelow}, goal}, false});
        }
        if (requiredTargetDir == MoveDirection::None || requiredTargetDir == MoveDirection::Right) {
            // Complex: right -> around -> right to goal
            options.push_back({{start, {safeZones.xRight, start.y}, {safeZones.xRight, safeZones.yAbove},
                                {safeZones.xLeft, safeZones.yAbove}, {safeZones.xLeft, goal.y}, goal}, false});
        }
    }

    // Validate all options
    for (auto& [path, valid] : options) {
        valid = validatePath(path);
    }

    // Find shortest valid path
    std::vector<GridPoint>* bestPath = nullptr;
    int bestLen = INT_MAX;

    for (auto& [path, valid] : options) {
        if (valid) {
            int len = pathLength(path);
            if (len < bestLen) {
                bestLen = len;
                bestPath = &path;
            }
        }
    }

    if (bestPath) {
        result.found = true;
        result.path = simplifyPath(*bestPath);
        result.bendCount = countBends(result.path);
        result.totalCost = bestLen;
    }

    return result;
}

int AStarPathFinder::manhattanDistance(const GridPoint& a, const GridPoint& b) {
    return std::abs(a.x - b.x) + std::abs(a.y - b.y);
}

std::vector<std::pair<GridPoint, MoveDirection>> AStarPathFinder::getNeighbors(const GridPoint& p) {
    return {
        {{p.x, p.y - 1}, MoveDirection::Up},
        {{p.x, p.y + 1}, MoveDirection::Down},
        {{p.x - 1, p.y}, MoveDirection::Left},
        {{p.x + 1, p.y}, MoveDirection::Right}
    };
}

bool AStarPathFinder::validateSegment(
    const GridPoint& from,
    const GridPoint& to,
    const IObstacleProvider& obstacles) const {

    // No exclusions - all cells must be clear
    std::unordered_set<NodeId> empty;
    return !obstacles.segmentBlocked(from.x, from.y, to.x, to.y, empty);
}

bool AStarPathFinder::validateSegmentWithEndpoints(
    const GridPoint& from,
    const GridPoint& to,
    const IObstacleProvider& obstacles,
    NodeId sourceNode,
    NodeId targetNode,
    bool isFirstSegment,
    bool isLastSegment,
    const std::unordered_set<NodeId>& extraStartExcludes,
    const std::unordered_set<NodeId>& extraGoalExcludes) const {

    // Build exclude sets for start and end cells
    std::unordered_set<NodeId> startExcludes = extraStartExcludes;
    startExcludes.insert(sourceNode);

    std::unordered_set<NodeId> endExcludes = extraGoalExcludes;
    endExcludes.insert(targetNode);

    // For orthogonal segments, check each cell along the path
    int dx = (to.x > from.x) ? 1 : (to.x < from.x) ? -1 : 0;
    int dy = (to.y > from.y) ? 1 : (to.y < from.y) ? -1 : 0;

    int x = from.x;
    int y = from.y;

    while (true) {
        // Determine exclude set for this cell
        std::unordered_set<NodeId> cellExclude;
        bool isStartCell = (x == from.x && y == from.y);
        bool isEndCell = (x == to.x && y == to.y);

        if (isFirstSegment && isStartCell) {
            cellExclude = startExcludes;
        }
        if (isLastSegment && isEndCell) {
            // Merge with any existing excludes
            for (NodeId n : endExcludes) {
                cellExclude.insert(n);
            }
        }

        if (obstacles.isBlocked(x, y, cellExclude)) {
            return false;
        }

        if (x == to.x && y == to.y) {
            break;
        }
        x += dx;
        y += dy;
    }

    return true;
}

int AStarPathFinder::countBends(const std::vector<GridPoint>& path) {
    if (path.size() < 3) {
        return 0;
    }

    int bends = 0;
    for (size_t i = 1; i + 1 < path.size(); ++i) {
        int dx1 = path[i].x - path[i-1].x;
        int dy1 = path[i].y - path[i-1].y;
        int dx2 = path[i+1].x - path[i].x;
        int dy2 = path[i+1].y - path[i].y;

        if (dx1 != 0) dx1 = dx1 > 0 ? 1 : -1;
        if (dy1 != 0) dy1 = dy1 > 0 ? 1 : -1;
        if (dx2 != 0) dx2 = dx2 > 0 ? 1 : -1;
        if (dy2 != 0) dy2 = dy2 > 0 ? 1 : -1;

        if (dx1 != dx2 || dy1 != dy2) {
            ++bends;
        }
    }

    return bends;
}

std::vector<GridPoint> AStarPathFinder::simplifyPath(const std::vector<GridPoint>& path) {
    if (path.size() <= 2) {
        return path;
    }

    std::vector<GridPoint> simplified;
    simplified.push_back(path[0]);

    for (size_t i = 1; i + 1 < path.size(); ++i) {
        int dx1 = path[i].x - path[i-1].x;
        int dy1 = path[i].y - path[i-1].y;
        int dx2 = path[i+1].x - path[i].x;
        int dy2 = path[i+1].y - path[i].y;

        if (dx1 != 0) dx1 = dx1 > 0 ? 1 : -1;
        if (dy1 != 0) dy1 = dy1 > 0 ? 1 : -1;
        if (dx2 != 0) dx2 = dx2 > 0 ? 1 : -1;
        if (dy2 != 0) dy2 = dy2 > 0 ? 1 : -1;

        if (dx1 != dx2 || dy1 != dy2) {
            simplified.push_back(path[i]);
        }
    }

    simplified.push_back(path.back());
    return simplified;
}

}  // namespace algorithms
}  // namespace arborvia
