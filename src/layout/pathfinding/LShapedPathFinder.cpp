#include "LShapedPathFinder.h"

namespace arborvia {


PathResult LShapedPathFinder::findPath(
    const GridPoint& start,
    const GridPoint& goal,
    const IObstacleProvider& obstacles,
    NodeId sourceNode,
    NodeId targetNode,
    NodeEdge sourceEdge,
    NodeEdge targetEdge,
    const std::unordered_set<NodeId>& extraStartExcludes,
    const std::unordered_set<NodeId>& extraGoalExcludes) const {

    // Trivial case: start == goal
    if (start == goal) {
        PathResult result;
        result.found = true;
        result.path = {start};
        result.bendCount = 0;
        result.totalCost = 0;
        return result;
    }

    // Build PathContext
    PathContext ctx;
    ctx.start = start;
    ctx.goal = goal;
    ctx.obstacles = &obstacles;
    ctx.sourceNode = sourceNode;
    ctx.targetNode = targetNode;
    ctx.sourceEdge = sourceEdge;
    ctx.targetEdge = targetEdge;
    ctx.buildExcludes(extraStartExcludes, extraGoalExcludes);

    // Get required directions
    MoveDirection exitDir = getExitDirection(sourceEdge);

    // Determine which L-shape to try first based on direction constraints
    bool preferHorizontalFirst = (exitDir == MoveDirection::Left || exitDir == MoveDirection::Right);
    bool preferVerticalFirst = (exitDir == MoveDirection::Up || exitDir == MoveDirection::Down);

    // Try L-shaped paths based on preference
    PathResult result;

    if (preferHorizontalFirst) {
        result = tryHorizontalFirst(ctx);
        if (result.found) return result;

        result = tryVerticalFirst(ctx);
        if (result.found) return result;
    } else if (preferVerticalFirst) {
        result = tryVerticalFirst(ctx);
        if (result.found) return result;

        result = tryHorizontalFirst(ctx);
        if (result.found) return result;
    } else {
        // No strong preference, try both
        result = tryHorizontalFirst(ctx);
        if (result.found) return result;

        result = tryVerticalFirst(ctx);
        if (result.found) return result;
    }

    // Try U-shaped paths
    // Horizontal U-shape (detour left or right)
    result = tryUShapeHorizontal(ctx, -1);
    if (result.found) return result;

    result = tryUShapeHorizontal(ctx, +1);
    if (result.found) return result;

    // Vertical U-shape (detour up or down)
    result = tryUShapeVertical(ctx, -1);
    if (result.found) return result;

    result = tryUShapeVertical(ctx, +1);
    if (result.found) return result;

    // Fallback to safe zone routing
    return findPathViaSafeZone(start, goal, obstacles, sourceNode, targetNode,
                               sourceEdge, targetEdge, extraStartExcludes, extraGoalExcludes);
}

PathResult LShapedPathFinder::tryHorizontalFirst(const PathContext& ctx) const {
    PathResult result;
    result.found = false;

    const auto& start = ctx.start;
    const auto& goal = ctx.goal;

    // L-shape: start -> corner (horizontal) -> goal (vertical)
    GridPoint corner{goal.x, start.y};

    // Check direction constraints
    MoveDirection firstDir = (goal.x > start.x) ? MoveDirection::Right : MoveDirection::Left;
    MoveDirection secondDir = (goal.y > start.y) ? MoveDirection::Down : MoveDirection::Up;

    if (start.x == goal.x) {
        // Already aligned horizontally, this is a straight vertical line
        firstDir = MoveDirection::None;
    }
    if (start.y == goal.y) {
        // Already aligned vertically, this is a straight horizontal line
        secondDir = MoveDirection::None;
    }

    // Check if first direction is compatible with source edge
    if (firstDir != MoveDirection::None && !isDirectionCompatible(ctx.sourceEdge, firstDir)) {
        return result;
    }

    // Check if second direction is compatible with target edge (arrival direction)
    if (secondDir != MoveDirection::None) {
        MoveDirection requiredEntry = getEntryDirection(ctx.targetEdge);
        if (requiredEntry != MoveDirection::None && requiredEntry != secondDir) {
            return result;
        }
    }

    // Check first segment (start to corner)
    if (start != corner) {
        if (!isSegmentClear(start, corner, *ctx.obstacles, ctx.startExcludes, {})) {
            return result;
        }
    }

    // Check second segment (corner to goal)
    if (corner != goal) {
        if (!isSegmentClear(corner, goal, *ctx.obstacles, {}, ctx.goalExcludes)) {
            return result;
        }
    }

    // Build path
    result.found = true;
    result.path.push_back(start);
    if (start != corner && corner != goal) {
        result.path.push_back(corner);
    }
    if (start != goal) {
        result.path.push_back(goal);
    }
    result.bendCount = countBends(result.path);
    result.totalCost = std::abs(goal.x - start.x) + std::abs(goal.y - start.y);

    return result;
}

PathResult LShapedPathFinder::tryVerticalFirst(const PathContext& ctx) const {
    PathResult result;
    result.found = false;

    const auto& start = ctx.start;
    const auto& goal = ctx.goal;

    // L-shape: start -> corner (vertical) -> goal (horizontal)
    GridPoint corner{start.x, goal.y};

    // Check direction constraints
    MoveDirection firstDir = (goal.y > start.y) ? MoveDirection::Down : MoveDirection::Up;
    MoveDirection secondDir = (goal.x > start.x) ? MoveDirection::Right : MoveDirection::Left;

    if (start.y == goal.y) {
        firstDir = MoveDirection::None;
    }
    if (start.x == goal.x) {
        secondDir = MoveDirection::None;
    }

    // Check if first direction is compatible with source edge
    if (firstDir != MoveDirection::None && !isDirectionCompatible(ctx.sourceEdge, firstDir)) {
        return result;
    }

    // Check if second direction is compatible with target edge (arrival direction)
    if (secondDir != MoveDirection::None) {
        MoveDirection requiredEntry = getEntryDirection(ctx.targetEdge);
        if (requiredEntry != MoveDirection::None && requiredEntry != secondDir) {
            return result;
        }
    }

    // Check first segment (start to corner)
    if (start != corner) {
        if (!isSegmentClear(start, corner, *ctx.obstacles, ctx.startExcludes, {})) {
            return result;
        }
    }

    // Check second segment (corner to goal)
    if (corner != goal) {
        if (!isSegmentClear(corner, goal, *ctx.obstacles, {}, ctx.goalExcludes)) {
            return result;
        }
    }

    // Build path
    result.found = true;
    result.path.push_back(start);
    if (start != corner && corner != goal) {
        result.path.push_back(corner);
    }
    if (start != goal) {
        result.path.push_back(goal);
    }
    result.bendCount = countBends(result.path);
    result.totalCost = std::abs(goal.x - start.x) + std::abs(goal.y - start.y);

    return result;
}

PathResult LShapedPathFinder::tryUShapeHorizontal(const PathContext& ctx, int detourDirection) const {
    PathResult result;
    result.found = false;

    const auto& start = ctx.start;
    const auto& goal = ctx.goal;

    // U-shape with horizontal detour:
    // start -> p1 (vertical) -> p2 (horizontal) -> goal (vertical)
    // detourDirection: -1 = left, +1 = right

    const auto& safeZones = ctx.obstacles->safeZones();
    int detourX = (detourDirection < 0) ? safeZones.xLeft - MIN_DETOUR_DISTANCE
                                        : safeZones.xRight + MIN_DETOUR_DISTANCE;

    GridPoint p1{start.x, start.y};  // Will be adjusted
    GridPoint p2{detourX, goal.y};

    // Adjust p1 based on source edge direction
    MoveDirection exitDir = getExitDirection(ctx.sourceEdge);
    if (exitDir == MoveDirection::Up || exitDir == MoveDirection::Down) {
        // Need to go vertical first, then horizontal
        int midY = (exitDir == MoveDirection::Up) ? std::min(start.y, goal.y) - MIN_DETOUR_DISTANCE
                                                  : std::max(start.y, goal.y) + MIN_DETOUR_DISTANCE;
        p1 = {start.x, midY};
        p2 = {detourX, midY};
    } else {
        // Go horizontal first
        p1 = {detourX, start.y};
        p2 = {detourX, goal.y};
    }

    // Check all segments
    if (!isSegmentClear(start, p1, *ctx.obstacles, ctx.startExcludes, {})) return result;
    if (!isSegmentClear(p1, p2, *ctx.obstacles, {}, {})) return result;
    if (!isSegmentClear(p2, goal, *ctx.obstacles, {}, ctx.goalExcludes)) return result;

    // Build path
    result.found = true;
    result.path = {start, p1, p2, goal};
    result.bendCount = countBends(result.path);
    result.totalCost = std::abs(p1.x - start.x) + std::abs(p1.y - start.y)
                     + std::abs(p2.x - p1.x) + std::abs(p2.y - p1.y)
                     + std::abs(goal.x - p2.x) + std::abs(goal.y - p2.y);

    return result;
}

PathResult LShapedPathFinder::tryUShapeVertical(const PathContext& ctx, int detourDirection) const {
    PathResult result;
    result.found = false;

    const auto& start = ctx.start;
    const auto& goal = ctx.goal;

    // U-shape with vertical detour:
    // start -> p1 (horizontal) -> p2 (vertical) -> goal (horizontal)
    // detourDirection: -1 = up, +1 = down

    const auto& safeZones = ctx.obstacles->safeZones();
    int detourY = (detourDirection < 0) ? safeZones.yAbove - MIN_DETOUR_DISTANCE
                                        : safeZones.yBelow + MIN_DETOUR_DISTANCE;

    GridPoint p1{start.x, start.y};  // Will be adjusted
    GridPoint p2{goal.x, detourY};

    // Adjust p1 based on source edge direction
    MoveDirection exitDir = getExitDirection(ctx.sourceEdge);
    if (exitDir == MoveDirection::Left || exitDir == MoveDirection::Right) {
        // Need to go horizontal first, then vertical
        int midX = (exitDir == MoveDirection::Left) ? std::min(start.x, goal.x) - MIN_DETOUR_DISTANCE
                                                    : std::max(start.x, goal.x) + MIN_DETOUR_DISTANCE;
        p1 = {midX, start.y};
        p2 = {midX, detourY};
    } else {
        // Go vertical first
        p1 = {start.x, detourY};
        p2 = {goal.x, detourY};
    }

    // Check all segments
    if (!isSegmentClear(start, p1, *ctx.obstacles, ctx.startExcludes, {})) return result;
    if (!isSegmentClear(p1, p2, *ctx.obstacles, {}, {})) return result;
    if (!isSegmentClear(p2, goal, *ctx.obstacles, {}, ctx.goalExcludes)) return result;

    // Build path
    result.found = true;
    result.path = {start, p1, p2, goal};
    result.bendCount = countBends(result.path);
    result.totalCost = std::abs(p1.x - start.x) + std::abs(p1.y - start.y)
                     + std::abs(p2.x - p1.x) + std::abs(p2.y - p1.y)
                     + std::abs(goal.x - p2.x) + std::abs(goal.y - p2.y);

    return result;
}

PathResult LShapedPathFinder::findPathViaSafeZone(
    const GridPoint& start,
    const GridPoint& goal,
    const IObstacleProvider& obstacles,
    NodeId sourceNode,
    NodeId targetNode,
    NodeEdge sourceEdge,
    [[maybe_unused]] NodeEdge targetEdge,
    const std::unordered_set<NodeId>& extraStartExcludes,
    const std::unordered_set<NodeId>& extraGoalExcludes) const {

    PathResult result;
    result.found = false;

    const auto& safeZones = obstacles.safeZones();

    // Route via safe zone boundary
    // Choose the closest safe zone edge
    int safeY = (std::abs(start.y - safeZones.yAbove) < std::abs(start.y - safeZones.yBelow))
                ? safeZones.yAbove - 1 : safeZones.yBelow + 1;
    int safeX = (std::abs(start.x - safeZones.xLeft) < std::abs(start.x - safeZones.xRight))
                ? safeZones.xLeft - 1 : safeZones.xRight + 1;

    // Try vertical safe zone route
    MoveDirection exitDir = getExitDirection(sourceEdge);

    // Simple 4-point route through safe zone
    GridPoint p1, p2;

    if (exitDir == MoveDirection::Up || exitDir == MoveDirection::Down) {
        // Exit vertically
        p1 = {start.x, safeY};
        p2 = {goal.x, safeY};
    } else {
        // Exit horizontally
        p1 = {safeX, start.y};
        p2 = {safeX, goal.y};
    }

    // Build exclusion sets
    std::unordered_set<NodeId> startExcludes = extraStartExcludes;
    startExcludes.insert(sourceNode);
    std::unordered_set<NodeId> goalExcludes = extraGoalExcludes;
    goalExcludes.insert(targetNode);

    // Verify segments (safe zone should be clear by definition)
    if (!isSegmentClear(start, p1, obstacles, startExcludes, {})) {
        // Try alternative safe zone
        if (exitDir == MoveDirection::Up || exitDir == MoveDirection::Down) {
            safeY = (safeY == safeZones.yAbove - 1) ? safeZones.yBelow + 1 : safeZones.yAbove - 1;
            p1 = {start.x, safeY};
            p2 = {goal.x, safeY};
        } else {
            safeX = (safeX == safeZones.xLeft - 1) ? safeZones.xRight + 1 : safeZones.xLeft - 1;
            p1 = {safeX, start.y};
            p2 = {safeX, goal.y};
        }
    }

    // Build path
    result.found = true;
    result.path = {start, p1, p2, goal};
    result.bendCount = countBends(result.path);
    result.totalCost = std::abs(p1.x - start.x) + std::abs(p1.y - start.y)
                     + std::abs(p2.x - p1.x) + std::abs(p2.y - p1.y)
                     + std::abs(goal.x - p2.x) + std::abs(goal.y - p2.y);

    return result;
}

bool LShapedPathFinder::isSegmentClear(
    const GridPoint& from,
    const GridPoint& to,
    const IObstacleProvider& obstacles,
    const std::unordered_set<NodeId>& excludeAtFrom,
    const std::unordered_set<NodeId>& excludeAtTo) const {

    // Check if segment is horizontal or vertical
    if (from.x != to.x && from.y != to.y) {
        // Diagonal segment not supported
        return false;
    }

    // Check each cell along the segment
    int dx = (to.x > from.x) ? 1 : (to.x < from.x) ? -1 : 0;
    int dy = (to.y > from.y) ? 1 : (to.y < from.y) ? -1 : 0;

    int x = from.x;
    int y = from.y;

    while (true) {
        // Determine exclusions for this cell
        const std::unordered_set<NodeId>* excludes = nullptr;
        if (x == from.x && y == from.y) {
            excludes = &excludeAtFrom;
        } else if (x == to.x && y == to.y) {
            excludes = &excludeAtTo;
        }

        bool blocked = excludes ? obstacles.isBlocked(x, y, *excludes)
                                : obstacles.isBlocked(x, y);

        if (blocked) {
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

bool LShapedPathFinder::isDirectionCompatible(NodeEdge edge, MoveDirection dir) {
    switch (edge) {
        case NodeEdge::Top:    return dir == MoveDirection::Up;
        case NodeEdge::Bottom: return dir == MoveDirection::Down;
        case NodeEdge::Left:   return dir == MoveDirection::Left;
        case NodeEdge::Right:  return dir == MoveDirection::Right;
    }
    return true;
}

MoveDirection LShapedPathFinder::getExitDirection(NodeEdge edge) {
    switch (edge) {
        case NodeEdge::Top:    return MoveDirection::Up;
        case NodeEdge::Bottom: return MoveDirection::Down;
        case NodeEdge::Left:   return MoveDirection::Left;
        case NodeEdge::Right:  return MoveDirection::Right;
    }
    return MoveDirection::None;
}

MoveDirection LShapedPathFinder::getEntryDirection(NodeEdge edge) {
    // Entry direction is opposite of exit direction
    // e.g., to enter from Top edge, must approach from above (moving Down)
    switch (edge) {
        case NodeEdge::Top:    return MoveDirection::Down;
        case NodeEdge::Bottom: return MoveDirection::Up;
        case NodeEdge::Left:   return MoveDirection::Right;
        case NodeEdge::Right:  return MoveDirection::Left;
    }
    return MoveDirection::None;
}

int LShapedPathFinder::countBends(const std::vector<GridPoint>& path) {
    if (path.size() < 3) return 0;

    int bends = 0;
    for (size_t i = 1; i < path.size() - 1; ++i) {
        const auto& prev = path[i - 1];
        const auto& curr = path[i];
        const auto& next = path[i + 1];

        // Check if direction changes
        int dx1 = curr.x - prev.x;
        int dx2 = next.x - curr.x;

        // Direction changes if movement axis changes
        bool wasHorizontal = (dx1 != 0);
        bool nowHorizontal = (dx2 != 0);

        if (wasHorizontal != nowHorizontal) {
            ++bends;
        }
    }

    return bends;
}


}  // namespace arborvia
