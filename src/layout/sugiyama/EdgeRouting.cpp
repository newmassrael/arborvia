#include "EdgeRouting.h"
#include "SnapIndexManager.h"
#include "ObstacleMap.h"
#include "AStarPathFinder.h"
#include "AStarEdgeOptimizer.h"
#include "GeometricEdgeOptimizer.h"
#include "EdgeValidator.h"
#include "SelfLoopRouter.h"
#include "PathCleanup.h"
#include "arborvia/core/GeometryUtils.h"
#include "arborvia/layout/IEdgeOptimizer.h"
#include "arborvia/layout/EdgeConstraintManager.h"
#include "arborvia/layout/LayoutTypes.h"
#include "arborvia/layout/LayoutUtils.h"
#include "arborvia/layout/PathRoutingCoordinator.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <set>
#include <unordered_map>

// Debug flag for edge routing
#ifndef EDGE_ROUTING_DEBUG
#define EDGE_ROUTING_DEBUG 1
#endif

namespace arborvia {


// =============================================================================
// Routing Constants
// =============================================================================

namespace {
    /// Tolerance for floating point comparisons in path calculations
    constexpr float EPSILON = 0.1f;

    // =========================================================================
    // Grid-Relative Offset Functions
    // =========================================================================
    // These functions return offsets that are guaranteed to be grid-aligned
    // when gridSize > 0, preventing post-hoc snap from creating spikes/duplicates.

    /// Get effective grid size for calculations
    // =========================================================================
    // Grid-Unit Constants for Quantized-First Calculations
    // =========================================================================
    // These constants define offsets in GRID UNITS (integers).
    // All routing calculations use these directly without conversion.

    /// Returns PATHFINDING_GRID_SIZE when grid is disabled (gridSize <= 0)
    /// This ensures A* pathfinding uses a coarse enough grid for performance
    inline float getEffectiveGridSize(float gridSize) {
        return gridSize > 0.0f ? gridSize : constants::PATHFINDING_GRID_SIZE;
    }

    // =========================================================================
    // Grid Coordinate Types for Quantized-First Calculations
    // =========================================================================

    /// Node bounds in grid units (integers) for quantized calculations
    /// Using floor for left/top and ceil for right/bottom ensures the bounds
    /// fully contain the node when converted back to pixels.
    struct GridNodeBounds {
        int left;
        int top;
        int right;
        int bottom;

        /// Create from NodeLayout using grid quantization
        static GridNodeBounds fromNodeLayout(const NodeLayout& layout, float gridSize) {
            return {
                static_cast<int>(std::floor(layout.position.x / gridSize)),
                static_cast<int>(std::floor(layout.position.y / gridSize)),
                static_cast<int>(std::ceil((layout.position.x + layout.size.width) / gridSize)),
                static_cast<int>(std::ceil((layout.position.y + layout.size.height) / gridSize))
            };
        }

        int width() const { return right - left; }
        int height() const { return bottom - top; }
    };

    /// Distribute snap points evenly in grid coordinates (preserves symmetry)
    /// Uses integer arithmetic to avoid round() accumulation errors
    /// @param edgeStart Start of edge in grid units
    /// @param edgeEnd End of edge in grid units
    /// @param count Number of snap points to distribute
    /// @param outPositions Output vector for grid positions
    inline void distributeSnapPointsQuantized(
        int edgeStart,
        int edgeEnd,
        int count,
        std::vector<int>& outPositions
    ) {
        if (count <= 0) return;
        int length = edgeEnd - edgeStart;
        int divisor = count + 1;

        for (int i = 0; i < count; ++i) {
            // Integer division with rounding: (a * b + divisor/2) / divisor
            int gridPos = edgeStart + (length * (i + 1) * 2 + divisor) / (2 * divisor);
            outPositions.push_back(gridPos);
        }
    }

    /// Get the grid coordinate range for a node edge
    /// @param node Node layout
    /// @param edge Which edge to get range for
    /// @param gridSize Grid cell size
    /// @return Pair of (start, end) in grid units
    inline std::pair<int, int> getEdgeGridRange(
        const NodeLayout& node,
        NodeEdge edge,
        float gridSize
    ) {
        // For Top/Bottom edges: range is along X axis
        // For Left/Right edges: range is along Y axis
        switch (edge) {
            case NodeEdge::Top:
            case NodeEdge::Bottom: {
                int gridLeft = static_cast<int>(std::ceil(node.position.x / gridSize));
                int gridRight = static_cast<int>(std::floor((node.position.x + node.size.width) / gridSize));
                return {gridLeft, gridRight};
            }
            case NodeEdge::Left:
            case NodeEdge::Right: {
                int gridTop = static_cast<int>(std::ceil(node.position.y / gridSize));
                int gridBottom = static_cast<int>(std::floor((node.position.y + node.size.height) / gridSize));
                return {gridTop, gridBottom};
            }
        }
        return {0, 0};
    }

    /// Calculate snap point position using quantized (grid-first) calculation
    /// This preserves symmetry by doing all arithmetic in grid units
    /// @param node Node layout
    /// @param edge Which edge the snap point is on
    /// @param snapIndex 0-based index of this snap point
    /// @param totalCount Total number of snap points on this edge
    /// @param rangeStart Relative start of range (0.0-1.0, for separated mode)
    /// @param rangeEnd Relative end of range (0.0-1.0, for separated mode)
    /// @param gridSize Grid cell size
    /// @return Snap point position in pixel coordinates (grid-aligned)
    inline Point calculateSnapPositionQuantized(
        const NodeLayout& node,
        NodeEdge edge,
        int snapIndex,
        int totalCount,
        float rangeStart,
        float rangeEnd,
        float gridSize
    ) {
        auto [fullStart, fullEnd] = getEdgeGridRange(node, edge, gridSize);
        int fullLength = fullEnd - fullStart;

        // Calculate the sub-range in grid units
        int rangeGridStart = fullStart + static_cast<int>(std::round(fullLength * rangeStart));
        int rangeGridEnd = fullStart + static_cast<int>(std::round(fullLength * rangeEnd));

        // Distribute evenly within the sub-range using integer arithmetic
        int rangeLength = rangeGridEnd - rangeGridStart;
        int divisor = totalCount + 1;
        int gridPos = rangeGridStart + (rangeLength * (snapIndex + 1) * 2 + divisor) / (2 * divisor);

        // Convert back to pixel coordinates
        float pixelCoord = gridPos * gridSize;

        // Create point based on edge type
        // The edge position (fixed coordinate) must also be snapped to grid
        switch (edge) {
            case NodeEdge::Top: {
                float edgeY = std::round(node.position.y / gridSize) * gridSize;
                return {pixelCoord, edgeY};
            }
            case NodeEdge::Bottom: {
                float edgeY = std::round((node.position.y + node.size.height) / gridSize) * gridSize;
                return {pixelCoord, edgeY};
            }
            case NodeEdge::Left: {
                float edgeX = std::round(node.position.x / gridSize) * gridSize;
                return {edgeX, pixelCoord};
            }
            case NodeEdge::Right: {
                float edgeX = std::round((node.position.x + node.size.width) / gridSize) * gridSize;
                return {edgeX, pixelCoord};
            }
        }
        return node.center();
    }

    // =========================================================================
    // Segment-Node Intersection (Namespace Version)
    // =========================================================================

    /// Check if an orthogonal segment intersects a node's interior (excluding boundary)
    // =========================================================================
    // Axis Abstraction for Horizontal/Vertical Routing
    // =========================================================================

    /// Configuration for axis-agnostic routing operations
    /// Horizontal layout: primary = X, secondary = Y
    /// Vertical layout: primary = Y, secondary = X
    struct AxisConfig {
        bool isHorizontal;

        /// Get primary coordinate (X for horizontal, Y for vertical)
        float primary(const Point& p) const {
            return isHorizontal ? p.x : p.y;
        }

        /// Get secondary coordinate (Y for horizontal, X for vertical)
        float secondary(const Point& p) const {
            return isHorizontal ? p.y : p.x;
        }

        /// Set primary coordinate on a point
        void setPrimary(Point& p, float value) const {
            if (isHorizontal) p.x = value;
            else p.y = value;
        }

        /// Set secondary coordinate on a point
        void setSecondary(Point& p, float value) const {
            if (isHorizontal) p.y = value;
            else p.x = value;
        }

        /// Create point from primary and secondary coordinates
        Point makePoint(float primaryCoord, float secondaryCoord) const {
            return isHorizontal ? Point{primaryCoord, secondaryCoord}
                                : Point{secondaryCoord, primaryCoord};
        }

        /// Get node min on primary axis
        float nodeMin(const NodeLayout& node) const {
            return isHorizontal ? node.position.x : node.position.y;
        }

        /// Get node max on primary axis
        float nodeMax(const NodeLayout& node) const {
            return isHorizontal ? (node.position.x + node.size.width)
                                : (node.position.y + node.size.height);
        }

        /// Get node min on secondary axis
        float nodeSecondaryMin(const NodeLayout& node) const {
            return isHorizontal ? node.position.y : node.position.x;
        }

        /// Get node max on secondary axis
        float nodeSecondaryMax(const NodeLayout& node) const {
            return isHorizontal ? (node.position.y + node.size.height)
                                : (node.position.x + node.size.width);
        }

        /// Check if edge is a "positive direction" edge (Right/Bottom)
        bool isPositiveEdge(NodeEdge edge) const {
            return isHorizontal ? (edge == NodeEdge::Right) : (edge == NodeEdge::Bottom);
        }

        /// Check if edge is a lower bound (positive direction = lower bound)
        /// Right/Bottom edges: constraint is LOWER bound (value must be >= constraint)
        /// Left/Top edges: constraint is UPPER bound (value must be <= constraint)
        bool isLowerBound(NodeEdge edge) const {
            return isPositiveEdge(edge);
        }

        /// Get channel coordinate from EdgeLayout (channelY stores both X and Y depending on orientation)
        float getChannel(const EdgeLayout& layout) const {
            return layout.channelY;  // channelY is used for both orientations
        }

        /// Get source point primary coordinate
        float sourcePrimary(const EdgeLayout& layout) const {
            return primary(layout.sourcePoint);
        }

        /// Get target point primary coordinate
        float targetPrimary(const EdgeLayout& layout) const {
            return primary(layout.targetPoint);
        }

        /// Get source point secondary coordinate
        float sourceSecondary(const EdgeLayout& layout) const {
            return secondary(layout.sourcePoint);
        }

        /// Get target point secondary coordinate
        float targetSecondary(const EdgeLayout& layout) const {
            return secondary(layout.targetPoint);
        }

        // =========================================================================
        // Grid-Unit Operations for Quantized-First Calculations
        // =========================================================================

        /// Convert pixel coordinate to grid unit (round to nearest)
        int toGrid(float pixel, float gridSize) const {
            return static_cast<int>(std::round(pixel / gridSize));
        }

        /// Convert grid unit to pixel coordinate
        float toPixel(int grid, float gridSize) const {
            return grid * gridSize;
        }

        /// Get primary coordinate in grid units
        int primaryGrid(const Point& p, float gridSize) const {
            return toGrid(primary(p), gridSize);
        }

        /// Get secondary coordinate in grid units
        int secondaryGrid(const Point& p, float gridSize) const {
            return toGrid(secondary(p), gridSize);
        }

        /// Create point from grid coordinates
        Point makePointFromGrid(int gridPrimary, int gridSecondary, float gridSize) const {
            return makePoint(toPixel(gridPrimary, gridSize), toPixel(gridSecondary, gridSize));
        }

        /// Get node bounds in grid units (uses floor for min, ceil for max)
        void getNodeGridBounds(const NodeLayout& node, float gridSize,
                              int& gridMin, int& gridMax,
                              int& gridSecondaryMin, int& gridSecondaryMax) const {
            if (isHorizontal) {
                gridMin = static_cast<int>(std::floor(node.position.x / gridSize));
                gridMax = static_cast<int>(std::ceil((node.position.x + node.size.width) / gridSize));
                gridSecondaryMin = static_cast<int>(std::floor(node.position.y / gridSize));
                gridSecondaryMax = static_cast<int>(std::ceil((node.position.y + node.size.height) / gridSize));
            } else {
                gridMin = static_cast<int>(std::floor(node.position.y / gridSize));
                gridMax = static_cast<int>(std::ceil((node.position.y + node.size.height) / gridSize));
                gridSecondaryMin = static_cast<int>(std::floor(node.position.x / gridSize));
                gridSecondaryMax = static_cast<int>(std::ceil((node.position.x + node.size.width) / gridSize));
            }
        }
    };

    // =========================================================================
    // Grid-Based Routing Functions (Quantized-First)
    // =========================================================================
    // These functions perform all calculations in grid units (integers).
    // Conversion to pixel coordinates happens only at the final BendPoint creation.










    struct PairHash {
        template <typename T1, typename T2>
        std::size_t operator()(const std::pair<T1, T2>& p) const {
            auto h1 = std::hash<T1>{}(p.first);
            auto h2 = std::hash<T2>{}(p.second);
            return h1 ^ (h2 << 1);
        }
    };

}

// =============================================================================
// Constructor
// =============================================================================

EdgeRouting::EdgeRouting(std::shared_ptr<IPathFinder> pathFinder)
    : pathFinder_(pathFinder ? std::move(pathFinder)
                             : std::make_shared<AStarPathFinder>()) {
}

EdgeRouting::EdgeRouting(PathRoutingCoordinator* coordinator)
    : pathFinder_(std::make_shared<AStarPathFinder>())
    , coordinator_(coordinator) {
}

const IPathFinder& EdgeRouting::pathFinder() const {
    return activePathFinder();
}

void EdgeRouting::setRoutingCoordinator(PathRoutingCoordinator* coordinator) {
    coordinator_ = coordinator;
}

PathRoutingCoordinator* EdgeRouting::routingCoordinator() const {
    return coordinator_;
}

void EdgeRouting::setEdgeOptimizer(std::shared_ptr<IEdgeOptimizer> optimizer) {
    edgeOptimizer_ = std::move(optimizer);
}

IEdgeOptimizer* EdgeRouting::edgeOptimizer() const {
    return edgeOptimizer_.get();
}

IPathFinder& EdgeRouting::activePathFinder() const {
    if (coordinator_) {
        return coordinator_->currentPathFinder();
    }
    return *pathFinder_;
}

// =============================================================================
// Static Helper Function Implementations
// =============================================================================

Point EdgeRouting::calculateSnapPosition(const NodeLayout& node, NodeEdge edge, float position) {
    return LayoutUtils::calculateSnapPointFromPosition(node, edge, position);
}

int EdgeRouting::unifiedToLocalIndex(int unifiedIdx, int offset, int count) {
    // Delegate to SnapIndexManager for centralized logic
    return SnapIndexManager::unifiedToLocal(unifiedIdx, offset, count);
}

// =============================================================================
// Main recalculateBendPoints Method
// =============================================================================

void EdgeRouting::recalculateBendPoints(
    EdgeLayout& layout,
    const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
    float gridSize) {
    layout.bendPoints.clear();

    // Handle self-loops specially - they need to route around the node corner
    // CONSTRAINT: Self-loops use adjacent edges (sourceEdge != targetEdge)
    if (layout.from == layout.to) {
        auto nodeIt = nodeLayouts.find(layout.from);
        if (nodeIt != nodeLayouts.end()) {
            const NodeLayout& node = nodeIt->second;
            // Self-loop offset: use the larger of gridSize or default offset
            float offset = std::max(gridSize, SelfLoopRouter::DEFAULT_OFFSET);

            float nodeLeft = node.position.x;
            float nodeRight = node.position.x + node.size.width;
            float nodeTop = node.position.y;
            float nodeBottom = node.position.y + node.size.height;

            // Route based on sourceEdge → targetEdge combination (adjacent edges)
            if (layout.sourceEdge == NodeEdge::Right && layout.targetEdge == NodeEdge::Top) {
                // Right → Top (top-right corner)
                layout.bendPoints.push_back({{nodeRight + offset, layout.sourcePoint.y}});
                layout.bendPoints.push_back({{nodeRight + offset, nodeTop - offset}});
                layout.bendPoints.push_back({{layout.targetPoint.x, nodeTop - offset}});
            }
            else if (layout.sourceEdge == NodeEdge::Right && layout.targetEdge == NodeEdge::Bottom) {
                // Right → Bottom (bottom-right corner)
                layout.bendPoints.push_back({{nodeRight + offset, layout.sourcePoint.y}});
                layout.bendPoints.push_back({{nodeRight + offset, nodeBottom + offset}});
                layout.bendPoints.push_back({{layout.targetPoint.x, nodeBottom + offset}});
            }
            else if (layout.sourceEdge == NodeEdge::Left && layout.targetEdge == NodeEdge::Top) {
                // Left → Top (top-left corner)
                layout.bendPoints.push_back({{nodeLeft - offset, layout.sourcePoint.y}});
                layout.bendPoints.push_back({{nodeLeft - offset, nodeTop - offset}});
                layout.bendPoints.push_back({{layout.targetPoint.x, nodeTop - offset}});
            }
            else if (layout.sourceEdge == NodeEdge::Left && layout.targetEdge == NodeEdge::Bottom) {
                // Left → Bottom (bottom-left corner)
                layout.bendPoints.push_back({{nodeLeft - offset, layout.sourcePoint.y}});
                layout.bendPoints.push_back({{nodeLeft - offset, nodeBottom + offset}});
                layout.bendPoints.push_back({{layout.targetPoint.x, nodeBottom + offset}});
            }
            else if (layout.sourceEdge == NodeEdge::Top && layout.targetEdge == NodeEdge::Right) {
                // Top → Right (top-right corner)
                layout.bendPoints.push_back({{layout.sourcePoint.x, nodeTop - offset}});
                layout.bendPoints.push_back({{nodeRight + offset, nodeTop - offset}});
                layout.bendPoints.push_back({{nodeRight + offset, layout.targetPoint.y}});
            }
            else if (layout.sourceEdge == NodeEdge::Top && layout.targetEdge == NodeEdge::Left) {
                // Top → Left (top-left corner)
                layout.bendPoints.push_back({{layout.sourcePoint.x, nodeTop - offset}});
                layout.bendPoints.push_back({{nodeLeft - offset, nodeTop - offset}});
                layout.bendPoints.push_back({{nodeLeft - offset, layout.targetPoint.y}});
            }
            else if (layout.sourceEdge == NodeEdge::Bottom && layout.targetEdge == NodeEdge::Right) {
                // Bottom → Right (bottom-right corner)
                layout.bendPoints.push_back({{layout.sourcePoint.x, nodeBottom + offset}});
                layout.bendPoints.push_back({{nodeRight + offset, nodeBottom + offset}});
                layout.bendPoints.push_back({{nodeRight + offset, layout.targetPoint.y}});
            }
            else if (layout.sourceEdge == NodeEdge::Bottom && layout.targetEdge == NodeEdge::Left) {
                // Bottom → Left (bottom-left corner)
                layout.bendPoints.push_back({{layout.sourcePoint.x, nodeBottom + offset}});
                layout.bendPoints.push_back({{nodeLeft - offset, nodeBottom + offset}});
                layout.bendPoints.push_back({{nodeLeft - offset, layout.targetPoint.y}});
            }
            else {
                // Fallback for legacy same-edge self-loops (should not happen with new constraint)
                switch (layout.sourceEdge) {
                    case NodeEdge::Right:
                        layout.bendPoints.push_back({{nodeRight + offset, layout.sourcePoint.y}});
                        layout.bendPoints.push_back({{nodeRight + offset, layout.targetPoint.y}});
                        break;
                    case NodeEdge::Left:
                        layout.bendPoints.push_back({{nodeLeft - offset, layout.sourcePoint.y}});
                        layout.bendPoints.push_back({{nodeLeft - offset, layout.targetPoint.y}});
                        break;
                    case NodeEdge::Top:
                        layout.bendPoints.push_back({{layout.sourcePoint.x, nodeTop - offset}});
                        layout.bendPoints.push_back({{layout.targetPoint.x, nodeTop - offset}});
                        break;
                    case NodeEdge::Bottom:
                        layout.bendPoints.push_back({{layout.sourcePoint.x, nodeBottom + offset}});
                        layout.bendPoints.push_back({{layout.targetPoint.x, nodeBottom + offset}});
                        break;
                    default:
                        break;
                }
            }
        }
        return;  // Self-loops don't need further processing
    }

    // Use effective grid size for routing calculations
    float effectiveGridSize = gridSize > 0.0f ? gridSize : constants::PATHFINDING_GRID_SIZE;

    // Build obstacle map from node layouts
    ObstacleMap obstacles;
    obstacles.buildFromNodes(nodeLayouts, effectiveGridSize, 0);  // margin = 0 (no extra padding)

    // Convert source/target points to grid coordinates
    GridPoint startGrid = obstacles.pixelToGrid(layout.sourcePoint);
    GridPoint goalGrid = obstacles.pixelToGrid(layout.targetPoint);

    // Find additional nodes to exclude at start/goal cells
    // These are nodes (other than source/target) that contain the source/target points
    std::unordered_set<NodeId> extraStartExcludes;
    std::unordered_set<NodeId> extraGoalExcludes;

    for (const auto& [nodeId, node] : nodeLayouts) {
        if (nodeId == layout.from || nodeId == layout.to) continue;

        // Check if sourcePoint is inside this node
        if (layout.sourcePoint.x >= node.position.x &&
            layout.sourcePoint.x <= node.position.x + node.size.width &&
            layout.sourcePoint.y >= node.position.y &&
            layout.sourcePoint.y <= node.position.y + node.size.height) {
            extraStartExcludes.insert(nodeId);
        }

        // Check if targetPoint is inside this node
        if (layout.targetPoint.x >= node.position.x &&
            layout.targetPoint.x <= node.position.x + node.size.width &&
            layout.targetPoint.y >= node.position.y &&
            layout.targetPoint.y <= node.position.y + node.size.height) {
            extraGoalExcludes.insert(nodeId);
        }
    }

    // Use PathFinder with direction constraints:
    // - sourceEdge constrains first move direction
    // - targetEdge constrains arrival direction
    // - sourceNode (layout.from) + extraStartExcludes excluded only at start cell
    // - targetNode (layout.to) + extraGoalExcludes excluded only at goal cell
    PathResult pathResult = activePathFinder().findPath(startGrid, goalGrid, obstacles,
                                                  layout.from, layout.to,
                                                  layout.sourceEdge, layout.targetEdge,
                                                  extraStartExcludes, extraGoalExcludes);

    if (pathResult.found && pathResult.path.size() >= 2) {
        // Convert grid path to pixel bend points
        // Skip first (source) and last (target) points - they're the endpoints
        layout.bendPoints.clear();
        for (size_t i = 1; i + 1 < pathResult.path.size(); ++i) {
            Point pixelPoint = obstacles.gridToPixel(pathResult.path[i].x, pathResult.path[i].y);
            layout.bendPoints.push_back({pixelPoint});
        }
    } else {
        // PathFinder couldn't find a valid path.
        // Create fallback path that avoids intermediate nodes.
        layout.bendPoints.clear();

        bool sourceVertical = (layout.sourceEdge == NodeEdge::Top || layout.sourceEdge == NodeEdge::Bottom);
        bool targetVertical = (layout.targetEdge == NodeEdge::Top || layout.targetEdge == NodeEdge::Bottom);

        bool sourceGoesDown = (layout.sourceEdge == NodeEdge::Bottom);
        bool targetEntersDown = (layout.targetEdge == NodeEdge::Top);

        bool isHorizontal = std::abs(layout.sourcePoint.y - layout.targetPoint.y) < 0.1f;
        bool isVertical = std::abs(layout.sourcePoint.x - layout.targetPoint.x) < 0.1f;

        // Find intermediate nodes that might block the path
        float minBlockingX = std::numeric_limits<float>::max();
        float maxBlockingX = std::numeric_limits<float>::lowest();
        bool hasBlockingNodes = false;

        float pathMinY = std::min(layout.sourcePoint.y, layout.targetPoint.y);
        float pathMaxY = std::max(layout.sourcePoint.y, layout.targetPoint.y);
        float pathMinX = std::min(layout.sourcePoint.x, layout.targetPoint.x);
        float pathMaxX = std::max(layout.sourcePoint.x, layout.targetPoint.x);

        for (const auto& [nodeId, node] : nodeLayouts) {
            if (nodeId == layout.from || nodeId == layout.to) continue;

            // Check if node is in the vertical path range
            float nodeTop = node.position.y;
            float nodeBottom = node.position.y + node.size.height;
            float nodeLeft = node.position.x;
            float nodeRight = node.position.x + node.size.width;

            // Node blocks path if it's between source and target Y-wise
            // and overlaps with the X range of source or target
            if (nodeBottom > pathMinY && nodeTop < pathMaxY) {
                if (nodeRight > pathMinX - effectiveGridSize && nodeLeft < pathMaxX + effectiveGridSize) {
                    hasBlockingNodes = true;
                    minBlockingX = std::min(minBlockingX, nodeLeft);
                    maxBlockingX = std::max(maxBlockingX, nodeRight);
                }
            }
        }

        if (isHorizontal || isVertical) {
            // Direct line - no bends needed if collinear
        } else if (sourceVertical && targetVertical && hasBlockingNodes) {
            // Route around blocking nodes
            float bypassX;
            float srcX = layout.sourcePoint.x;
            float tgtX = layout.targetPoint.x;
            float avgX = (srcX + tgtX) / 2.0f;
            float centerBlocking = (minBlockingX + maxBlockingX) / 2.0f;

            // Choose which side to bypass
            if (avgX < centerBlocking) {
                bypassX = minBlockingX - effectiveGridSize;
            } else {
                bypassX = maxBlockingX + effectiveGridSize;
            }

            if (effectiveGridSize > 0) {
                bypassX = std::round(bypassX / effectiveGridSize) * effectiveGridSize;
            }

            // Create bypass path with 4 bends
            float exitY = layout.sourcePoint.y + (sourceGoesDown ? effectiveGridSize : -effectiveGridSize);
            float entryY = layout.targetPoint.y + (targetEntersDown ? -effectiveGridSize : effectiveGridSize);

            if (effectiveGridSize > 0) {
                exitY = std::round(exitY / effectiveGridSize) * effectiveGridSize;
                entryY = std::round(entryY / effectiveGridSize) * effectiveGridSize;
            }

            layout.bendPoints.push_back({{layout.sourcePoint.x, exitY}});
            layout.bendPoints.push_back({{bypassX, exitY}});
            layout.bendPoints.push_back({{bypassX, entryY}});
            layout.bendPoints.push_back({{layout.targetPoint.x, entryY}});

        } else if (sourceVertical && targetVertical) {
            // No blocking nodes - simple Z-path
            float midY;
            if (sourceGoesDown && targetEntersDown) {
                midY = layout.targetPoint.y - effectiveGridSize;
            } else {
                midY = (layout.sourcePoint.y + layout.targetPoint.y) / 2.0f;
            }

            if (effectiveGridSize > 0) {
                midY = std::round(midY / effectiveGridSize) * effectiveGridSize;
            }

            layout.bendPoints.push_back({{layout.sourcePoint.x, midY}});
            layout.bendPoints.push_back({{layout.targetPoint.x, midY}});

        } else if (!sourceVertical && !targetVertical) {
            // Both horizontal edges
            float midX = (layout.sourcePoint.x + layout.targetPoint.x) / 2.0f;

            if (effectiveGridSize > 0) {
                midX = std::round(midX / effectiveGridSize) * effectiveGridSize;
            }

            layout.bendPoints.push_back({{midX, layout.sourcePoint.y}});
            layout.bendPoints.push_back({{midX, layout.targetPoint.y}});

        } else {
            // Different orientations - simple L-path
            Point corner;
            if (sourceVertical) {
                corner = {layout.sourcePoint.x, layout.targetPoint.y};
            } else {
                corner = {layout.targetPoint.x, layout.sourcePoint.y};
            }
            layout.bendPoints.push_back({corner});
        }
    }

    // Cleanup: remove spikes and duplicates first
    {
        std::vector<Point> allPoints;
        allPoints.push_back(layout.sourcePoint);
        for (const auto& bend : layout.bendPoints) {
            allPoints.push_back(bend.position);
        }
        allPoints.push_back(layout.targetPoint);

        // Clean up any spikes or duplicates
        PathCleanup::removeSpikesAndDuplicates(allPoints);

        // Reconstruct bendPoints from allPoints
        layout.bendPoints.clear();
        for (size_t i = 1; i + 1 < allPoints.size(); ++i) {
            layout.bendPoints.push_back({allPoints[i]});
        }
    }

    // CRITICAL: Check if PathCleanup violated direction constraints
    // - First segment must match source edge direction (vertical for Top/Bottom, horizontal for Left/Right)
    // - Last segment must match target edge direction
    // If violated, clear bendPoints so the empty-check logic below recreates proper shape
    if (!layout.bendPoints.empty()) {
        bool sourceVertical = (layout.sourceEdge == NodeEdge::Top || layout.sourceEdge == NodeEdge::Bottom);
        bool targetVertical = (layout.targetEdge == NodeEdge::Top || layout.targetEdge == NodeEdge::Bottom);

        const Point& firstBend = layout.bendPoints.front().position;
        const Point& lastBend = layout.bendPoints.back().position;

        constexpr float DIRECTION_TOLERANCE = 0.5f;
        bool firstSegmentVertical = std::abs(layout.sourcePoint.x - firstBend.x) < DIRECTION_TOLERANCE;
        bool lastSegmentVertical = std::abs(lastBend.x - layout.targetPoint.x) < DIRECTION_TOLERANCE;

        // Check if direction constraints are violated
        bool sourceViolated = (sourceVertical != firstSegmentVertical);
        bool targetViolated = (targetVertical != lastSegmentVertical);

        if (sourceViolated || targetViolated) {
#if EDGE_ROUTING_DEBUG
            std::cout << "[EdgeRouting] recalculateBendPoints: Direction constraint violated!"
                      << " sourceViolated=" << sourceViolated
                      << " targetViolated=" << targetViolated
                      << " - clearing bendPoints for recreation" << std::endl;
#endif
            layout.bendPoints.clear();
        }
    }

    // Post-processing: ensure first bend has proper clearance from source
    // This handles cases where PathFinder or fallback paths don't maintain direction clearance
    // When clearance is needed, INSERT a clearance point rather than modifying existing bends
    // to maintain orthogonality of the entire path
    if (!layout.bendPoints.empty()) {
        Point firstBend = layout.bendPoints[0].position;
        float minClearance = std::max(effectiveGridSize, constants::PATHFINDING_GRID_SIZE);

        switch (layout.sourceEdge) {
            case NodeEdge::Top:
                // First bend must be ABOVE source (smaller Y)
                if (firstBend.y > layout.sourcePoint.y - minClearance) {
                    float newY = layout.sourcePoint.y - minClearance;
                    // Insert clearance point: go up from source, then horizontal to first bend
                    Point clearancePoint = {layout.sourcePoint.x, newY};
                    Point connectionPoint = {firstBend.x, newY};
                    layout.bendPoints.insert(layout.bendPoints.begin(), {connectionPoint});
                    layout.bendPoints.insert(layout.bendPoints.begin(), {clearancePoint});
                }
                break;
            case NodeEdge::Bottom:
                // First bend must be BELOW source (larger Y)
                if (firstBend.y < layout.sourcePoint.y + minClearance) {
                    float newY = layout.sourcePoint.y + minClearance;
                    // Insert clearance point: go down from source, then horizontal to first bend
                    Point clearancePoint = {layout.sourcePoint.x, newY};
                    Point connectionPoint = {firstBend.x, newY};
                    layout.bendPoints.insert(layout.bendPoints.begin(), {connectionPoint});
                    layout.bendPoints.insert(layout.bendPoints.begin(), {clearancePoint});
                }
                break;
            case NodeEdge::Left:
                // First bend must be LEFT of source (smaller X)
                if (firstBend.x > layout.sourcePoint.x - minClearance) {
                    float newX = layout.sourcePoint.x - minClearance;
                    // Insert clearance point: go left from source, then vertical to first bend Y
                    Point clearancePoint = {newX, layout.sourcePoint.y};
                    Point connectionPoint = {newX, firstBend.y};
                    layout.bendPoints.insert(layout.bendPoints.begin(), {connectionPoint});
                    layout.bendPoints.insert(layout.bendPoints.begin(), {clearancePoint});
                }
                break;
            case NodeEdge::Right:
                // First bend must be RIGHT of source (larger X)
                if (firstBend.x < layout.sourcePoint.x + minClearance) {
                    float newX = layout.sourcePoint.x + minClearance;
                    // Insert clearance point: go right from source, then vertical to first bend Y
                    Point clearancePoint = {newX, layout.sourcePoint.y};
                    Point connectionPoint = {newX, firstBend.y};
                    layout.bendPoints.insert(layout.bendPoints.begin(), {connectionPoint});
                    layout.bendPoints.insert(layout.bendPoints.begin(), {clearancePoint});
                }
                break;
        }

        // Clean up any duplicate points that may have been introduced
        std::vector<Point> allPoints;
        allPoints.push_back(layout.sourcePoint);
        for (const auto& bend : layout.bendPoints) {
            allPoints.push_back(bend.position);
        }
        allPoints.push_back(layout.targetPoint);

        PathCleanup::removeSpikesAndDuplicates(allPoints);

        layout.bendPoints.clear();
        for (size_t i = 1; i + 1 < allPoints.size(); ++i) {
            layout.bendPoints.push_back({allPoints[i]});
        }
    }

    // CRITICAL: Ensure orthogonality when bendPoints is empty but source/target misaligned
    // This can happen when A* returns a 2-point path or PathCleanup removes all intermediate points
    if (layout.bendPoints.empty()) {
        const float EPSILON = 0.1f;
        float dx = std::abs(layout.targetPoint.x - layout.sourcePoint.x);
        float dy = std::abs(layout.targetPoint.y - layout.sourcePoint.y);

        // If both dx and dy are significant, we need bend points for orthogonal routing
        if (dx > EPSILON && dy > EPSILON) {
            // Check both source and target edge directions
            bool sourceVertical = (layout.sourceEdge == NodeEdge::Top || layout.sourceEdge == NodeEdge::Bottom);
            bool targetVertical = (layout.targetEdge == NodeEdge::Top || layout.targetEdge == NodeEdge::Bottom);

            if (sourceVertical == targetVertical) {
                // Same orientation (both horizontal or both vertical)
                // Need 2 bend points to form a Z-shape or S-shape
                float midX = (layout.sourcePoint.x + layout.targetPoint.x) / 2.0f;
                float midY = (layout.sourcePoint.y + layout.targetPoint.y) / 2.0f;

                if (sourceVertical) {
                    // Both vertical: vertical → horizontal → vertical
                    layout.bendPoints.push_back({{layout.sourcePoint.x, midY}});
                    layout.bendPoints.push_back({{layout.targetPoint.x, midY}});
                } else {
                    // Both horizontal: horizontal → vertical → horizontal
                    layout.bendPoints.push_back({{midX, layout.sourcePoint.y}});
                    layout.bendPoints.push_back({{midX, layout.targetPoint.y}});
                }
#if EDGE_ROUTING_DEBUG
                std::cout << "[EdgeRouting] recalculateBendPoints: SAME ORIENTATION FIX (2 bends)!" << std::endl;
                std::cout << "  src=(" << layout.sourcePoint.x << "," << layout.sourcePoint.y << ") "
                          << "tgt=(" << layout.targetPoint.x << "," << layout.targetPoint.y << ")" << std::endl;
                std::cout << "  sourceVertical=" << sourceVertical << " targetVertical=" << targetVertical << std::endl;
#endif
            } else {
                // Different orientation: Create L-shaped path (1 bend point)
                Point bendPoint;
                if (sourceVertical) {
                    // Vertical first, then horizontal: bend at (source.x, target.y)
                    bendPoint = {layout.sourcePoint.x, layout.targetPoint.y};
                } else {
                    // Horizontal first, then vertical: bend at (target.x, source.y)
                    bendPoint = {layout.targetPoint.x, layout.sourcePoint.y};
                }
                layout.bendPoints.push_back({bendPoint});
#if EDGE_ROUTING_DEBUG
                std::cout << "[EdgeRouting] recalculateBendPoints: ORTHOGONALITY FIX (1 bend)!" << std::endl;
                std::cout << "  src=(" << layout.sourcePoint.x << "," << layout.sourcePoint.y << ") "
                          << "tgt=(" << layout.targetPoint.x << "," << layout.targetPoint.y << ")" << std::endl;
                std::cout << "  dx=" << dx << " dy=" << dy << " sourceVertical=" << sourceVertical << std::endl;
                std::cout << "  Added bend: (" << bendPoint.x << "," << bendPoint.y << ")" << std::endl;
#endif
            }
        }
#if EDGE_ROUTING_DEBUG
        else {
            std::cout << "[EdgeRouting] recalculateBendPoints: bendPoints empty, no fix needed "
                      << "(dx=" << dx << " dy=" << dy << ")" << std::endl;
        }
#endif
    }
}

void EdgeRouting::recalculateBendPointsWithOverlapAvoidance(
    EdgeLayout& layout,
    const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
    const std::unordered_map<EdgeId, EdgeLayout>& otherEdges,
    float gridSize) {

    // First, calculate the basic path
    recalculateBendPoints(layout, nodeLayouts, gridSize);

    // If no other edges, nothing to check
    if (otherEdges.empty()) {
        return;
    }

    // Need at least 2 bend points to have a middle segment we can adjust
    if (layout.bendPoints.size() < 2) {
        return;
    }

    float effectiveGridSize = gridSize > 0.0f ? gridSize : constants::PATHFINDING_GRID_SIZE;

    // Check ONLY middle segments (between bend points)
    // We can only shift if the adjacent segments are perpendicular to the shift direction
    bool adjusted = false;

    for (size_t bendIdx = 0; bendIdx + 1 < layout.bendPoints.size(); ++bendIdx) {
        Point& p1 = layout.bendPoints[bendIdx].position;
        Point& p2 = layout.bendPoints[bendIdx + 1].position;

        bool isVertical = std::abs(p1.x - p2.x) < 0.5f;
        bool isHorizontal = std::abs(p1.y - p2.y) < 0.5f;

        if (!isVertical && !isHorizontal) {
            continue;  // Non-orthogonal segment, skip
        }

        // Get adjacent points (prev connects to p1, next connects to p2)
        Point prev = (bendIdx == 0) ? layout.sourcePoint : layout.bendPoints[bendIdx - 1].position;
        Point next = (bendIdx + 2 >= layout.bendPoints.size()) ? layout.targetPoint
                                                                : layout.bendPoints[bendIdx + 2].position;

        // Check if we can safely shift this segment
        // For vertical segment (shift in X): adjacent segments must be horizontal (same Y)
        // For horizontal segment (shift in Y): adjacent segments must be vertical (same X)
        bool canShift = false;
        if (isVertical) {
            // To shift X, prev->p1 must be horizontal (same Y) and p2->next must be horizontal (same Y)
            bool prevHorizontal = std::abs(prev.y - p1.y) < 0.5f;
            bool nextHorizontal = std::abs(p2.y - next.y) < 0.5f;
            canShift = prevHorizontal && nextHorizontal;
        } else {
            // To shift Y, prev->p1 must be vertical (same X) and p2->next must be vertical (same X)
            bool prevVertical = std::abs(prev.x - p1.x) < 0.5f;
            bool nextVertical = std::abs(p2.x - next.x) < 0.5f;
            canShift = prevVertical && nextVertical;
        }

        if (!canShift) {
            continue;  // Can't shift without breaking orthogonality
        }

        float segMin, segMax;
        float segCoord;  // The fixed coordinate (x for vertical, y for horizontal)

        if (isVertical) {
            segCoord = p1.x;
            segMin = std::min(p1.y, p2.y);
            segMax = std::max(p1.y, p2.y);
        } else {
            segCoord = p1.y;
            segMin = std::min(p1.x, p2.x);
            segMax = std::max(p1.x, p2.x);
        }

        // Check for overlaps with other edges' segments
        bool hasOverlap = false;
        for (const auto& [otherId, otherLayout] : otherEdges) {
            if (otherId == layout.id) continue;

            // Build other edge's path
            std::vector<Point> otherPath;
            otherPath.push_back(otherLayout.sourcePoint);
            for (const auto& bp : otherLayout.bendPoints) {
                otherPath.push_back(bp.position);
            }
            otherPath.push_back(otherLayout.targetPoint);

            // Check each segment of the other edge
            for (size_t j = 0; j + 1 < otherPath.size(); ++j) {
                const Point& op1 = otherPath[j];
                const Point& op2 = otherPath[j + 1];

                bool otherVertical = std::abs(op1.x - op2.x) < 0.5f;
                bool otherHorizontal = std::abs(op1.y - op2.y) < 0.5f;

                // Only check same-orientation segments for collinear overlap
                if (isVertical && otherVertical) {
                    if (std::abs(segCoord - op1.x) < effectiveGridSize * 0.5f) {
                        float otherMin = std::min(op1.y, op2.y);
                        float otherMax = std::max(op1.y, op2.y);
                        if (segMax > otherMin + 1.0f && segMin < otherMax - 1.0f) {
                            hasOverlap = true;
                            break;
                        }
                    }
                } else if (isHorizontal && otherHorizontal) {
                    if (std::abs(segCoord - op1.y) < effectiveGridSize * 0.5f) {
                        float otherMin = std::min(op1.x, op2.x);
                        float otherMax = std::max(op1.x, op2.x);
                        if (segMax > otherMin + 1.0f && segMin < otherMax - 1.0f) {
                            hasOverlap = true;
                            break;
                        }
                    }
                }
            }
            if (hasOverlap) break;
        }

        // If overlap detected and we can shift safely, do it
        if (hasOverlap) {
            float shift = effectiveGridSize;

            if (isVertical) {
                // Shift in X direction
                float midX = (layout.sourcePoint.x + layout.targetPoint.x) / 2.0f;
                if (segCoord < midX) {
                    shift = -effectiveGridSize;
                }
                p1.x += shift;
                p2.x += shift;
            } else {
                // Shift in Y direction
                float midY = (layout.sourcePoint.y + layout.targetPoint.y) / 2.0f;
                if (segCoord < midY) {
                    shift = -effectiveGridSize;
                }
                p1.y += shift;
                p2.y += shift;
            }
            adjusted = true;
        }
    }

#if EDGE_ROUTING_DEBUG
    if (adjusted) {
        std::cout << "[EdgeRouting] recalculateBendPointsWithOverlapAvoidance: "
                  << "Adjusted edge " << layout.id << " to avoid overlap" << std::endl;
    }
#endif
}

std::pair<int, int> EdgeRouting::countConnectionsOnNodeEdge(
    const std::unordered_map<EdgeId, EdgeLayout>& edgeLayouts,
    NodeId nodeId,
    NodeEdge nodeEdge) {

    // Delegate to SnapIndexManager for centralized logic
    auto connections = SnapIndexManager::getConnections(edgeLayouts, nodeId, nodeEdge);
    return {connections.incomingCount(), connections.outgoingCount()};
}

// =============================================================================
// Edge Routing Core Methods
// =============================================================================

// =============================================================================
// Segment-Node Intersection Detection and Avoidance
// =============================================================================

bool EdgeRouting::segmentIntersectsNode(
    const Point& p1,
    const Point& p2,
    const NodeLayout& node,
    float margin) {
    // Delegate to EdgeValidator
    return EdgeValidator::segmentIntersectsNode(p1, p2, node, margin);
}

EdgeRouting::Result EdgeRouting::route(
    const Graph& graph,
    const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
    const std::unordered_set<EdgeId>& reversedEdges,
    const LayoutOptions& options) {

    Result result;

    // Allocate channels for all edges
    auto channelAssignments = allocateChannels(graph, nodeLayouts, reversedEdges, options);
    result.channelAssignments = channelAssignments;

    // Track self-loop counts per node
    std::map<NodeId, int> selfLoopIndices;

    for (EdgeId edgeId : graph.edges()) {
        const EdgeData edge = graph.getEdge(edgeId);

        auto fromIt = nodeLayouts.find(edge.from);
        auto toIt = nodeLayouts.find(edge.to);

        if (fromIt == nodeLayouts.end() || toIt == nodeLayouts.end()) {
            continue;
        }

        bool isReversed = reversedEdges.count(edgeId) > 0;

        EdgeLayout layout;

        // Self-loop handling
        if (edge.from == edge.to) {
            int loopIndex = selfLoopIndices[edge.from]++;
            layout = routeSelfLoop(edge, fromIt->second, loopIndex, options);
        } else {
            // Regular edge with channel assignment
            auto channelIt = channelAssignments.find(edgeId);
            if (channelIt != channelAssignments.end()) {
                layout = routeChannelOrthogonal(edge, fromIt->second, toIt->second,
                                               isReversed, channelIt->second, options, &nodeLayouts);
            } else {
                // Fallback: create simple orthogonal routing without channel
                layout = routeChannelOrthogonal(edge, fromIt->second, toIt->second,
                                               isReversed, ChannelAssignment{}, options, &nodeLayouts);
            }
        }

        result.edgeLayouts[edgeId] = layout;
    }

    // Apply edge optimization if optimizer is injected or postDragAlgorithm is enabled
    IEdgeOptimizer* optimizer = edgeOptimizer_.get();
    std::unique_ptr<IEdgeOptimizer> fallbackOptimizer;

    // Use injected optimizer, or create based on postDragAlgorithm
    if (!optimizer && options.optimizationOptions.postDragAlgorithm != PostDragAlgorithm::None) {
        const float gridSize = options.gridConfig.cellSize;

        switch (options.optimizationOptions.postDragAlgorithm) {
            case PostDragAlgorithm::AStar:
                fallbackOptimizer = std::make_unique<AStarEdgeOptimizer>(
                    pathFinder_,  // Share pathfinder with optimizer
                    options.optimizationOptions.scoringWeights,
                    gridSize);
                break;
            case PostDragAlgorithm::None:
                break;
        }

        // Set up constraint manager with default constraints
        // All constraints apply automatically to all optimizers through this manager
        if (fallbackOptimizer) {
            auto constraintManager = std::make_shared<EdgeConstraintManager>(
                EdgeConstraintManager::createDefault());
            fallbackOptimizer->setConstraintManager(constraintManager);
        }

        optimizer = fallbackOptimizer.get();
    }

    if (optimizer && !result.edgeLayouts.empty()) {
        // Collect all edge IDs (skip self-loops which have fixed routing)
        std::vector<EdgeId> edgeIds;
        edgeIds.reserve(result.edgeLayouts.size());
        for (const auto& [edgeId, layout] : result.edgeLayouts) {
            if (layout.from != layout.to) {
                edgeIds.push_back(edgeId);
            }
        }

        // Optimize edge layouts (optimizer now calculates actual paths internally)
        auto optimizedLayouts = optimizer->optimize(edgeIds, result.edgeLayouts, nodeLayouts);

        // Merge optimized layouts (bend points already calculated by optimizer)
        for (auto& [edgeId, layout] : optimizedLayouts) {
            result.edgeLayouts[edgeId] = std::move(layout);
        }
    }

    return result;
}

// =============================================================================
// Snap Point Distribution Methods
// =============================================================================

void EdgeRouting::distributeAutoSnapPoints(
    Result& result,
    const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
    float gridSize,
    bool sortSnapPoints) {

    float effectiveGridSize = getEffectiveGridSize(gridSize);

    // Unified mode: all connections on same edge distributed together
    // Key: (nodeId, nodeEdge) -> list of (edgeId, isSource)
    std::map<std::pair<NodeId, NodeEdge>, std::vector<std::pair<EdgeId, bool>>> allConnections;

    for (auto& [edgeId, layout] : result.edgeLayouts) {
        allConnections[{layout.from, layout.sourceEdge}].push_back({edgeId, true});
        allConnections[{layout.to, layout.targetEdge}].push_back({edgeId, false});
    }

    for (auto& [key, connections] : allConnections) {
        auto [nodeId, nodeEdge] = key;

        // Sort snap points by other node position to minimize edge crossings
        if (sortSnapPoints && connections.size() > 1) {
            auto sortedPairs = SnapIndexManager::sortSnapPointsByOtherNode(
                nodeId, nodeEdge, result.edgeLayouts, nodeLayouts);

            // Replace original connections if sorting succeeded
            // sortedPairs contains (EdgeId, isSource) pairs in correct order
            if (sortedPairs.size() == connections.size()) {
                connections = std::move(sortedPairs);
            }
        }

        // Move self-loop endpoints to corner positions
        SelfLoopRouter::applySelfLoopCornerPositioning(connections, result.edgeLayouts, nodeEdge);

        auto nodeIt = nodeLayouts.find(nodeId);
        if (nodeIt == nodeLayouts.end()) continue;

        const NodeLayout& node = nodeIt->second;
        int count = static_cast<int>(connections.size());

        for (int i = 0; i < count; ++i) {
            auto [edgeId, isSource] = connections[i];
            EdgeLayout& layout = result.edgeLayouts[edgeId];

            // Quantized calculation: preserves symmetry by using integer arithmetic
            Point snapPoint = calculateSnapPositionQuantized(node, nodeEdge, i, count, 0.0f, 1.0f, effectiveGridSize);

            if (isSource) {
                layout.sourcePoint = snapPoint;
                layout.sourceSnapIndex = i;
            } else {
                layout.targetPoint = snapPoint;
                layout.targetSnapIndex = i;
            }
        }
    }

    // Recalculate bend points (snap points are already on grid from quantized calculation)
    for (auto& [edgeId, layout] : result.edgeLayouts) {
        recalculateBendPoints(layout, nodeLayouts, effectiveGridSize);

        // Grid mode: bend points are already orthogonal from quantized routing.
        // Remove spikes/duplicates for path cleanup.
        std::vector<Point> fullPath;
        fullPath.push_back(layout.sourcePoint);
        for (const auto& bp : layout.bendPoints) {
            fullPath.push_back(bp.position);
        }
        fullPath.push_back(layout.targetPoint);

        PathCleanup::removeSpikesAndDuplicates(fullPath);

        layout.bendPoints.clear();
        for (size_t i = 1; i + 1 < fullPath.size(); ++i) {
            layout.bendPoints.push_back({fullPath[i]});
        }

        // Ensure first bend has proper clearance from source
        if (!layout.bendPoints.empty()) {
            Point& firstBend = layout.bendPoints[0].position;
            float minClearance = std::max(effectiveGridSize, constants::PATHFINDING_GRID_SIZE);

            switch (layout.sourceEdge) {
                case NodeEdge::Top:
                    if (firstBend.y > layout.sourcePoint.y - minClearance) {
                        float newY = layout.sourcePoint.y - minClearance;
                        firstBend.y = newY;
                        if (layout.bendPoints.size() >= 2) {
                            layout.bendPoints[1].position.y = newY;
                        }
                    }
                    break;
                case NodeEdge::Bottom:
                    if (firstBend.y < layout.sourcePoint.y + minClearance) {
                        float newY = layout.sourcePoint.y + minClearance;
                        firstBend.y = newY;
                        if (layout.bendPoints.size() >= 2) {
                            layout.bendPoints[1].position.y = newY;
                        }
                    }
                    break;
                case NodeEdge::Left:
                    if (firstBend.x > layout.sourcePoint.x - minClearance) {
                        float newX = layout.sourcePoint.x - minClearance;
                        firstBend.x = newX;
                        if (layout.bendPoints.size() >= 2) {
                            layout.bendPoints[1].position.x = newX;
                        }
                    }
                    break;
                case NodeEdge::Right:
                    if (firstBend.x < layout.sourcePoint.x + minClearance) {
                        float newX = layout.sourcePoint.x + minClearance;
                        firstBend.x = newX;
                        if (layout.bendPoints.size() >= 2) {
                            layout.bendPoints[1].position.x = newX;
                        }
                    }
                    break;
            }
        }

        layout.labelPosition = LayoutUtils::calculateEdgeLabelPosition(layout);
    }
}

EdgeRouting::SnapUpdateResult EdgeRouting::updateSnapPositions(
    std::unordered_map<EdgeId, EdgeLayout>& edgeLayouts,
    const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
    const std::vector<EdgeId>& affectedEdges,
    const std::unordered_set<NodeId>& movedNodes,
    float gridSize) {

    SnapUpdateResult result;
    float effectiveGridSize = getEffectiveGridSize(gridSize);

    // Helper to check if a node should be updated
    auto shouldUpdateNode = [&movedNodes](NodeId nodeId) -> bool {
        return movedNodes.empty() || movedNodes.count(nodeId) > 0;
    };

    // === Phase 1: Collect affected connections by node-edge ===
    std::map<std::pair<NodeId, NodeEdge>, std::vector<std::pair<EdgeId, bool>>> affectedConnections;

    for (EdgeId edgeId : affectedEdges) {
        auto it = edgeLayouts.find(edgeId);
        if (it == edgeLayouts.end()) continue;
        const EdgeLayout& layout = it->second;
        affectedConnections[{layout.from, layout.sourceEdge}].push_back({edgeId, true});
        affectedConnections[{layout.to, layout.targetEdge}].push_back({edgeId, false});
    }

    // === Phase 2: Calculate snap positions and track redistributed edges ===
    for (auto& [key, connections] : affectedConnections) {
        auto [nodeId, nodeEdge] = key;

        // Check if any edge needs snap index redistribution:
        // 1. Has -1 (SNAP_INDEX_UNASSIGNED)
        // 2. Has duplicate snap index (two edges with same index on same node edge)
        bool needsRedistribution = false;
        std::set<int> usedIndices;
        for (const auto& [edgeId, isSource] : connections) {
            auto it = edgeLayouts.find(edgeId);
            if (it != edgeLayouts.end()) {
                int snapIdx = isSource ? it->second.sourceSnapIndex : it->second.targetSnapIndex;
                if (snapIdx < 0) {
                    needsRedistribution = true;
                    break;
                }
                // Check for duplicate
                if (usedIndices.count(snapIdx) > 0) {
                    needsRedistribution = true;
                    break;
                }
                usedIndices.insert(snapIdx);
            }
        }

        // Skip nodes that haven't moved AND don't need redistribution
        if (!shouldUpdateNode(nodeId) && !needsRedistribution) continue;

        auto nodeIt = nodeLayouts.find(nodeId);
        if (nodeIt == nodeLayouts.end()) continue;

        const NodeLayout& node = nodeIt->second;

        // Get ALL connections on this node-edge for correct totalCount calculation
        // CRITICAL: totalCount must always be the total number of connections,
        // not just the affected ones, for correct snap position calculation.
        auto allConnections = SnapIndexManager::getConnections(edgeLayouts, nodeId, nodeEdge);
        int totalCount = static_cast<int>(allConnections.incoming.size() + allConnections.outgoing.size());

        // Determine which edges to process
        std::vector<std::pair<EdgeId, bool>> edgesToProcess;
        if (needsRedistribution) {
            // Process ALL edges on this node-edge when redistribution needed
            for (EdgeId edgeId : allConnections.incoming) {
                edgesToProcess.push_back(std::make_pair(edgeId, false));  // incoming = target side
            }
            for (EdgeId edgeId : allConnections.outgoing) {
                edgesToProcess.push_back(std::make_pair(edgeId, true));   // outgoing = source side
            }
        } else {
            // Only process affected edges, but use totalCount from all connections
            edgesToProcess = connections;
        }

        // Move self-loop endpoints to corner positions
        SelfLoopRouter::applySelfLoopCornerPositioning(edgesToProcess, edgeLayouts, nodeEdge);

        for (size_t connIdx = 0; connIdx < edgesToProcess.size(); ++connIdx) {
            auto [edgeId, isSource] = edgesToProcess[connIdx];
            EdgeLayout& layout = edgeLayouts[edgeId];

            int snapIdx;
            if (needsRedistribution) {
                // When redistribution is needed (duplicates or -1 values),
                // assign new indices based on position in edgesToProcess
                snapIdx = static_cast<int>(connIdx);
            } else {
                // Use existing snap index if valid
                snapIdx = isSource ? layout.sourceSnapIndex : layout.targetSnapIndex;
                if (snapIdx < 0 || snapIdx >= totalCount) {
                    snapIdx = static_cast<int>(connIdx);
                }
            }

            // Quantized calculation: preserves symmetry
            Point snapPoint = calculateSnapPositionQuantized(node, nodeEdge, snapIdx, totalCount, 0.0f, 1.0f, effectiveGridSize);

            if (isSource) {
                layout.sourcePoint = snapPoint;
                layout.sourceSnapIndex = snapIdx;
            } else {
                layout.targetPoint = snapPoint;
                layout.targetSnapIndex = snapIdx;
            }

            // Track edges that were updated during redistribution
            // These need recalculateBendPoints even if not in affectedEdges
            if (needsRedistribution) {
                result.redistributedEdges.insert(edgeId);
            }
        }
    }

    // === Phase 3: Merge affected edges and redistributed edges for processing ===
    // When snap redistribution occurs, edges not in affectedEdges may have
    // their snap positions changed. These edges also need recalculateBendPoints.
    result.processedEdges.insert(affectedEdges.begin(), affectedEdges.end());
    result.processedEdges.insert(result.redistributedEdges.begin(), result.redistributedEdges.end());

#if EDGE_ROUTING_DEBUG
    if (!result.redistributedEdges.empty()) {
        std::cout << "[EdgeRouting] updateSnapPositions: redistributedEdges=" << result.redistributedEdges.size()
                  << ", processedEdges=" << result.processedEdges.size() << std::endl;
        if (result.hasIndirectUpdates(affectedEdges)) {
            std::cout << "  WARNING: Edges outside affectedEdges were updated due to redistribution" << std::endl;
        }
    }
#endif

    // Check which edges are bidirectional (have a reverse edge)
    // O(N) algorithm using unordered_map for O(1) lookup
    std::unordered_set<EdgeId> bidirectionalEdges;

    // Build edge lookup map using helper function
    auto edgeMap = buildEdgeMapFromLayouts<PairHash>(edgeLayouts);

    for (EdgeId edgeId : result.processedEdges) {
        auto it = edgeLayouts.find(edgeId);
        if (it == edgeLayouts.end()) continue;

        NodeId from = it->second.from;
        NodeId to = it->second.to;

        // Look for reverse edge in map - O(1) average case
        auto reverseIt = edgeMap.find({to, from});
        if (reverseIt != edgeMap.end()) {
            bidirectionalEdges.insert(edgeId);
        }
    }

    // === Phase 4: Recalculate bend points for all processed edges ===
    // Build "other edges" map for overlap detection
    std::unordered_map<EdgeId, EdgeLayout> otherEdges;
    for (const auto& [edgeId, layout] : edgeLayouts) {
        if (std::find(result.processedEdges.begin(), result.processedEdges.end(), edgeId)
            == result.processedEdges.end()) {
            otherEdges[edgeId] = layout;
        }
    }

    for (EdgeId edgeId : result.processedEdges) {
        auto it = edgeLayouts.find(edgeId);
        if (it == edgeLayouts.end()) continue;

        // Recalculate bend points with overlap avoidance
        recalculateBendPointsWithOverlapAvoidance(it->second, nodeLayouts, otherEdges, effectiveGridSize);

        // Add this edge to otherEdges so subsequent edges avoid overlapping with it
        otherEdges[edgeId] = it->second;

#if EDGE_ROUTING_DEBUG
        size_t bendsBeforeCleanup = it->second.bendPoints.size();
#endif

        // Grid mode: bend points are already orthogonal from quantized routing.
        // Remove spikes/duplicates for path cleanup.
        std::vector<Point> fullPath;
        fullPath.push_back(it->second.sourcePoint);
        for (const auto& bp : it->second.bendPoints) {
            fullPath.push_back(bp.position);
        }
        fullPath.push_back(it->second.targetPoint);

        PathCleanup::removeSpikesAndDuplicates(fullPath);

        it->second.bendPoints.clear();
        for (size_t i = 1; i + 1 < fullPath.size(); ++i) {
            it->second.bendPoints.push_back({fullPath[i]});
        }

        // CRITICAL: Validate direction constraints after PathCleanup
        // PathCleanup may remove bend points needed for direction constraints
        // If violated, recalculate with proper Z-shape
        if (!it->second.bendPoints.empty()) {
            bool sourceVertical = (it->second.sourceEdge == NodeEdge::Top || 
                                   it->second.sourceEdge == NodeEdge::Bottom);
            bool targetVertical = (it->second.targetEdge == NodeEdge::Top || 
                                   it->second.targetEdge == NodeEdge::Bottom);

            const Point& firstBend = it->second.bendPoints.front().position;
            const Point& lastBend = it->second.bendPoints.back().position;

            constexpr float DIRECTION_TOLERANCE = 0.5f;
            bool firstSegmentVertical = std::abs(it->second.sourcePoint.x - firstBend.x) < DIRECTION_TOLERANCE;
            bool lastSegmentVertical = std::abs(lastBend.x - it->second.targetPoint.x) < DIRECTION_TOLERANCE;

            if (sourceVertical != firstSegmentVertical || targetVertical != lastSegmentVertical) {
                // Direction constraint violated - recreate proper path
                it->second.bendPoints.clear();
                recalculateBendPoints(it->second, nodeLayouts, effectiveGridSize);
            }
        }

#if EDGE_ROUTING_DEBUG
        if (bendsBeforeCleanup != it->second.bendPoints.size()) {
            std::cout << "[EdgeRouting] updateSnapPositions: PathCleanup changed bendPoints for edge " << edgeId
                      << " from " << bendsBeforeCleanup << " to " << it->second.bendPoints.size() << std::endl;
        }
        // Check for diagonal after cleanup
        float dx = std::abs(it->second.targetPoint.x - it->second.sourcePoint.x);
        float dy = std::abs(it->second.targetPoint.y - it->second.sourcePoint.y);
        if (dx > 0.1f && dy > 0.1f && it->second.bendPoints.empty()) {
            std::cout << "[EdgeRouting] updateSnapPositions: WARNING! Edge " << edgeId
                      << " has DIAGONAL after PathCleanup! dx=" << dx << " dy=" << dy << std::endl;
        }
#endif

        it->second.labelPosition = LayoutUtils::calculateEdgeLabelPosition(it->second);
    }

    return result;
}

void EdgeRouting::updateEdgeRoutingWithOptimization(
    std::unordered_map<EdgeId, EdgeLayout>& edgeLayouts,
    const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
    const std::vector<EdgeId>& affectedEdges,
    const LayoutOptions& options,
    const std::unordered_set<NodeId>& movedNodes) {

#if EDGE_ROUTING_DEBUG
    std::cout << "\n[EdgeRouting] updateEdgeRoutingWithOptimization called" << std::endl;
    std::cout << "  affectedEdges: " << affectedEdges.size() << std::endl;
    std::cout << "  movedNodes: " << movedNodes.size() << std::endl;
    for (EdgeId edgeId : affectedEdges) {
        auto it = edgeLayouts.find(edgeId);
        if (it != edgeLayouts.end()) {
            const auto& e = it->second;
            std::cout << "  Edge " << edgeId << " BEFORE: "
                      << "src=(" << e.sourcePoint.x << "," << e.sourcePoint.y << ") "
                      << "tgt=(" << e.targetPoint.x << "," << e.targetPoint.y << ") "
                      << "bends=" << e.bendPoints.size() << std::endl;
        }
    }
#endif

    float gridSize = options.gridConfig.cellSize;

    // Apply drag algorithm if enabled
    if (options.optimizationOptions.dragAlgorithm != DragAlgorithm::None && !affectedEdges.empty()) {
        std::unique_ptr<IEdgeOptimizer> optimizer;

        switch (options.optimizationOptions.dragAlgorithm) {
            case DragAlgorithm::Geometric:
                optimizer = std::make_unique<GeometricEdgeOptimizer>(
                    options.optimizationOptions.scoringWeights);
                break;
            case DragAlgorithm::None:
                // No optimization
                break;
        }

        // Set up constraint manager with default constraints
        // All constraints apply automatically to all optimizers through this manager
        if (optimizer) {
            auto constraintManager = std::make_shared<EdgeConstraintManager>(
                EdgeConstraintManager::createDefault());
            optimizer->setConstraintManager(constraintManager);
        }

        if (optimizer) {
            // During drag operations, preserve existing edge directions
            // Only optimize path routing, not source/target edge selection
            optimizer->setPreserveDirections(true);

            // Collect non-self-loop edges
            std::vector<EdgeId> edgesToOptimize;
            edgesToOptimize.reserve(affectedEdges.size());
            for (EdgeId edgeId : affectedEdges) {
                auto it = edgeLayouts.find(edgeId);
                if (it != edgeLayouts.end() && it->second.from != it->second.to) {
                    edgesToOptimize.push_back(edgeId);
                }
            }

            // Step 1: Optimizer calculates best paths while preserving edge directions
            auto optimizedLayouts = optimizer->optimize(edgesToOptimize, edgeLayouts, nodeLayouts);

            // Merge optimized sourceEdge/targetEdge choices from optimizer
            // Note: bendPoints from optimizer are not preserved here - they will be
            // recalculated in updateSnapPositions to ensure orthogonality with snap positions
            for (auto& [edgeId, layout] : optimizedLayouts) {
                auto it = edgeLayouts.find(edgeId);
                if (it != edgeLayouts.end()) {
                    it->second.sourceEdge = layout.sourceEdge;
                    it->second.targetEdge = layout.targetEdge;
                    // Reset snap indices for redistribution
                    it->second.sourceSnapIndex = constants::SNAP_INDEX_UNASSIGNED;
                    it->second.targetSnapIndex = constants::SNAP_INDEX_UNASSIGNED;
                }
            }

            // Step 2: Snap distribution for new edge combinations
            // Note: bendPoints will be recalculated to maintain orthogonality with snap positions
            updateSnapPositions(edgeLayouts, nodeLayouts, edgesToOptimize, movedNodes, gridSize);

            // Step 3: Update self-loops separately (they don't go through optimizer)
            std::vector<EdgeId> selfLoops;
            for (EdgeId edgeId : affectedEdges) {
                auto it = edgeLayouts.find(edgeId);
                if (it != edgeLayouts.end() && it->second.from == it->second.to) {
                    selfLoops.push_back(edgeId);
                }
            }
            if (!selfLoops.empty()) {
                updateSnapPositions(edgeLayouts, nodeLayouts, selfLoops, movedNodes, gridSize);
            }

            // Note: bendPoints already set by GeometricEdgeOptimizer with obstacle avoidance
            // No need to recalculate here
        }
    } else {
        // No optimizer: just update snap positions
        updateSnapPositions(edgeLayouts, nodeLayouts, affectedEdges, movedNodes, gridSize);
    }

#if EDGE_ROUTING_DEBUG
    std::cout << "[EdgeRouting] updateEdgeRoutingWithOptimization DONE" << std::endl;
    for (EdgeId edgeId : affectedEdges) {
        auto it = edgeLayouts.find(edgeId);
        if (it != edgeLayouts.end()) {
            const auto& e = it->second;
            float dx = std::abs(e.targetPoint.x - e.sourcePoint.x);
            float dy = std::abs(e.targetPoint.y - e.sourcePoint.y);
            bool needsBend = (dx > 0.1f && dy > 0.1f);
            bool hasBend = !e.bendPoints.empty();
            std::cout << "  Edge " << edgeId << " AFTER: "
                      << "src=(" << e.sourcePoint.x << "," << e.sourcePoint.y << ") "
                      << "tgt=(" << e.targetPoint.x << "," << e.targetPoint.y << ") "
                      << "bends=" << e.bendPoints.size();
            if (needsBend && !hasBend) {
                std::cout << " [WARNING: DIAGONAL! dx=" << dx << " dy=" << dy << "]";
            }
            std::cout << std::endl;
            for (size_t i = 0; i < e.bendPoints.size(); ++i) {
                std::cout << "    bend[" << i << "]: (" << e.bendPoints[i].position.x
                          << "," << e.bendPoints[i].position.y << ")" << std::endl;
            }
        }
    }
#endif
}

// =============================================================================
// Edge Layout Validation (delegated to EdgeValidator)
// =============================================================================

std::string EdgeRouting::ValidationResult::getErrorDescription() const {
    // Create equivalent EdgeValidator result for delegation
    EdgeValidator::ValidationResult evResult;
    evResult.valid = valid;
    evResult.orthogonal = orthogonal;
    evResult.noNodeIntersection = noNodeIntersection;
    evResult.sourceDirectionOk = sourceDirectionOk;
    evResult.targetDirectionOk = targetDirectionOk;
    return evResult.getErrorDescription();
}

EdgeRouting::ValidationResult EdgeRouting::validateEdgeLayout(
    const EdgeLayout& layout,
    const std::unordered_map<NodeId, NodeLayout>& nodeLayouts) {
    // Delegate to EdgeValidator and convert result
    auto evResult = EdgeValidator::validate(layout, nodeLayouts);

    ValidationResult result;
    result.valid = evResult.valid;
    result.orthogonal = evResult.orthogonal;
    result.noNodeIntersection = evResult.noNodeIntersection;
    result.sourceDirectionOk = evResult.sourceDirectionOk;
    result.targetDirectionOk = evResult.targetDirectionOk;
    return result;
}

// =============================================================================
// Channel-Based Routing Methods
// =============================================================================

std::vector<ChannelRegion> EdgeRouting::computeChannelRegions(
    const Graph& graph,
    const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
    const std::unordered_set<EdgeId>& reversedEdges,
    Direction direction) {

    bool isHorizontal = (direction == Direction::LeftToRight || direction == Direction::RightToLeft);

    // Find min/max position for each layer (Y for vertical, X for horizontal)
    std::map<int, std::pair<float, float>> layerBounds;  // layer -> (min, max)

    for (const auto& [nodeId, layout] : nodeLayouts) {
        int layer = layout.layer;
        float nodeStart, nodeEnd;

        if (isHorizontal) {
            nodeStart = layout.position.x;
            nodeEnd = layout.position.x + layout.size.width;
        } else {
            nodeStart = layout.position.y;
            nodeEnd = layout.position.y + layout.size.height;
        }

        auto it = layerBounds.find(layer);
        if (it == layerBounds.end()) {
            layerBounds[layer] = {nodeStart, nodeEnd};
        } else {
            it->second.first = std::min(it->second.first, nodeStart);
            it->second.second = std::max(it->second.second, nodeEnd);
        }
    }

    if (layerBounds.empty()) {
        return {};
    }

    // Create channel regions between adjacent layers
    std::vector<ChannelRegion> regions;
    std::vector<int> sortedLayers;
    for (const auto& [layer, bounds] : layerBounds) {
        sortedLayers.push_back(layer);
    }
    std::sort(sortedLayers.begin(), sortedLayers.end());

    for (size_t i = 0; i + 1 < sortedLayers.size(); ++i) {
        int fromLayer = sortedLayers[i];
        int toLayer = sortedLayers[i + 1];

        ChannelRegion region;
        region.fromLayer = fromLayer;
        region.toLayer = toLayer;
        region.regionStart = layerBounds[fromLayer].second;  // Bottom of upper layer
        region.regionEnd = layerBounds[toLayer].first;       // Top of lower layer

        regions.push_back(region);
    }

    // Assign edges to regions based on their layer span
    for (EdgeId edgeId : graph.edges()) {
        const EdgeData edge = graph.getEdge(edgeId);

        auto fromIt = nodeLayouts.find(edge.from);
        auto toIt = nodeLayouts.find(edge.to);
        if (fromIt == nodeLayouts.end() || toIt == nodeLayouts.end()) {
            continue;
        }

        int fromLayer = fromIt->second.layer;
        int toLayer = toIt->second.layer;

        // Skip self-loops (handled separately)
        if (edge.from == edge.to) {
            continue;
        }

        // Handle reversed edges (swap layers for routing purposes)
        if (reversedEdges.count(edgeId) > 0) {
            std::swap(fromLayer, toLayer);
        }

        // Find regions this edge passes through
        int minLayer = std::min(fromLayer, toLayer);
        int maxLayer = std::max(fromLayer, toLayer);

        for (auto& region : regions) {
            if (region.fromLayer >= minLayer && region.toLayer <= maxLayer) {
                region.edges.push_back(edgeId);
            }
        }
    }

    return regions;
}

std::unordered_map<EdgeId, ChannelAssignment> EdgeRouting::allocateChannels(
    const Graph& graph,
    const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
    const std::unordered_set<EdgeId>& reversedEdges,
    const LayoutOptions& options) {

    std::unordered_map<EdgeId, ChannelAssignment> assignments;

    // Handle self-loops first
    std::map<NodeId, int> selfLoopCounts;
    for (EdgeId edgeId : graph.edges()) {
        const EdgeData edge = graph.getEdge(edgeId);
        if (edge.from == edge.to) {
            auto nodeIt = nodeLayouts.find(edge.from);
            if (nodeIt != nodeLayouts.end()) {
                ChannelAssignment assignment;
                assignment.channel = selfLoopCounts[edge.from]++;
                assignment.sourceLayer = nodeIt->second.layer;
                assignment.targetLayer = nodeIt->second.layer;
                assignment.isSelfLoop = true;
                assignments[edgeId] = assignment;
            }
        }
    }

    // Compute channel regions
    auto regions = computeChannelRegions(graph, nodeLayouts, reversedEdges, options.direction);

    // Identify bidirectional edge pairs (edges going in opposite directions between same nodes)
    // Track which edges should get offset channels
    // O(N) algorithm using unordered_map for O(1) lookup
    std::unordered_set<EdgeId> offsetEdges;
    std::unordered_set<EdgeId> processedBidirectional;

    // Build edge lookup map using helper function
    auto edgeMap = buildEdgeMapFromGraph<PairHash>(graph);

    // Find bidirectional pairs by checking for reverse edges (O(1) lookup)
    for (EdgeId edgeId : graph.edges()) {
        const EdgeData edge = graph.getEdge(edgeId);
        if (edge.from == edge.to) continue; // Skip self-loops
        if (processedBidirectional.count(edgeId) > 0) continue; // Already processed

        // Look for reverse edge in map - O(1) average case
        auto reverseIt = edgeMap.find({edge.to, edge.from});
        if (reverseIt != edgeMap.end()) {
            EdgeId otherId = reverseIt->second;
            // Found bidirectional pair - mark the second one for offset
            offsetEdges.insert(otherId);
            processedBidirectional.insert(edgeId);
            processedBidirectional.insert(otherId);
        }
    }

    // Sort edges within each region by source X coordinate to minimize crossings
    for (auto& region : regions) {
        std::sort(region.edges.begin(), region.edges.end(),
            [&](EdgeId a, EdgeId b) {
                const EdgeData edgeA = graph.getEdge(a);
                const EdgeData edgeB = graph.getEdge(b);

                auto fromAIt = nodeLayouts.find(edgeA.from);
                auto fromBIt = nodeLayouts.find(edgeB.from);

                if (fromAIt == nodeLayouts.end() || fromBIt == nodeLayouts.end()) {
                    return a < b;
                }

                return fromAIt->second.position.x < fromBIt->second.position.x;
            });

        // Assign channels sequentially
        int maxChannels = options.channelRouting.maxChannelsPerRegion;
        for (size_t i = 0; i < region.edges.size(); ++i) {
            EdgeId edgeId = region.edges[i];

            // Skip if already has assignment (e.g., from another region)
            if (assignments.find(edgeId) != assignments.end()) {
                continue;
            }

            const EdgeData edge = graph.getEdge(edgeId);
            auto fromIt = nodeLayouts.find(edge.from);
            auto toIt = nodeLayouts.find(edge.to);

            ChannelAssignment assignment;
            assignment.channel = static_cast<int>(i) % maxChannels;

            // If this edge is marked for offset (part of bidirectional pair), add offset
            if (offsetEdges.count(edgeId) > 0) {
                assignment.channel += 1;
                if (assignment.channel >= maxChannels) {
                    assignment.channel = maxChannels - 1;
                }
            }

            assignment.sourceLayer = fromIt->second.layer;
            assignment.targetLayer = toIt->second.layer;
            assignment.isSelfLoop = false;

            // Compute Y position using region info (quantized when gridSize > 0)
            assignment.yPosition = computeChannelY(region, assignment.channel,
                                                   options.channelRouting,
                                                   options.gridConfig.cellSize);

            assignments[edgeId] = assignment;
        }
    }

    return assignments;
}

float EdgeRouting::computeChannelY(
    const ChannelRegion& region,
    int channelIndex,
    const ChannelRoutingOptions& opts,
    float gridSize) {

    // Edge count in this region
    int count = static_cast<int>(region.edges.size());

    // === Quantized-First Calculation (when gridSize > 0) ===
    if (gridSize > 0.0f) {
        // Convert region bounds to grid units
        int gridStart = static_cast<int>(std::round(region.regionStart / gridSize));
        int gridEnd = static_cast<int>(std::round(region.regionEnd / gridSize));
        int gridLength = gridEnd - gridStart;

        // Single edge centered
        if (count == 1 && opts.centerSingleEdge) {
            int gridCenter = gridStart + gridLength / 2;
            return gridCenter * gridSize;
        }

        // Offset in grid units (convert from pixel offset)
        int gridOffset = static_cast<int>(std::round(opts.channelOffset / gridSize));
        int usableLength = gridLength - 2 * gridOffset;
        if (usableLength < 0) {
            usableLength = gridLength;
            gridOffset = 0;
        }

        // Distribute evenly within usable region using integer arithmetic
        // Same formula as distributeSnapPointsQuantized for symmetry preservation
        int divisor = (count > 1) ? (count - 1) : 1;
        int gridPos;
        if (count <= 1) {
            gridPos = gridStart + gridLength / 2;  // Center single channel
        } else {
            // Evenly spaced: gridStart + offset + (usableLength * channelIndex) / (count-1)
            // Use rounding: (usableLength * channelIndex * 2 + divisor) / (2 * divisor)
            gridPos = gridStart + gridOffset +
                      (usableLength * channelIndex * 2 + divisor) / (2 * divisor);
        }

        return gridPos * gridSize;
    }

    // === Legacy Float Calculation (when gridSize == 0) ===
    float regionHeight = region.regionEnd - region.regionStart;

    // Single edge centered
    if (count == 1 && opts.centerSingleEdge) {
        return region.regionStart + regionHeight / 2.0f;
    }

    // Compute usable height (excluding offset margins)
    float usableHeight = regionHeight - 2 * opts.channelOffset;
    if (usableHeight < 0) {
        usableHeight = regionHeight;  // Fall back if region too small
    }

    // Compute spacing between channels
    float spacing = opts.channelSpacing;
    if (count > 1) {
        float maxSpacing = usableHeight / static_cast<float>(count - 1);
        spacing = std::min(spacing, maxSpacing);
    }

    // Compute channel Y position
    float startY = region.regionStart + opts.channelOffset;
    if (usableHeight < regionHeight) {
        startY = region.regionStart + (regionHeight - (count - 1) * spacing) / 2.0f;
    }

    return startY + static_cast<float>(channelIndex) * spacing;
}

EdgeLayout EdgeRouting::routeChannelOrthogonal(
    const EdgeData& edge,
    const NodeLayout& fromLayout,
    const NodeLayout& toLayout,
    bool isReversed,
    const ChannelAssignment& channel,
    const LayoutOptions& options,
    const std::unordered_map<NodeId, NodeLayout>* allNodeLayouts) {

    EdgeLayout layout;
    layout.id = edge.id;
    layout.from = edge.from;
    layout.to = edge.to;

    Point fromCenter = fromLayout.center();
    Point toCenter = toLayout.center();

    bool isVertical = (options.direction == Direction::TopToBottom ||
                       options.direction == Direction::BottomToTop);

    if (isVertical) {
        // Vertical layout: source bottom, target top
        if (fromCenter.y < toCenter.y) {
            layout.sourcePoint = {fromCenter.x, fromLayout.position.y + fromLayout.size.height};
            layout.targetPoint = {toCenter.x, toLayout.position.y};
            layout.sourceEdge = NodeEdge::Bottom;
            layout.targetEdge = NodeEdge::Top;
        } else {
            layout.sourcePoint = {fromCenter.x, fromLayout.position.y};
            layout.targetPoint = {toCenter.x, toLayout.position.y + toLayout.size.height};
            layout.sourceEdge = NodeEdge::Top;
            layout.targetEdge = NodeEdge::Bottom;
        }

        // Store channel Y for recalculation (already grid-aligned from computeChannelY)
        layout.channelY = channel.yPosition;
    } else {
        // Horizontal layout: source right, target left
        if (fromCenter.x < toCenter.x) {
            layout.sourcePoint = {fromLayout.position.x + fromLayout.size.width, fromCenter.y};
            layout.targetPoint = {toLayout.position.x, toCenter.y};
            layout.sourceEdge = NodeEdge::Right;
            layout.targetEdge = NodeEdge::Left;
        } else {
            layout.sourcePoint = {fromLayout.position.x, fromCenter.y};
            layout.targetPoint = {toLayout.position.x + toLayout.size.width, toCenter.y};
            layout.sourceEdge = NodeEdge::Left;
            layout.targetEdge = NodeEdge::Right;
        }

        // Store channel X (stored in channelY field, already grid-aligned from computeChannelY)
        layout.channelY = channel.yPosition;
    }

    // Calculate bend points using the SINGLE algorithm
    // This ensures consistency across initial layout, drag, and post-drag scenarios
    float gridSize = options.gridConfig.cellSize;
    if (allNodeLayouts) {
        recalculateBendPoints(layout, *allNodeLayouts, gridSize);
    } else {
        // Fallback: create empty map if no node layouts provided
        std::unordered_map<NodeId, NodeLayout> emptyMap;
        recalculateBendPoints(layout, emptyMap, gridSize);
    }

    // Mark as reversed if needed (for arrow rendering)
    (void)isReversed;  // Could be used for visual indication

    // Final validation: Verify no segments intersect node interiors
    if (allNodeLayouts && !allNodeLayouts->empty()) {
        std::vector<Point> allPoints;
        allPoints.push_back(layout.sourcePoint);
        for (const auto& bend : layout.bendPoints) {
            allPoints.push_back(bend.position);
        }
        allPoints.push_back(layout.targetPoint);

        // Check ALL segments for intersection (skip only source/target nodes appropriately)
        bool hasIntersection = false;
        NodeId intersectingNode = 0;
        for (size_t i = 0; i + 1 < allPoints.size(); ++i) {
            const Point& p1 = allPoints[i];
            const Point& p2 = allPoints[i + 1];

            for (const auto& [nodeId, nodeLayout] : *allNodeLayouts) {
                // Skip source node for first segment, target node for last segment
                if (i == 0 && nodeId == layout.from) continue;
                if (i == allPoints.size() - 2 && nodeId == layout.to) continue;

                if (segmentIntersectsNode(p1, p2, nodeLayout)) {
                    hasIntersection = true;
                    intersectingNode = nodeId;
                    break;
                }
            }
            if (hasIntersection) break;
        }

        // If intersection found, create bypass path directly (don't rely on pathfinding)
        if (hasIntersection) {
            auto nodeIt = allNodeLayouts->find(intersectingNode);
            if (nodeIt != allNodeLayouts->end()) {
                const NodeLayout& blockingNode = nodeIt->second;

                // Calculate bypass route to the left or right of all blocking nodes
                float margin = gridSize > 0.0f ? gridSize : constants::PATHFINDING_GRID_SIZE;

                // Find leftmost and rightmost edges of all intermediate nodes
                float leftmostEdge = blockingNode.position.x;
                float rightmostEdge = blockingNode.position.x + blockingNode.size.width;

                for (const auto& [nid, nlayout] : *allNodeLayouts) {
                    if (nid == layout.from || nid == layout.to) continue;
                    leftmostEdge = std::min(leftmostEdge, nlayout.position.x);
                    rightmostEdge = std::max(rightmostEdge, nlayout.position.x + nlayout.size.width);
                }

                // Choose side based on source and target positions
                float sourceX = layout.sourcePoint.x;
                float targetX = layout.targetPoint.x;
                float avgX = (sourceX + targetX) / 2.0f;
                float centerNodes = (leftmostEdge + rightmostEdge) / 2.0f;

                float bypassX;
                if (avgX < centerNodes) {
                    // Prefer left bypass
                    bypassX = leftmostEdge - margin;
                } else {
                    // Prefer right bypass
                    bypassX = rightmostEdge + margin;
                }

                // Snap to grid if enabled
                if (gridSize > 0.0f) {
                    bypassX = std::round(bypassX / gridSize) * gridSize;
                }

                // Create bypass path directly: source -> (bypassX, src.y) -> (bypassX, tgt.y) -> target
                layout.bendPoints.clear();

                float exitY, entryY;
                if (layout.sourceEdge == NodeEdge::Bottom) {
                    exitY = layout.sourcePoint.y + margin;
                } else if (layout.sourceEdge == NodeEdge::Top) {
                    exitY = layout.sourcePoint.y - margin;
                } else {
                    exitY = layout.sourcePoint.y;
                }

                if (layout.targetEdge == NodeEdge::Top) {
                    entryY = layout.targetPoint.y - margin;
                } else if (layout.targetEdge == NodeEdge::Bottom) {
                    entryY = layout.targetPoint.y + margin;
                } else {
                    entryY = layout.targetPoint.y;
                }

                // Snap Y coordinates to grid
                if (gridSize > 0.0f) {
                    exitY = std::round(exitY / gridSize) * gridSize;
                    entryY = std::round(entryY / gridSize) * gridSize;
                }

                // Create bend points for bypass route
                layout.bendPoints.push_back({{layout.sourcePoint.x, exitY}});
                layout.bendPoints.push_back({{bypassX, exitY}});
                layout.bendPoints.push_back({{bypassX, entryY}});
                layout.bendPoints.push_back({{layout.targetPoint.x, entryY}});
            }
        }
    }

    // Final clearance check: ensure first bend has minimum distance from source
    float minClearance = gridSize > 0.0f ? gridSize : constants::PATHFINDING_GRID_SIZE;
    if (!layout.bendPoints.empty()) {
        Point& firstBend = layout.bendPoints[0].position;

        switch (layout.sourceEdge) {
            case NodeEdge::Top:
                if (firstBend.y > layout.sourcePoint.y - minClearance) {
                    float newY = layout.sourcePoint.y - minClearance;
                    firstBend.y = newY;
                    if (layout.bendPoints.size() >= 2) {
                        layout.bendPoints[1].position.y = newY;
                    }
                }
                break;
            case NodeEdge::Bottom:
                if (firstBend.y < layout.sourcePoint.y + minClearance) {
                    float newY = layout.sourcePoint.y + minClearance;
                    firstBend.y = newY;
                    if (layout.bendPoints.size() >= 2) {
                        layout.bendPoints[1].position.y = newY;
                    }
                }
                break;
            case NodeEdge::Left:
                if (firstBend.x > layout.sourcePoint.x - minClearance) {
                    float newX = layout.sourcePoint.x - minClearance;
                    firstBend.x = newX;
                    if (layout.bendPoints.size() >= 2) {
                        layout.bendPoints[1].position.x = newX;
                    }
                }
                break;
            case NodeEdge::Right:
                if (firstBend.x < layout.sourcePoint.x + minClearance) {
                    float newX = layout.sourcePoint.x + minClearance;
                    firstBend.x = newX;
                    if (layout.bendPoints.size() >= 2) {
                        layout.bendPoints[1].position.x = newX;
                    }
                }
                break;
        }
    }

    // Calculate label position
    layout.labelPosition = LayoutUtils::calculateEdgeLabelPosition(layout);

    return layout;
}

EdgeLayout EdgeRouting::routeSelfLoop(
    const EdgeData& edge,
    const NodeLayout& nodeLayout,
    int loopIndex,
    const LayoutOptions& options) {
    // Delegate to SelfLoopRouter
    return SelfLoopRouter::route(edge.id, edge.from, nodeLayout, loopIndex, options);
}


}  // namespace arborvia
