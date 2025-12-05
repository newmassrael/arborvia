#include "EdgeRouting.h"
#include "SnapIndexManager.h"
#include "ObstacleMap.h"
#include "AStarPathFinder.h"
#include "AStarEdgeOptimizer.h"
#include "GeometricEdgeOptimizer.h"
#include "arborvia/core/GeometryUtils.h"
#include "arborvia/layout/IEdgeOptimizer.h"
#include "arborvia/layout/LayoutTypes.h"
#include "arborvia/layout/LayoutUtils.h"
#include "arborvia/layout/PathRoutingCoordinator.h"

#include <algorithm>
#include <cmath>
#include <set>
#include <unordered_map>

namespace arborvia {


// =============================================================================
// Routing Constants
// =============================================================================

namespace {
    /// Tolerance for floating point comparisons in path calculations
    constexpr float EPSILON = 0.1f;



    /// Self-loop spacing between source and target points
    constexpr float SELF_LOOP_SPACING = 10.0f;

    /// Default offset for self-loop routing when grid is disabled
    /// Self-loops route outside the node by this amount
    constexpr float DEFAULT_SELF_LOOP_OFFSET = 20.0f;


    // =========================================================================
    // Grid-Relative Offset Functions
    // =========================================================================
    // These functions return offsets that are guaranteed to be grid-aligned
    // when gridSize > 0, preventing post-hoc snap from creating spikes/duplicates.


    /// Self-loop spacing: distance between source and target on self-loops
    inline float getSelfLoopSpacing(float gridSize) {
        return gridSize > 0.0f ? gridSize : SELF_LOOP_SPACING;
    }


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

    /// Check if two points are essentially the same (within tolerance)
    bool isPointDuplicate(const Point& a, const Point& b) {
        return std::abs(a.x - b.x) < EPSILON && std::abs(a.y - b.y) < EPSILON;
    }

    /// Check if three collinear points form a spike (direction reversal)
    bool isSpike(const Point& a, const Point& b, const Point& c) {
        // Check vertical spike (all on same X)
        bool sameX = (std::abs(a.x - b.x) < EPSILON && std::abs(b.x - c.x) < EPSILON);
        if (sameX) {
            bool aToB_down = b.y > a.y;
            bool bToC_down = c.y > b.y;
            if (aToB_down != bToC_down) {
                return true;
            }
        }

        // Check horizontal spike (all on same Y)
        bool sameY = (std::abs(a.y - b.y) < EPSILON && std::abs(b.y - c.y) < EPSILON);
        if (sameY) {
            bool aToB_right = b.x > a.x;
            bool bToC_right = c.x > b.x;
            if (aToB_right != bToC_right) {
                return true;
            }
        }

        return false;
    }

    /// Remove spikes, duplicate points, and circular detours from a path
    /// A circular detour is when the path returns to a previously visited point
    void removeSpikesAndDuplicates(std::vector<Point>& points) {
        bool modified = true;
        while (modified && points.size() > 2) {
            modified = false;

            // Remove duplicate consecutive points
            for (size_t i = 0; i + 1 < points.size(); ++i) {
                if (isPointDuplicate(points[i], points[i + 1])) {
                    points.erase(points.begin() + i + 1);
                    modified = true;
                    break;
                }
            }
            if (modified) continue;

            // Remove spike points (direction reversal on same line)
            for (size_t i = 0; i + 2 < points.size(); ++i) {
                if (isSpike(points[i], points[i + 1], points[i + 2])) {
                    points.erase(points.begin() + i + 1);
                    modified = true;
                    break;
                }
            }
            if (modified) continue;

            // Remove redundant collinear intermediate points
            // If three consecutive points are on the same line (vertical or horizontal),
            // the middle point can be removed as it doesn't affect the path
            for (size_t i = 0; i + 2 < points.size(); ++i) {
                const Point& a = points[i];
                const Point& b = points[i + 1];
                const Point& c = points[i + 2];

                // Check if all three are on same vertical line
                bool sameX = (std::abs(a.x - b.x) < EPSILON && std::abs(b.x - c.x) < EPSILON);
                // Check if all three are on same horizontal line
                bool sameY = (std::abs(a.y - b.y) < EPSILON && std::abs(b.y - c.y) < EPSILON);

                if (sameX || sameY) {
                    // Middle point is redundant, remove it
                    points.erase(points.begin() + i + 1);
                    modified = true;
                    break;
                }
            }
            if (modified) continue;

            // Remove non-consecutive duplicate points and the circular detour between them
            // This is safe because if two points are identical, the path between them
            // is a closed loop that accomplishes nothing
            for (size_t i = 0; i + 2 < points.size(); ++i) {
                for (size_t j = i + 2; j < points.size(); ++j) {
                    if (isPointDuplicate(points[i], points[j])) {
                        // Found duplicate at i and j - remove all points from i+1 to j (inclusive)
                        // This eliminates the circular detour between them
                        points.erase(points.begin() + i + 1, points.begin() + j + 1);
                        modified = true;
                        break;
                    }
                }
                if (modified) break;
            }
        }
    }


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

    // Handle self-loops specially - they need to route around the same node
    if (layout.from == layout.to) {
        auto nodeIt = nodeLayouts.find(layout.from);
        if (nodeIt != nodeLayouts.end()) {
            const NodeLayout& node = nodeIt->second;
            // Self-loop offset: use the larger of gridSize or DEFAULT_SELF_LOOP_OFFSET
            // This ensures sufficient clearance regardless of grid settings
            float offset = std::max(gridSize, DEFAULT_SELF_LOOP_OFFSET);

            // Determine direction from sourceEdge
            switch (layout.sourceEdge) {
                case NodeEdge::Right:
                    layout.bendPoints.push_back({{node.position.x + node.size.width + offset,
                                                  layout.sourcePoint.y}});
                    layout.bendPoints.push_back({{node.position.x + node.size.width + offset,
                                                  layout.targetPoint.y}});
                    break;
                case NodeEdge::Left:
                    layout.bendPoints.push_back({{node.position.x - offset,
                                                  layout.sourcePoint.y}});
                    layout.bendPoints.push_back({{node.position.x - offset,
                                                  layout.targetPoint.y}});
                    break;
                case NodeEdge::Top:
                    layout.bendPoints.push_back({{layout.sourcePoint.x,
                                                  node.position.y - offset}});
                    layout.bendPoints.push_back({{layout.targetPoint.x,
                                                  node.position.y - offset}});
                    break;
                case NodeEdge::Bottom:
                    layout.bendPoints.push_back({{layout.sourcePoint.x,
                                                  node.position.y + node.size.height + offset}});
                    layout.bendPoints.push_back({{layout.targetPoint.x,
                                                  node.position.y + node.size.height + offset}});
                    break;
                default:
                    // For unknown edge, default to right
                    layout.bendPoints.push_back({{node.position.x + node.size.width + offset,
                                                  layout.sourcePoint.y}});
                    layout.bendPoints.push_back({{node.position.x + node.size.width + offset,
                                                  layout.targetPoint.y}});
                    break;
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
        removeSpikesAndDuplicates(allPoints);

        // Reconstruct bendPoints from allPoints
        layout.bendPoints.clear();
        for (size_t i = 1; i + 1 < allPoints.size(); ++i) {
            layout.bendPoints.push_back({allPoints[i]});
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

        removeSpikesAndDuplicates(allPoints);

        layout.bendPoints.clear();
        for (size_t i = 1; i + 1 < allPoints.size(); ++i) {
            layout.bendPoints.push_back({allPoints[i]});
        }
    }
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

    // Expand node bounds by margin for proximity detection
    float nodeXmin = node.position.x - margin;
    float nodeXmax = node.position.x + node.size.width + margin;
    float nodeYmin = node.position.y - margin;
    float nodeYmax = node.position.y + node.size.height + margin;

    bool isHorizontal = (std::abs(p1.y - p2.y) < EPSILON);
    bool isVertical = (std::abs(p1.x - p2.x) < EPSILON);

    if (isHorizontal) {
        // Horizontal segment: check if Y is strictly inside node Y-range AND X-ranges overlap
        float y = p1.y;
        float xMin = std::min(p1.x, p2.x);
        float xMax = std::max(p1.x, p2.x);

        // Y must be strictly inside (not on boundary) AND X-ranges must overlap
        return (y > nodeYmin && y < nodeYmax && xMin < nodeXmax && xMax > nodeXmin);
    }
    else if (isVertical) {
        // Vertical segment: check if X is strictly inside node X-range AND Y-ranges overlap
        float x = p1.x;
        float yMin = std::min(p1.y, p2.y);
        float yMax = std::max(p1.y, p2.y);

        // X must be strictly inside (not on boundary) AND Y-ranges must overlap
        return (x > nodeXmin && x < nodeXmax && yMin < nodeYmax && yMax > nodeYmin);
    }

    return false;  // No diagonal segments in orthogonal routing
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

        removeSpikesAndDuplicates(fullPath);

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

void EdgeRouting::updateSnapPositions(
    std::unordered_map<EdgeId, EdgeLayout>& edgeLayouts,
    const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
    const std::vector<EdgeId>& affectedEdges,
    const std::unordered_set<NodeId>& movedNodes,
    float gridSize) {

    float effectiveGridSize = getEffectiveGridSize(gridSize);

    // Helper to check if a node should be updated
    auto shouldUpdateNode = [&movedNodes](NodeId nodeId) -> bool {
        return movedNodes.empty() || movedNodes.count(nodeId) > 0;
    };

    // Unified mode: all connections on same edge distributed together
    std::map<std::pair<NodeId, NodeEdge>, std::vector<std::pair<EdgeId, bool>>> affectedConnections;

    for (EdgeId edgeId : affectedEdges) {
        auto it = edgeLayouts.find(edgeId);
        if (it == edgeLayouts.end()) continue;
        const EdgeLayout& layout = it->second;
        affectedConnections[{layout.from, layout.sourceEdge}].push_back({edgeId, true});
        affectedConnections[{layout.to, layout.targetEdge}].push_back({edgeId, false});
    }

    for (auto& [key, connections] : affectedConnections) {
        auto [nodeId, nodeEdge] = key;

        // Check if any edge needs snap index redistribution (has -1)
        bool needsRedistribution = false;
        for (const auto& [edgeId, isSource] : connections) {
            auto it = edgeLayouts.find(edgeId);
            if (it != edgeLayouts.end()) {
                int snapIdx = isSource ? it->second.sourceSnapIndex : it->second.targetSnapIndex;
                if (snapIdx == constants::SNAP_INDEX_UNASSIGNED) {
                    needsRedistribution = true;
                    break;
                }
            }
        }

        // Skip nodes that haven't moved AND don't need redistribution
        if (!shouldUpdateNode(nodeId) && !needsRedistribution) continue;

        auto nodeIt = nodeLayouts.find(nodeId);
        if (nodeIt == nodeLayouts.end()) continue;

        const NodeLayout& node = nodeIt->second;

        // Get TOTAL count from all edgeLayouts, not just affected edges
        auto [totalIn, totalOut] = countConnectionsOnNodeEdge(edgeLayouts, nodeId, nodeEdge);
        int totalCount = totalIn + totalOut;

        for (const auto& [edgeId, isSource] : connections) {
            EdgeLayout& layout = edgeLayouts[edgeId];

            // Use existing snap index
            int snapIdx = isSource ? layout.sourceSnapIndex : layout.targetSnapIndex;
            if (snapIdx < 0 || snapIdx >= totalCount) {  // includes SNAP_INDEX_UNASSIGNED
                // Fallback: find index based on current position in connections
                snapIdx = 0;
                for (size_t i = 0; i < connections.size(); ++i) {
                    if (connections[i].first == edgeId) {
                        snapIdx = static_cast<int>(i);
                        break;
                    }
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
        }
    }

    // Check which edges are bidirectional (have a reverse edge)
    // O(N) algorithm using unordered_map for O(1) lookup
    std::unordered_set<EdgeId> bidirectionalEdges;

    // Build edge lookup map using helper function
    auto edgeMap = buildEdgeMapFromLayouts<PairHash>(edgeLayouts);

    // Check affected edges for bidirectionality (O(1) lookup per edge)
    for (EdgeId edgeId : affectedEdges) {
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

    // Recalculate bend points for affected edges
    // Note: Snap points are already on grid from quantized calculation (if gridSize > 0)
    for (EdgeId edgeId : affectedEdges) {
        auto it = edgeLayouts.find(edgeId);
        if (it == edgeLayouts.end()) continue;

        // Preserve channel routing for ALL edges (bidirectional and non-bidirectional)
        // channelY stores absolute coordinates (Y for vertical, X for horizontal layout)
        // and is already set correctly by allocateChannels(), so we keep it unchanged.
        // recalculateBendPoints() will use this absolute position to maintain consistent routing.
        //
        // ARCHITECTURAL PRINCIPLE: All edges maintain their channel assignment across
        // initial layout, drag, and post-drag scenarios for complete consistency.

        recalculateBendPoints(it->second, nodeLayouts, effectiveGridSize);

        // Grid mode: bend points are already orthogonal from quantized routing.
        // Remove spikes/duplicates for path cleanup.
        std::vector<Point> fullPath;
        fullPath.push_back(it->second.sourcePoint);
        for (const auto& bp : it->second.bendPoints) {
            fullPath.push_back(bp.position);
        }
        fullPath.push_back(it->second.targetPoint);

        removeSpikesAndDuplicates(fullPath);

        it->second.bendPoints.clear();
        for (size_t i = 1; i + 1 < fullPath.size(); ++i) {
            it->second.bendPoints.push_back({fullPath[i]});
        }

        it->second.labelPosition = LayoutUtils::calculateEdgeLabelPosition(it->second);
    }
}

void EdgeRouting::updateEdgeRoutingWithOptimization(
    std::unordered_map<EdgeId, EdgeLayout>& edgeLayouts,
    const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
    const std::vector<EdgeId>& affectedEdges,
    const LayoutOptions& options,
    const std::unordered_set<NodeId>& movedNodes) {

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

        if (optimizer) {
            // Collect non-self-loop edges
            std::vector<EdgeId> edgesToOptimize;
            edgesToOptimize.reserve(affectedEdges.size());
            for (EdgeId edgeId : affectedEdges) {
                auto it = edgeLayouts.find(edgeId);
                if (it != edgeLayouts.end() && it->second.from != it->second.to) {
                    edgesToOptimize.push_back(edgeId);
                }
            }

            // Step 1: Optimizer selects best edge combinations (sourceEdge/targetEdge only)
            auto optimizedLayouts = optimizer->optimize(edgesToOptimize, edgeLayouts, nodeLayouts);

            // Merge optimized edge combinations (including obstacle-avoidance bendPoints)
            for (auto& [edgeId, layout] : optimizedLayouts) {
                auto it = edgeLayouts.find(edgeId);
                if (it != edgeLayouts.end()) {
                    it->second.sourceEdge = layout.sourceEdge;
                    it->second.targetEdge = layout.targetEdge;
                    it->second.bendPoints = layout.bendPoints;  // Use obstacle-avoidance path
                    // Reset snap indices for redistribution
                    it->second.sourceSnapIndex = constants::SNAP_INDEX_UNASSIGNED;
                    it->second.targetSnapIndex = constants::SNAP_INDEX_UNASSIGNED;
                }
            }

            // Step 2: Snap distribution for new edge combinations
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
}

// =============================================================================
// Edge Layout Validation
// =============================================================================

std::string EdgeRouting::ValidationResult::getErrorDescription() const {
    if (valid) return "Valid";

    std::string desc;
    if (!orthogonal) desc += "Non-orthogonal segments; ";
    if (!noNodeIntersection) desc += "Segments pass through nodes; ";
    if (!sourceDirectionOk) desc += "First segment direction violates sourceEdge; ";
    if (!targetDirectionOk) desc += "Last segment direction violates targetEdge; ";

    if (!desc.empty() && desc.back() == ' ') {
        desc.pop_back();
        desc.pop_back();  // Remove trailing "; "
    }
    return desc;
}

EdgeRouting::ValidationResult EdgeRouting::validateEdgeLayout(
    const EdgeLayout& layout,
    const std::unordered_map<NodeId, NodeLayout>& nodeLayouts) {

    ValidationResult result;

    // Build path points
    std::vector<Point> path;
    path.push_back(layout.sourcePoint);
    for (const auto& bp : layout.bendPoints) {
        path.push_back(bp.position);
    }
    path.push_back(layout.targetPoint);

    // Need at least 2 points
    if (path.size() < 2) {
        result.valid = false;
        result.orthogonal = false;
        return result;
    }

    // === Check 1: Source direction constraint ===
    // First segment must exit in the direction indicated by sourceEdge
    {
        float dx = path[1].x - path[0].x;
        float dy = path[1].y - path[0].y;

        switch (layout.sourceEdge) {
            case NodeEdge::Top:    result.sourceDirectionOk = (dy < -0.1f); break;
            case NodeEdge::Bottom: result.sourceDirectionOk = (dy > 0.1f);  break;
            case NodeEdge::Left:   result.sourceDirectionOk = (dx < -0.1f); break;
            case NodeEdge::Right:  result.sourceDirectionOk = (dx > 0.1f);  break;
        }
    }

    // === Check 2: Target direction constraint ===
    // Last segment must enter in the direction indicated by targetEdge
    {
        size_t last = path.size() - 1;
        float dx = path[last].x - path[last - 1].x;
        float dy = path[last].y - path[last - 1].y;

        switch (layout.targetEdge) {
            case NodeEdge::Top:    result.targetDirectionOk = (dy > 0.1f);  break;  // Enter from above
            case NodeEdge::Bottom: result.targetDirectionOk = (dy < -0.1f); break;  // Enter from below
            case NodeEdge::Left:   result.targetDirectionOk = (dx > 0.1f);  break;  // Enter from left
            case NodeEdge::Right:  result.targetDirectionOk = (dx < -0.1f); break;  // Enter from right
        }
    }

    // === Check 3: Orthogonality and node intersection ===
    constexpr float TOLERANCE = 1.0f;

    for (size_t i = 0; i + 1 < path.size(); ++i) {
        const Point& p1 = path[i];
        const Point& p2 = path[i + 1];

        bool isHoriz = std::abs(p1.y - p2.y) < 0.1f;
        bool isVert = std::abs(p1.x - p2.x) < 0.1f;

        // Orthogonality check
        if (!isHoriz && !isVert) {
            result.orthogonal = false;
        }

        // Node intersection check
        bool isFirstSeg = (i == 0);
        bool isLastSeg = (i == path.size() - 2);

        for (const auto& [nodeId, node] : nodeLayouts) {
            // Skip source node at first segment start, target node at last segment end
            bool skipAtP1 = (isFirstSeg && nodeId == layout.from);
            bool skipAtP2 = (isLastSeg && nodeId == layout.to);

            // Check if segment passes through node interior
            float xmin = node.position.x;
            float xmax = node.position.x + node.size.width;
            float ymin = node.position.y;
            float ymax = node.position.y + node.size.height;

            bool intersects = false;

            if (isHoriz) {
                float y = p1.y;
                float segXmin = std::min(p1.x, p2.x);
                float segXmax = std::max(p1.x, p2.x);

                // Shrink segment if endpoints should be excluded
                if (skipAtP1) {
                    if (segXmin == p1.x) segXmin += TOLERANCE;
                    else segXmax -= TOLERANCE;
                }
                if (skipAtP2) {
                    if (segXmax == p2.x) segXmax -= TOLERANCE;
                    else segXmin += TOLERANCE;
                }

                // Check if y is strictly inside node and x ranges overlap
                intersects = (y > ymin && y < ymax && segXmin < xmax && segXmax > xmin);
            } else if (isVert) {
                float x = p1.x;
                float segYmin = std::min(p1.y, p2.y);
                float segYmax = std::max(p1.y, p2.y);

                // Shrink segment if endpoints should be excluded
                if (skipAtP1) {
                    if (segYmin == p1.y) segYmin += TOLERANCE;
                    else segYmax -= TOLERANCE;
                }
                if (skipAtP2) {
                    if (segYmax == p2.y) segYmax -= TOLERANCE;
                    else segYmin += TOLERANCE;
                }

                // Check if x is strictly inside node and y ranges overlap
                intersects = (x > xmin && x < xmax && segYmin < ymax && segYmax > ymin);
            }

            if (intersects) {
                result.noNodeIntersection = false;
            }
        }
    }

    // Final validity
    result.valid = result.orthogonal &&
                   result.noNodeIntersection &&
                   result.sourceDirectionOk &&
                   result.targetDirectionOk;

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

    EdgeLayout layout;
    layout.id = edge.id;
    layout.from = edge.from;
    layout.to = edge.to;

    const auto& config = options.channelRouting.selfLoop;
    float offset = config.loopOffset + static_cast<float>(loopIndex) * config.stackSpacing;

    // Determine direction based on config or auto-detect
    SelfLoopDirection dir = config.preferredDirection;

    // For auto, default to Right (most common for state diagrams)
    if (dir == SelfLoopDirection::Auto) {
        dir = SelfLoopDirection::Right;
    }

    Point center = nodeLayout.center();
    float gridSize = options.gridConfig.cellSize;
    float spacing = getSelfLoopSpacing(gridSize);

    switch (dir) {
        case SelfLoopDirection::Right:
            layout.sourcePoint = {nodeLayout.position.x + nodeLayout.size.width,
                                  center.y + spacing};
            layout.targetPoint = {nodeLayout.position.x + nodeLayout.size.width,
                                  center.y - spacing};
            layout.sourceEdge = NodeEdge::Right;
            layout.targetEdge = NodeEdge::Right;
            layout.bendPoints.push_back({{nodeLayout.position.x + nodeLayout.size.width + offset,
                                          layout.sourcePoint.y}});
            layout.bendPoints.push_back({{nodeLayout.position.x + nodeLayout.size.width + offset,
                                          layout.targetPoint.y}});
            break;

        case SelfLoopDirection::Left:
            layout.sourcePoint = {nodeLayout.position.x, center.y - spacing};
            layout.targetPoint = {nodeLayout.position.x, center.y + spacing};
            layout.sourceEdge = NodeEdge::Left;
            layout.targetEdge = NodeEdge::Left;
            layout.bendPoints.push_back({{nodeLayout.position.x - offset, layout.sourcePoint.y}});
            layout.bendPoints.push_back({{nodeLayout.position.x - offset, layout.targetPoint.y}});
            break;

        case SelfLoopDirection::Top:
            layout.sourcePoint = {center.x - spacing, nodeLayout.position.y};
            layout.targetPoint = {center.x + spacing, nodeLayout.position.y};
            layout.sourceEdge = NodeEdge::Top;
            layout.targetEdge = NodeEdge::Top;
            layout.bendPoints.push_back({{layout.sourcePoint.x, nodeLayout.position.y - offset}});
            layout.bendPoints.push_back({{layout.targetPoint.x, nodeLayout.position.y - offset}});
            break;

        case SelfLoopDirection::Bottom:
            layout.sourcePoint = {center.x + spacing,
                                  nodeLayout.position.y + nodeLayout.size.height};
            layout.targetPoint = {center.x - spacing,
                                  nodeLayout.position.y + nodeLayout.size.height};
            layout.sourceEdge = NodeEdge::Bottom;
            layout.targetEdge = NodeEdge::Bottom;
            layout.bendPoints.push_back({{layout.sourcePoint.x,
                                          nodeLayout.position.y + nodeLayout.size.height + offset}});
            layout.bendPoints.push_back({{layout.targetPoint.x,
                                          nodeLayout.position.y + nodeLayout.size.height + offset}});
            break;

        case SelfLoopDirection::Auto:
            // Auto is converted to Right above, this case should never be reached
            break;
    }

    // Calculate label position
    layout.labelPosition = LayoutUtils::calculateEdgeLabelPosition(layout);

    return layout;
}


}  // namespace arborvia
