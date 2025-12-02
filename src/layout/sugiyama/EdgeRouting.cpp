#include "EdgeRouting.h"
#include "SnapIndexManager.h"
#include "arborvia/layout/LayoutTypes.h"
#include "arborvia/layout/LayoutUtils.h"

#include <algorithm>
#include <array>
#include <climits>
#include <cmath>
#include <optional>
#include <iostream>
#include <set>
#include <unordered_map>

namespace arborvia {
namespace algorithms {

// =============================================================================
// Routing Constants
// =============================================================================

namespace {
    /// Tolerance for floating point comparisons in path calculations
    constexpr float EPSILON = 0.1f;
    
    /// Margin for avoiding node intersections during routing
    constexpr float SAFE_MARGIN = 20.0f;
    
    /// Offset for direction correction when inserting bend points
    constexpr float DIRECTION_OFFSET = 20.0f;
    
    /// Margin for rerouting around nodes when direction is wrong
    constexpr float REROUTE_MARGIN = 30.0f;
    
    /// Minimum perpendicular distance from snap point for directional routing
    constexpr float MIN_PERPENDICULAR_OFFSET = 10.0f;
    
    /// Margin for segment-node intersection avoidance
    constexpr float AVOIDANCE_MARGIN = 15.0f;
    
    /// Self-loop spacing between source and target points
    constexpr float SELF_LOOP_SPACING = 10.0f;
    
    /// Default offset for self-loop routing when grid is disabled
    /// Self-loops route outside the node by this amount
    constexpr float DEFAULT_SELF_LOOP_OFFSET = 20.0f;
    
    /// Tolerance for safe avoidance checks
    constexpr float SAFE_TOLERANCE = 5.0f;
    
    /// Maximum number of bend points before stopping iteration
    constexpr int MAX_PATH_POINTS = 30;
    
    /// Maximum number of intersection fixing passes
    constexpr int MAX_FIX_PASSES = 5;

    // =========================================================================
    // Grid-Relative Offset Functions
    // =========================================================================
    // These functions return offsets that are guaranteed to be grid-aligned
    // when gridSize > 0, preventing post-hoc snap from creating spikes/duplicates.

    /// Direction offset: minimum distance from node edge for directional routing
    inline float getDirectionOffset(float gridSize) {
        return gridSize > 0.0f ? gridSize : DIRECTION_OFFSET;
    }

    /// Reroute margin: distance for routing around when direction conflicts
    inline float getRerouteMargin(float gridSize) {
        return gridSize > 0.0f ? gridSize * 2.0f : REROUTE_MARGIN;
    }

    /// Avoidance margin: distance for avoiding node intersections
    inline float getAvoidanceMargin(float gridSize) {
        return gridSize > 0.0f ? gridSize : AVOIDANCE_MARGIN;
    }

    /// Self-loop spacing: distance between source and target on self-loops
    inline float getSelfLoopSpacing(float gridSize) {
        return gridSize > 0.0f ? gridSize : SELF_LOOP_SPACING;
    }

    /// Perpendicular offset: minimum distance from snap point for directional routing
    inline float getPerpendicularOffset(float gridSize) {
        return gridSize > 0.0f ? gridSize : MIN_PERPENDICULAR_OFFSET;
    }

    /// Get effective grid size for calculations
    /// Returns 1.0f when grid is disabled (gridSize <= 0) to prevent division by zero
    /// This means grid unit = pixel coordinate when grid is disabled
    inline float getEffectiveGridSize(float gridSize) {
        return gridSize > 0.0f ? gridSize : 1.0f;
    }

    // =========================================================================
    // Grid-Unit Constants for Quantized-First Calculations
    // =========================================================================
    // These constants define offsets in GRID UNITS (integers).
    // All routing calculations use these directly without conversion.

    constexpr int GRID_PERPENDICULAR_OFFSET = 1;  // 1 grid cell for directional constraints
    constexpr int GRID_AVOIDANCE_MARGIN = 1;      // 1 grid cell for node avoidance
    constexpr int GRID_REROUTE_MARGIN = 2;        // 2 grid cells for routing around
    constexpr int GRID_SELF_LOOP_SPACING = 1;     // 1 grid cell for self-loops

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

    /// Apply directional constraint in grid units
    /// Returns the constrained coordinate in grid units
    /// @param gridChannelPos Channel position in grid units
    /// @param gridSnapPoint Snap point position in grid units
    /// @param edge Which edge of the node
    /// @param offset Perpendicular offset in grid units (typically GRID_PERPENDICULAR_OFFSET)
    /// @return Constrained position in grid units
    int applyDirectionalConstraintGrid(int gridChannelPos, int gridSnapPoint, NodeEdge edge, int offset = GRID_PERPENDICULAR_OFFSET) {
        switch (edge) {
            case NodeEdge::Top:
            case NodeEdge::Left:
                // Bend points must be toward smaller coordinate
                return std::min(gridChannelPos, gridSnapPoint - offset);
            case NodeEdge::Bottom:
            case NodeEdge::Right:
                // Bend points must be toward larger coordinate
                return std::max(gridChannelPos, gridSnapPoint + offset);
            default:
                return gridChannelPos;
        }
    }

    /// Check if a three-segment grid path intersects any node
    /// @param gridOrthogonal The orthogonal coordinate in grid units
    /// @param gridSrcSecondary Source secondary coordinate in grid units
    /// @param gridTgtSecondary Target secondary coordinate in grid units
    /// @param axis Axis configuration
    /// @param nodeLayouts All node layouts
    /// @param excludeFrom Source node ID to exclude
    /// @param excludeTo Target node ID to exclude
    /// @param gridSize Grid cell size
    bool gridPathIntersectsNodes(
        int gridOrthogonal,
        int gridSrcSecondary,
        int gridTgtSecondary,
        const AxisConfig& axis,
        const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
        NodeId excludeFrom,
        NodeId excludeTo,
        float gridSize) {

        // Convert to pixel coordinates for intersection checking
        float orthogonal = axis.toPixel(gridOrthogonal, gridSize);
        float srcSecondary = axis.toPixel(gridSrcSecondary, gridSize);
        float tgtSecondary = axis.toPixel(gridTgtSecondary, gridSize);

        Point seg1Start = axis.makePoint(orthogonal, srcSecondary);
        Point seg1End = axis.makePoint(orthogonal, tgtSecondary);

        for (const auto& [nodeId, nodeLayout] : nodeLayouts) {
            if (nodeId == excludeFrom || nodeId == excludeTo) continue;
            if (EdgeRouting::segmentIntersectsNode(seg1Start, seg1End, nodeLayout)) {
                return true;
            }
        }
        return false;
    }

    /// Add bend points from grid coordinates (conversion to pixel at the end)
    void addBendPointsFromGrid(
        EdgeLayout& layout,
        int gridOrthogonal,
        int gridSrcSecondary,
        int gridTgtSecondary,
        const AxisConfig& axis,
        float gridSize) {

        layout.bendPoints.push_back({axis.makePointFromGrid(gridOrthogonal, gridSrcSecondary, gridSize)});
        layout.bendPoints.push_back({axis.makePointFromGrid(gridOrthogonal, gridTgtSecondary, gridSize)});
    }

    /// Route edge with same bound constraints using grid-first calculation (Case 1)
    void routeSameBoundCaseGrid(
        EdgeLayout& layout,
        int gridSourceConstraint,
        int gridTargetConstraint,
        int gridSrcSecondary,
        int gridTgtSecondary,
        const AxisConfig& axis,
        const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
        float gridSize) {

        // Try min constraint first
        int gridOrthogonal = std::min(gridSourceConstraint, gridTargetConstraint);

        if (gridPathIntersectsNodes(gridOrthogonal, gridSrcSecondary, gridTgtSecondary,
                                    axis, nodeLayouts, layout.from, layout.to, gridSize)) {
            // Try max constraint
            gridOrthogonal = std::max(gridSourceConstraint, gridTargetConstraint);

            if (gridPathIntersectsNodes(gridOrthogonal, gridSrcSecondary, gridTgtSecondary,
                                        axis, nodeLayouts, layout.from, layout.to, gridSize)) {
                // Find bounds outside all blocking nodes
                int gridMinBound = INT_MIN / 2;  // Avoid overflow
                int gridMaxBound = INT_MAX / 2;

                for (const auto& [nodeId, nodeLayout] : nodeLayouts) {
                    if (nodeId == layout.from || nodeId == layout.to) continue;
                    int nodeMin, nodeMax, nodeSecMin, nodeSecMax;
                    axis.getNodeGridBounds(nodeLayout, gridSize, nodeMin, nodeMax, nodeSecMin, nodeSecMax);
                    gridMinBound = std::max(gridMinBound, nodeMax + GRID_AVOIDANCE_MARGIN);
                    gridMaxBound = std::min(gridMaxBound, nodeMin - GRID_AVOIDANCE_MARGIN);
                }

                // Choose direction requiring less deviation
                int distMin = std::abs(gridOrthogonal - gridMinBound);
                int distMax = std::abs(gridOrthogonal - gridMaxBound);
                gridOrthogonal = (distMin < distMax) ? gridMinBound : gridMaxBound;
            }
        }

        addBendPointsFromGrid(layout, gridOrthogonal, gridSrcSecondary, gridTgtSecondary, axis, gridSize);
    }

    /// Route edge with compatible bound constraints using grid-first calculation (Case 2)
    void routeCompatibleBoundsCaseGrid(
        EdgeLayout& layout,
        int gridLowerBound,
        int gridUpperBound,
        int gridSrcSecondary,
        int gridTgtSecondary,
        const AxisConfig& axis,
        const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
        float gridSize) {

        // Try midpoint first (integer division preserves grid alignment)
        int gridOrthogonal = (gridLowerBound + gridUpperBound) / 2;

        if (gridPathIntersectsNodes(gridOrthogonal, gridSrcSecondary, gridTgtSecondary,
                                    axis, nodeLayouts, layout.from, layout.to, gridSize)) {
            // Try lower bound
            gridOrthogonal = gridLowerBound;

            if (gridPathIntersectsNodes(gridOrthogonal, gridSrcSecondary, gridTgtSecondary,
                                        axis, nodeLayouts, layout.from, layout.to, gridSize)) {
                // Try upper bound
                gridOrthogonal = gridUpperBound;

                if (gridPathIntersectsNodes(gridOrthogonal, gridSrcSecondary, gridTgtSecondary,
                                            axis, nodeLayouts, layout.from, layout.to, gridSize)) {
                    // Find safe coordinate outside all nodes
                    int gridMinBound = INT_MIN / 2;
                    int gridMaxBound = INT_MAX / 2;

                    for (const auto& [nodeId, nodeLayout] : nodeLayouts) {
                        if (nodeId == layout.from || nodeId == layout.to) continue;
                        int nodeMin, nodeMax, nodeSecMin, nodeSecMax;
                        axis.getNodeGridBounds(nodeLayout, gridSize, nodeMin, nodeMax, nodeSecMin, nodeSecMax);
                        gridMinBound = std::max(gridMinBound, nodeMax + GRID_AVOIDANCE_MARGIN);
                        gridMaxBound = std::min(gridMaxBound, nodeMin - GRID_AVOIDANCE_MARGIN);
                    }

                    int distMin = std::abs(gridOrthogonal - gridMinBound);
                    int distMax = std::abs(gridOrthogonal - gridMaxBound);
                    gridOrthogonal = (distMin < distMax) ? gridMinBound : gridMaxBound;
                }
            }
        }

        addBendPointsFromGrid(layout, gridOrthogonal, gridSrcSecondary, gridTgtSecondary, axis, gridSize);
    }

    /// Route edge with conflicting bound constraints using grid-first calculation (Case 3)
    void routeConflictingBoundsCaseGrid(
        EdgeLayout& layout,
        int gridExitCoord,      // lowerBound in original
        int gridApproachCoord,  // upperBound in original
        int gridSrcSecondary,
        int gridTgtSecondary,
        int gridTgtPrimary,
        const AxisConfig& axis,
        const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
        float gridSize) {

        // Check for spike-inducing geometry
        bool wouldCreateSpike = false;
        bool sourceIsPositive = axis.isPositiveEdge(layout.sourceEdge);
        bool targetIsPositive = axis.isPositiveEdge(layout.targetEdge);

        if (sourceIsPositive && !targetIsPositive) {
            wouldCreateSpike = (gridExitCoord >= gridTgtPrimary);
        } else if (!sourceIsPositive && targetIsPositive) {
            wouldCreateSpike = (gridExitCoord <= gridTgtPrimary);
        }

        if (wouldCreateSpike) {
            // Alternative routing: Go perpendicular first
            int gridDetourSecondary;
            if (gridSrcSecondary < gridTgtSecondary) {
                gridDetourSecondary = gridSrcSecondary - GRID_REROUTE_MARGIN;
            } else {
                gridDetourSecondary = gridSrcSecondary + GRID_REROUTE_MARGIN;
            }

            // Create 4-point detour path (convert to pixels)
            layout.bendPoints.push_back({axis.makePointFromGrid(gridExitCoord, gridSrcSecondary, gridSize)});
            layout.bendPoints.push_back({axis.makePointFromGrid(gridExitCoord, gridDetourSecondary, gridSize)});
            layout.bendPoints.push_back({axis.makePointFromGrid(gridApproachCoord, gridDetourSecondary, gridSize)});
            layout.bendPoints.push_back({axis.makePointFromGrid(gridApproachCoord, gridTgtSecondary, gridSize)});
        } else {
            // Check for node intersection on the exit segment
            bool hasIntersection = false;
            int blockingNodeSecMax = 0;

            for (const auto& [nodeId, nodeLayout] : nodeLayouts) {
                if (nodeId == layout.from || nodeId == layout.to) continue;

                Point segStart = axis.makePointFromGrid(gridExitCoord, gridSrcSecondary, gridSize);
                Point segEnd = axis.makePointFromGrid(gridExitCoord, gridTgtSecondary, gridSize);

                if (EdgeRouting::segmentIntersectsNode(segStart, segEnd, nodeLayout)) {
                    hasIntersection = true;
                    int nodeMin, nodeMax, nodeSecMin, nodeSecMax;
                    axis.getNodeGridBounds(nodeLayout, gridSize, nodeMin, nodeMax, nodeSecMin, nodeSecMax);
                    (void)nodeSecMin;  // Only nodeSecMax used for avoidance
                    blockingNodeSecMax = nodeSecMax;
                    break;
                }
            }

            if (hasIntersection) {
                // Route around blocking node
                int gridAvoidSecondary = blockingNodeSecMax + GRID_AVOIDANCE_MARGIN;

                layout.bendPoints.push_back({axis.makePointFromGrid(gridExitCoord, gridSrcSecondary, gridSize)});
                layout.bendPoints.push_back({axis.makePointFromGrid(gridExitCoord, gridAvoidSecondary, gridSize)});
                layout.bendPoints.push_back({axis.makePointFromGrid(gridApproachCoord, gridAvoidSecondary, gridSize)});
                layout.bendPoints.push_back({axis.makePointFromGrid(gridApproachCoord, gridTgtSecondary, gridSize)});
            } else {
                // Simple 3-point path
                layout.bendPoints.push_back({axis.makePointFromGrid(gridExitCoord, gridSrcSecondary, gridSize)});
                layout.bendPoints.push_back({axis.makePointFromGrid(gridExitCoord, gridTgtSecondary, gridSize)});
                layout.bendPoints.push_back({axis.makePointFromGrid(gridApproachCoord, gridTgtSecondary, gridSize)});
            }
        }
    }

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
    
    /// Check if edge needs vertical or horizontal first/last segment
    bool needsVerticalSegment(NodeEdge edge) {
        return edge == NodeEdge::Top || edge == NodeEdge::Bottom;
    }
    
    /// Check if segment is vertical (same X coordinates)
    bool isVerticalSegment(const Point& p1, const Point& p2) {
        return std::abs(p1.x - p2.x) < EPSILON;
    }
    
    /// Check if a point is strictly inside a node (not on boundary)
    bool isPointInsideNode(const Point& pt, const NodeLayout& node) {
        return (pt.x > node.position.x && pt.x < node.position.x + node.size.width &&
                pt.y > node.position.y && pt.y < node.position.y + node.size.height);
    }
    
    /// Check if a point is inside any node in the layout
    /// @param pt Point to check
    /// @param nodeLayouts All node layouts
    /// @param excludeNodes Optional nodes to exclude from checking
    /// @return Pointer to blocking node if found, nullptr otherwise
    const NodeLayout* findBlockingNode(
        const Point& pt,
        const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
        const std::unordered_set<NodeId>& excludeNodes = {}) {
        
        for (const auto& [nodeId, nodeLayout] : nodeLayouts) {
            if (excludeNodes.count(nodeId) > 0) continue;
            if (isPointInsideNode(pt, nodeLayout)) {
                return &nodeLayout;
            }
        }
        return nullptr;
    }
    
    /// Move a point outside a blocking node to the nearest edge
    /// @param pt Point to move (modified in place)
    /// @param node The blocking node
    void movePointOutsideNode(Point& pt, const NodeLayout& node, float gridSize = 0.0f) {
        float nodeLeft = node.position.x;
        float nodeRight = node.position.x + node.size.width;
        float nodeTop = node.position.y;
        float nodeBottom = node.position.y + node.size.height;
        float avoidMargin = getAvoidanceMargin(gridSize);
        
        // Calculate distances to each edge
        float distToLeft = pt.x - nodeLeft;
        float distToRight = nodeRight - pt.x;
        float distToTop = pt.y - nodeTop;
        float distToBottom = nodeBottom - pt.y;
        
        // Find minimum distance and move that way
        float minDist = std::min({distToLeft, distToRight, distToTop, distToBottom});
        
        if (minDist == distToTop) {
            pt.y = nodeTop - avoidMargin;
        } else if (minDist == distToBottom) {
            pt.y = nodeBottom + avoidMargin;
        } else if (minDist == distToLeft) {
            pt.x = nodeLeft - avoidMargin;
        } else {
            pt.x = nodeRight + avoidMargin;
        }
    }
    
    /// Find valid orthogonal intermediate point for non-orthogonal segment
    /// @param prev Previous point
    /// @param curr Current point  
    /// @param nodeLayouts All node layouts for intersection checking
    /// @return Valid intermediate point, or nullopt if neither simple option works
    std::optional<Point> findSimpleOrthogonalIntermediate(
        const Point& prev,
        const Point& curr,
        const std::unordered_map<NodeId, NodeLayout>& nodeLayouts) {
        
        Point option1 = {curr.x, prev.y};  // Horizontal first
        Point option2 = {prev.x, curr.y};  // Vertical first
        
        bool option1Ok = true;
        bool option2Ok = true;
        
        for (const auto& [nodeId, nodeLayout] : nodeLayouts) {
            if (isPointInsideNode(option1, nodeLayout)) option1Ok = false;
            if (isPointInsideNode(option2, nodeLayout)) option2Ok = false;
            if (!option1Ok && !option2Ok) break;
        }
        
        if (option1Ok) return option1;
        if (option2Ok) return option2;
        return std::nullopt;
    }
    
    /// Compute safe Y coordinate for routing around blocking nodes
    /// @param prev Previous point
    /// @param curr Current point
    /// @param nodeLayouts All node layouts
    /// @return Pair of (safeY, success) where success indicates if valid Y was found
    std::pair<float, bool> computeSafeY(
        const Point& prev,
        const Point& curr,
        const std::unordered_map<NodeId, NodeLayout>& nodeLayouts) {
        
        float minSafeY = std::numeric_limits<float>::max();
        float maxSafeY = std::numeric_limits<float>::lowest();
        float minX = std::min(prev.x, curr.x);
        float maxX = std::max(prev.x, curr.x);
        
        for (const auto& [nodeId, nodeLayout] : nodeLayouts) {
            float nodeTop = nodeLayout.position.y;
            float nodeBottom = nodeLayout.position.y + nodeLayout.size.height;
            float nodeLeft = nodeLayout.position.x;
            float nodeRight = nodeLayout.position.x + nodeLayout.size.width;
            
            // If node overlaps X range, we need to avoid its Y range
            if (nodeRight > minX && nodeLeft < maxX) {
                minSafeY = std::min(minSafeY, nodeTop);
                maxSafeY = std::max(maxSafeY, nodeBottom);
            }
        }
        
        if (minSafeY == std::numeric_limits<float>::max()) {
            return {0.0f, false};  // No blocking nodes
        }
        
        // Choose safe Y: go above or below all blocking nodes
        float distToAbove = std::abs(prev.y - (minSafeY - SAFE_MARGIN));
        float distToBelow = std::abs(prev.y - (maxSafeY + SAFE_MARGIN));
        
        float safeY = (distToAbove < distToBelow) ? (minSafeY - SAFE_MARGIN) : (maxSafeY + SAFE_MARGIN);
        return {safeY, true};
    }
    
    /// Compute safe X coordinate for routing around blocking nodes
    /// @param prev Previous point
    /// @param curr Current point
    /// @param nodeLayouts All node layouts
    /// @return Pair of (safeX, success) where success indicates if valid X was found
    std::pair<float, bool> computeSafeX(
        const Point& prev,
        const Point& curr,
        const std::unordered_map<NodeId, NodeLayout>& nodeLayouts) {
        
        float minSafeX = std::numeric_limits<float>::max();
        float maxSafeX = std::numeric_limits<float>::lowest();
        float minY = std::min(prev.y, curr.y);
        float maxY = std::max(prev.y, curr.y);
        
        for (const auto& [nodeId, nodeLayout] : nodeLayouts) {
            float nodeLeft = nodeLayout.position.x;
            float nodeRight = nodeLayout.position.x + nodeLayout.size.width;
            float nodeTop = nodeLayout.position.y;
            float nodeBottom = nodeLayout.position.y + nodeLayout.size.height;
            
            // If node overlaps Y range, we need to avoid its X range
            if (nodeBottom > minY && nodeTop < maxY) {
                minSafeX = std::min(minSafeX, nodeLeft);
                maxSafeX = std::max(maxSafeX, nodeRight);
            }
        }
        
        if (minSafeX == std::numeric_limits<float>::max()) {
            return {0.0f, false};  // No blocking nodes
        }
        
        float distToLeft = std::abs(prev.x - (minSafeX - SAFE_MARGIN));
        float distToRight = std::abs(prev.x - (maxSafeX + SAFE_MARGIN));
        
        float safeX = (distToLeft < distToRight) ? (minSafeX - SAFE_MARGIN) : (maxSafeX + SAFE_MARGIN);
        return {safeX, true};
    }
    
    /// Get ranges of existing segments at a given coordinate
    /// For horizontal segments at Y, returns X ranges
    /// For vertical segments at X, returns Y ranges
    std::vector<std::pair<float, float>> getSegmentRanges(
        const std::vector<Point>& points,
        float coord,
        bool horizontal) {
        std::vector<std::pair<float, float>> ranges;
        for (size_t j = 1; j < points.size(); ++j) {
            const Point& p1 = points[j - 1];
            const Point& p2 = points[j];
            if (horizontal) {
                // Looking for horizontal segments at Y=coord
                if (std::abs(p1.y - p2.y) < EPSILON && std::abs(p1.y - coord) < EPSILON) {
                    ranges.push_back({std::min(p1.x, p2.x), std::max(p1.x, p2.x)});
                }
            } else {
                // Looking for vertical segments at X=coord
                if (std::abs(p1.x - p2.x) < EPSILON && std::abs(p1.x - coord) < EPSILON) {
                    ranges.push_back({std::min(p1.y, p2.y), std::max(p1.y, p2.y)});
                }
            }
        }
        return ranges;
    }
    
    /// Check if a range overlaps with any of the given ranges
    bool rangesOverlap(const std::vector<std::pair<float, float>>& ranges, float minVal, float maxVal) {
        for (const auto& range : ranges) {
            if (std::max(minVal, range.first) < std::min(maxVal, range.second) - EPSILON) {
                return true;
            }
        }
        return false;
    }
    
    /// Find a safe coordinate that doesn't overlap with existing segments
    float findSafeCoordinate(
        const std::vector<Point>& points,
        float preferredCoord,
        float rangeMin,
        float rangeMax,
        bool horizontal) {
        
        auto ranges = getSegmentRanges(points, preferredCoord, horizontal);
        if (ranges.empty()) return preferredCoord;
        
        if (!rangesOverlap(ranges, rangeMin, rangeMax)) return preferredCoord;
        
        // Try shifting away from existing segments
        float rightCoord = preferredCoord + SAFE_MARGIN;
        float leftCoord = preferredCoord - SAFE_MARGIN;
        
        for (int attempt = 0; attempt < 10; ++attempt) {
            auto rightRanges = getSegmentRanges(points, rightCoord, horizontal);
            if (!rangesOverlap(rightRanges, rangeMin, rangeMax)) return rightCoord;
            
            auto leftRanges = getSegmentRanges(points, leftCoord, horizontal);
            if (!rangesOverlap(leftRanges, rangeMin, rangeMax)) return leftCoord;
            
            rightCoord += SAFE_MARGIN;
            leftCoord -= SAFE_MARGIN;
        }
        
        return preferredCoord;
    }
    
    /// Check if a proposed segment would overlap with existing path segments
    /// @param points The path points to check against
    /// @param newP1 Start of proposed segment
    /// @param newP2 End of proposed segment
    /// @param skipSegmentIdx Index of segment to skip (the one being replaced)
    /// @return True if overlap detected
    bool wouldSegmentsOverlap(
        const std::vector<Point>& points,
        const Point& newP1,
        const Point& newP2,
        int skipSegmentIdx) {
        
        bool newIsVertical = std::abs(newP1.x - newP2.x) < EPSILON;
        bool newIsHorizontal = std::abs(newP1.y - newP2.y) < EPSILON;
        
        for (int j = 1; j < static_cast<int>(points.size()); ++j) {
            if (j == skipSegmentIdx) continue;
            
            const Point& existP1 = points[j - 1];
            const Point& existP2 = points[j];
            
            bool existIsVertical = std::abs(existP1.x - existP2.x) < EPSILON;
            bool existIsHorizontal = std::abs(existP1.y - existP2.y) < EPSILON;
            
            // Check if both segments are on the same vertical line
            if (newIsVertical && existIsVertical && std::abs(newP1.x - existP1.x) < EPSILON) {
                float newMin = std::min(newP1.y, newP2.y);
                float newMax = std::max(newP1.y, newP2.y);
                float existMin = std::min(existP1.y, existP2.y);
                float existMax = std::max(existP1.y, existP2.y);
                
                float overlapStart = std::max(newMin, existMin);
                float overlapEnd = std::min(newMax, existMax);
                if (overlapEnd - overlapStart > EPSILON) {
                    return true;
                }
            }
            
            // Check if both segments are on the same horizontal line
            if (newIsHorizontal && existIsHorizontal && std::abs(newP1.y - existP1.y) < EPSILON) {
                float newMin = std::min(newP1.x, newP2.x);
                float newMax = std::max(newP1.x, newP2.x);
                float existMin = std::min(existP1.x, existP2.x);
                float existMax = std::max(existP1.x, existP2.x);
                
                float overlapStart = std::max(newMin, existMin);
                float overlapEnd = std::min(newMax, existMax);
                if (overlapEnd - overlapStart > EPSILON) {
                    return true;
                }
            }
        }
        return false;
    }
    
    /// Check if an avoidance path around a blocking node would intersect any nodes
    /// @param p1 Start point of original segment
    /// @param p2 End point of original segment
    /// @param avoidCoord The coordinate to route around (X for vertical segments, Y for horizontal)
    /// @param isVerticalSegment True if original segment is vertical (needs horizontal avoidance)
    /// @param nodeLayouts All node layouts to check against
    /// @return True if avoidance path is clear (no intersections)
    bool checkAvoidancePathClear(
        const Point& p1,
        const Point& p2,
        float avoidCoord,
        bool isVerticalSegment,
        const std::unordered_map<NodeId, NodeLayout>& nodeLayouts) {
        
        Point seg1Start, seg1End, seg2Start, seg2End, seg3Start, seg3End;
        
        if (isVerticalSegment) {
            // Vertical segment: create horizontal-vertical-horizontal path
            seg1Start = p1;
            seg1End = {avoidCoord, p1.y};
            seg2Start = {avoidCoord, p1.y};
            seg2End = {avoidCoord, p2.y};
            seg3Start = {avoidCoord, p2.y};
            seg3End = p2;
        } else {
            // Horizontal segment: create vertical-horizontal-vertical path
            seg1Start = p1;
            seg1End = {p1.x, avoidCoord};
            seg2Start = {p1.x, avoidCoord};
            seg2End = {p2.x, avoidCoord};
            seg3Start = {p2.x, avoidCoord};
            seg3End = p2;
        }
        
        for (const auto& [nodeId, nodeLayout] : nodeLayouts) {
            if (EdgeRouting::segmentIntersectsNode(seg1Start, seg1End, nodeLayout) ||
                EdgeRouting::segmentIntersectsNode(seg2Start, seg2End, nodeLayout) ||
                EdgeRouting::segmentIntersectsNode(seg3Start, seg3End, nodeLayout)) {
                return false;
            }
        }
        return true;
    }
    
    /// Calculate safe coordinate for rerouting around a node
    /// @param nodeMin Minimum bound of node (left or top)
    /// @param nodeMax Maximum bound of node (right or bottom)
    /// @param referenceCoord Reference coordinate to decide which side to go
    /// @param goToMin True to go toward min side, false to go toward max side
    /// @param gridSize Grid cell size for grid-aligned offsets
    float calculateSafeCoordinate(float nodeMin, float nodeMax, float referenceCoord, bool goToMin, float gridSize = 0.0f) {
        float nodeCenter = (nodeMin + nodeMax) / 2;
        float rerouteMargin = getRerouteMargin(gridSize);
        if (goToMin || referenceCoord < nodeCenter) {
            return nodeMin - rerouteMargin;
        } else {
            return nodeMax + rerouteMargin;
        }
    }
    
    /// Insert reroute points at the beginning of path (after source point)
    /// Used when source exit direction is wrong
    void insertSourceReroutePoints(
        std::vector<Point>& points,
        const Point& p1,
        const Point& p2,
        const Point& p3) {
        points.insert(points.begin() + 1, p3);
        points.insert(points.begin() + 1, p2);
        points.insert(points.begin() + 1, p1);
    }
    
    /// Insert reroute points at the end of path (before target point)
    /// Used when target entry direction is wrong
    void insertTargetReroutePoints(
        std::vector<Point>& points,
        const Point& p1,
        const Point& p2,
        const Point& p3) {
        points.insert(points.end() - 1, p3);
        points.insert(points.end() - 2, p2);
        points.insert(points.end() - 3, p1);
    }
    
    /// Correct source exit direction when orientation is wrong
    /// @param points Path points to modify
    /// @param sourceEdge Which edge of source node
    /// @param gridSize Grid cell size for grid-aligned offsets
    void correctSourceOrientation(
        std::vector<Point>& points,
        NodeEdge sourceEdge,
        float gridSize = 0.0f) {
        
        if (points.size() < 2) return;
        
        const Point& srcPt = points[0];
        const Point& firstBend = points[1];
        
        bool needsVertical = needsVerticalSegment(sourceEdge);
        bool isVertical = isVerticalSegment(srcPt, firstBend);
        float dirOffset = getDirectionOffset(gridSize);
        
        if (needsVertical && !isVertical) {
            // Need to insert a vertical segment before the horizontal one
            float newY = (sourceEdge == NodeEdge::Top) ?
                         (srcPt.y - dirOffset) : (srcPt.y + dirOffset);
            Point newBend1 = {srcPt.x, newY};
            Point newBend2 = {firstBend.x, newY};
            points.insert(points.begin() + 1, newBend2);
            points.insert(points.begin() + 1, newBend1);
        } else if (!needsVertical && isVertical) {
            // Need to insert a horizontal segment before the vertical one
            float newX = (sourceEdge == NodeEdge::Left) ?
                         (srcPt.x - dirOffset) : (srcPt.x + dirOffset);
            Point newBend1 = {newX, srcPt.y};
            Point newBend2 = {newX, firstBend.y};
            points.insert(points.begin() + 1, newBend2);
            points.insert(points.begin() + 1, newBend1);
        }
    }
    
    /// Correct source exit direction when direction is wrong (correct orientation but wrong direction)
    /// @param points Path points to modify
    /// @param sourceEdge Which edge of source node
    /// @param sourceNode Layout of source node
    /// @param gridSize Grid cell size for grid-aligned offsets
    void correctSourceDirection(
        std::vector<Point>& points,
        NodeEdge sourceEdge,
        const NodeLayout& sourceNode,
        float gridSize = 0.0f) {
        
        if (points.size() < 2) return;
        
        const Point& srcPt = points[0];
        const Point& firstBend = points[1];
        float dx = firstBend.x - srcPt.x;
        float dy = firstBend.y - srcPt.y;
        
        bool needsVertical = needsVerticalSegment(sourceEdge);
        bool isVertical = isVerticalSegment(srcPt, firstBend);
        
        // Only handle case where orientation is correct but direction is wrong
        if (needsVertical != isVertical) return;
        
        float nodeTop = sourceNode.position.y;
        float nodeBottom = sourceNode.position.y + sourceNode.size.height;
        float nodeLeft = sourceNode.position.x;
        float nodeRight = sourceNode.position.x + sourceNode.size.width;
        float rerouteMargin = getRerouteMargin(gridSize);
        
        if (needsVertical) {
            // Check vertical direction: Top -> up (dy < 0), Bottom -> down (dy > 0)
            bool directionCorrect = (sourceEdge == NodeEdge::Top) ?
                                    (dy < -EPSILON) : (dy > EPSILON);
            if (directionCorrect) return;
            
            float safeY = (sourceEdge == NodeEdge::Top) ?
                          (nodeTop - rerouteMargin) : (nodeBottom + rerouteMargin);
            float safeX = calculateSafeCoordinate(nodeLeft, nodeRight, firstBend.x, false, gridSize);
            
            insertSourceReroutePoints(points,
                {srcPt.x, safeY},
                {safeX, safeY},
                {safeX, firstBend.y});
        } else {
            // Check horizontal direction: Left -> left (dx < 0), Right -> right (dx > 0)
            bool directionCorrect = (sourceEdge == NodeEdge::Left) ?
                                    (dx < -EPSILON) : (dx > EPSILON);
            if (directionCorrect) return;
            
            float safeX = (sourceEdge == NodeEdge::Left) ?
                          (nodeLeft - rerouteMargin) : (nodeRight + rerouteMargin);
            float safeY = calculateSafeCoordinate(nodeTop, nodeBottom, firstBend.y, false, gridSize);
            
            insertSourceReroutePoints(points,
                {safeX, srcPt.y},
                {safeX, safeY},
                {firstBend.x, safeY});
        }
    }
    
    /// Correct target entry direction when orientation is wrong
    /// @param points Path points to modify
    /// @param targetEdge Which edge of target node
    /// @param gridSize Grid cell size for grid-aligned offsets
    void correctTargetOrientation(
        std::vector<Point>& points,
        NodeEdge targetEdge,
        float gridSize = 0.0f) {
        
        if (points.size() < 2) return;
        
        size_t lastIdx = points.size() - 1;
        const Point& lastBend = points[lastIdx - 1];
        const Point& tgtPt = points[lastIdx];
        
        bool needsVertical = needsVerticalSegment(targetEdge);
        bool isVertical = isVerticalSegment(lastBend, tgtPt);
        float dirOffset = getDirectionOffset(gridSize);
        
        if (needsVertical && !isVertical) {
            // Need to insert bend points to make last segment vertical
            float newY = (targetEdge == NodeEdge::Top) ?
                         (tgtPt.y - dirOffset) : (tgtPt.y + dirOffset);
            Point newBend1 = {lastBend.x, newY};
            Point newBend2 = {tgtPt.x, newY};
            points[lastIdx - 1] = newBend1;
            points.insert(points.end() - 1, newBend2);
        } else if (!needsVertical && isVertical) {
            // Need to insert bend points to make last segment horizontal
            float newX = (targetEdge == NodeEdge::Left) ?
                         (tgtPt.x - dirOffset) : (tgtPt.x + dirOffset);
            Point newBend1 = {newX, lastBend.y};
            Point newBend2 = {newX, tgtPt.y};
            points[lastIdx - 1] = newBend1;
            points.insert(points.end() - 1, newBend2);
        }
    }
    
    /// Correct target entry direction when direction is wrong (correct orientation but wrong direction)
    /// @param points Path points to modify
    /// @param targetEdge Which edge of target node
    /// @param targetNode Layout of target node
    /// @param gridSize Grid cell size for grid-aligned offsets
    void correctTargetDirection(
        std::vector<Point>& points,
        NodeEdge targetEdge,
        const NodeLayout& targetNode,
        float gridSize = 0.0f) {
        
        if (points.size() < 2) return;
        
        size_t lastIdx = points.size() - 1;
        const Point& lastBend = points[lastIdx - 1];
        const Point& tgtPt = points[lastIdx];
        float dx = tgtPt.x - lastBend.x;
        float dy = tgtPt.y - lastBend.y;
        
        bool needsVertical = needsVerticalSegment(targetEdge);
        bool isVertical = isVerticalSegment(lastBend, tgtPt);
        
        // Only handle case where orientation is correct but direction is wrong
        if (needsVertical != isVertical) return;
        
        float nodeTop = targetNode.position.y;
        float nodeBottom = targetNode.position.y + targetNode.size.height;
        float nodeLeft = targetNode.position.x;
        float nodeRight = targetNode.position.x + targetNode.size.width;
        float rerouteMargin = getRerouteMargin(gridSize);
        
        if (needsVertical) {
            // Check vertical direction: Top -> down (dy > 0), Bottom -> up (dy < 0)
            bool directionCorrect = (targetEdge == NodeEdge::Top) ?
                                    (dy > EPSILON) : (dy < -EPSILON);
            if (directionCorrect) return;
            
            float safeY = (targetEdge == NodeEdge::Top) ?
                          (nodeTop - rerouteMargin) : (nodeBottom + rerouteMargin);
            float safeX = calculateSafeCoordinate(nodeLeft, nodeRight, lastBend.x, false, gridSize);
            
            insertTargetReroutePoints(points,
                {safeX, lastBend.y},
                {safeX, safeY},
                {tgtPt.x, safeY});
        } else {
            // Check horizontal direction: Left -> right (dx > 0), Right -> left (dx < 0)
            bool directionCorrect = (targetEdge == NodeEdge::Left) ?
                                    (dx > EPSILON) : (dx < -EPSILON);
            if (directionCorrect) return;
            
            float safeX = (targetEdge == NodeEdge::Left) ?
                          (nodeLeft - rerouteMargin) : (nodeRight + rerouteMargin);
            float safeY = calculateSafeCoordinate(nodeTop, nodeBottom, lastBend.y, false, gridSize);
            
            insertTargetReroutePoints(points,
                {lastBend.x, safeY},
                {safeX, safeY},
                {safeX, tgtPt.y});
        }
    }
}

// =============================================================================
// Static Helper Function Implementations
// =============================================================================

Point EdgeRouting::calculateSnapPosition(const NodeLayout& node, NodeEdge edge, float position) {
    return LayoutUtils::calculateSnapPointFromPosition(node, edge, position);
}

float EdgeRouting::calculateRelativePosition(int snapIdx, int count, float rangeStart, float rangeEnd) {
    // Delegate to SnapIndexManager for centralized logic
    SnapRange range{rangeStart, rangeEnd};
    return SnapIndexManager::calculatePosition(snapIdx, count, range);
}

int EdgeRouting::unifiedToLocalIndex(int unifiedIdx, int offset, int count) {
    // Delegate to SnapIndexManager for centralized logic
    return SnapIndexManager::unifiedToLocal(unifiedIdx, offset, count);
}

// =============================================================================
// recalculateBendPoints Helper Methods (SRP Decomposition)
// =============================================================================

void EdgeRouting::performChannelBasedRouting(
    EdgeLayout& layout,
    bool isHorizontal,
    const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
    float gridSize) {

    float effectiveGridSize = getEffectiveGridSize(gridSize);

    AxisConfig axis{isHorizontal};

    // Convert all coordinates to grid units at the start
    int gridChannel = axis.toGrid(layout.channelY, effectiveGridSize);
    int gridSrcPrimary = axis.primaryGrid(layout.sourcePoint, effectiveGridSize);
    int gridTgtPrimary = axis.primaryGrid(layout.targetPoint, effectiveGridSize);
    int gridSrcSecondary = axis.secondaryGrid(layout.sourcePoint, effectiveGridSize);
    int gridTgtSecondary = axis.secondaryGrid(layout.targetPoint, effectiveGridSize);

    // Calculate perpendicular offset in grid units
    // When grid is disabled (effectiveGridSize = 1.0f), this ensures minimum pixel clearance
    int gridOffset = static_cast<int>(std::ceil(MIN_PERPENDICULAR_OFFSET / effectiveGridSize));

    // Apply directional constraints in grid units
    int gridSourceConstraint = applyDirectionalConstraintGrid(gridChannel, gridSrcPrimary, layout.sourceEdge, gridOffset);
    int gridTargetConstraint = applyDirectionalConstraintGrid(gridChannel, gridTgtPrimary, layout.targetEdge, gridOffset);

    bool sourceIsLowerBound = axis.isLowerBound(layout.sourceEdge);
    bool targetIsLowerBound = axis.isLowerBound(layout.targetEdge);

    if (sourceIsLowerBound == targetIsLowerBound) {
        routeSameBoundCaseGrid(layout, gridSourceConstraint, gridTargetConstraint,
                               gridSrcSecondary, gridTgtSecondary, axis, nodeLayouts, effectiveGridSize);
    } else {
        int gridLowerBound = sourceIsLowerBound ? gridSourceConstraint : gridTargetConstraint;
        int gridUpperBound = sourceIsLowerBound ? gridTargetConstraint : gridSourceConstraint;

        if (gridLowerBound <= gridUpperBound) {
            routeCompatibleBoundsCaseGrid(layout, gridLowerBound, gridUpperBound,
                                          gridSrcSecondary, gridTgtSecondary, axis, nodeLayouts, effectiveGridSize);
        } else {
            routeConflictingBoundsCaseGrid(layout, gridLowerBound, gridUpperBound,
                                           gridSrcSecondary, gridTgtSecondary, gridTgtPrimary,
                                           axis, nodeLayouts, effectiveGridSize);
        }
    }
}

void EdgeRouting::performFallbackRouting(
    EdgeLayout& layout,
    bool isHorizontal,
    const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
    float gridSize) {

    float effectiveGridSize = getEffectiveGridSize(gridSize);

    AxisConfig axis{isHorizontal};

    // Convert all coordinates to grid units at the start
    int gridSrcPrimary = axis.primaryGrid(layout.sourcePoint, effectiveGridSize);
    int gridTgtPrimary = axis.primaryGrid(layout.targetPoint, effectiveGridSize);
    int gridSrcSecondary = axis.secondaryGrid(layout.sourcePoint, effectiveGridSize);
    int gridTgtSecondary = axis.secondaryGrid(layout.targetPoint, effectiveGridSize);

    // Calculate midpoint in grid units (integer division preserves alignment)
    int gridMid = (gridSrcPrimary + gridTgtPrimary) / 2;

    // Calculate perpendicular offset in grid units
    // When grid is disabled (effectiveGridSize = 1.0f), this ensures minimum pixel clearance
    int gridOffset = static_cast<int>(std::ceil(MIN_PERPENDICULAR_OFFSET / effectiveGridSize));

    // Apply directional constraints in grid units
    int gridSourceConstraint = applyDirectionalConstraintGrid(gridMid, gridSrcPrimary, layout.sourceEdge, gridOffset);
    int gridTargetConstraint = applyDirectionalConstraintGrid(gridMid, gridTgtPrimary, layout.targetEdge, gridOffset);

    routeSameBoundCaseGrid(layout, gridSourceConstraint, gridTargetConstraint,
                           gridSrcSecondary, gridTgtSecondary, axis, nodeLayouts, effectiveGridSize);
}

void EdgeRouting::moveIntermediatePointsOutsideNodes(
    std::vector<Point>& allPoints,
    const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
    float gridSize) {

    (void)gridSize;  // Reserved for future grid-aligned movement
    for (size_t idx = 1; idx + 1 < allPoints.size(); ++idx) {
        Point& pt = allPoints[idx];
        const NodeLayout* blockingNode = findBlockingNode(pt, nodeLayouts);
        if (blockingNode) {
            movePointOutsideNode(pt, *blockingNode, gridSize);
        }
    }
}

void EdgeRouting::fixNonOrthogonalSegments(
    std::vector<Point>& allPoints,
    const std::unordered_map<NodeId, NodeLayout>& nodeLayouts) {

    for (size_t idx = 1; idx < allPoints.size(); ++idx) {
        const Point& prev = allPoints[idx - 1];
        Point& curr = allPoints[idx];

        bool isHoriz = std::abs(prev.y - curr.y) < EPSILON;
        bool isVert = std::abs(prev.x - curr.x) < EPSILON;

        if (!isHoriz && !isVert) {
            // Non-orthogonal segment - try simple orthogonal intermediates first
            auto simpleIntermediate = findSimpleOrthogonalIntermediate(prev, curr, nodeLayouts);

            if (simpleIntermediate) {
                allPoints.insert(allPoints.begin() + idx, *simpleIntermediate);
                idx++;
            } else {
                // Both simple options inside nodes - try Y-based routing
                auto [safeY, foundY] = computeSafeY(prev, curr, nodeLayouts);

                if (foundY) {
                    Point pt1 = {prev.x, safeY};
                    Point pt2 = {curr.x, safeY};

                    if (!findBlockingNode(pt1, nodeLayouts) && !findBlockingNode(pt2, nodeLayouts)) {
                        allPoints.insert(allPoints.begin() + idx, pt2);
                        allPoints.insert(allPoints.begin() + idx, pt1);
                        idx += 2;
                        continue;
                    }
                }

                // Last resort: try X-based routing
                auto [safeX, foundX] = computeSafeX(prev, curr, nodeLayouts);
                (void)foundX;  // Always use the result
                Point ptA = {safeX, prev.y};
                Point ptB = {safeX, curr.y};

                allPoints.insert(allPoints.begin() + idx, ptB);
                allPoints.insert(allPoints.begin() + idx, ptA);
                idx += 2;
            }
        }
    }
}

void EdgeRouting::fixSegmentNodeIntersections(
    std::vector<Point>& allPoints,
    const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
    float gridSize) {

    for (int pass = 0; pass < MAX_FIX_PASSES && allPoints.size() < MAX_PATH_POINTS; ++pass) {
        bool foundIntersection = false;

        for (int i = 1; i < static_cast<int>(allPoints.size()); ++i) {
            const Point p1 = allPoints[i - 1];
            const Point p2 = allPoints[i];

            NodeLayout blockingNode = nodeLayouts.begin()->second;
            bool segmentHasIntersection = false;

            for (const auto& [nodeId, nodeLayout] : nodeLayouts) {
                if (segmentIntersectsNode(p1, p2, nodeLayout)) {
                    blockingNode = nodeLayout;
                    segmentHasIntersection = true;
                    break;
                }
            }

            if (segmentHasIntersection) {
                bool isVerticalSeg = std::abs(p1.x - p2.x) < 0.1f;

                // Get target point to determine optimal avoidance direction
                const Point& targetPoint = allPoints.back();

                if (isVerticalSeg) {
                    // Vertical segment: go around horizontally
                    // Calculate both avoidance coordinates and pick the one closer to target
                    float avoidXRight = calculateAvoidanceCoordinate(
                        blockingNode.position.x,
                        blockingNode.position.x + blockingNode.size.width,
                        true, gridSize);
                    float avoidXLeft = calculateAvoidanceCoordinate(
                        blockingNode.position.x,
                        blockingNode.position.x + blockingNode.size.width,
                        false, gridSize);

                    // Prefer the direction that results in less total distance to target
                    float distRight = std::abs(avoidXRight - targetPoint.x);
                    float distLeft = std::abs(avoidXLeft - targetPoint.x);
                    bool preferRight = distRight <= distLeft;

                    float avoidXFirst = preferRight ? avoidXRight : avoidXLeft;
                    float avoidXSecond = preferRight ? avoidXLeft : avoidXRight;

                    float avoidX = avoidXFirst;
                    bool firstWorks = checkAvoidancePathClear(p1, p2, avoidX, true, nodeLayouts);

                    if (!firstWorks) {
                        avoidX = avoidXSecond;

                        if (!checkAvoidancePathClear(p1, p2, avoidX, true, nodeLayouts)) {
                            // 4-point path around the node
                            float safeY = calculateAvoidanceCoordinate(
                                blockingNode.position.y,
                                blockingNode.position.y + blockingNode.size.height,
                                p1.y < blockingNode.position.y, gridSize);

                            float safeX = calculateAvoidanceCoordinate(
                                blockingNode.position.x,
                                blockingNode.position.x + blockingNode.size.width,
                                true, gridSize);

                            Point newPt1 = {safeX, p1.y};
                            Point newPt2 = {safeX, safeY};
                            Point newPt3 = {p2.x, safeY};

                            bool wouldOverlap =
                                wouldSegmentsOverlap(allPoints, p1, newPt1, i) ||
                                wouldSegmentsOverlap(allPoints, newPt1, newPt2, i) ||
                                wouldSegmentsOverlap(allPoints, newPt2, newPt3, i) ||
                                wouldSegmentsOverlap(allPoints, newPt3, p2, i);

                            if (!wouldOverlap) {
                                allPoints.insert(allPoints.begin() + i, {newPt1});
                                allPoints.insert(allPoints.begin() + i + 1, {newPt2});
                                allPoints.insert(allPoints.begin() + i + 2, {newPt3});
                                foundIntersection = true;
                            }
                            break;
                        }
                    }

                    float safeAvoidX = findSafeCoordinate(allPoints, avoidX, std::min(p1.y, p2.y), std::max(p1.y, p2.y), false);

                    bool safeXAvoidsNode = (safeAvoidX > blockingNode.position.x + blockingNode.size.width + SAFE_TOLERANCE) ||
                                           (safeAvoidX < blockingNode.position.x - SAFE_TOLERANCE);

                    if (safeXAvoidsNode) {
                        Point newPtA = {safeAvoidX, p1.y};
                        Point newPtB = {safeAvoidX, p2.y};

                        bool segmentsOk = true;
                        for (const auto& [nodeId, nodeLayout] : nodeLayouts) {
                            if (segmentIntersectsNode(p1, newPtA, nodeLayout) ||
                                segmentIntersectsNode(newPtA, newPtB, nodeLayout) ||
                                segmentIntersectsNode(newPtB, p2, nodeLayout)) {
                                segmentsOk = false;
                                break;
                            }
                        }

                        if (segmentsOk) {
                            allPoints.insert(allPoints.begin() + i, {newPtA});
                            allPoints.insert(allPoints.begin() + i + 1, {newPtB});
                            foundIntersection = true;
                        }
                    }
                } else {
                    // Horizontal segment: go around vertically
                    // Calculate both avoidance coordinates and pick the one closer to target
                    float avoidYDown = calculateAvoidanceCoordinate(
                        blockingNode.position.y,
                        blockingNode.position.y + blockingNode.size.height,
                        true, gridSize);
                    float avoidYUp = calculateAvoidanceCoordinate(
                        blockingNode.position.y,
                        blockingNode.position.y + blockingNode.size.height,
                        false, gridSize);

                    // Prefer the direction that results in less total distance to target
                    float distDown = std::abs(avoidYDown - targetPoint.y);
                    float distUp = std::abs(avoidYUp - targetPoint.y);
                    bool preferDown = distDown <= distUp;

                    float avoidYFirst = preferDown ? avoidYDown : avoidYUp;
                    float avoidYSecond = preferDown ? avoidYUp : avoidYDown;

                    float avoidY = avoidYFirst;
                    bool firstWorks = checkAvoidancePathClear(p1, p2, avoidY, false, nodeLayouts);

                    if (!firstWorks) {
                        avoidY = avoidYSecond;

                        if (!checkAvoidancePathClear(p1, p2, avoidY, false, nodeLayouts)) {
                            float avoidX = calculateAvoidanceCoordinate(
                                blockingNode.position.x,
                                blockingNode.position.x + blockingNode.size.width,
                                p1.x < blockingNode.position.x, gridSize);

                            float safeY = calculateAvoidanceCoordinate(
                                blockingNode.position.y,
                                blockingNode.position.y + blockingNode.size.height,
                                true, gridSize);

                            Point newPt1 = {p1.x, safeY};
                            Point newPt2 = {avoidX, safeY};
                            Point newPt3 = {avoidX, p2.y};

                            bool wouldOverlap =
                                wouldSegmentsOverlap(allPoints, p1, newPt1, i) ||
                                wouldSegmentsOverlap(allPoints, newPt1, newPt2, i) ||
                                wouldSegmentsOverlap(allPoints, newPt2, newPt3, i) ||
                                wouldSegmentsOverlap(allPoints, newPt3, p2, i);

                            if (!wouldOverlap) {
                                allPoints.insert(allPoints.begin() + i, {newPt1});
                                allPoints.insert(allPoints.begin() + i + 1, {newPt2});
                                allPoints.insert(allPoints.begin() + i + 2, {newPt3});
                                foundIntersection = true;
                            }
                            break;
                        }
                    }

                    float safeAvoidY = findSafeCoordinate(allPoints, avoidY, std::min(p1.x, p2.x), std::max(p1.x, p2.x), true);

                    bool safeYAvoidsNode = (safeAvoidY > blockingNode.position.y + blockingNode.size.height + SAFE_TOLERANCE) ||
                                           (safeYAvoidsNode < blockingNode.position.y - SAFE_TOLERANCE);

                    if (safeYAvoidsNode) {
                        Point newPtA = {p1.x, safeAvoidY};
                        Point newPtB = {p2.x, safeAvoidY};

                        bool segmentsOk = true;
                        for (const auto& [nodeId, nodeLayout] : nodeLayouts) {
                            if (segmentIntersectsNode(p1, newPtA, nodeLayout) ||
                                segmentIntersectsNode(newPtA, newPtB, nodeLayout) ||
                                segmentIntersectsNode(newPtB, p2, nodeLayout)) {
                                segmentsOk = false;
                                break;
                            }
                        }

                        if (segmentsOk) {
                            allPoints.insert(allPoints.begin() + i, {newPtA});
                            allPoints.insert(allPoints.begin() + i + 1, {newPtB});
                            foundIntersection = true;
                        }
                    }
                }

                if (foundIntersection) {
                    break;
                }
            }
        }

        if (!foundIntersection) break;
    }
}

void EdgeRouting::fixIntermediateNodeIntersections(
    std::vector<Point>& allPoints,
    const EdgeLayout& layout,
    const std::unordered_map<NodeId, NodeLayout>& nodeLayouts) {

    for (int maxIter = 0; maxIter < 10; ++maxIter) {
        bool foundNodeIntersection = false;

        for (size_t i = 0; i + 1 < allPoints.size() && !foundNodeIntersection; ++i) {
            const Point& p1 = allPoints[i];
            const Point& p2 = allPoints[i + 1];

            bool isHoriz = std::abs(p1.y - p2.y) < EPSILON;
            bool isVert = std::abs(p1.x - p2.x) < EPSILON;

            for (const auto& [nodeId, nodeLayout] : nodeLayouts) {
                if (i == 0 && nodeId == layout.from) continue;
                if (i == allPoints.size() - 2 && nodeId == layout.to) continue;

                float nodeXmin = nodeLayout.position.x;
                float nodeXmax = nodeLayout.position.x + nodeLayout.size.width;
                float nodeYmin = nodeLayout.position.y;
                float nodeYmax = nodeLayout.position.y + nodeLayout.size.height;

                bool intersects = false;

                if (isVert) {
                    float x = p1.x;
                    float segYmin = std::min(p1.y, p2.y);
                    float segYmax = std::max(p1.y, p2.y);
                    intersects = (x > nodeXmin && x < nodeXmax && segYmin < nodeYmax && segYmax > nodeYmin);
                } else if (isHoriz) {
                    float y = p1.y;
                    float segXmin = std::min(p1.x, p2.x);
                    float segXmax = std::max(p1.x, p2.x);
                    intersects = (y > nodeYmin && y < nodeYmax && segXmin < nodeXmax && segXmax > nodeXmin);
                }

                if (intersects) {
                    if (isHoriz) {
                        float safeY = (p1.y < nodeYmin) ? (nodeYmin - SAFE_MARGIN) : (nodeYmax + SAFE_MARGIN);
                        bool fromLeft = (std::min(p1.x, p2.x) < nodeXmin);
                        bool fromRight = (std::max(p1.x, p2.x) > nodeXmax);

                        if (fromLeft && fromRight) {
                            float avoidX1 = nodeXmin - SAFE_MARGIN;
                            float avoidX2 = nodeXmax + SAFE_MARGIN;
                            float detourX = (std::abs(p1.x - avoidX1) < std::abs(p1.x - avoidX2)) ? avoidX1 : avoidX2;

                            Point newPt1 = {detourX, p1.y};
                            Point newPt2 = {detourX, safeY};
                            Point newPt3 = {p2.x, safeY};

                            allPoints.insert(allPoints.begin() + i + 1, newPt3);
                            allPoints.insert(allPoints.begin() + i + 1, newPt2);
                            allPoints.insert(allPoints.begin() + i + 1, newPt1);
                            foundNodeIntersection = true;
                            break;
                        }
                    } else if (isVert) {
                        float safeX = (p1.x < nodeXmin) ? (nodeXmin - SAFE_MARGIN) : (nodeXmax + SAFE_MARGIN);
                        bool fromTop = (std::min(p1.y, p2.y) < nodeYmin);
                        bool fromBottom = (std::max(p1.y, p2.y) > nodeYmax);

                        if (fromTop && fromBottom) {
                            float avoidY1 = nodeYmin - SAFE_MARGIN;
                            float avoidY2 = nodeYmax + SAFE_MARGIN;
                            float detourY = (std::abs(p1.y - avoidY1) < std::abs(p1.y - avoidY2)) ? avoidY1 : avoidY2;

                            Point newPt1 = {p1.x, detourY};
                            Point newPt2 = {safeX, detourY};
                            Point newPt3 = {safeX, p2.y};

                            allPoints.insert(allPoints.begin() + i + 1, newPt3);
                            allPoints.insert(allPoints.begin() + i + 1, newPt2);
                            allPoints.insert(allPoints.begin() + i + 1, newPt1);
                            foundNodeIntersection = true;
                            break;
                        }
                    }
                }
            }
        }

        if (!foundNodeIntersection) break;
    }
}

void EdgeRouting::finalizeEdgePath(
    std::vector<Point>& allPoints,
    const EdgeLayout& layout,
    const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
    float gridSize) {

    // Simplify path by removing spikes and duplicate points
    removeSpikesAndDuplicates(allPoints);

    // Correct source exit direction (orientation + direction)
    correctSourceOrientation(allPoints, layout.sourceEdge, gridSize);
    auto sourceIt = nodeLayouts.find(layout.from);
    if (sourceIt != nodeLayouts.end()) {
        correctSourceDirection(allPoints, layout.sourceEdge, sourceIt->second, gridSize);
    }

    // Correct target entry direction (orientation + direction)
    correctTargetOrientation(allPoints, layout.targetEdge, gridSize);
    auto targetIt = nodeLayouts.find(layout.to);
    if (targetIt != nodeLayouts.end()) {
        correctTargetDirection(allPoints, layout.targetEdge, targetIt->second, gridSize);
    }

    // Final cleanup: remove any spikes created during direction correction
    removeSpikesAndDuplicates(allPoints);
}

// =============================================================================
// Main recalculateBendPoints Method (Refactored)
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

    // Determine layout direction from source/target edges
    bool isHorizontal = (layout.sourceEdge == NodeEdge::Right ||
                         layout.sourceEdge == NodeEdge::Left);

    // Step 1: Initial routing based on channel or fallback
    bool hasSignificantDistance =
        std::abs(layout.sourcePoint.x - layout.targetPoint.x) > 1.0f ||
        std::abs(layout.sourcePoint.y - layout.targetPoint.y) > 1.0f;

    if (hasSignificantDistance) {
        if (layout.channelY >= 0.0f) {
            // Channel-based routing
            performChannelBasedRouting(layout, isHorizontal, nodeLayouts, gridSize);
        } else {
            // Fallback routing for non-channel edges
            performFallbackRouting(layout, isHorizontal, nodeLayouts, gridSize);
        }
    }

    // Step 2: Post-processing - validate and fix path
    if (!nodeLayouts.empty() && !layout.bendPoints.empty()) {
        // Build complete path from source through bends to target
        std::vector<Point> allPoints;
        allPoints.push_back(layout.sourcePoint);
        for (const auto& bend : layout.bendPoints) {
            allPoints.push_back(bend.position);
        }
        allPoints.push_back(layout.targetPoint);

        // Step 2a: Move intermediate points outside nodes
        moveIntermediatePointsOutsideNodes(allPoints, nodeLayouts, gridSize);

        // Step 2b: Fix non-orthogonal segments
        fixNonOrthogonalSegments(allPoints, nodeLayouts);

        // Step 2c: Fix segments that intersect nodes
        fixSegmentNodeIntersections(allPoints, nodeLayouts, gridSize);

        // Step 2d: Final pass for intermediate node intersections
        fixIntermediateNodeIntersections(allPoints, layout, nodeLayouts);

        // Step 2e: Finalize path with direction corrections
        finalizeEdgePath(allPoints, layout, nodeLayouts, gridSize);

        // Reconstruct bendPoints from allPoints
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
    const NodeLayout& node) {
    
    
    float nodeXmin = node.position.x;
    float nodeXmax = node.position.x + node.size.width;
    float nodeYmin = node.position.y;
    float nodeYmax = node.position.y + node.size.height;
    
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

float EdgeRouting::calculateAvoidanceCoordinate(
    float nodeMin,
    float nodeMax,
    bool goPositive,
    float gridSize) {
    float margin = getAvoidanceMargin(gridSize);
    return goPositive ? (nodeMax + margin) : (nodeMin - margin);
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

    return result;
}

Point EdgeRouting::computeSnapPoint(
    const NodeLayout& node,
    Direction direction,
    bool isSource) {
    
    Point center = node.center();
    
    switch (direction) {
        case Direction::TopToBottom:
            return isSource 
                ? Point{center.x, node.position.y + node.size.height}
                : Point{center.x, node.position.y};
        case Direction::BottomToTop:
            return isSource 
                ? Point{center.x, node.position.y}
                : Point{center.x, node.position.y + node.size.height};
        case Direction::LeftToRight:
            return isSource 
                ? Point{node.position.x + node.size.width, center.y}
                : Point{node.position.x, center.y};
        case Direction::RightToLeft:
            return isSource 
                ? Point{node.position.x, center.y}
                : Point{node.position.x + node.size.width, center.y};
    }
    
    return center;
}

// =============================================================================
// Snap Point Distribution Methods
// =============================================================================

void EdgeRouting::distributeAutoSnapPoints(
    Result& result,
    const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
    SnapDistribution distribution,
    float gridSize) {

    float effectiveGridSize = getEffectiveGridSize(gridSize);
    
    if (distribution == SnapDistribution::Unified) {
        // Unified mode: all connections on same edge distributed together
        // Key: (nodeId, nodeEdge) -> list of (edgeId, isSource)
        std::map<std::pair<NodeId, NodeEdge>, std::vector<std::pair<EdgeId, bool>>> allConnections;
        
        for (auto& [edgeId, layout] : result.edgeLayouts) {
            allConnections[{layout.from, layout.sourceEdge}].push_back({edgeId, true});
            allConnections[{layout.to, layout.targetEdge}].push_back({edgeId, false});
        }
        
        for (auto& [key, connections] : allConnections) {
            auto [nodeId, nodeEdge] = key;
            
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
    } else {
        // Separated mode: incoming on first half, outgoing on second half
        // Key: (nodeId, nodeEdge) -> (incoming edges, outgoing edges)
        std::map<std::pair<NodeId, NodeEdge>, std::pair<std::vector<EdgeId>, std::vector<EdgeId>>> separatedConnections;
        
        for (auto& [edgeId, layout] : result.edgeLayouts) {
            // Outgoing from source node
            separatedConnections[{layout.from, layout.sourceEdge}].second.push_back(edgeId);
            // Incoming to target node
            separatedConnections[{layout.to, layout.targetEdge}].first.push_back(edgeId);
        }
        
        for (auto& [key, connections] : separatedConnections) {
            auto [nodeId, nodeEdge] = key;
            
            auto nodeIt = nodeLayouts.find(nodeId);
            if (nodeIt == nodeLayouts.end()) continue;
            
            const NodeLayout& node = nodeIt->second;
            auto& [incoming, outgoing] = connections;
            
            int inCount = static_cast<int>(incoming.size());
            int outCount = static_cast<int>(outgoing.size());
            
            // Determine ranges based on what exists
            float inStart = 0.0f, inEnd = 0.5f;
            float outStart = 0.5f, outEnd = 1.0f;
            
            if (inCount > 0 && outCount == 0) {
                inStart = 0.0f; inEnd = 1.0f;
            } else if (outCount > 0 && inCount == 0) {
                outStart = 0.0f; outEnd = 1.0f;
            }
            
            // Unified snap index counter for this node edge
            int unifiedIndex = 0;
            
            // Incoming edges
            for (int i = 0; i < inCount; ++i) {
                EdgeId edgeId = incoming[i];
                EdgeLayout& layout = result.edgeLayouts[edgeId];
                // Quantized calculation: preserves symmetry
                layout.targetPoint = calculateSnapPositionQuantized(node, nodeEdge, i, inCount, inStart, inEnd, effectiveGridSize);
                layout.targetSnapIndex = unifiedIndex++;
            }

            // Outgoing edges (continue from where incoming left off)
            for (int i = 0; i < outCount; ++i) {
                EdgeId edgeId = outgoing[i];
                EdgeLayout& layout = result.edgeLayouts[edgeId];
                // Quantized calculation: preserves symmetry
                layout.sourcePoint = calculateSnapPositionQuantized(node, nodeEdge, i, outCount, outStart, outEnd, effectiveGridSize);
                layout.sourceSnapIndex = unifiedIndex++;
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

        layout.labelPosition = LayoutUtils::calculateEdgeLabelPosition(layout);
    }
}

void EdgeRouting::updateEdgePositions(
    std::unordered_map<EdgeId, EdgeLayout>& edgeLayouts,
    const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
    const std::vector<EdgeId>& affectedEdges,
    SnapDistribution distribution,
    const std::unordered_set<NodeId>& movedNodes,
    float gridSize) {

    float effectiveGridSize = getEffectiveGridSize(gridSize);
    
    // Helper to check if a node should be updated
    auto shouldUpdateNode = [&movedNodes](NodeId nodeId) -> bool {
        return movedNodes.empty() || movedNodes.count(nodeId) > 0;
    };
    
    if (distribution == SnapDistribution::Unified) {
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
            
            // Skip nodes that haven't moved
            if (!shouldUpdateNode(nodeId)) continue;
            
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
                if (snapIdx < 0 || snapIdx >= totalCount) {
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
                } else {
                    layout.targetPoint = snapPoint;
                }
            }
        }
    } else {
        // Separated mode: incoming on first half, outgoing on second half
        std::map<std::pair<NodeId, NodeEdge>, std::pair<std::vector<EdgeId>, std::vector<EdgeId>>> affectedConnections;
        
        for (EdgeId edgeId : affectedEdges) {
            auto it = edgeLayouts.find(edgeId);
            if (it == edgeLayouts.end()) continue;
            const EdgeLayout& layout = it->second;
            // Outgoing from source node
            affectedConnections[{layout.from, layout.sourceEdge}].second.push_back(edgeId);
            // Incoming to target node
            affectedConnections[{layout.to, layout.targetEdge}].first.push_back(edgeId);
        }
        
        for (auto& [key, connections] : affectedConnections) {
            auto [nodeId, nodeEdge] = key;
            
            // Skip nodes that haven't moved
            if (!shouldUpdateNode(nodeId)) continue;
            
            auto nodeIt = nodeLayouts.find(nodeId);
            if (nodeIt == nodeLayouts.end()) continue;
            
            const NodeLayout& node = nodeIt->second;
            auto& [incoming, outgoing] = connections;
            
            // Get TOTAL counts from all edgeLayouts for this node edge
            auto [totalIn, totalOut] = countConnectionsOnNodeEdge(edgeLayouts, nodeId, nodeEdge);
            
            // Determine ranges based on TOTAL counts
            float inStart = 0.0f, inEnd = 0.5f;
            float outStart = 0.5f, outEnd = 1.0f;
            
            if (totalIn > 0 && totalOut == 0) {
                inStart = 0.0f; inEnd = 1.0f;
            } else if (totalOut > 0 && totalIn == 0) {
                outStart = 0.0f; outEnd = 1.0f;
            }
            
            // Update incoming edges (target point on this node)
            // Note: In Separated mode, snap indices are unified:
            //   incoming get [0, totalIn), outgoing get [totalIn, totalIn+totalOut)
            for (EdgeId edgeId : incoming) {
                EdgeLayout& layout = edgeLayouts[edgeId];
                int localIdx = unifiedToLocalIndex(layout.targetSnapIndex, 0, totalIn);
                // Quantized calculation: preserves symmetry
                layout.targetPoint = calculateSnapPositionQuantized(node, nodeEdge, localIdx, totalIn, inStart, inEnd, effectiveGridSize);
            }

            // Update outgoing edges (source point on this node)
            for (EdgeId edgeId : outgoing) {
                EdgeLayout& layout = edgeLayouts[edgeId];
                int localIdx = unifiedToLocalIndex(layout.sourceSnapIndex, totalIn, totalOut);
                // Quantized calculation: preserves symmetry
                layout.sourcePoint = calculateSnapPositionQuantized(node, nodeEdge, localIdx, totalOut, outStart, outEnd, effectiveGridSize);
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
        
        // Quick check: Do any segments (except first and last) intersect nodes?
        bool hasIntersection = false;
        for (size_t i = 1; i + 1 < allPoints.size(); ++i) {
            const Point& p1 = allPoints[i - 1];
            const Point& p2 = allPoints[i];
            
            for (const auto& [nodeId, nodeLayout] : *allNodeLayouts) {
                if (nodeId == layout.from || nodeId == layout.to) continue;
                if (segmentIntersectsNode(p1, p2, nodeLayout)) {
                    hasIntersection = true;
                    break;
                }
            }
            if (hasIntersection) break;
        }
        
        // If intersection found and we used channel routing, try alternative channel
        if (hasIntersection && layout.channelY >= 0.0f) {
            // Offset channel position and recalculate
            float offset = 30.0f;
            if (layout.sourceEdge == NodeEdge::Right || layout.sourceEdge == NodeEdge::Left) {
                // Horizontal layout: adjust X channel
                layout.channelY += offset;
            } else {
                // Vertical layout: adjust Y channel
                layout.channelY += offset;
            }
            
            // Recalculate with new channel position
            recalculateBendPoints(layout, *allNodeLayouts, gridSize);
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
            // Already handled above, but keep for completeness
            break;
    }

    // Calculate label position
    layout.labelPosition = LayoutUtils::calculateEdgeLabelPosition(layout);

    return layout;
}

SelfLoopDirection EdgeRouting::analyzeSelfLoopDirection(
    const NodeLayout& node,
    const std::unordered_map<EdgeId, EdgeLayout>& existingLayouts,
    const std::unordered_map<NodeId, NodeLayout>& allNodes) {

    // Cost analysis for each direction
    std::map<SelfLoopDirection, float> costs;
    costs[SelfLoopDirection::Right] = 0.0f;
    costs[SelfLoopDirection::Left] = 0.0f;
    costs[SelfLoopDirection::Top] = 0.0f;
    costs[SelfLoopDirection::Bottom] = 0.0f;

    // Check for edges using each direction
    for (const auto& [edgeId, layout] : existingLayouts) {
        if (layout.from == node.id || layout.to == node.id) {
            // Penalize directions with existing connections
            if (layout.sourceEdge == NodeEdge::Right || layout.targetEdge == NodeEdge::Right) {
                costs[SelfLoopDirection::Right] += 10.0f;
            }
            if (layout.sourceEdge == NodeEdge::Left || layout.targetEdge == NodeEdge::Left) {
                costs[SelfLoopDirection::Left] += 10.0f;
            }
            if (layout.sourceEdge == NodeEdge::Top || layout.targetEdge == NodeEdge::Top) {
                costs[SelfLoopDirection::Top] += 10.0f;
            }
            if (layout.sourceEdge == NodeEdge::Bottom || layout.targetEdge == NodeEdge::Bottom) {
                costs[SelfLoopDirection::Bottom] += 10.0f;
            }
        }
    }

    // Check for nearby nodes in each direction
    Point center = node.center();
    for (const auto& [otherId, otherNode] : allNodes) {
        if (otherId == node.id) continue;

        Point otherCenter = otherNode.center();
        float dx = otherCenter.x - center.x;
        float dy = otherCenter.y - center.y;

        // Proximity penalty based on direction
        float proximity = 100.0f / (std::abs(dx) + std::abs(dy) + 1.0f);

        if (dx > 0 && std::abs(dx) > std::abs(dy)) {
            costs[SelfLoopDirection::Right] += proximity;
        }
        if (dx < 0 && std::abs(dx) > std::abs(dy)) {
            costs[SelfLoopDirection::Left] += proximity;
        }
        if (dy < 0 && std::abs(dy) > std::abs(dx)) {
            costs[SelfLoopDirection::Top] += proximity;
        }
        if (dy > 0 && std::abs(dy) > std::abs(dx)) {
            costs[SelfLoopDirection::Bottom] += proximity;
        }
    }

    // Find minimum cost direction
    SelfLoopDirection bestDir = SelfLoopDirection::Right;
    float minCost = costs[SelfLoopDirection::Right];

    for (const auto& [dir, cost] : costs) {
        if (cost < minCost) {
            minCost = cost;
            bestDir = dir;
        }
    }

    return bestDir;
}

}  // namespace algorithms
}  // namespace arborvia
