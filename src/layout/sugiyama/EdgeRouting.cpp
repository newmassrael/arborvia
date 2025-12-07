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
#include "arborvia/layout/LayoutTypes.h"
#include "arborvia/layout/LayoutUtils.h"
#include "arborvia/layout/PathRoutingCoordinator.h"
#include "arborvia/layout/OptimizerRegistry.h"
#include "arborvia/layout/OptimizerConfig.h"
#include "arborvia/layout/EdgePenaltySystem.h"
#include "arborvia/layout/EdgeNudger.h"

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
        // NOTE: Use floor instead of round to avoid half-grid transitions.
        // round() causes transitions at gridSize/2 boundaries, floor() only at gridSize boundaries.
        switch (edge) {
            case NodeEdge::Top: {
                float edgeY = std::floor(node.position.y / gridSize) * gridSize;
                return {pixelCoord, edgeY};
            }
            case NodeEdge::Bottom: {
                float edgeY = std::ceil((node.position.y + node.size.height) / gridSize) * gridSize;
                return {pixelCoord, edgeY};
            }
            case NodeEdge::Left: {
                float edgeX = std::floor(node.position.x / gridSize) * gridSize;
                return {edgeX, pixelCoord};
            }
            case NodeEdge::Right: {
                float edgeX = std::ceil((node.position.x + node.size.width) / gridSize) * gridSize;
                return {edgeX, pixelCoord};
            }
        }
        return node.center();
    }

    // =========================================================================
    // Segment-Node Intersection (Namespace Version)
    // =========================================================================

    /// Check if an orthogonal segment passes through a node's INTERIOR (not just touching boundary)
    /// Returns true only if the segment enters the interior region, not just touches edges.
    /// Uses a small margin (1px) to avoid false positives from boundary-touching segments.
    /// @param p1 First point of the segment
    /// @param p2 Second point of the segment
    /// @param node The node to check against
    /// @return true if segment passes through node interior
    inline bool segmentPenetratesNodeInterior(const Point& p1, const Point& p2, const NodeLayout& node) {
        constexpr float MARGIN = 1.0f;  // Margin to distinguish interior from boundary
        
        float left = node.position.x;
        float right = node.position.x + node.size.width;
        float top = node.position.y;
        float bottom = node.position.y + node.size.height;

        // Check vertical segment (same X coordinate)
        if (std::abs(p1.x - p2.x) < 0.1f) {
            float x = p1.x;
            float minY = std::min(p1.y, p2.y);
            float maxY = std::max(p1.y, p2.y);
            
            // Segment X must be strictly inside node horizontal bounds (not on boundary)
            if (x > left + MARGIN && x < right - MARGIN) {
                // Segment Y range must overlap with node Y range (with margin)
                if (minY < bottom - MARGIN && maxY > top + MARGIN) {
                    return true;
                }
            }
        }
        // Check horizontal segment (same Y coordinate)
        else if (std::abs(p1.y - p2.y) < 0.1f) {
            float y = p1.y;
            float minX = std::min(p1.x, p2.x);
            float maxX = std::max(p1.x, p2.x);
            
            // Segment Y must be strictly inside node vertical bounds (not on boundary)
            if (y > top + MARGIN && y < bottom - MARGIN) {
                // Segment X range must overlap with node X range (with margin)
                if (minX < right - MARGIN && maxX > left + MARGIN) {
                    return true;
                }
            }
        }
        return false;
    }

    /// Check if shifting a segment would cause it to penetrate any node's interior
    /// @param p1 First point of segment (will be shifted)
    /// @param p2 Second point of segment (will be shifted)
    /// @param isVertical Whether this is a vertical segment (shift in X direction)
    /// @param shift The shift amount to apply
    /// @param nodeLayouts All node layouts to check against
    /// @param excludeFrom Node ID to exclude (source node)
    /// @param excludeTo Node ID to exclude (target node)
    /// @return true if shifted segment would penetrate any node
    inline bool shiftWouldPenetrateNode(
        const Point& p1, const Point& p2,
        bool isVertical, float shift,
        const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
        NodeId excludeFrom, NodeId excludeTo) {
        
        Point shiftedP1 = p1;
        Point shiftedP2 = p2;
        
        if (isVertical) {
            shiftedP1.x += shift;
            shiftedP2.x += shift;
        } else {
            shiftedP1.y += shift;
            shiftedP2.y += shift;
        }
        
        for (const auto& [nodeId, nodeLayout] : nodeLayouts) {
            // Skip source and target nodes
            if (nodeId == excludeFrom || nodeId == excludeTo) continue;
            
            if (segmentPenetratesNodeInterior(shiftedP1, shiftedP2, nodeLayout)) {
                return true;
            }
        }
        return false;
    }

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

    // =========================================================================
    // BendPoints Validity Check
    // =========================================================================

    /// Check if an EdgeLayout has fresh (non-stale) bendPoints.
    /// Stale bendPoints occur when sourcePoint was updated (e.g., due to node drag)
    /// but bendPoints still reflect the old position, creating a diagonal path.
    /// @param layout The edge layout to check
    /// @param gridSize Grid size for diagonal detection threshold
    /// @return true if bendPoints are fresh and consistent with sourcePoint
    inline bool hasFreshBendPoints(const EdgeLayout& layout, float gridSize) {
        if (layout.bendPoints.empty()) {
            return true;  // No bendPoints = direct connection, considered fresh
        }

        // Check source side: sourcePoint → firstBend
        const Point& src = layout.sourcePoint;
        const Point& firstBend = layout.bendPoints[0].position;
        float dx_src = std::abs(src.x - firstBend.x);
        float dy_src = std::abs(src.y - firstBend.y);
        bool sourceDiagonal = (dx_src > gridSize && dy_src > gridSize);

        // Check target side: lastBend → targetPoint
        const Point& lastBend = layout.bendPoints.back().position;
        const Point& tgt = layout.targetPoint;
        float dx_tgt = std::abs(lastBend.x - tgt.x);
        float dy_tgt = std::abs(lastBend.y - tgt.y);
        bool targetDiagonal = (dx_tgt > gridSize && dy_tgt > gridSize);

        // If either side has diagonal, bendPoints are stale
        return !(sourceDiagonal || targetDiagonal);
    }

    // =========================================================================
    // Edge Count on NodeEdge
    // =========================================================================

    /// Count how many edges are connected to a specific NodeEdge.
    /// Used for proper snap index allocation when switching NodeEdges.
    /// @param edgeLayouts All edge layouts
    /// @param nodeId The node to check
    /// @param edge Which edge of the node
    /// @param excludeId Edge ID to exclude from counting (the edge being routed)
    /// @return Number of edges connected to this NodeEdge
    inline int countEdgesOnNodeEdge(
        const std::unordered_map<EdgeId, EdgeLayout>& edgeLayouts,
        NodeId nodeId,
        NodeEdge edge,
        EdgeId excludeId) {

        int count = 0;
        for (const auto& [id, layout] : edgeLayouts) {
            if (id == excludeId) continue;

            // Check source side
            if (layout.from == nodeId && layout.sourceEdge == edge) {
                count++;
            }
            // Check target side
            if (layout.to == nodeId && layout.targetEdge == edge) {
                count++;
            }
        }
        return count;
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
    float gridSize,
    const std::unordered_map<EdgeId, EdgeLayout>* otherEdges) {
    
    // Use effective grid size for routing calculations
    float effectiveGridSize = gridSize > 0.0f ? gridSize : constants::PATHFINDING_GRID_SIZE;

    // Get source node for snap point calculations
    auto srcNodeIt = nodeLayouts.find(layout.from);
    if (srcNodeIt == nodeLayouts.end()) {
        return;  // Can't route without source node
    }
    const NodeLayout& srcNode = srcNodeIt->second;

    // === Self-loop handling ===
    // Self-loops need special orthogonal routing that wraps around the node corner
    if (layout.from == layout.to) {
        constexpr float BASE_OFFSET = 30.0f;
        
        // Generate orthogonal path for self-loop
        // The path goes: sourcePoint -> srcExt -> corner -> tgtExt -> targetPoint
        Point srcExt, tgtExt;
        
        // Calculate source extension point (perpendicular to source edge)
        switch (layout.sourceEdge) {
            case NodeEdge::Top:
                srcExt = {layout.sourcePoint.x, layout.sourcePoint.y - BASE_OFFSET};
                break;
            case NodeEdge::Bottom:
                srcExt = {layout.sourcePoint.x, layout.sourcePoint.y + BASE_OFFSET};
                break;
            case NodeEdge::Left:
                srcExt = {layout.sourcePoint.x - BASE_OFFSET, layout.sourcePoint.y};
                break;
            case NodeEdge::Right:
                srcExt = {layout.sourcePoint.x + BASE_OFFSET, layout.sourcePoint.y};
                break;
        }
        
        // Calculate target extension point (perpendicular to target edge)
        switch (layout.targetEdge) {
            case NodeEdge::Top:
                tgtExt = {layout.targetPoint.x, layout.targetPoint.y - BASE_OFFSET};
                break;
            case NodeEdge::Bottom:
                tgtExt = {layout.targetPoint.x, layout.targetPoint.y + BASE_OFFSET};
                break;
            case NodeEdge::Left:
                tgtExt = {layout.targetPoint.x - BASE_OFFSET, layout.targetPoint.y};
                break;
            case NodeEdge::Right:
                tgtExt = {layout.targetPoint.x + BASE_OFFSET, layout.targetPoint.y};
                break;
        }
        
        // Snap to grid
        srcExt.x = std::round(srcExt.x / effectiveGridSize) * effectiveGridSize;
        srcExt.y = std::round(srcExt.y / effectiveGridSize) * effectiveGridSize;
        tgtExt.x = std::round(tgtExt.x / effectiveGridSize) * effectiveGridSize;
        tgtExt.y = std::round(tgtExt.y / effectiveGridSize) * effectiveGridSize;
        
        layout.bendPoints.clear();
        
        // Check if we need a corner point (srcExt and tgtExt not aligned)
        constexpr float EPSILON = 1.0f;
        if (std::abs(srcExt.x - tgtExt.x) > EPSILON && std::abs(srcExt.y - tgtExt.y) > EPSILON) {
            // Need corner - pick the one outside the node
            Point cornerA = {srcExt.x, tgtExt.y};
            Point cornerB = {tgtExt.x, srcExt.y};
            
            float nodeLeft = srcNode.position.x;
            float nodeRight = srcNode.position.x + srcNode.size.width;
            float nodeTop = srcNode.position.y;
            float nodeBottom = srcNode.position.y + srcNode.size.height;
            
            bool cornerAInside = (cornerA.x >= nodeLeft && cornerA.x <= nodeRight &&
                                  cornerA.y >= nodeTop && cornerA.y <= nodeBottom);
            
            Point corner = cornerAInside ? cornerB : cornerA;
            
            layout.bendPoints.push_back({srcExt});
            layout.bendPoints.push_back({corner});
            layout.bendPoints.push_back({tgtExt});
        } else {
            // srcExt and tgtExt are aligned (either same X or same Y)
            layout.bendPoints.push_back({srcExt});
            layout.bendPoints.push_back({tgtExt});
        }
        
#if EDGE_ROUTING_DEBUG
        std::cout << "[recalculateBendPoints] Edge " << layout.id << " SELF-LOOP: "
                  << "src=(" << layout.sourcePoint.x << "," << layout.sourcePoint.y << ") "
                  << "tgt=(" << layout.targetPoint.x << "," << layout.targetPoint.y << ") "
                  << "bends=" << layout.bendPoints.size() << std::endl;
#endif
        return;  // Self-loop routing complete
    }
    
    // Calculate edge length for snap point positions
    float edgeLength = 0.0f;
    if (layout.sourceEdge == NodeEdge::Top || layout.sourceEdge == NodeEdge::Bottom) {
        edgeLength = srcNode.size.width;
    } else {
        edgeLength = srcNode.size.height;
    }
    
    // Generate snap positions to try (current position first, then alternatives)
    std::vector<float> snapPositions;
    
    // Calculate current position ratio
    float currentRatio = 0.5f;
    if (layout.sourceEdge == NodeEdge::Top || layout.sourceEdge == NodeEdge::Bottom) {
        currentRatio = (layout.sourcePoint.x - srcNode.position.x) / edgeLength;
    } else {
        currentRatio = (layout.sourcePoint.y - srcNode.position.y) / edgeLength;
    }
    
    // Add current position first
    snapPositions.push_back(currentRatio);
    
    // Add alternative positions (spread across the edge)
    std::vector<float> alternatives = {0.2f, 0.4f, 0.5f, 0.6f, 0.8f, 0.1f, 0.9f, 0.3f, 0.7f};
    for (float alt : alternatives) {
        if (std::abs(alt - currentRatio) > 0.05f) {  // Skip if too close to current
            snapPositions.push_back(alt);
        }
    }
    
    // Try each snap position
    Point originalSourcePoint = layout.sourcePoint;
    bool pathFound = false;
    
    for (size_t attempt = 0; attempt < snapPositions.size() && !pathFound; ++attempt) {
        float ratio = snapPositions[attempt];
        
        // Calculate source point for this ratio
        Point trySourcePoint;
        if (layout.sourceEdge == NodeEdge::Top) {
            trySourcePoint = {srcNode.position.x + edgeLength * ratio, srcNode.position.y};
        } else if (layout.sourceEdge == NodeEdge::Bottom) {
            trySourcePoint = {srcNode.position.x + edgeLength * ratio, srcNode.position.y + srcNode.size.height};
        } else if (layout.sourceEdge == NodeEdge::Left) {
            trySourcePoint = {srcNode.position.x, srcNode.position.y + edgeLength * ratio};
        } else {  // Right
            trySourcePoint = {srcNode.position.x + srcNode.size.width, srcNode.position.y + edgeLength * ratio};
        }
        
        // Quantize to grid
        trySourcePoint.x = std::round(trySourcePoint.x / effectiveGridSize) * effectiveGridSize;
        trySourcePoint.y = std::round(trySourcePoint.y / effectiveGridSize) * effectiveGridSize;
        
#if EDGE_ROUTING_DEBUG
        if (attempt > 0) {
            std::cout << "[A*] Edge " << layout.id << " RETRY attempt " << attempt 
                      << " ratio=" << ratio
                      << " tryPoint=(" << trySourcePoint.x << "," << trySourcePoint.y << ")" << std::endl;
        }
#endif
        
        // Build obstacle map (fresh for each attempt)
        ObstacleMap obstacles;
        obstacles.buildFromNodes(nodeLayouts, effectiveGridSize, 0);
        
        // Convert points to grid coordinates
        GridPoint startGrid = obstacles.pixelToGrid(trySourcePoint);
        GridPoint goalGrid = obstacles.pixelToGrid(layout.targetPoint);
        
        // Find additional nodes to exclude at start/goal cells
        std::unordered_set<NodeId> extraStartExcludes;
        std::unordered_set<NodeId> extraGoalExcludes;
        
        for (const auto& [nodeId, node] : nodeLayouts) {
            if (nodeId == layout.from || nodeId == layout.to) continue;
            
            if (trySourcePoint.x >= node.position.x &&
                trySourcePoint.x <= node.position.x + node.size.width &&
                trySourcePoint.y >= node.position.y &&
                trySourcePoint.y <= node.position.y + node.size.height) {
                extraStartExcludes.insert(nodeId);
            }
            
            if (layout.targetPoint.x >= node.position.x &&
                layout.targetPoint.x <= node.position.x + node.size.width &&
                layout.targetPoint.y >= node.position.y &&
                layout.targetPoint.y <= node.position.y + node.size.height) {
                extraGoalExcludes.insert(nodeId);
            }
        }
        
        // Add other edges as obstacles for ALL attempts (same constraints for retry)
        if (otherEdges) {
            std::unordered_map<EdgeId, EdgeLayout> freshEdges;
            for (const auto& [edgeId, edgeLayout] : *otherEdges) {
                if (hasFreshBendPoints(edgeLayout, effectiveGridSize)) {
                    freshEdges[edgeId] = edgeLayout;
                }
            }
            obstacles.addEdgeSegments(freshEdges, layout.id);
        }
        
        // Try A* pathfinding
        PathResult pathResult = activePathFinder().findPath(startGrid, goalGrid, obstacles,
                                                      layout.from, layout.to,
                                                      layout.sourceEdge, layout.targetEdge,
                                                      extraStartExcludes, extraGoalExcludes);
        
        if (pathResult.found && pathResult.path.size() >= 2) {
            // Success! Update layout
            layout.sourcePoint = trySourcePoint;
            layout.bendPoints.clear();
            for (size_t i = 1; i + 1 < pathResult.path.size(); ++i) {
                Point pixelPoint = obstacles.gridToPixel(pathResult.path[i].x, pathResult.path[i].y);
                layout.bendPoints.push_back({pixelPoint});
            }
            pathFound = true;
            
#if EDGE_ROUTING_DEBUG
            std::cout << "[A*] Edge " << layout.id << " SUCCESS"
                      << (attempt > 0 ? " (after retry)" : "")
                      << ": path.size=" << pathResult.path.size()
                      << " bends=" << layout.bendPoints.size()
                      << " src=(" << layout.sourcePoint.x << "," << layout.sourcePoint.y << ")"
                      << " tgt=(" << layout.targetPoint.x << "," << layout.targetPoint.y << ")" << std::endl;
#endif
        }
    }
    
    if (!pathFound) {
        // All attempts failed - restore original source point
        layout.sourcePoint = originalSourcePoint;
        layout.bendPoints.clear();
        
#if EDGE_ROUTING_DEBUG
        std::cout << "[A*] Edge " << layout.id << " ALL RETRIES FAILED!"
                  << " src=(" << layout.sourcePoint.x << "," << layout.sourcePoint.y << ")"
                  << " tgt=(" << layout.targetPoint.x << "," << layout.targetPoint.y << ")" << std::endl;
#endif
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
// A* Retry Methods
// =============================================================================

SnapRetryResult EdgeRouting::tryAStarRetry(
    EdgeLayout& layout,
    std::unordered_map<EdgeId, EdgeLayout>& edgeLayouts,
    const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
    float gridSize,
    const std::unordered_map<EdgeId, EdgeLayout>* otherEdges,
    const SnapRetryConfig& config) {

#if EDGE_ROUTING_DEBUG
    std::cout << "[A* RETRY] Edge " << layout.id << " starting retry sequence" << std::endl;
#endif

    // Layer 1: Snap Index Retry (concentric expansion on same NodeEdge)
    auto result = trySnapIndexRetry(layout, edgeLayouts, nodeLayouts, gridSize, otherEdges, config);
    if (result.success) {
#if EDGE_ROUTING_DEBUG
        std::cout << "[A* RETRY] Edge " << layout.id << " Layer 1 SUCCESS: snap index changed to "
                  << result.newSourceSnapIndex << std::endl;
#endif
        return result;
    }

    // Layer 2: Neighbor Adjustment (adjust neighbor transition coordinates)
    result = tryNeighborAdjustment(layout, edgeLayouts, nodeLayouts, gridSize, otherEdges, config);
    if (result.success) {
#if EDGE_ROUTING_DEBUG
        std::cout << "[A* RETRY] Edge " << layout.id << " Layer 2 SUCCESS: neighbor "
                  << result.adjustedNeighborId << " adjusted" << std::endl;
#endif
        return result;
    }

#if EDGE_ROUTING_DEBUG
    std::cout << "[A* RETRY] Edge " << layout.id << " Layer 2 COMPLETED (failed), moving to Layer 3" << std::endl;
#endif

    // Layer 3: NodeEdge Switch (16 combinations, only if preserveDirections=false)
    if (config.enableNodeEdgeSwitch && !config.preserveDirections) {
        result = tryNodeEdgeSwitch(layout, nodeLayouts, gridSize, otherEdges, config);
        if (result.success) {
#if EDGE_ROUTING_DEBUG
            std::cout << "[A* RETRY] Edge " << layout.id << " Layer 3 SUCCESS: NodeEdge switched to src="
                      << static_cast<int>(result.newSourceEdge) << " tgt="
                      << static_cast<int>(result.newTargetEdge) << std::endl;
#endif
            return result;
        }
    }

#if EDGE_ROUTING_DEBUG
    std::cout << "[A* RETRY] Edge " << layout.id << " ALL LAYERS FAILED: snapRetries="
              << result.snapRetryCount << " neighborAdjusts=" << result.neighborAdjustCount
              << " edgeCombinations=" << result.edgeCombinationCount << std::endl;
#endif

    return result;  // All retries failed
}

SnapRetryResult EdgeRouting::trySnapIndexRetry(
    EdgeLayout& layout,
    const std::unordered_map<EdgeId, EdgeLayout>& edgeLayouts,
    const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
    float gridSize,
    const std::unordered_map<EdgeId, EdgeLayout>* otherEdges,
    const SnapRetryConfig& config) {

    SnapRetryResult result;

    auto srcNodeIt = nodeLayouts.find(layout.from);
    if (srcNodeIt == nodeLayouts.end()) {
        return result;
    }

    const NodeLayout& srcNode = srcNodeIt->second;

    // Get total snap point count for source NodeEdge
    auto srcConnections = SnapIndexManager::getConnections(edgeLayouts, layout.from, layout.sourceEdge);
    int srcTotalCount = srcConnections.totalCount();
    
    // CRITICAL FIX: Even with srcTotalCount=1, try virtual snap positions
    // This handles cases where the current snap position is blocked by node obstacles
    // We try at least 3 positions to give room for alternatives
    int minVirtualPositions = 3;
    int effectiveTotalCount = std::max(srcTotalCount, minVirtualPositions);
    
#if EDGE_ROUTING_DEBUG
    std::cout << "[A* RETRY] Edge " << layout.id << " trySnapIndexRetry: srcTotalCount="
              << srcTotalCount << " effectiveTotalCount=" << effectiveTotalCount
              << " currentIdx=" << layout.sourceSnapIndex
              << " sourceEdge=" << static_cast<int>(layout.sourceEdge) << std::endl;
#endif

    int currentIdx = layout.sourceSnapIndex;
    float effectiveGridSize = getEffectiveGridSize(gridSize);

    // Concentric expansion: try ±1, ±2, ...
    // Use effectiveTotalCount to allow virtual snap positions beyond current edge count
    for (int offset = 1; offset <= config.maxSnapRetries; ++offset) {
        for (int sign : {1, -1}) {
            int tryIdx = currentIdx + (offset * sign);
            if (tryIdx < 0 || tryIdx >= effectiveTotalCount) continue;

            // Check if this index is already used by another edge
            bool indexOccupied = false;
            for (const auto& [edgeId, otherLayout] : edgeLayouts) {
                if (edgeId == layout.id) continue;
                if (otherLayout.from == layout.from &&
                    otherLayout.sourceEdge == layout.sourceEdge &&
                    otherLayout.sourceSnapIndex == tryIdx) {
                    indexOccupied = true;
                    break;
                }
            }
            if (indexOccupied) continue;

            result.snapRetryCount++;

            // Calculate new snap position using effectiveTotalCount for proper spacing
            Point newSrcPoint = calculateSnapPositionQuantized(
                srcNode, layout.sourceEdge, tryIdx, effectiveTotalCount, 0.0f, 1.0f, effectiveGridSize);

            // Create obstacle map and try A*
            ObstacleMap obstacles;
            obstacles.buildFromNodes(nodeLayouts, effectiveGridSize);
            GridPoint startGrid = obstacles.pixelToGrid(newSrcPoint);
            GridPoint goalGrid = obstacles.pixelToGrid(layout.targetPoint);

            // Option B: Don't add other edge segments as obstacles during retry
            // The retry tests if a path is POSSIBLE with new snap point.
            // After success, full re-routing will resolve edge overlaps.
            // (Other edges' segments block the new snap point unfairly
            //  because they were routed before snap point change)

            PathResult pathResult = activePathFinder().findPath(
                startGrid, goalGrid, obstacles,
                layout.from, layout.to,
                layout.sourceEdge, layout.targetEdge,
                {}, {});

            if (pathResult.found && pathResult.path.size() >= 2) {
                // Success - update result
                result.success = true;
                result.snapIndexChanged = true;
                result.newSourceSnapIndex = tryIdx;
                result.newSourcePoint = newSrcPoint;  // Store exact point used
                result.needsFullReroute = true;  // Need full re-route because no edge segments were obstacles

                // Don't store bendPoints - they will be recalculated during full re-route
                // The bendPoints from this retry are invalid because other edges weren't obstacles
                result.bendPoints.clear();

                std::cout << "[DEBUG RETRY SUCCESS] Edge " << layout.id 
                          << " effectiveTotalCount=" << effectiveTotalCount
                          << " snapIdx " << currentIdx << " -> " << tryIdx 
                          << " newSrcPoint=(" << newSrcPoint.x << "," << newSrcPoint.y << ")"
                          << " needsFullReroute=true"
                          << std::endl;
                return result;
            }
        }
    }

#if EDGE_ROUTING_DEBUG
    std::cout << "[A* RETRY] Edge " << layout.id << " snap index retry FAILED after "
              << result.snapRetryCount << " attempts" << std::endl;
#endif

    return result;
}

SnapRetryResult EdgeRouting::tryNeighborAdjustment(
    EdgeLayout& layout,
    std::unordered_map<EdgeId, EdgeLayout>& edgeLayouts,
    const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
    float gridSize,
    const std::unordered_map<EdgeId, EdgeLayout>* otherEdges,
    const SnapRetryConfig& config) {

    SnapRetryResult result;
    float effectiveGridSize = getEffectiveGridSize(gridSize);

    // Collect ALL neighbor edges (from same source OR same target node)
    std::vector<EdgeId> neighborEdges;
    for (const auto& [edgeId, edgeLayout] : edgeLayouts) {
        if (edgeId != layout.id) {
            // Include edges from same source node OR targeting same target node
            if (edgeLayout.from == layout.from || edgeLayout.to == layout.to ||
                edgeLayout.from == layout.to || edgeLayout.to == layout.from) {
                neighborEdges.push_back(edgeId);
            }
        }
    }

    if (neighborEdges.empty()) {
#if EDGE_ROUTING_DEBUG
        std::cout << "[A* RETRY] Edge " << layout.id << " no neighbor edges to adjust" << std::endl;
#endif
        return result;
    }

#if EDGE_ROUTING_DEBUG
    std::cout << "[A* RETRY] Edge " << layout.id << " trying " << neighborEdges.size()
              << " potential blocking neighbors:";
    for (EdgeId nid : neighborEdges) {
        std::cout << " " << nid;
    }
    std::cout << std::endl;
#endif

    constexpr std::array<NodeEdge, 4> allNodeEdges = {
        NodeEdge::Top, NodeEdge::Bottom, NodeEdge::Left, NodeEdge::Right
    };

    // For each neighbor, try rerouting it to different NodeEdges to make room for X
    for (EdgeId neighborId : neighborEdges) {
        auto neighborIt = edgeLayouts.find(neighborId);
        if (neighborIt == edgeLayouts.end()) continue;

        EdgeLayout& neighbor = neighborIt->second;

#if EDGE_ROUTING_DEBUG
        std::cout << "[A* RETRY] Trying neighbor " << neighborId
                  << " (from=" << neighbor.from << " to=" << neighbor.to
                  << " srcEdge=" << static_cast<int>(neighbor.sourceEdge)
                  << " tgtEdge=" << static_cast<int>(neighbor.targetEdge) << ")" << std::endl;
#endif

        // Save original state
        NodeEdge origSourceEdge = neighbor.sourceEdge;
        NodeEdge origTargetEdge = neighbor.targetEdge;
        Point origSourcePoint = neighbor.sourcePoint;
        Point origTargetPoint = neighbor.targetPoint;
        int origSourceSnapIdx = neighbor.sourceSnapIndex;
        int origTargetSnapIdx = neighbor.targetSnapIndex;
        auto origBends = neighbor.bendPoints;

        // Try different NodeEdge combinations for the neighbor
        for (NodeEdge newSrcEdge : allNodeEdges) {
            for (NodeEdge newTgtEdge : allNodeEdges) {
                // Skip current combination
                if (newSrcEdge == origSourceEdge && newTgtEdge == origTargetEdge) {
                    continue;
                }

                result.neighborAdjustCount++;

                // Calculate new source/target points for neighbor
                // Account for existing edges on the new NodeEdge
                auto srcNodeIt = nodeLayouts.find(neighbor.from);
                auto tgtNodeIt = nodeLayouts.find(neighbor.to);
                if (srcNodeIt == nodeLayouts.end() || tgtNodeIt == nodeLayouts.end()) continue;

                const NodeLayout& srcNode = srcNodeIt->second;
                const NodeLayout& tgtNode = tgtNodeIt->second;

                // Count existing edges on the new source NodeEdge (excluding this neighbor)
                int srcEdgeCount = 0;
                for (const auto& [otherId, otherLayout] : edgeLayouts) {
                    if (otherId != neighborId && otherLayout.from == neighbor.from &&
                        otherLayout.sourceEdge == newSrcEdge) {
                        srcEdgeCount++;
                    }
                }

                // Count existing edges on the new target NodeEdge (excluding this neighbor)
                int tgtEdgeCount = 0;
                for (const auto& [otherId, otherLayout] : edgeLayouts) {
                    if (otherId != neighborId && otherLayout.to == neighbor.to &&
                        otherLayout.targetEdge == newTgtEdge) {
                        tgtEdgeCount++;
                    }
                }

                // Place neighbor at the end (last index)
                int newSrcIdx = srcEdgeCount;
                int newTgtIdx = tgtEdgeCount;
                int newSrcTotal = srcEdgeCount + 1;
                int newTgtTotal = tgtEdgeCount + 1;

                Point newSrcPoint = calculateSnapPositionQuantized(
                    srcNode, newSrcEdge, newSrcIdx, newSrcTotal, 0.0f, 1.0f, effectiveGridSize);
                Point newTgtPoint = calculateSnapPositionQuantized(
                    tgtNode, newTgtEdge, newTgtIdx, newTgtTotal, 0.0f, 1.0f, effectiveGridSize);

                // Try rerouting neighbor with new NodeEdges
                ObstacleMap neighborObstacles;
                neighborObstacles.buildFromNodes(nodeLayouts, effectiveGridSize);

                // Add all other edges (except neighbor, X, and edges with stale bendPoints)
                for (const auto& [otherId, otherLayout] : edgeLayouts) {
                    if (otherId != neighborId && otherId != layout.id) {
                        // Skip edges with stale bendPoints (diagonal from sourcePoint to first bend)
                        if (!hasFreshBendPoints(otherLayout, effectiveGridSize)) {
#if EDGE_ROUTING_DEBUG
                            std::cout << "[A* RETRY] Skipping edge " << otherId << " with stale bendPoints" << std::endl;
#endif
                            continue;
                        }
                        neighborObstacles.addEdgeSegments({{otherId, otherLayout}}, neighborId);
                    }
                }

                GridPoint neighborStart = neighborObstacles.pixelToGrid(newSrcPoint);
                GridPoint neighborGoal = neighborObstacles.pixelToGrid(newTgtPoint);

                PathResult neighborPath = activePathFinder().findPath(
                    neighborStart, neighborGoal, neighborObstacles,
                    neighbor.from, neighbor.to,
                    newSrcEdge, newTgtEdge,
                    {}, {});

                if (!neighborPath.found || neighborPath.path.size() < 2) {
#if EDGE_ROUTING_DEBUG
                    std::cout << "[A* RETRY] neighbor " << neighborId
                              << " can't route srcEdge=" << static_cast<int>(newSrcEdge)
                              << " tgtEdge=" << static_cast<int>(newTgtEdge) << std::endl;
                    std::cout << "[A* RETRY]   start=(" << neighborStart.x << "," << neighborStart.y
                              << ") goal=(" << neighborGoal.x << "," << neighborGoal.y << ")" << std::endl;
                    std::cout << "[A* RETRY]   srcPoint=(" << newSrcPoint.x << "," << newSrcPoint.y
                              << ") tgtPoint=(" << newTgtPoint.x << "," << newTgtPoint.y << ")" << std::endl;
                    std::cout << "[A* RETRY]   obstacles from edges:";
                    for (const auto& [otherId, otherLayout] : edgeLayouts) {
                        if (otherId != neighborId && otherId != layout.id) {
                            if (hasFreshBendPoints(otherLayout, effectiveGridSize)) {
                                std::cout << " " << otherId;
                            } else {
                                std::cout << " (" << otherId << "-stale)";
                            }
                        }
                    }
                    std::cout << std::endl;
                    std::cout << "[A* RETRY]   path.found=" << neighborPath.found
                              << " path.size=" << neighborPath.path.size() << std::endl;
#endif
                    continue;  // Neighbor can't route with this NodeEdge combo
                }

                // Build neighbor's new bendPoints
                std::vector<BendPoint> newNeighborBends;
                for (size_t i = 1; i + 1 < neighborPath.path.size(); ++i) {
                    Point pixelPoint = neighborObstacles.gridToPixel(neighborPath.path[i].x, neighborPath.path[i].y);
                    newNeighborBends.push_back({pixelPoint});
                }

                // Check neighbor's path is valid (no diagonal)
                bool neighborValid = true;
                if (!newNeighborBends.empty()) {
                    float dx = std::abs(newSrcPoint.x - newNeighborBends[0].position.x);
                    float dy = std::abs(newSrcPoint.y - newNeighborBends[0].position.y);
                    if (dx > 1.0f && dy > 1.0f) {
                        neighborValid = false;
                    }
                }
                if (!neighborValid) continue;

#if EDGE_ROUTING_DEBUG
                std::cout << "[A* RETRY] Edge " << layout.id << " trying neighbor " << neighborId
                          << " with srcEdge " << static_cast<int>(newSrcEdge)
                          << " tgtEdge " << static_cast<int>(newTgtEdge) << std::endl;
#endif

                // Temporarily apply neighbor's new route
                neighbor.sourceEdge = newSrcEdge;
                neighbor.targetEdge = newTgtEdge;
                neighbor.sourcePoint = newSrcPoint;
                neighbor.targetPoint = newTgtPoint;
                neighbor.sourceSnapIndex = newSrcIdx;
                neighbor.targetSnapIndex = newTgtIdx;
                neighbor.bendPoints = newNeighborBends;

                // Now try A* for X with neighbor's NEW path as obstacle
                // CRITICAL FIX: Also try multiple snap positions for edge X
                // The original position might be blocked by node obstacles
                
                auto xSrcNodeIt = nodeLayouts.find(layout.from);
                if (xSrcNodeIt == nodeLayouts.end()) continue;
                const NodeLayout& xSrcNode = xSrcNodeIt->second;
                
                // Try current position first, then virtual positions
                int xMinPositions = 3;
                auto xSrcConnections = SnapIndexManager::getConnections(edgeLayouts, layout.from, layout.sourceEdge);
                int xEffectiveTotalCount = std::max(xSrcConnections.totalCount(), xMinPositions);
                
                bool foundXPath = false;
                Point successSrcPoint = layout.sourcePoint;
                int successSnapIdx = layout.sourceSnapIndex;
                PathResult xPath;
                
                // Try current index first, then ±1, ±2, ...
                std::vector<int> tryIndices = {layout.sourceSnapIndex};
                for (int offset = 1; offset <= 2; ++offset) {
                    tryIndices.push_back(layout.sourceSnapIndex + offset);
                    tryIndices.push_back(layout.sourceSnapIndex - offset);
                }
                
                for (int xTryIdx : tryIndices) {
                    if (xTryIdx < 0 || xTryIdx >= xEffectiveTotalCount) continue;
                    
                    Point xSrcPoint = (xTryIdx == layout.sourceSnapIndex) 
                        ? layout.sourcePoint 
                        : calculateSnapPositionQuantized(xSrcNode, layout.sourceEdge, xTryIdx, xEffectiveTotalCount, 0.0f, 1.0f, effectiveGridSize);
                    
                    ObstacleMap testObstacles;
                    testObstacles.buildFromNodes(nodeLayouts, effectiveGridSize);

                    // Add all edges (including neighbor's new path)
                    for (const auto& [otherId, otherLayout] : edgeLayouts) {
                        if (otherId != layout.id) {
                            testObstacles.addEdgeSegments({{otherId, otherLayout}}, layout.id);
                        }
                    }

                    GridPoint startGrid = testObstacles.pixelToGrid(xSrcPoint);
                    GridPoint goalGrid = testObstacles.pixelToGrid(layout.targetPoint);

                    xPath = activePathFinder().findPath(
                        startGrid, goalGrid, testObstacles,
                        layout.from, layout.to,
                        layout.sourceEdge, layout.targetEdge,
                        {}, {});
                    
                    if (xPath.found && xPath.path.size() >= 2) {
                        foundXPath = true;
                        successSrcPoint = xSrcPoint;
                        successSnapIdx = xTryIdx;
                        break;
                    }
                }
                
                if (!foundXPath) {
                    // Rollback neighbor and try next combination
                    neighbor.sourceEdge = origSourceEdge;
                    neighbor.targetEdge = origTargetEdge;
                    neighbor.sourcePoint = origSourcePoint;
                    neighbor.targetPoint = origTargetPoint;
                    neighbor.sourceSnapIndex = origSourceSnapIdx;
                    neighbor.targetSnapIndex = origTargetSnapIdx;
                    neighbor.bendPoints = origBends;
                    continue;
                }

                // Build testObstacles for validation
                ObstacleMap finalObstacles;
                finalObstacles.buildFromNodes(nodeLayouts, effectiveGridSize);
                for (const auto& [otherId, otherLayout] : edgeLayouts) {
                    if (otherId != layout.id) {
                        finalObstacles.addEdgeSegments({{otherId, otherLayout}}, layout.id);
                    }
                }

#if EDGE_ROUTING_DEBUG
                std::cout << "[A* RETRY] neighbor " << neighborId << " rerouted to ("
                          << static_cast<int>(newSrcEdge) << "," << static_cast<int>(newTgtEdge)
                          << ") -> trying Edge " << layout.id << " at snapIdx=" << successSnapIdx << std::endl;
#endif
                // Build X's bendPoints
                std::vector<BendPoint> xBends;
                for (size_t i = 1; i + 1 < xPath.path.size(); ++i) {
                    Point pixelPoint = finalObstacles.gridToPixel(xPath.path[i].x, xPath.path[i].y);
                    xBends.push_back({pixelPoint});
                }

                // Check X's path is valid (no diagonal)
                bool xValid = true;
                if (!xBends.empty()) {
                    float dx = std::abs(successSrcPoint.x - xBends[0].position.x);
                    float dy = std::abs(successSrcPoint.y - xBends[0].position.y);
                    if (dx > 1.0f && dy > 1.0f) {
                        xValid = false;
#if EDGE_ROUTING_DEBUG
                        std::cout << "[A* RETRY] neighbor " << neighborId << " rerouted OK but Edge "
                                  << layout.id << " path has DIAGONAL" << std::endl;
#endif
                    }
                }

                if (xValid) {
#if EDGE_ROUTING_DEBUG
                    std::cout << "[A* RETRY] neighbor " << neighborId << " rerouted to ("
                              << static_cast<int>(newSrcEdge) << "," << static_cast<int>(newTgtEdge)
                              << ") -> Edge " << layout.id << " found path!" << std::endl;
#endif
                    // SUCCESS! Keep neighbor's new route
                    result.success = true;
                    result.neighborAdjusted = true;
                    result.adjustedNeighborId = neighborId;
                    result.bendPoints = xBends;
                    
                    // Also record if we changed X's snap position
                    if (successSnapIdx != layout.sourceSnapIndex) {
                        result.snapIndexChanged = true;
                        result.newSourceSnapIndex = successSnapIdx;
                    }

#if EDGE_ROUTING_DEBUG
                    std::cout << "[A* RETRY] Edge " << layout.id << " SUCCESS! Rerouted neighbor "
                              << neighborId << " from edges (" << static_cast<int>(origSourceEdge)
                              << "," << static_cast<int>(origTargetEdge) << ") to ("
                              << static_cast<int>(newSrcEdge) << "," << static_cast<int>(newTgtEdge) << ")"
                              << " xSnapIdx=" << successSnapIdx << std::endl;
#endif
                    return result;
                }

                // Rollback neighbor
                neighbor.sourceEdge = origSourceEdge;
                neighbor.targetEdge = origTargetEdge;
                neighbor.sourcePoint = origSourcePoint;
                neighbor.targetPoint = origTargetPoint;
                neighbor.sourceSnapIndex = origSourceSnapIdx;
                neighbor.targetSnapIndex = origTargetSnapIdx;
                neighbor.bendPoints = origBends;
            }
        }
    }

#if EDGE_ROUTING_DEBUG
    std::cout << "[A* RETRY] Edge " << layout.id << " neighbor adjustment FAILED after "
              << result.neighborAdjustCount << " attempts" << std::endl;
#endif

    return result;
}

SnapRetryResult EdgeRouting::tryNodeEdgeSwitch(
    EdgeLayout& layout,
    const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
    float gridSize,
    const std::unordered_map<EdgeId, EdgeLayout>* otherEdges,
    const SnapRetryConfig& config) {

    SnapRetryResult result;

    auto srcNodeIt = nodeLayouts.find(layout.from);
    auto tgtNodeIt = nodeLayouts.find(layout.to);
    if (srcNodeIt == nodeLayouts.end() || tgtNodeIt == nodeLayouts.end()) {
        return result;
    }

    const NodeLayout& srcNode = srcNodeIt->second;
    const NodeLayout& tgtNode = tgtNodeIt->second;
    float effectiveGridSize = getEffectiveGridSize(gridSize);

#if EDGE_ROUTING_DEBUG
    std::cout << "[A* RETRY] Edge " << layout.id << " ENTERING tryNodeEdgeSwitch (Layer 3)" << std::endl;
#endif

    constexpr std::array<NodeEdge, 4> allEdges = {
        NodeEdge::Top, NodeEdge::Bottom, NodeEdge::Left, NodeEdge::Right
    };

    NodeEdge origSrcEdge = layout.sourceEdge;
    NodeEdge origTgtEdge = layout.targetEdge;

    for (NodeEdge srcEdge : allEdges) {
        for (NodeEdge tgtEdge : allEdges) {
            // Skip current combination (already tried)
            if (srcEdge == origSrcEdge && tgtEdge == origTgtEdge) {
                continue;
            }

            result.edgeCombinationCount++;

            // Count existing edges on each NodeEdge for proper snap index allocation
            int srcEdgeCount = 0;
            int tgtEdgeCount = 0;
            if (otherEdges) {
                srcEdgeCount = countEdgesOnNodeEdge(*otherEdges, layout.from, srcEdge, layout.id);
                tgtEdgeCount = countEdgesOnNodeEdge(*otherEdges, layout.to, tgtEdge, layout.id);
            }

            // CRITICAL FIX: Try multiple snap positions per NodeEdge combination
            // The single position might be blocked by node obstacles
            int minPositions = 3;
            int srcEffectiveTotalCount = std::max(srcEdgeCount + 1, minPositions);
            int tgtEffectiveTotalCount = std::max(tgtEdgeCount + 1, minPositions);
            
            // Try several snap position combinations for this NodeEdge pair
            bool foundPath = false;
            Point successSrcPoint, successTgtPoint;
            int successSrcSnapIdx = 0, successTgtSnapIdx = 0;
            PathResult pathResult;
            
            for (int srcSnapIdx = 0; srcSnapIdx < srcEffectiveTotalCount && !foundPath; ++srcSnapIdx) {
                for (int tgtSnapIdx = 0; tgtSnapIdx < tgtEffectiveTotalCount && !foundPath; ++tgtSnapIdx) {
                    Point srcPoint = calculateSnapPositionQuantized(srcNode, srcEdge, srcSnapIdx, srcEffectiveTotalCount, 0.0f, 1.0f, effectiveGridSize);
                    Point tgtPoint = calculateSnapPositionQuantized(tgtNode, tgtEdge, tgtSnapIdx, tgtEffectiveTotalCount, 0.0f, 1.0f, effectiveGridSize);

#if EDGE_ROUTING_DEBUG
                    if (srcSnapIdx == 0 && tgtSnapIdx == 0) {
                        std::cout << "[A* RETRY] Edge " << layout.id << " trying srcEdge=" << static_cast<int>(srcEdge)
                                  << " tgtEdge=" << static_cast<int>(tgtEdge)
                                  << " srcSnapIdx=" << srcSnapIdx << "/" << srcEffectiveTotalCount
                                  << " tgtSnapIdx=" << tgtSnapIdx << "/" << tgtEffectiveTotalCount << std::endl;
                    }
#endif

                    ObstacleMap obstacles;
                    obstacles.buildFromNodes(nodeLayouts, effectiveGridSize);
                    GridPoint startGrid = obstacles.pixelToGrid(srcPoint);
                    GridPoint goalGrid = obstacles.pixelToGrid(tgtPoint);

                    // Add other edges as obstacles (filter stale bendPoints)
                    if (otherEdges) {
                        std::unordered_map<EdgeId, EdgeLayout> freshEdges;
                        for (const auto& [edgeId, edgeLayout] : *otherEdges) {
                            if (hasFreshBendPoints(edgeLayout, effectiveGridSize)) {
                                freshEdges[edgeId] = edgeLayout;
                            }
                        }
                        obstacles.addEdgeSegments(freshEdges, layout.id);
                    }

                    pathResult = activePathFinder().findPath(
                        startGrid, goalGrid, obstacles,
                        layout.from, layout.to,
                        srcEdge, tgtEdge,
                        {}, {});
                    
                    if (pathResult.found && pathResult.path.size() >= 2) {
                        foundPath = true;
                        successSrcPoint = srcPoint;
                        successTgtPoint = tgtPoint;
                        successSrcSnapIdx = srcSnapIdx;
                        successTgtSnapIdx = tgtSnapIdx;
                    }
                }
            }
            
            if (!foundPath) {
                continue;  // Try next NodeEdge combination
            }

            // foundPath == true, use the success variables
            result.success = true;
            result.nodeEdgeChanged = true;
            result.newSourceEdge = srcEdge;
            result.newTargetEdge = tgtEdge;
            result.newSourceSnapIndex = successSrcSnapIdx;
            result.newTargetSnapIndex = successTgtSnapIdx;
            result.snapIndexChanged = true;  // Also changing snap indices

            // Create obstacles map for coordinate conversion
            ObstacleMap coordObstacles;
            coordObstacles.buildFromNodes(nodeLayouts, effectiveGridSize);
            
            result.bendPoints.clear();
            for (size_t i = 1; i + 1 < pathResult.path.size(); ++i) {
                Point pixelPoint = coordObstacles.gridToPixel(pathResult.path[i].x, pathResult.path[i].y);
                result.bendPoints.push_back({pixelPoint});
            }

#if EDGE_ROUTING_DEBUG
            std::cout << "[A* RETRY] Edge " << layout.id << " NodeEdge switch: src="
                      << static_cast<int>(origSrcEdge) << "->" << static_cast<int>(srcEdge)
                      << " tgt=" << static_cast<int>(origTgtEdge) << "->" << static_cast<int>(tgtEdge)
                      << " snapIdx=(" << successSrcSnapIdx << "," << successTgtSnapIdx << ")" << std::endl;
#endif
            return result;
        }
    }

#if EDGE_ROUTING_DEBUG
    std::cout << "[A* RETRY] Edge " << layout.id << " NodeEdge switch FAILED after "
              << result.edgeCombinationCount << " combinations" << std::endl;
#endif

    return result;
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
    const LayoutOptions& options,
    bool skipOptimization) {

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

    // Skip optimization if requested (optimization will be called later after snap distribution)
    if (skipOptimization) {
        return result;
    }

    // Apply edge optimization if optimizer is injected or postDragAlgorithm is enabled
    IEdgeOptimizer* optimizer = edgeOptimizer_.get();
    std::unique_ptr<IEdgeOptimizer> fallbackOptimizer;

    // Use injected optimizer, or create based on postDragAlgorithm
    if (!optimizer && options.optimizationOptions.postDragAlgorithm != PostDragAlgorithm::None) {
        const float gridSize = options.gridConfig.cellSize;

        switch (options.optimizationOptions.postDragAlgorithm) {
            case PostDragAlgorithm::AStar: {
                // Create optimizer with penalty system for unified constraint handling
                OptimizerConfig config = OptimizerConfig::balanced();
                config.gridSize = gridSize;
                config.pathFinder = pathFinder_;  // Share pathfinder with optimizer
                config.penaltySystem = EdgePenaltySystem::createDefault();
                fallbackOptimizer = OptimizerRegistry::instance().create("AStar", config);
                break;
            }
            case PostDragAlgorithm::None:
                break;
        }

        // Note: Penalty system is already configured during creation via OptimizerRegistry
        // The EdgePenaltySystem replaces both EdgeConstraintManager and EdgeScorer
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

void EdgeRouting::optimizeRouting(
    Result& result,
    const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
    const LayoutOptions& options) {

    // Create optimizer based on postDragAlgorithm
    IEdgeOptimizer* optimizer = edgeOptimizer_.get();
    std::unique_ptr<IEdgeOptimizer> fallbackOptimizer;

    if (!optimizer && options.optimizationOptions.postDragAlgorithm != PostDragAlgorithm::None) {
        const float gridSize = options.gridConfig.cellSize;

        switch (options.optimizationOptions.postDragAlgorithm) {
            case PostDragAlgorithm::AStar: {
                OptimizerConfig config = OptimizerConfig::balanced();
                config.gridSize = gridSize;
                config.pathFinder = pathFinder_;
                config.penaltySystem = EdgePenaltySystem::createDefault();
                fallbackOptimizer = OptimizerRegistry::instance().create("AStar", config);
                break;
            }
            case PostDragAlgorithm::None:
                break;
        }
        optimizer = fallbackOptimizer.get();
    }

    if (!optimizer || result.edgeLayouts.empty()) {
        return;
    }

    // Collect all edge IDs for optimization
    std::vector<EdgeId> edgeIds;
    edgeIds.reserve(result.edgeLayouts.size());
    for (const auto& [edgeId, layout] : result.edgeLayouts) {
        edgeIds.push_back(edgeId);
    }

    // Run optimizer on edges with their final snap positions
    auto optimizedLayouts = optimizer->optimize(edgeIds, result.edgeLayouts, nodeLayouts);

    // Merge optimized layouts
    for (auto& [edgeId, layout] : optimizedLayouts) {
        result.edgeLayouts[edgeId] = std::move(layout);
    }

    // Update label positions after optimization
    for (auto& [edgeId, layout] : result.edgeLayouts) {
        layout.labelPosition = LayoutUtils::calculateEdgeLabelPosition(layout);
    }
}

EdgeRouting::SnapUpdateResult EdgeRouting::updateSnapPositions(
    std::unordered_map<EdgeId, EdgeLayout>& edgeLayouts,
    const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
    const std::vector<EdgeId>& affectedEdges,
    const std::unordered_set<NodeId>& movedNodes,
    float gridSize,
    bool skipBendPointRecalc) {

    static int callCounter = 0;
    int thisCallId = ++callCounter;

    SnapUpdateResult result;
    float effectiveGridSize = getEffectiveGridSize(gridSize);

#if EDGE_ROUTING_DEBUG
    std::cout << "\n[updateSnapPositions CALL #" << thisCallId << "] "
              << "affectedEdges=" << affectedEdges.size()
              << " skipBendPointRecalc=" << (skipBendPointRecalc ? "true" : "false")
              << " edges: ";
    for (EdgeId eid : affectedEdges) std::cout << eid << " ";
    std::cout << std::endl;
#endif

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
    // When skipBendPointRecalc is true, optimizer has already set snap points.
    // We only need to assign snap INDICES, not recalculate snap POINTS.
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

            // When skipBendPointRecalc is true, optimizer has already set snap points.
            // However, for MOVED nodes, we must recalculate snap points to follow the node.
            // We only preserve optimizer snap points for UNMOVED nodes.
            bool nodeHasMoved = shouldUpdateNode(nodeId);
            
            if (skipBendPointRecalc && !nodeHasMoved) {
                // Node hasn't moved - preserve optimizer's snap point, only assign index
                if (isSource) {
                    layout.sourceSnapIndex = snapIdx;
#if EDGE_ROUTING_DEBUG
                    std::cout << "[CALL #" << thisCallId << " SNAP] Edge " << edgeId
                              << " sourceSnapIndex=" << snapIdx
                              << " (preserving optimizer point: "
                              << layout.sourcePoint.x << "," << layout.sourcePoint.y << ")" << std::endl;
#endif
                } else {
                    layout.targetSnapIndex = snapIdx;
#if EDGE_ROUTING_DEBUG
                    std::cout << "[CALL #" << thisCallId << " SNAP] Edge " << edgeId
                              << " targetSnapIndex=" << snapIdx
                              << " (preserving optimizer point: "
                              << layout.targetPoint.x << "," << layout.targetPoint.y << ")" << std::endl;
#endif
                }
            } else {
                // Normal mode: calculate snap point from snap index
                Point snapPoint = calculateSnapPositionQuantized(node, nodeEdge, snapIdx, totalCount, 0.0f, 1.0f, effectiveGridSize);

                if (isSource) {
#if EDGE_ROUTING_DEBUG
                    if (std::abs(layout.sourcePoint.x - snapPoint.x) > 0.1f || 
                        std::abs(layout.sourcePoint.y - snapPoint.y) > 0.1f) {
                        std::cout << "[CALL #" << thisCallId << " SNAP] Edge " << edgeId
                                  << " sourcePoint CHANGED: ("
                                  << layout.sourcePoint.x << "," << layout.sourcePoint.y << ") -> ("
                                  << snapPoint.x << "," << snapPoint.y << ")"
                                  << " needsRedist=" << (needsRedistribution ? "Y" : "N") << std::endl;
                    }
#endif
                    layout.sourcePoint = snapPoint;
                    layout.sourceSnapIndex = snapIdx;
                } else {
#if EDGE_ROUTING_DEBUG
                    if (std::abs(layout.targetPoint.x - snapPoint.x) > 0.1f || 
                        std::abs(layout.targetPoint.y - snapPoint.y) > 0.1f) {
                        std::cout << "[CALL #" << thisCallId << " SNAP] Edge " << edgeId
                                  << " targetPoint CHANGED: ("
                                  << layout.targetPoint.x << "," << layout.targetPoint.y << ") -> ("
                                  << snapPoint.x << "," << snapPoint.y << ")"
                                  << " needsRedist=" << (needsRedistribution ? "Y" : "N") << std::endl;
                    }
#endif
                    layout.targetPoint = snapPoint;
                    layout.targetSnapIndex = snapIdx;
                }
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
    std::cout << "[CALL #" << thisCallId << " Phase2 DONE] redistributedEdges: ";
    for (EdgeId eid : result.redistributedEdges) std::cout << eid << " ";
    std::cout << std::endl;

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
    // Skip this phase when bendPoints are already set by optimizer (e.g., during drag)
    if (skipBendPointRecalc) {
#if EDGE_ROUTING_DEBUG
        std::cout << "[CALL #" << thisCallId << " Phase4] SKIPPED for optimizer-processed edges" << std::endl;
#endif
        // Collect all edges that need bendPoints regeneration:
        // 1. affectedEdges - optimizer-processed edges with potentially stale bendPoints
        // 2. redistributedEdges NOT in affectedEdges - snap positions changed by redistribution
        std::unordered_set<EdgeId> affectedSet(affectedEdges.begin(), affectedEdges.end());
        std::vector<EdgeId> edgesToRegenerate;
        
        // Add all affected edges
        for (EdgeId edgeId : affectedEdges) {
            edgesToRegenerate.push_back(edgeId);
        }
        
        // Add redistributed edges that are NOT in affectedEdges
        for (EdgeId edgeId : result.redistributedEdges) {
            if (affectedSet.find(edgeId) == affectedSet.end()) {
#if EDGE_ROUTING_DEBUG
                std::cout << "[CALL #" << thisCallId << " Phase4] Edge " << edgeId
                          << " redistributed but not affected - adding to regeneration list" << std::endl;
#endif
                edgesToRegenerate.push_back(edgeId);
            }
        }

        // Regenerate bendPoints using optimizer's path generation algorithm
        // This delegates to the appropriate algorithm (Geometric or A*) based on
        // what optimizer is set, maintaining algorithm separation during drag.
        // 
        // Note: Full optimize() would be ideal for cooperative rerouting, but it
        // would need to be called with special handling to preserve the redistributed
        // snap positions. For now, use regenerateBendPoints which keeps snap positions
        // and only regenerates paths.
        if (edgeOptimizer_ && !edgesToRegenerate.empty()) {
#if EDGE_ROUTING_DEBUG
            std::cout << "[CALL #" << thisCallId << " Phase4] Regenerating bendPoints for "
                      << edgesToRegenerate.size() << " edges via " << edgeOptimizer_->algorithmName()
                      << " optimizer: ";
            for (EdgeId eid : edgesToRegenerate) std::cout << eid << " ";
            std::cout << std::endl;
#endif
            edgeOptimizer_->regenerateBendPoints(edgesToRegenerate, edgeLayouts, nodeLayouts);
        }

        // Update label positions for all processed edges
        for (EdgeId edgeId : result.processedEdges) {
            auto it = edgeLayouts.find(edgeId);
            if (it != edgeLayouts.end()) {
                it->second.labelPosition = LayoutUtils::calculateEdgeLabelPosition(it->second);
            }
        }
        return result;
    }

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

#if EDGE_ROUTING_DEBUG
        std::cout << "[Phase4] Edge " << edgeId << " BEFORE recalc: src=("
                  << it->second.sourcePoint.x << "," << it->second.sourcePoint.y << ") tgt=("
                  << it->second.targetPoint.x << "," << it->second.targetPoint.y << ") bends="
                  << it->second.bendPoints.size() << std::endl;
#endif

        // Recalculate bend points with overlap avoidance
#if EDGE_ROUTING_DEBUG
        std::cout << "[Phase4] Edge " << edgeId << " otherEdges.size=" << otherEdges.size() << " contains: ";
        for (const auto& [eid, _] : otherEdges) std::cout << eid << " ";
        std::cout << std::endl;
#endif
        recalculateBendPoints(it->second, nodeLayouts, effectiveGridSize, &otherEdges);

#if EDGE_ROUTING_DEBUG
        size_t bendsAfterAStar = it->second.bendPoints.size();
        std::cout << "[Phase4] Edge " << edgeId << " AFTER A*: bends=" << bendsAfterAStar << std::endl;
#endif

        // Check for DIAGONAL (A* failure indicator) and retry if detected
        // Must check BOTH source→firstBend AND lastBend→target for diagonals
        // Also check direct source→target diagonal when bendPoints is empty (A* complete failure)
        bool needsRetry = false;

        if (it->second.bendPoints.empty()) {
            // A* complete failure: check direct source→target diagonal
            float dx = std::abs(it->second.sourcePoint.x - it->second.targetPoint.x);
            float dy = std::abs(it->second.sourcePoint.y - it->second.targetPoint.y);
            if (dx > 1.0f && dy > 1.0f) {
                needsRetry = true;
#if EDGE_ROUTING_DEBUG
                std::cout << "[Phase4] Edge " << edgeId << " COMPLETE DIAGONAL (A* failed):"
                          << " dx=" << dx << " dy=" << dy << " - invoking SWAP retry" << std::endl;
#endif
            }
        } else {
            // Check source side: sourcePoint → firstBend
            float dx_src = std::abs(it->second.sourcePoint.x - it->second.bendPoints[0].position.x);
            float dy_src = std::abs(it->second.sourcePoint.y - it->second.bendPoints[0].position.y);

            // Check target side: lastBend → targetPoint
            const auto& lastBend = it->second.bendPoints.back();
            float dx_tgt = std::abs(lastBend.position.x - it->second.targetPoint.x);
            float dy_tgt = std::abs(lastBend.position.y - it->second.targetPoint.y);

            bool sourceDiagonal = (dx_src > 1.0f && dy_src > 1.0f);
            bool targetDiagonal = (dx_tgt > 1.0f && dy_tgt > 1.0f);

            if (sourceDiagonal || targetDiagonal) {
                needsRetry = true;
#if EDGE_ROUTING_DEBUG
                std::cout << "[Phase4] Edge " << edgeId << " DIAGONAL detected!";
                if (sourceDiagonal) {
                    std::cout << " SRC: dx=" << dx_src << " dy=" << dy_src;
                }
                if (targetDiagonal) {
                    std::cout << " TGT: dx=" << dx_tgt << " dy=" << dy_tgt;
                }
                std::cout << " - invoking retry system" << std::endl;
#endif
            }
        }

        if (needsRetry) {
            // SNAP SWAP RETRY: Find edges on the same NodeEdge and swap positions
            // Strategy: Try source-side swaps first, then target-side swaps

            // Save original state for restoration
            Point originalSourcePoint = it->second.sourcePoint;
            Point originalTargetPoint = it->second.targetPoint;
            int originalSourceSnapIndex = it->second.sourceSnapIndex;
            int originalTargetSnapIndex = it->second.targetSnapIndex;
            std::vector<BendPoint> originalBendPoints = it->second.bendPoints;

            bool swapSucceeded = false;

            // === PHASE 1: Try source-side swaps ===
            std::vector<EdgeId> srcSwapCandidates;
            for (const auto& [otherEdgeId, otherLayout] : edgeLayouts) {
                if (otherEdgeId == edgeId) continue;
                if (otherLayout.from == it->second.from &&
                    otherLayout.sourceEdge == it->second.sourceEdge) {
                    srcSwapCandidates.push_back(otherEdgeId);
                }
            }

            // === PHASE 2: Try target-side swaps ===
            std::vector<EdgeId> tgtSwapCandidates;
            for (const auto& [otherEdgeId, otherLayout] : edgeLayouts) {
                if (otherEdgeId == edgeId) continue;
                if (otherLayout.to == it->second.to &&
                    otherLayout.targetEdge == it->second.targetEdge) {
                    tgtSwapCandidates.push_back(otherEdgeId);
                }
            }

#if EDGE_ROUTING_DEBUG
            std::cout << "[Phase4] Edge " << edgeId << " SWAP RETRY: src_candidates="
                      << srcSwapCandidates.size() << " tgt_candidates=" << tgtSwapCandidates.size() << std::endl;
#endif

            // Try source-side swaps
            for (EdgeId swapWithId : srcSwapCandidates) {
                if (swapSucceeded) break;
                auto swapIt = edgeLayouts.find(swapWithId);
                if (swapIt == edgeLayouts.end()) continue;

                Point swapOriginalSourcePoint = swapIt->second.sourcePoint;
                int swapOriginalSnapIndex = swapIt->second.sourceSnapIndex;
                std::vector<BendPoint> swapOriginalBendPoints = swapIt->second.bendPoints;

#if EDGE_ROUTING_DEBUG
                std::cout << "[Phase4] Edge " << edgeId << " trying SRC SWAP with Edge " << swapWithId << std::endl;
#endif

                std::swap(it->second.sourcePoint, swapIt->second.sourcePoint);
                std::swap(it->second.sourceSnapIndex, swapIt->second.sourceSnapIndex);

                recalculateBendPoints(it->second, nodeLayouts, effectiveGridSize, nullptr);
                recalculateBendPoints(swapIt->second, nodeLayouts, effectiveGridSize, nullptr);

                bool stillDiagonal = false;
                if (it->second.bendPoints.empty()) {
                    float dx = std::abs(it->second.sourcePoint.x - it->second.targetPoint.x);
                    float dy = std::abs(it->second.sourcePoint.y - it->second.targetPoint.y);
                    stillDiagonal = (dx > 1.0f && dy > 1.0f);
                }

                bool swapBecameDiagonal = false;
                if (swapIt->second.bendPoints.empty()) {
                    float dx = std::abs(swapIt->second.sourcePoint.x - swapIt->second.targetPoint.x);
                    float dy = std::abs(swapIt->second.sourcePoint.y - swapIt->second.targetPoint.y);
                    swapBecameDiagonal = (dx > 1.0f && dy > 1.0f);
                }

                if (!stillDiagonal && !swapBecameDiagonal) {
#if EDGE_ROUTING_DEBUG
                    std::cout << "[Phase4] Edge " << edgeId << " SRC SWAP SUCCESS" << std::endl;
#endif
                    swapSucceeded = true;
                } else {
                    it->second.sourcePoint = originalSourcePoint;
                    it->second.sourceSnapIndex = originalSourceSnapIndex;
                    it->second.bendPoints = originalBendPoints;
                    swapIt->second.sourcePoint = swapOriginalSourcePoint;
                    swapIt->second.sourceSnapIndex = swapOriginalSnapIndex;
                    swapIt->second.bendPoints = swapOriginalBendPoints;
                }
            }

            // Try target-side swaps if source swaps failed
            for (EdgeId swapWithId : tgtSwapCandidates) {
                if (swapSucceeded) break;
                auto swapIt = edgeLayouts.find(swapWithId);
                if (swapIt == edgeLayouts.end()) continue;

                Point swapOriginalTargetPoint = swapIt->second.targetPoint;
                int swapOriginalSnapIndex = swapIt->second.targetSnapIndex;
                std::vector<BendPoint> swapOriginalBendPoints = swapIt->second.bendPoints;

#if EDGE_ROUTING_DEBUG
                std::cout << "[Phase4] Edge " << edgeId << " trying TGT SWAP with Edge " << swapWithId << std::endl;
#endif

                std::swap(it->second.targetPoint, swapIt->second.targetPoint);
                std::swap(it->second.targetSnapIndex, swapIt->second.targetSnapIndex);

                recalculateBendPoints(it->second, nodeLayouts, effectiveGridSize, nullptr);
                recalculateBendPoints(swapIt->second, nodeLayouts, effectiveGridSize, nullptr);

                bool stillDiagonal = false;
                if (it->second.bendPoints.empty()) {
                    float dx = std::abs(it->second.sourcePoint.x - it->second.targetPoint.x);
                    float dy = std::abs(it->second.sourcePoint.y - it->second.targetPoint.y);
                    stillDiagonal = (dx > 1.0f && dy > 1.0f);
                }

                bool swapBecameDiagonal = false;
                if (swapIt->second.bendPoints.empty()) {
                    float dx = std::abs(swapIt->second.sourcePoint.x - swapIt->second.targetPoint.x);
                    float dy = std::abs(swapIt->second.sourcePoint.y - swapIt->second.targetPoint.y);
                    swapBecameDiagonal = (dx > 1.0f && dy > 1.0f);
                }

                if (!stillDiagonal && !swapBecameDiagonal) {
#if EDGE_ROUTING_DEBUG
                    std::cout << "[Phase4] Edge " << edgeId << " TGT SWAP SUCCESS" << std::endl;
#endif
                    swapSucceeded = true;
                } else {
                    it->second.targetPoint = originalTargetPoint;
                    it->second.targetSnapIndex = originalTargetSnapIndex;
                    it->second.bendPoints = originalBendPoints;
                    swapIt->second.targetPoint = swapOriginalTargetPoint;
                    swapIt->second.targetSnapIndex = swapOriginalSnapIndex;
                    swapIt->second.bendPoints = swapOriginalBendPoints;
                }
            }

            // NO FALLBACK: If all snap swaps failed, try exhaustive NodeEdge + snap index search
            // This follows the principle: no geometric fallback, only A* with different snap positions
            if (!swapSucceeded) {
                // === PHASE 3: Exhaustive NodeEdge and snap index search ===
                // Try ALL NodeEdge combinations and ALL snap positions
                auto srcNodeIt = nodeLayouts.find(it->second.from);
                auto tgtNodeIt = nodeLayouts.find(it->second.to);
                
                if (srcNodeIt != nodeLayouts.end() && tgtNodeIt != nodeLayouts.end()) {
                    const auto& srcNode = srcNodeIt->second;
                    const auto& tgtNode = tgtNodeIt->second;
                    
                    constexpr std::array<NodeEdge, 4> allEdges = {
                        NodeEdge::Top, NodeEdge::Bottom, NodeEdge::Left, NodeEdge::Right
                    };
                    
                    NodeEdge origSrcEdge = it->second.sourceEdge;
                    NodeEdge origTgtEdge = it->second.targetEdge;
                    
#if EDGE_ROUTING_DEBUG
                    std::cout << "[Phase4] Edge " << edgeId << " EXHAUSTIVE NodeEdge SEARCH starting" << std::endl;
#endif
                    
                    bool exhaustiveSuccess = false;
                    
                    // Try all 16 NodeEdge combinations
                    for (NodeEdge srcEdge : allEdges) {
                        if (exhaustiveSuccess) break;
                        for (NodeEdge tgtEdge : allEdges) {
                            if (exhaustiveSuccess) break;
                            
                            // Calculate edge lengths for this combination
                            float srcEdgeLength = (srcEdge == NodeEdge::Top || srcEdge == NodeEdge::Bottom) 
                                                  ? srcNode.size.width : srcNode.size.height;
                            float tgtEdgeLength = (tgtEdge == NodeEdge::Top || tgtEdge == NodeEdge::Bottom) 
                                                  ? tgtNode.size.width : tgtNode.size.height;
                            
                            int srcMaxIndex = std::max(1, static_cast<int>(srcEdgeLength / effectiveGridSize));
                            int tgtMaxIndex = std::max(1, static_cast<int>(tgtEdgeLength / effectiveGridSize));
                            
                            // Try all snap index combinations for this NodeEdge pair
                            for (int srcIdx = 0; srcIdx <= srcMaxIndex && !exhaustiveSuccess; ++srcIdx) {
                                for (int tgtIdx = 0; tgtIdx <= tgtMaxIndex && !exhaustiveSuccess; ++tgtIdx) {
                                    // Skip original combination
                                    if (srcEdge == origSrcEdge && tgtEdge == origTgtEdge && 
                                        srcIdx == originalSourceSnapIndex && tgtIdx == originalTargetSnapIndex) {
                                        continue;
                                    }
                                    
                                    // Calculate new positions
                                    Point newSrc = calculateSnapPositionQuantized(srcNode, srcEdge, srcIdx, srcMaxIndex + 1, 0.0f, 1.0f, effectiveGridSize);
                                    Point newTgt = calculateSnapPositionQuantized(tgtNode, tgtEdge, tgtIdx, tgtMaxIndex + 1, 0.0f, 1.0f, effectiveGridSize);
                                    
                                    // Rebuild obstacle map with margin=0 (no buffer around nodes)
                                    // This allows paths to start/end at node boundaries
                                    ObstacleMap obstacles;
                                    obstacles.buildFromNodes(nodeLayouts, effectiveGridSize, 0);
                                    
                                    GridPoint startGrid = obstacles.pixelToGrid(newSrc);
                                    GridPoint goalGrid = obstacles.pixelToGrid(newTgt);
                                    
                                    // Try A* with new positions and NodeEdges
                                    PathResult pathResult = activePathFinder().findPath(
                                        startGrid, goalGrid, obstacles,
                                        it->second.from, it->second.to,
                                        srcEdge, tgtEdge,
                                        {}, {});
                                    
                                    if (pathResult.found && pathResult.path.size() >= 2) {
                                        // Success! Apply the new configuration
                                        it->second.sourceEdge = srcEdge;
                                        it->second.targetEdge = tgtEdge;
                                        it->second.sourcePoint = newSrc;
                                        it->second.targetPoint = newTgt;
                                        it->second.sourceSnapIndex = srcIdx;
                                        it->second.targetSnapIndex = tgtIdx;
                                        it->second.bendPoints.clear();
                                        
                                        // Convert grid path to pixel bend points
                                        for (size_t i = 1; i + 1 < pathResult.path.size(); ++i) {
                                            const auto& gp = pathResult.path[i];
                                            Point pixelPoint = obstacles.gridToPixel(gp.x, gp.y);
                                            it->second.bendPoints.push_back(BendPoint{pixelPoint, false});
                                        }
                                        exhaustiveSuccess = true;
#if EDGE_ROUTING_DEBUG
                                        std::cout << "[Phase4] Edge " << edgeId << " EXHAUSTIVE SUCCESS: srcEdge=" 
                                                  << static_cast<int>(srcEdge) << " tgtEdge=" << static_cast<int>(tgtEdge)
                                                  << " srcIdx=" << srcIdx << " tgtIdx=" << tgtIdx << std::endl;
#endif
                                    }
                                }
                            }
                        }
                    }
                    
                    // If exhaustive search failed, restore original
                    if (!exhaustiveSuccess) {
                        it->second.sourcePoint = originalSourcePoint;
                        it->second.targetPoint = originalTargetPoint;
                        it->second.sourceSnapIndex = originalSourceSnapIndex;
                        it->second.targetSnapIndex = originalTargetSnapIndex;
                        it->second.sourceEdge = origSrcEdge;
                        it->second.targetEdge = origTgtEdge;
                        it->second.bendPoints = originalBendPoints;
#if EDGE_ROUTING_DEBUG
                        std::cout << "[Phase4] Edge " << edgeId << " EXHAUSTIVE SEARCH FAILED - no valid path in any NodeEdge combination" << std::endl;
#endif
                    }
                }
            }
        }

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

#if EDGE_ROUTING_DEBUG
        std::cout << "[Phase4] Edge " << edgeId << " AFTER PathCleanup: bends=" << it->second.bendPoints.size() << std::endl;
#endif

        // CRITICAL: Validate direction constraints after PathCleanup
        // PathCleanup may remove bend points needed for direction constraints
        // Check BOTH orthogonality AND direction (up/down for vertical, left/right for horizontal)
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
            
            // Check orthogonality
            bool orthogonalViolation = (sourceVertical != firstSegmentVertical || 
                                        targetVertical != lastSegmentVertical);
            
            // Check direction for sourceEdge (up/down for vertical, left/right for horizontal)
            bool sourceDirectionViolation = false;
            if (!orthogonalViolation && sourceVertical) {
                float dyFirst = firstBend.y - it->second.sourcePoint.y;
                bool shouldGoUp = (it->second.sourceEdge == NodeEdge::Top);
                bool shouldGoDown = (it->second.sourceEdge == NodeEdge::Bottom);
                if ((shouldGoUp && dyFirst > DIRECTION_TOLERANCE) || 
                    (shouldGoDown && dyFirst < -DIRECTION_TOLERANCE)) {
                    sourceDirectionViolation = true;
                }
            } else if (!orthogonalViolation && !sourceVertical) {
                float dxFirst = firstBend.x - it->second.sourcePoint.x;
                bool shouldGoLeft = (it->second.sourceEdge == NodeEdge::Left);
                bool shouldGoRight = (it->second.sourceEdge == NodeEdge::Right);
                if ((shouldGoLeft && dxFirst > DIRECTION_TOLERANCE) || 
                    (shouldGoRight && dxFirst < -DIRECTION_TOLERANCE)) {
                    sourceDirectionViolation = true;
                }
            }
            
            // Check direction for targetEdge (approach direction)
            bool targetDirectionViolation = false;
            if (!orthogonalViolation && targetVertical) {
                float dyLast = it->second.targetPoint.y - lastBend.y;
                bool shouldEnterFromAbove = (it->second.targetEdge == NodeEdge::Top);  // dy > 0
                bool shouldEnterFromBelow = (it->second.targetEdge == NodeEdge::Bottom);  // dy < 0
                if ((shouldEnterFromAbove && dyLast < -DIRECTION_TOLERANCE) || 
                    (shouldEnterFromBelow && dyLast > DIRECTION_TOLERANCE)) {
                    targetDirectionViolation = true;
                }
            } else if (!orthogonalViolation && !targetVertical) {
                float dxLast = it->second.targetPoint.x - lastBend.x;
                bool shouldEnterFromLeft = (it->second.targetEdge == NodeEdge::Left);  // dx > 0
                bool shouldEnterFromRight = (it->second.targetEdge == NodeEdge::Right);  // dx < 0
                if ((shouldEnterFromLeft && dxLast < -DIRECTION_TOLERANCE) || 
                    (shouldEnterFromRight && dxLast > DIRECTION_TOLERANCE)) {
                    targetDirectionViolation = true;
                }
            }

            // Direction constraint violated - FIX BY INSERTING CLEARANCE SEGMENTS
            // This approach maintains the declared direction while creating a valid path
            // We do NOT change sourceEdge/targetEdge (that would cause "direction changes")
            if (sourceDirectionViolation && !it->second.bendPoints.empty()) {
                // First segment goes in wrong direction - insert clearance to fix
                // The first segment must go in the declared direction, then turn
                bool sourceVertical = (it->second.sourceEdge == NodeEdge::Top || 
                                       it->second.sourceEdge == NodeEdge::Bottom);
                
                const Point& firstBend = it->second.bendPoints.front().position;
                Point clearancePoint;
                
                if (sourceVertical) {
                    // Need vertical first segment in correct direction
                    bool shouldGoUp = (it->second.sourceEdge == NodeEdge::Top);
                    float clearanceY = shouldGoUp 
                        ? std::min(it->second.sourcePoint.y - effectiveGridSize * 2, firstBend.y - effectiveGridSize)
                        : std::max(it->second.sourcePoint.y + effectiveGridSize * 2, firstBend.y + effectiveGridSize);
                    if (effectiveGridSize > 0) {
                        clearanceY = std::round(clearanceY / effectiveGridSize) * effectiveGridSize;
                    }
                    
                    // Check if sourcePoint and firstBend are on same vertical line
                    bool sameVerticalLine = std::abs(it->second.sourcePoint.x - firstBend.x) < 0.5f;
                    
                    if (sameVerticalLine) {
                        // Same X - adding clearance creates spike if directions differ
                        // Path: source → clearance → firstBend
                        // Spike if: direction(source→clearance) != direction(clearance→firstBend)
                        bool sourceToClearanceGoesUp = clearanceY < it->second.sourcePoint.y;
                        bool clearanceToFirstBendGoesUp = firstBend.y < clearanceY;
                        
                        // Spike occurs if path reverses direction
                        bool wouldCreateSpike = (sourceToClearanceGoesUp != clearanceToFirstBendGoesUp);
                        
                        if (wouldCreateSpike) {
                            // Add horizontal detour
                            float offsetX = effectiveGridSize > 0 ? effectiveGridSize * 2 : 40.0f;
                            // Choose direction based on target position
                            if (it->second.targetPoint.x > it->second.sourcePoint.x) {
                                offsetX = -offsetX;  // Target is right, detour left
                            }
                            
                            float detourX = it->second.sourcePoint.x + offsetX;
                            if (effectiveGridSize > 0) {
                                detourX = std::round(detourX / effectiveGridSize) * effectiveGridSize;
                            }
                            
                            clearancePoint = {it->second.sourcePoint.x, clearanceY};
                            Point detourPoint1 = {detourX, clearanceY};    // horizontal step
                            Point detourPoint2 = {detourX, firstBend.y};   // vertical step
                            // Connection point connects to firstBend on same Y
                            Point connectionPoint = {firstBend.x, firstBend.y};  // This is actually firstBend itself
                            
                            // Insert in reverse order since we're using insert at begin
                            // Final order: source → clearance → detour1 → detour2 → firstBend
                            it->second.bendPoints.insert(it->second.bendPoints.begin(), {detourPoint2});
                            it->second.bendPoints.insert(it->second.bendPoints.begin(), {detourPoint1});
                            it->second.bendPoints.insert(it->second.bendPoints.begin(), {clearancePoint});
                        } else {
                            // No spike - use simple clearance point
                            clearancePoint = {it->second.sourcePoint.x, clearanceY};
                            it->second.bendPoints.insert(it->second.bendPoints.begin(), {clearancePoint});
                        }
                    } else {
                        // Different X - original logic works fine
                        clearancePoint = {it->second.sourcePoint.x, clearanceY};
                        Point connectionPoint = {firstBend.x, clearanceY};
                        if (effectiveGridSize > 0) {
                            connectionPoint.x = std::round(connectionPoint.x / effectiveGridSize) * effectiveGridSize;
                        }
                        it->second.bendPoints.insert(it->second.bendPoints.begin(), {connectionPoint});
                        it->second.bendPoints.insert(it->second.bendPoints.begin(), {clearancePoint});
                    }
                } else {
                    // Need horizontal first segment in correct direction
                    bool shouldGoLeft = (it->second.sourceEdge == NodeEdge::Left);
                    float clearanceX = shouldGoLeft
                        ? std::min(it->second.sourcePoint.x - effectiveGridSize * 2, firstBend.x - effectiveGridSize)
                        : std::max(it->second.sourcePoint.x + effectiveGridSize * 2, firstBend.x + effectiveGridSize);
                    if (effectiveGridSize > 0) {
                        clearanceX = std::round(clearanceX / effectiveGridSize) * effectiveGridSize;
                    }
                    
                    // Check if sourcePoint and firstBend are on same horizontal line
                    bool sameHorizontalLine = std::abs(it->second.sourcePoint.y - firstBend.y) < 0.5f;
                    
                    if (sameHorizontalLine) {
                        // Same Y - adding clearance creates spike if directions differ
                        // Path: source → clearance → firstBend
                        bool sourceToClearanceGoesLeft = clearanceX < it->second.sourcePoint.x;
                        bool clearanceToFirstBendGoesLeft = firstBend.x < clearanceX;
                        
                        // Spike occurs if path reverses direction
                        bool wouldCreateSpike = (sourceToClearanceGoesLeft != clearanceToFirstBendGoesLeft);
                        
                        if (wouldCreateSpike) {
                            // Add vertical detour
                            float offsetY = effectiveGridSize > 0 ? effectiveGridSize * 2 : 40.0f;
                            // Choose direction based on target position
                            if (it->second.targetPoint.y > it->second.sourcePoint.y) {
                                offsetY = -offsetY;  // Target is below, detour up
                            }
                            
                            float detourY = it->second.sourcePoint.y + offsetY;
                            if (effectiveGridSize > 0) {
                                detourY = std::round(detourY / effectiveGridSize) * effectiveGridSize;
                            }
                            
                            clearancePoint = {clearanceX, it->second.sourcePoint.y};
                            Point detourPoint1 = {clearanceX, detourY};    // vertical step
                            Point detourPoint2 = {firstBend.x, detourY};   // horizontal step
                            
                            it->second.bendPoints.insert(it->second.bendPoints.begin(), {detourPoint2});
                            it->second.bendPoints.insert(it->second.bendPoints.begin(), {detourPoint1});
                            it->second.bendPoints.insert(it->second.bendPoints.begin(), {clearancePoint});
                        } else {
                            // No spike - use simple clearance point
                            clearancePoint = {clearanceX, it->second.sourcePoint.y};
                            it->second.bendPoints.insert(it->second.bendPoints.begin(), {clearancePoint});
                        }
                    } else {
                        // Different Y - original logic works fine
                        clearancePoint = {clearanceX, it->second.sourcePoint.y};
                        Point connectionPoint = {clearanceX, firstBend.y};
                        if (effectiveGridSize > 0) {
                            connectionPoint.y = std::round(connectionPoint.y / effectiveGridSize) * effectiveGridSize;
                        }
                        it->second.bendPoints.insert(it->second.bendPoints.begin(), {connectionPoint});
                        it->second.bendPoints.insert(it->second.bendPoints.begin(), {clearancePoint});
                    }
                }
            }
            
            if (targetDirectionViolation && !it->second.bendPoints.empty()) {
                // Last segment goes in wrong direction - insert clearance to fix
                bool targetVertical = (it->second.targetEdge == NodeEdge::Top || 
                                       it->second.targetEdge == NodeEdge::Bottom);
                
                const Point& lastBend = it->second.bendPoints.back().position;
                Point clearancePoint;
                
                if (targetVertical) {
                    // Need vertical last segment arriving from correct direction
                    bool shouldEnterFromAbove = (it->second.targetEdge == NodeEdge::Top);
                    float clearanceY = shouldEnterFromAbove
                        ? std::min(it->second.targetPoint.y - effectiveGridSize * 2, lastBend.y - effectiveGridSize)
                        : std::max(it->second.targetPoint.y + effectiveGridSize * 2, lastBend.y + effectiveGridSize);
                    if (effectiveGridSize > 0) {
                        clearanceY = std::round(clearanceY / effectiveGridSize) * effectiveGridSize;
                    }
                    
                    // Check if lastBend and targetPoint are on same vertical line
                    // This would create a spike: lastBend → connectionPoint → targetPoint all on same X
                    bool sameVerticalLine = std::abs(lastBend.x - it->second.targetPoint.x) < 0.5f;
                    
                    if (sameVerticalLine) {
                        // Same X - adding clearance creates spike if directions differ
                        // Path: lastBend → clearance → targetPoint
                        bool lastBendToClearanceGoesUp = clearanceY < lastBend.y;
                        bool clearanceToTargetGoesUp = it->second.targetPoint.y < clearanceY;
                        
                        // Spike occurs if path reverses direction
                        bool wouldCreateSpike = (lastBendToClearanceGoesUp != clearanceToTargetGoesUp);
                        
                        if (wouldCreateSpike) {
                            // Add horizontal detour: go sideways, then to clearanceY, then back to target X
                            float offsetX = effectiveGridSize > 0 ? effectiveGridSize * 2 : 40.0f;
                            // Choose direction based on source position (go opposite side)
                            if (it->second.sourcePoint.x > it->second.targetPoint.x) {
                                offsetX = -offsetX;  // Source is right, detour left
                            }
                            
                            float detourX = lastBend.x + offsetX;
                            if (effectiveGridSize > 0) {
                                detourX = std::round(detourX / effectiveGridSize) * effectiveGridSize;
                            }
                            
                            Point detourPoint1 = {detourX, lastBend.y};     // horizontal step
                            Point detourPoint2 = {detourX, clearanceY};    // vertical step
                            clearancePoint = {it->second.targetPoint.x, clearanceY};  // horizontal back
                            
                            it->second.bendPoints.push_back({detourPoint1});
                            it->second.bendPoints.push_back({detourPoint2});
                            it->second.bendPoints.push_back({clearancePoint});
                        } else {
                            // No spike - use original logic (connectionPoint equals clearancePoint)
                            clearancePoint = {it->second.targetPoint.x, clearanceY};
                            it->second.bendPoints.push_back({clearancePoint});
                        }
                    } else {
                        // Different X - original logic works fine
                        Point connectionPoint = {lastBend.x, clearanceY};
                        if (effectiveGridSize > 0) {
                            connectionPoint.x = std::round(connectionPoint.x / effectiveGridSize) * effectiveGridSize;
                        }
                        clearancePoint = {it->second.targetPoint.x, clearanceY};
                        it->second.bendPoints.push_back({connectionPoint});
                        it->second.bendPoints.push_back({clearancePoint});
                    }
                } else {
                    // Need horizontal last segment arriving from correct direction
                    bool shouldEnterFromLeft = (it->second.targetEdge == NodeEdge::Left);
                    float clearanceX = shouldEnterFromLeft
                        ? std::min(it->second.targetPoint.x - effectiveGridSize * 2, lastBend.x - effectiveGridSize)
                        : std::max(it->second.targetPoint.x + effectiveGridSize * 2, lastBend.x + effectiveGridSize);
                    if (effectiveGridSize > 0) {
                        clearanceX = std::round(clearanceX / effectiveGridSize) * effectiveGridSize;
                    }
                    
                    // Check if lastBend and targetPoint are on same horizontal line
                    // This would create a spike: lastBend → connectionPoint → targetPoint all on same Y
                    bool sameHorizontalLine = std::abs(lastBend.y - it->second.targetPoint.y) < 0.5f;
                    
                    if (sameHorizontalLine) {
                        // Same Y - adding clearance creates spike if directions differ
                        // Path: lastBend → clearance → targetPoint
                        bool lastBendToClearanceGoesLeft = clearanceX < lastBend.x;
                        bool clearanceToTargetGoesLeft = it->second.targetPoint.x < clearanceX;
                        
                        // Spike occurs if path reverses direction
                        bool wouldCreateSpike = (lastBendToClearanceGoesLeft != clearanceToTargetGoesLeft);
                        
                        if (wouldCreateSpike) {
                            // Add vertical detour: go up/down, then to clearanceX, then back to target Y
                            float offsetY = effectiveGridSize > 0 ? effectiveGridSize * 2 : 40.0f;
                            // Choose direction based on source position
                            if (it->second.sourcePoint.y > it->second.targetPoint.y) {
                                offsetY = -offsetY;  // Source is below, detour up
                            }
                            
                            float detourY = lastBend.y + offsetY;
                            if (effectiveGridSize > 0) {
                                detourY = std::round(detourY / effectiveGridSize) * effectiveGridSize;
                            }
                            
                            Point detourPoint1 = {lastBend.x, detourY};     // vertical step
                            Point detourPoint2 = {clearanceX, detourY};    // horizontal step
                            clearancePoint = {clearanceX, it->second.targetPoint.y};  // vertical back
                            
                            it->second.bendPoints.push_back({detourPoint1});
                            it->second.bendPoints.push_back({detourPoint2});
                            it->second.bendPoints.push_back({clearancePoint});
                        } else {
                            // No spike - use original logic
                            clearancePoint = {clearanceX, it->second.targetPoint.y};
                            it->second.bendPoints.push_back({clearancePoint});
                        }
                    } else {
                        // Different Y - original logic works fine
                        Point connectionPoint = {clearanceX, lastBend.y};
                        if (effectiveGridSize > 0) {
                            connectionPoint.y = std::round(connectionPoint.y / effectiveGridSize) * effectiveGridSize;
                        }
                        clearancePoint = {clearanceX, it->second.targetPoint.y};
                        it->second.bendPoints.push_back({connectionPoint});
                        it->second.bendPoints.push_back({clearancePoint});
                    }
                }
            }
            
            (void)orthogonalViolation;
            
            // Clean up spikes and duplicates created by direction fix
            // PathCleanup::removeSpikesAndDuplicates now protects first/last bends
            // for direction constraints, so it's safe to call after direction fix
            if (sourceDirectionViolation || targetDirectionViolation) {
                std::vector<Point> allPoints;
                allPoints.push_back(it->second.sourcePoint);
                for (const auto& bp : it->second.bendPoints) {
                    allPoints.push_back(bp.position);
                }
                allPoints.push_back(it->second.targetPoint);
                
                PathCleanup::removeSpikesAndDuplicates(allPoints);
                
                // Reconstruct bendPoints from cleaned path
                it->second.bendPoints.clear();
                for (size_t j = 1; j + 1 < allPoints.size(); ++j) {
                    it->second.bendPoints.push_back({allPoints[j]});
                }
                
                // Run overlap avoidance after direction fix created new segments
                recalculateBendPoints(it->second, nodeLayouts, effectiveGridSize, &otherEdges);
            }
        }

        // IMPORTANT: Add to otherEdges AFTER all modifications (PathCleanup, direction validation, overlap avoidance)
        // so subsequent edges see the FINAL state, not an intermediate state
        otherEdges[edgeId] = it->second;

#if EDGE_ROUTING_DEBUG
        if (bendsAfterAStar != it->second.bendPoints.size()) {
            std::cout << "[EdgeRouting] updateSnapPositions: PathCleanup changed bendPoints for edge " << edgeId
                      << " from " << bendsAfterAStar << " to " << it->second.bendPoints.size() << std::endl;
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

    // Option B: If retry required full re-routing, re-route all affected edges now
    // This ensures proper obstacle avoidance after snap point changes
    if (result.needsFullReroute && !result.edgesNeedingReroute.empty()) {
        std::cout << "[EdgeRouting] updateSnapPositions: Full re-route needed for "
                  << result.edgesNeedingReroute.size() << " edges after retry" << std::endl;

        // Build list of all edges to re-route (affected + those needing re-route)
        std::vector<EdgeId> edgesToReroute;
        for (EdgeId edgeId : affectedEdges) {
            if (edgeLayouts.find(edgeId) != edgeLayouts.end()) {
                edgesToReroute.push_back(edgeId);
            }
        }

        // Sort by priority (layer, etc.) to route in correct order
        std::sort(edgesToReroute.begin(), edgesToReroute.end(), [&](EdgeId a, EdgeId b) {
            auto itA = edgeLayouts.find(a);
            auto itB = edgeLayouts.find(b);
            if (itA == edgeLayouts.end() || itB == edgeLayouts.end()) return a < b;
            // Sort by source layer, then target layer
            auto srcA = nodeLayouts.find(itA->second.from);
            auto srcB = nodeLayouts.find(itB->second.from);
            if (srcA != nodeLayouts.end() && srcB != nodeLayouts.end()) {
                if (srcA->second.layer != srcB->second.layer) {
                    return srcA->second.layer < srcB->second.layer;
                }
            }
            return a < b;
        });

        // Re-route each edge with proper obstacle map
        std::unordered_map<EdgeId, EdgeLayout> otherEdges;
        for (EdgeId edgeId : edgesToReroute) {
            auto it = edgeLayouts.find(edgeId);
            if (it == edgeLayouts.end()) continue;

            // Build obstacle map with already-routed edges
            ObstacleMap obstacles;
            obstacles.buildFromNodes(nodeLayouts, effectiveGridSize);

            // Add segments from edges already routed in this pass
            if (!otherEdges.empty()) {
                obstacles.addEdgeSegments(otherEdges, edgeId);
            }

            // Also add segments from edges not being re-routed
            std::unordered_map<EdgeId, EdgeLayout> staticEdges;
            for (const auto& [otherId, otherLayout] : edgeLayouts) {
                if (otherId == edgeId) continue;
                if (std::find(edgesToReroute.begin(), edgesToReroute.end(), otherId) != edgesToReroute.end()) continue;
                if (hasFreshBendPoints(otherLayout, effectiveGridSize)) {
                    staticEdges[otherId] = otherLayout;
                }
            }
            obstacles.addEdgeSegments(staticEdges, edgeId);

            GridPoint startGrid = obstacles.pixelToGrid(it->second.sourcePoint);
            GridPoint goalGrid = obstacles.pixelToGrid(it->second.targetPoint);

            PathResult pathResult = activePathFinder().findPath(
                startGrid, goalGrid, obstacles,
                it->second.from, it->second.to,
                it->second.sourceEdge, it->second.targetEdge,
                {}, {});

            if (pathResult.found && pathResult.path.size() >= 2) {
                it->second.bendPoints.clear();
                for (size_t i = 1; i + 1 < pathResult.path.size(); ++i) {
                    Point pixelPoint = obstacles.gridToPixel(pathResult.path[i].x, pathResult.path[i].y);
                    it->second.bendPoints.push_back({pixelPoint});
                }
                std::cout << "[EdgeRouting] Full re-route Edge " << edgeId 
                          << " SUCCESS: bends=" << it->second.bendPoints.size() << std::endl;
            } else {
                std::cout << "[EdgeRouting] Full re-route Edge " << edgeId << " FAILED" << std::endl;
            }

            // Add this edge to otherEdges for subsequent routing
            otherEdges[edgeId] = it->second;

            // Update label position
            it->second.labelPosition = LayoutUtils::calculateEdgeLabelPosition(it->second);
        }
    }

    // Post-processing removed - A* should find correct paths with obstacles
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
    bool usedDragOptimizer = false;  // Track if we used drag optimizer (skip A* if true)

    // Apply drag algorithm if enabled
    if (options.optimizationOptions.dragAlgorithm != DragAlgorithm::None && !affectedEdges.empty()) {
        std::unique_ptr<IEdgeOptimizer> optimizer;

        switch (options.optimizationOptions.dragAlgorithm) {
            case DragAlgorithm::Geometric: {
                // Create optimizer with penalty system and direction preservation
                OptimizerConfig config = OptimizerConfig::aggressive();  // Fast for drag
                config.preserveDirections = true;  // Keep existing edge directions during drag
                config.penaltySystem = EdgePenaltySystem::createDefault();
                optimizer = OptimizerRegistry::instance().create("Geometric", config);
                break;
            }
            case DragAlgorithm::AStar: {
                // A* pathfinding after drop - optimal paths with full optimization
                OptimizerConfig config = OptimizerConfig::aggressive();
                config.preserveDirections = false;  // Try all 16 edge combinations for best path
                config.gridSize = options.gridConfig.cellSize;
                config.pathFinder = pathFinder_;  // Share pathfinder
                config.penaltySystem = EdgePenaltySystem::createDefault();
                optimizer = OptimizerRegistry::instance().create("AStar", config);
                break;
            }
            case DragAlgorithm::None:
                // No optimization
                break;
            case DragAlgorithm::HideUntilDrop:
                // Hide edges during drag - skip all processing
                // Edges will be recalculated with A* on drop via PostDragAlgorithm
                return;
        }

        // Note: Penalty system and preserveDirections already configured via OptimizerRegistry

        if (optimizer) {
            // Store optimizer in edgeOptimizer_ so updateSnapPositions can use it
            // for regenerating bendPoints of redistributed edges
            edgeOptimizer_ = std::move(optimizer);

            // Collect all edges (including self-loops - optimizer handles them with penalty system)
            std::vector<EdgeId> edgesToOptimize;
            edgesToOptimize.reserve(affectedEdges.size());
            for (EdgeId edgeId : affectedEdges) {
                auto it = edgeLayouts.find(edgeId);
                if (it != edgeLayouts.end()) {
                    edgesToOptimize.push_back(edgeId);
                }
            }

            // Step 1: Optimizer calculates best paths while preserving edge directions
            auto optimizedLayouts = edgeOptimizer_->optimize(edgesToOptimize, edgeLayouts, nodeLayouts);

            // Merge optimized layouts from optimizer - including bendPoints for drag
            // During drag, we use Geometric optimizer's bendPoints directly (fast)
            // This avoids expensive A* recalculation in updateSnapPositions
            for (auto& [edgeId, layout] : optimizedLayouts) {
                auto it = edgeLayouts.find(edgeId);
                if (it != edgeLayouts.end()) {
                    it->second.sourceEdge = layout.sourceEdge;
                    it->second.targetEdge = layout.targetEdge;
                    it->second.sourcePoint = layout.sourcePoint;
                    it->second.targetPoint = layout.targetPoint;
                    it->second.bendPoints = layout.bendPoints;
                    // Reset snap indices for redistribution
                    it->second.sourceSnapIndex = constants::SNAP_INDEX_UNASSIGNED;
                    it->second.targetSnapIndex = constants::SNAP_INDEX_UNASSIGNED;
                }
            }

            // Step 2: Snap distribution for new edge combinations
            // Skip bendPoint recalculation - already set by optimizer
            updateSnapPositions(edgeLayouts, nodeLayouts, edgesToOptimize, movedNodes, gridSize, true);

            // Note: Self-loops are now included in edgesToOptimize and go through optimizer
            // with penalty-based direction selection (SegmentOverlapPenalty etc.)

            usedDragOptimizer = true;  // Mark that drag optimizer was used
        }
    } else {
        // No optimizer: just update snap positions
        updateSnapPositions(edgeLayouts, nodeLayouts, affectedEdges, movedNodes, gridSize);
    }

    // === Skip A* post-processing when drag optimizer was used ===
    // During drag, we prioritize speed over perfect paths.
    // A* will run after drop (via debounce callback) to fix any issues.
    if (usedDragOptimizer) {
#if EDGE_ROUTING_DEBUG
        std::cout << "[EdgeRouting] Skipping A* post-processing (drag optimizer mode)" << std::endl;
#endif
        return;
    }

    // === Check if any non-affected edges now penetrate moved nodes ===
    // When a node is moved, edges that don't connect to it might now pass through it.
    // Detect and re-route such edges to avoid penetration.
    if (!movedNodes.empty()) {
        std::vector<EdgeId> penetratingEdges;
        
        for (const auto& [edgeId, edgeLayout] : edgeLayouts) {
            // Skip edges already updated
            if (std::find(affectedEdges.begin(), affectedEdges.end(), edgeId) != affectedEdges.end()) {
                continue;
            }
            // Skip self-loops
            if (edgeLayout.from == edgeLayout.to) {
                continue;
            }
            
            // Check if this edge penetrates any moved node
            bool penetrates = false;
            for (NodeId movedNodeId : movedNodes) {
                if (penetrates) break;
                
                // Skip if this edge connects to the moved node
                if (edgeLayout.from == movedNodeId || edgeLayout.to == movedNodeId) continue;
                
                auto nodeIt = nodeLayouts.find(movedNodeId);
                if (nodeIt == nodeLayouts.end()) continue;
                const NodeLayout& movedNode = nodeIt->second;
                
                // Check each segment
                std::vector<Point> path;
                path.push_back(edgeLayout.sourcePoint);
                for (const auto& bp : edgeLayout.bendPoints) {
                    path.push_back(bp.position);
                }
                path.push_back(edgeLayout.targetPoint);
                
                for (size_t i = 0; i + 1 < path.size(); ++i) {
                    if (segmentPenetratesNodeInterior(path[i], path[i+1], movedNode)) {
                        penetrates = true;
                        break;
                    }
                }
            }
            
            if (penetrates) {
                penetratingEdges.push_back(edgeId);
            }
        }
        
        // Re-route edges that penetrate moved nodes
        if (!penetratingEdges.empty()) {
#if EDGE_ROUTING_DEBUG
            std::cout << "[EdgeRouting] updateEdgeRoutingWithOptimization: "
                      << penetratingEdges.size() << " non-affected edges penetrate moved nodes, re-routing" << std::endl;
#endif
            updateSnapPositions(edgeLayouts, nodeLayouts, penetratingEdges, movedNodes, gridSize);
        }
    }
    
    // === Final validation: Check ALL edges against ALL nodes ===
    // After all routing is done, do a final sweep to ensure no edge penetrates any node.
    // This catches cases where re-routing created new penetrations with non-moved nodes.
    {
        std::vector<EdgeId> finalPenetratingEdges;
        
        for (auto& [edgeId, edgeLayout] : edgeLayouts) {
            // Skip self-loops
            if (edgeLayout.from == edgeLayout.to) continue;
            
            // Build path
            std::vector<Point> path;
            path.push_back(edgeLayout.sourcePoint);
            for (const auto& bp : edgeLayout.bendPoints) {
                path.push_back(bp.position);
            }
            path.push_back(edgeLayout.targetPoint);
            
            // Check against all nodes
            bool penetrates = false;
            for (const auto& [nodeId, nodeLayout] : nodeLayouts) {
                if (penetrates) break;
                // Skip source and target nodes
                if (nodeId == edgeLayout.from || nodeId == edgeLayout.to) continue;
                
                for (size_t i = 0; i + 1 < path.size(); ++i) {
                    if (segmentPenetratesNodeInterior(path[i], path[i+1], nodeLayout)) {
                        penetrates = true;
                        break;
                    }
                }
            }
            
            if (penetrates) {
                finalPenetratingEdges.push_back(edgeId);
            }
        }
        
        // For any remaining penetrating edges, recalculate with fresh A* pathfinding
        if (!finalPenetratingEdges.empty()) {
#if EDGE_ROUTING_DEBUG
            std::cout << "[EdgeRouting] updateEdgeRoutingWithOptimization: "
                      << "Final validation found " << finalPenetratingEdges.size() 
                      << " edges still penetrating nodes, forcing A* recalculation" << std::endl;
#endif
            for (EdgeId edgeId : finalPenetratingEdges) {
                auto it = edgeLayouts.find(edgeId);
                if (it != edgeLayouts.end()) {
                    // Force recalculation with A* pathfinder (no overlap avoidance)
                    recalculateBendPoints(it->second, nodeLayouts, gridSize, nullptr);
                }
            }
        }
    }

    // === Post-Nudging: Visual separation of overlapping segments ===
    // EdgeNudger applies small offsets to visually separate segments that share coordinates.
    // This is the final fallback after Rip-up and Reroute has eliminated most overlaps.
    if (options.optimizationOptions.enablePostNudging) {
        EdgeNudger::Config nudgeConfig;
        nudgeConfig.nudgeOffset = options.optimizationOptions.nudgeOffset;
        nudgeConfig.minSegmentLength = options.optimizationOptions.minNudgeSegmentLength;
        nudgeConfig.enabled = true;

        EdgeNudger nudger(nudgeConfig);
        auto nudgeResult = nudger.applyNudging(edgeLayouts);

#if EDGE_ROUTING_DEBUG
        if (nudgeResult.totalOverlaps > 0) {
            std::cout << "[EdgeNudger] Applied nudging to " << nudgeResult.totalOverlaps
                      << " overlap groups, " << nudgeResult.nudgedSegments.size()
                      << " segments adjusted" << std::endl;
        }
#endif

        // Apply nudged positions to edge layouts
        // Note: EdgeNudger returns NudgedSegment with edgeId and segmentIndex
        // We need to update the corresponding bendPoint positions
        for (const auto& nudged : nudgeResult.nudgedSegments) {
            auto it = edgeLayouts.find(nudged.edgeId);
            if (it == edgeLayouts.end()) continue;

            EdgeLayout& layout = it->second;
            
            // segmentIndex: 0 = sourcePoint→first bend (or target if no bends)
            //               1..n-1 = bend-to-bend segments
            //               n = last bend→targetPoint
            // We need to adjust the START point of the segment (which is bendPoints[segmentIndex-1])
            // and the END point of the segment (which is bendPoints[segmentIndex] or targetPoint)
            
            if (nudged.segmentIndex == 0) {
                // First segment: adjust sourcePoint
                layout.sourcePoint = nudged.p1;
                if (layout.bendPoints.empty()) {
                    layout.targetPoint = nudged.p2;
                } else {
                    layout.bendPoints[0].position = nudged.p2;
                }
            } else if (nudged.segmentIndex < layout.bendPoints.size()) {
                // Middle segment: adjust bend points
                layout.bendPoints[nudged.segmentIndex - 1].position = nudged.p1;
                layout.bendPoints[nudged.segmentIndex].position = nudged.p2;
            } else if (nudged.segmentIndex == layout.bendPoints.size()) {
                // Last segment: adjust last bendPoint and targetPoint
                if (!layout.bendPoints.empty()) {
                    layout.bendPoints.back().position = nudged.p1;
                }
                layout.targetPoint = nudged.p2;
            }
        }
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
