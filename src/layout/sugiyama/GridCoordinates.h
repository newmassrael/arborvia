#pragma once

#include "arborvia/layout/LayoutResult.h"
#include <cmath>

namespace arborvia {

/**
 * @brief Node bounds in grid units (integers) for quantized calculations.
 *
 * Using floor for left/top and ceil for right/bottom ensures the bounds
 * fully contain the node when converted back to pixels.
 */
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

/**
 * @brief Configuration for axis-agnostic routing operations.
 *
 * This struct provides a unified interface for operations that can work
 * in either horizontal or vertical orientation. It abstracts away the
 * axis differences so routing code can be written once and work for both.
 *
 * - Horizontal layout: primary = X, secondary = Y
 * - Vertical layout: primary = Y, secondary = X
 */
struct AxisConfig {
    bool isHorizontal;

    // =========================================================================
    // Basic Coordinate Access
    // =========================================================================

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

    // =========================================================================
    // Node Bounds Access
    // =========================================================================

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

    // =========================================================================
    // Edge Direction Helpers
    // =========================================================================

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

    // =========================================================================
    // EdgeLayout Access
    // =========================================================================

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

}  // namespace arborvia
