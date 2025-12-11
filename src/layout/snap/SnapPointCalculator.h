#pragma once

#include "arborvia/layout/config/LayoutResult.h"

namespace arborvia {

/**
 * @brief Unified snap point position calculator - A* standard implementation.
 *
 * This class provides the single source of truth for calculating snap point
 * pixel positions. The algorithm is extracted from UnifiedRetryChain::calculateSnapPointForRatio
 * which is the A* pathfinding implementation.
 *
 * All snap point coordinates are guaranteed to be grid-aligned (multiples of gridSize).
 * This ensures consistency across all algorithms (A*, Greedy, etc.).
 *
 * All methods are static as this is a stateless utility class.
 */
class SnapPointCalculator {
public:
    /**
     * @brief Calculate grid-aligned snap point from ratio (A* standard).
     *
     * This is the primary method for snap point calculation. It:
     * 1. Calculates base position from node layout and ratio
     * 2. Applies grid quantization: round(pos / gridSize) * gridSize
     *
     * @param node Node layout (position and size)
     * @param edge Which edge the snap point is on (Top, Bottom, Left, Right)
     * @param ratio Position on edge (0.0 = left/top edge, 1.0 = right/bottom edge)
     * @param gridSize Grid cell size in pixels (must be > 0)
     * @return Grid-aligned snap point position
     */
    static Point calculateFromRatio(
        const NodeLayout& node,
        NodeEdge edge,
        float ratio,
        float gridSize);

    /**
     * @brief Calculate grid-aligned snap point from snap index.
     *
     * Converts a snap index to a ratio and then calculates the grid-aligned position.
     * This is useful when you have a stored snapIndex and totalSnapPoints.
     *
     * @param node Node layout (position and size)
     * @param edge Which edge the snap point is on
     * @param snapIndex 0-based snap index
     * @param totalSnapPoints Total number of snap points on this edge
     * @param gridSize Grid cell size in pixels (must be > 0)
     * @return Grid-aligned snap point position
     */
    static Point calculateFromIndex(
        const NodeLayout& node,
        NodeEdge edge,
        int snapIndex,
        int totalSnapPoints,
        float gridSize);

    /**
     * @brief Get the length of a node edge.
     *
     * @param node Node layout
     * @param edge Which edge (Top/Bottom returns width, Left/Right returns height)
     * @return Edge length in pixels
     */
    static float getEdgeLength(const NodeLayout& node, NodeEdge edge);

    /**
     * @brief Calculate total snap points for an edge (A* standard formula).
     *
     * Formula: max(1, floor(edgeLength / gridSize) - 1)
     * This excludes corner positions and ensures at least 1 snap point.
     *
     * @param edgeLength Length of the edge in pixels
     * @param gridSize Grid cell size in pixels (must be > 0)
     * @return Number of snap points on this edge
     */
    static int calculateTotalSnapPoints(float edgeLength, float gridSize);

    /**
     * @brief Calculate total snap points for a node edge (convenience overload).
     *
     * @param node Node layout
     * @param edge Which edge
     * @param gridSize Grid cell size in pixels (must be > 0)
     * @return Number of snap points on this edge
     */
    static int calculateTotalSnapPoints(const NodeLayout& node, NodeEdge edge, float gridSize);
};

}  // namespace arborvia
