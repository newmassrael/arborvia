#pragma once

#include "arborvia/layout/config/LayoutResult.h"
#include <vector>
#include <utility>

namespace arborvia {

/**
 * @brief Single source of truth for grid-based snap point calculations.
 *
 * This class consolidates all snap point calculation logic that was previously
 * scattered across EdgeRouting, LayoutUtils, and SnapIndexManager. It uses a
 * fixed grid-based candidate system where snap points are allocated at grid
 * positions (excluding corners) rather than being connection-order dependent.
 *
 * The "fixed candidate" model:
 * - Each node edge has a fixed number of grid-aligned candidate positions
 * - Candidates are determined by grid intersections within the edge bounds
 * - Connections are distributed evenly among available candidates
 * - Snap indices stored in EdgeLayout refer to candidate indices (not connection order)
 *
 * All methods are static as this is a stateless utility class.
 */
class GridSnapCalculator {
public:
    // =========================================================================
    // Low-level Grid Operations
    // =========================================================================

    /**
     * @brief Get the grid coordinate range for a node edge.
     *
     * For Top/Bottom edges: returns X range (horizontal positions)
     * For Left/Right edges: returns Y range (vertical positions)
     *
     * Uses ceil for start and floor for end to get interior grid points
     * that are strictly within the edge bounds (excluding corners).
     *
     * @param node Node layout
     * @param edge Which edge to get range for
     * @param gridSize Grid cell size (must be > 0)
     * @return Pair of (start, end) in grid units
     */
    static std::pair<int, int> getEdgeGridRange(
        const NodeLayout& node,
        NodeEdge edge,
        float gridSize);

    /**
     * @brief Get the number of fixed snap point candidates on a node edge.
     *
     * Returns the count of grid-aligned positions available for snap points,
     * excluding corners. This is calculated as: gridEnd - gridStart + 1
     *
     * @param node Node layout
     * @param edge Which edge to count candidates for
     * @param gridSize Grid cell size (must be > 0)
     * @return Number of available snap point candidates (>= 0)
     */
    static int getCandidateCount(
        const NodeLayout& node,
        NodeEdge edge,
        float gridSize);

    // =========================================================================
    // Mid-level Candidate Management
    // =========================================================================

    /**
     * @brief Select candidate indices for distributing connections among fixed candidates.
     *
     * Distributes `connectionCount` connections evenly across `candidateCount` candidates.
     * When connections >= candidates: uses round-robin distribution
     * When connections < candidates: uses even spacing formula
     *
     * @param candidateCount Total number of fixed candidates available
     * @param connectionCount Number of connections to distribute
     * @return Vector of candidate indices (size == connectionCount)
     */
    static std::vector<int> selectCandidateIndices(
        int candidateCount,
        int connectionCount);

    /**
     * @brief Get snap point position from a candidate index.
     *
     * Converts a candidate index (0-based within the candidate range)
     * to pixel coordinates on the node edge.
     *
     * @param node Node layout
     * @param edge Which edge the snap point is on
     * @param candidateIndex 0-based index into fixed candidates
     * @param gridSize Grid cell size
     * @return Snap point position in pixel coordinates (grid-aligned)
     */
    static Point getPositionFromCandidateIndex(
        const NodeLayout& node,
        NodeEdge edge,
        int candidateIndex,
        float gridSize);

    /**
     * @brief Get snap point position from a stored snap index.
     *
     * Use this when you have a previously stored snapIndex (which represents
     * a candidate index). Handles out-of-range clamping automatically.
     *
     * @param node Node layout
     * @param edge Which edge the snap point is on
     * @param storedSnapIndex Previously stored snap index (candidate index)
     * @param gridSize Grid cell size
     * @return Snap point position in pixel coordinates (grid-aligned)
     */
    static Point getPositionFromStoredIndex(
        const NodeLayout& node,
        NodeEdge edge,
        int storedSnapIndex,
        float gridSize);

    /**
     * @brief Get candidate index from a pixel position.
     *
     * Finds the closest grid-aligned candidate position for a given pixel coordinate.
     * This is the inverse of getPositionFromCandidateIndex.
     *
     * @param node Node layout
     * @param edge Which edge the position is on
     * @param position Pixel position to map to candidate index
     * @param gridSize Grid cell size
     * @return Candidate index (0-based, clamped to valid range)
     */
    static int getCandidateIndexFromPosition(
        const NodeLayout& node,
        NodeEdge edge,
        const Point& position,
        float gridSize);

    // =========================================================================
    // High-level Snap Calculation (Main Entry Point)
    // =========================================================================

    /**
     * @brief Calculate snap point position using fixed grid-based candidates.
     *
     * This is the main entry point for snap point calculation. It:
     * 1. Gets the candidate count for the edge
     * 2. Selects appropriate candidates based on connection count
     * 3. Returns the position for the specified connection index
     *
     * @param node Node layout
     * @param edge Which edge the snap point is on
     * @param connectionIndex 0-based index of this connection (0, 1, 2, ...)
     * @param totalConnections Total number of connections on this edge
     * @param gridSize Grid cell size
     * @param outCandidateIndex Optional output parameter to receive the selected candidate index
     * @return Snap point position in pixel coordinates (grid-aligned)
     */
    static Point calculateSnapPosition(
        const NodeLayout& node,
        NodeEdge edge,
        int connectionIndex,
        int totalConnections,
        float gridSize,
        int* outCandidateIndex = nullptr);

    /**
     * @brief Calculate snap point position by analyzing connection context.
     *
     * This convenience method automatically determines connectionIndex and totalConnections
     * by analyzing all edge layouts. Useful when you have an edge layout but don't know
     * its position among other connections on the same node edge.
     *
     * @param nodeLayouts All node layouts (to look up source/target nodes)
     * @param targetEdge The edge layout to calculate snap position for
     * @param allEdgeLayouts All edge layouts (to count connections on same node edge)
     * @param isSource True to calculate source point, false for target point
     * @param gridSize Grid cell size
     * @param outCandidateIndex Optional output parameter to receive the selected candidate index
     * @return Snap point position in pixel coordinates (grid-aligned)
     */
    static Point calculatePositionInContext(
        const std::vector<NodeLayout>& nodeLayouts,
        const EdgeLayout& targetEdge,
        const std::vector<EdgeLayout>& allEdgeLayouts,
        bool isSource,
        float gridSize,
        int* outCandidateIndex = nullptr);

    // =========================================================================
    // Utility Functions
    // =========================================================================

    /**
     * @brief Distribute positions evenly in grid coordinates using integer arithmetic.
     *
     * Uses integer division with rounding to avoid floating-point accumulation errors.
     * Preserves symmetry when distributing snap points.
     *
     * @param start Start of range in grid units
     * @param end End of range in grid units
     * @param count Number of positions to distribute
     * @param outPositions Output vector for grid positions
     */
    static void distributePositionsQuantized(
        int start,
        int end,
        int count,
        std::vector<int>& outPositions);

    // NOTE: getEffectiveGridSize() moved to constants::effectiveGridSize() in GeometryUtils.h
};

}  // namespace arborvia
