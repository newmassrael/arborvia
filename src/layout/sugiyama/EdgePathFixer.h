#pragma once

#include "arborvia/layout/LayoutResult.h"
#include "arborvia/layout/IPathFinder.h"
#include <unordered_map>
#include <unordered_set>

namespace arborvia {

/**
 * @brief Fixes edge path issues including diagonal segments and direction constraint violations.
 *
 * This class encapsulates the logic for:
 * 1. Detecting and fixing diagonal (non-orthogonal) segments by trying snap swaps
 *    and exhaustive NodeEdge search
 * 2. Validating and fixing direction constraints to ensure edges exit/enter nodes
 *    in the correct direction based on NodeEdge
 */
class EdgePathFixer {
public:
    /**
     * @brief Construct an EdgePathFixer with a pathfinder reference.
     * @param pathFinder Reference to the pathfinder for A* calculations.
     */
    explicit EdgePathFixer(IPathFinder& pathFinder);

    /**
     * @brief Detect and fix diagonal segments in an edge layout.
     *
     * Attempts to fix diagonal segments by:
     * 1. Swapping source snap points with other edges on same node edge
     * 2. Swapping target snap points with other edges on same node edge
     * 3. Exhaustive search through all NodeEdge combinations
     *
     * @param edgeId The edge to fix.
     * @param edgeLayouts All edge layouts (may be modified for swaps).
     * @param nodeLayouts All node layouts.
     * @param movedNodes Set of nodes that were moved (empty = all).
     * @param effectiveGridSize Grid size for snapping.
     * @param otherEdges Other edges to consider as obstacles.
     * @return true if edge was successfully fixed or didn't need fixing.
     */
    bool detectAndFixDiagonals(
        EdgeId edgeId,
        std::unordered_map<EdgeId, EdgeLayout>& edgeLayouts,
        const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
        const std::unordered_set<NodeId>& movedNodes,
        float effectiveGridSize,
        std::unordered_map<EdgeId, EdgeLayout>& otherEdges);

    /**
     * @brief Validate and fix direction constraint violations.
     *
     * Ensures that:
     * - First segment direction matches source NodeEdge (Top=up, Bottom=down, etc.)
     * - Last segment direction matches target NodeEdge arrival requirements
     *
     * Fixes violations by inserting clearance points and detour paths.
     *
     * @param layout The edge layout to validate and fix.
     * @param effectiveGridSize Grid size for calculating clearance.
     * @return true if a direction violation was found and fixed.
     */
    bool validateAndFixDirectionConstraints(
        EdgeLayout& layout,
        float effectiveGridSize);

private:
    /**
     * @brief Recalculate bend points for an edge layout.
     * @param layout Edge layout to update.
     * @param nodeLayouts All node layouts.
     * @param effectiveGridSize Grid size for calculations.
     * @param otherEdges Other edges to avoid (can be nullptr).
     */
    void recalculateBendPoints(
        EdgeLayout& layout,
        const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
        float effectiveGridSize,
        const std::unordered_map<EdgeId, EdgeLayout>* otherEdges);

    IPathFinder& pathFinder_;
};

} // namespace arborvia
