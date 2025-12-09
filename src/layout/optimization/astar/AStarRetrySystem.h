#pragma once

#include "../../sugiyama/routing/EdgeRouting.h"  // For SnapRetryConfig, SnapRetryResult

#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace arborvia {

// Forward declarations
class IPathFinder;

/**
 * @brief 3-layer A* retry system for edge pathfinding failures.
 *
 * When A* pathfinding fails to find a path for an edge, this system
 * attempts recovery through three increasingly aggressive strategies:
 *
 * Layer 1 - Snap Index Retry:
 *   Try alternative snap positions on the same node edge using
 *   concentric expansion from the original position.
 *
 * Layer 2 - Neighbor Adjustment:
 *   Adjust transition coordinates of neighboring edges that may
 *   be blocking the path.
 *
 * Layer 3 - NodeEdge Switch:
 *   Try different source/target edge combinations (16 total).
 *   Only used when preserveDirections is false.
 */
class AStarRetrySystem {
public:
    /**
     * @brief Constructor with pathfinder dependency.
     * @param pathFinder Pathfinder to use for A* routing (not owned)
     * @param gridSize Grid cell size for calculations
     */
    AStarRetrySystem(IPathFinder& pathFinder, float gridSize);

    /**
     * @brief Main entry point for retry system.
     *
     * Orchestrates the 3-layer retry strategy:
     * 1. Snap Index Retry
     * 2. Neighbor Adjustment
     * 3. NodeEdge Switch
     *
     * @param layout Edge layout to fix (may be modified)
     * @param edgeLayouts All edge layouts (may be modified for neighbor adjustment)
     * @param nodeLayouts All node layouts
     * @param otherEdges Optional additional edges to avoid
     * @param config Retry configuration
     * @return Result with success status and applied changes
     */
    SnapRetryResult tryRetry(
        EdgeLayout& layout,
        std::unordered_map<EdgeId, EdgeLayout>& edgeLayouts,
        const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
        const std::unordered_map<EdgeId, EdgeLayout>* otherEdges,
        const SnapRetryConfig& config);

private:
    /**
     * @brief Layer 1: Try alternative snap indices on same NodeEdge.
     *
     * Uses concentric expansion from the original snap position,
     * trying positions at increasing distances.
     */
    SnapRetryResult trySnapIndexRetry(
        EdgeLayout& layout,
        const std::unordered_map<EdgeId, EdgeLayout>& edgeLayouts,
        const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
        const std::unordered_map<EdgeId, EdgeLayout>* otherEdges,
        const SnapRetryConfig& config);

    /**
     * @brief Layer 2: Try adjusting neighbor transition coordinates.
     *
     * Identifies neighboring edges that share the same source node-edge
     * and attempts to adjust their positions to create space.
     */
    SnapRetryResult tryNeighborAdjustment(
        EdgeLayout& layout,
        std::unordered_map<EdgeId, EdgeLayout>& edgeLayouts,
        const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
        const std::unordered_map<EdgeId, EdgeLayout>* otherEdges,
        const SnapRetryConfig& config);

    /**
     * @brief Layer 3: Try different NodeEdge combinations.
     *
     * Tests all 16 combinations of source/target edges
     * (4 source edges x 4 target edges).
     * Only used when config.preserveDirections is false.
     */
    SnapRetryResult tryNodeEdgeSwitch(
        EdgeLayout& layout,
        const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
        const std::unordered_map<EdgeId, EdgeLayout>* otherEdges,
        const SnapRetryConfig& config);

    // =========================================================================
    // Helper Methods
    // =========================================================================

    /**
     * @brief Check if bendPoints are fresh (consistent with sourcePoint).
     *
     * Stale bendPoints occur when sourcePoint was updated but bendPoints
     * still reflect the old position, creating a diagonal path.
     */
    static bool hasFreshBendPoints(const EdgeLayout& layout, float gridSize);

    /**
     * @brief Get effective grid size for routing.
     */
    float getEffectiveGridSize() const;

    /// Pathfinder reference (not owned)
    IPathFinder& pathFinder_;

    /// Grid size for calculations
    float gridSize_;
};

}  // namespace arborvia
