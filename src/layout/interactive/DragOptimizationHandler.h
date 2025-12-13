#pragma once

#include "arborvia/layout/config/LayoutResult.h"
#include "arborvia/layout/config/LayoutOptions.h"
#include "arborvia/layout/api/IEdgeOptimizer.h"
#include "arborvia/layout/api/IPathFinder.h"
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <functional>
#include <memory>

namespace arborvia {

/**
 * @brief Handles edge routing optimization during and after drag operations.
 *
 * This class encapsulates the logic for:
 * 1. Applying drag algorithms (Geometric, AStar, HideUntilDrop)
 * 2. Detecting and re-routing edges that penetrate moved nodes
 * 3. Final validation sweep for node penetration
 * 4. Post-nudging for visual separation of overlapping segments
 */
class DragOptimizationHandler {
public:
    /// Function type for updating snap positions
    using SnapUpdateFunc = std::function<void(
        std::unordered_map<EdgeId, EdgeLayout>& edgeLayouts,
        const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
        const std::vector<EdgeId>& affectedEdges,
        const std::unordered_set<NodeId>& movedNodes,
        float gridSize,
        bool skipBendPointRecalc)>;

    /// Function type for recalculating bend points
    /// @param movedNodes Set of nodes that were moved (for soft constraint on endpoint modification)
    using RecalcBendPointsFunc = std::function<void(
        EdgeLayout& layout,
        const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
        float gridSize,
        const std::unordered_map<EdgeId, EdgeLayout>* otherEdges,
        const std::unordered_set<NodeId>* movedNodes)>;

    /**
     * @brief Construct a DragOptimizationHandler.
     * @param pathFinder Shared pathfinder for A* calculations.
     * @param snapUpdateFunc Function to call for snap position updates.
     * @param recalcFunc Function to call for bend point recalculation.
     */
    DragOptimizationHandler(
        std::shared_ptr<IPathFinder> pathFinder,
        SnapUpdateFunc snapUpdateFunc,
        RecalcBendPointsFunc recalcFunc);

    /**
     * @brief Update edge routing with optimization during/after drag.
     *
     * @param edgeLayouts All edge layouts (modified in place).
     * @param nodeLayouts All node layouts.
     * @param affectedEdges Edges affected by the drag operation.
     * @param options Layout options including drag algorithm settings.
     * @param movedNodes Set of nodes that were moved.
     * @param edgeOptimizer Output parameter for the created optimizer (for external use).
     */
    void updateEdgeRoutingWithOptimization(
        std::unordered_map<EdgeId, EdgeLayout>& edgeLayouts,
        const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
        const std::vector<EdgeId>& affectedEdges,
        const LayoutOptions& options,
        const std::unordered_set<NodeId>& movedNodes,
        std::shared_ptr<IEdgeOptimizer>& edgeOptimizer);

private:
    /**
     * @brief Check if a segment penetrates the interior of a node.
     * @param p1 First point of the segment.
     * @param p2 Second point of the segment.
     * @param node The node to check against.
     * @return true if segment penetrates node interior.
     */
    static bool segmentPenetratesNodeInterior(
        const Point& p1, const Point& p2, const NodeLayout& node);

    /**
     * @brief Collect edges that penetrate specified nodes.
     * @param edgeLayouts All edge layouts.
     * @param nodeLayouts All node layouts.
     * @param nodesToCheck Nodes to check penetration against.
     * @param excludeEdges Edges to exclude from checking.
     * @return Vector of edge IDs that penetrate the nodes.
     */
    std::vector<EdgeId> collectPenetratingEdges(
        const std::unordered_map<EdgeId, EdgeLayout>& edgeLayouts,
        const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
        const std::unordered_set<NodeId>& nodesToCheck,
        const std::vector<EdgeId>& excludeEdges);

    /**
     * @brief Collect all edges that penetrate any node (final validation).
     */
    std::vector<EdgeId> collectAllPenetratingEdges(
        const std::unordered_map<EdgeId, EdgeLayout>& edgeLayouts,
        const std::unordered_map<NodeId, NodeLayout>& nodeLayouts);

    std::shared_ptr<IPathFinder> pathFinder_;
    SnapUpdateFunc snapUpdateFunc_;
    RecalcBendPointsFunc recalcFunc_;
};

} // namespace arborvia
