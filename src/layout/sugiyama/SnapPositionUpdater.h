#pragma once

#include "arborvia/layout/LayoutResult.h"
#include "arborvia/layout/IEdgeOptimizer.h"
#include "arborvia/layout/IPathFinder.h"
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <set>
#include <map>
#include <functional>
#include <memory>

namespace arborvia {

/**
 * @brief Result structure for snap position updates.
 */
struct SnapUpdateResult {
    std::set<EdgeId> processedEdges;     ///< Edges that were processed
    std::set<EdgeId> redistributedEdges; ///< Edges that had snap positions redistributed
    bool needsFullReroute = false;       ///< If true, full re-route needed after retry
    std::set<EdgeId> edgesNeedingReroute;///< Edges needing re-route after retry

    bool hasIndirectUpdates(const std::vector<EdgeId>& affectedEdges) const {
        for (EdgeId edgeId : redistributedEdges) {
            if (std::find(affectedEdges.begin(), affectedEdges.end(), edgeId) == affectedEdges.end()) {
                return true;
            }
        }
        return false;
    }
};

/**
 * @brief Handles snap position calculation and updating for edges.
 *
 * This class encapsulates the logic for:
 * 1. Collecting affected connections by node-edge
 * 2. Calculating and distributing snap positions
 * 3. Recalculating bend points after snap position changes
 * 4. Handling diagonal detection and direction constraint fixes
 */
class SnapPositionUpdater {
public:
    /// Function type for recalculating bend points
    using RecalcBendPointsFunc = std::function<void(
        EdgeLayout& layout,
        const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
        float gridSize,
        const std::unordered_map<EdgeId, EdgeLayout>* otherEdges)>;

    /// Function type for detecting and fixing diagonals
    using DetectFixDiagonalsFunc = std::function<bool(
        EdgeId edgeId,
        std::unordered_map<EdgeId, EdgeLayout>& edgeLayouts,
        const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
        const std::unordered_set<NodeId>& movedNodes,
        float effectiveGridSize,
        std::unordered_map<EdgeId, EdgeLayout>& otherEdges)>;

    /// Function type for validating and fixing direction constraints
    using ValidateFixDirectionFunc = std::function<bool(
        EdgeLayout& layout,
        float effectiveGridSize)>;

    /**
     * @brief Construct a SnapPositionUpdater.
     * @param pathFinder Shared pathfinder for A* calculations.
     * @param recalcFunc Function to call for bend point recalculation.
     * @param detectFixFunc Function to call for diagonal detection/fixing.
     * @param validateFixFunc Function to call for direction constraint validation/fixing.
     */
    SnapPositionUpdater(
        std::shared_ptr<IPathFinder> pathFinder,
        RecalcBendPointsFunc recalcFunc,
        DetectFixDiagonalsFunc detectFixFunc,
        ValidateFixDirectionFunc validateFixFunc);

    /**
     * @brief Update snap positions for affected edges.
     *
     * @param edgeLayouts All edge layouts (modified in place).
     * @param nodeLayouts All node layouts.
     * @param affectedEdges Edges that need snap position updates.
     * @param movedNodes Set of nodes that were moved.
     * @param gridSize Grid size for snapping.
     * @param skipBendPointRecalc If true, skip bend point recalculation (optimizer handles it).
     * @param edgeOptimizer Optional optimizer for bend point regeneration.
     * @return Result containing processed and redistributed edges.
     */
    SnapUpdateResult updateSnapPositions(
        std::unordered_map<EdgeId, EdgeLayout>& edgeLayouts,
        const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
        const std::vector<EdgeId>& affectedEdges,
        const std::unordered_set<NodeId>& movedNodes,
        float gridSize,
        bool skipBendPointRecalc = false,
        IEdgeOptimizer* edgeOptimizer = nullptr);

private:
    /// Map type for affected connections: (nodeId, nodeEdge) -> [(edgeId, isSource)]
    using AffectedConnectionsMap = std::map<std::pair<NodeId, NodeEdge>, std::vector<std::pair<EdgeId, bool>>>;

    /**
     * @brief Collect affected connections grouped by node-edge.
     */
    AffectedConnectionsMap collectAffectedConnections(
        const std::unordered_map<EdgeId, EdgeLayout>& edgeLayouts,
        const std::vector<EdgeId>& affectedEdges);

    /**
     * @brief Calculate snap positions for a single node-edge.
     */
    void calculateSnapPositionsForNodeEdge(
        const std::pair<NodeId, NodeEdge>& key,
        const std::vector<std::pair<EdgeId, bool>>& connections,
        std::unordered_map<EdgeId, EdgeLayout>& edgeLayouts,
        const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
        const std::unordered_set<NodeId>& movedNodes,
        float effectiveGridSize,
        SnapUpdateResult& result);

    /**
     * @brief Check if an EdgeLayout has fresh (non-stale) bendPoints.
     */
    static bool hasFreshBendPoints(const EdgeLayout& layout, float gridSize);

    std::shared_ptr<IPathFinder> pathFinder_;
    RecalcBendPointsFunc recalcFunc_;
    DetectFixDiagonalsFunc detectFixFunc_;
    ValidateFixDirectionFunc validateFixFunc_;
};

} // namespace arborvia
