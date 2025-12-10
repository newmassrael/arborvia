#pragma once

#include "arborvia/layout/config/LayoutResult.h"
#include "arborvia/layout/api/IPathFinder.h"
#include <memory>
#include <unordered_map>
#include <unordered_set>

namespace arborvia {

// Forward declaration
class UnifiedRetryChain;

/**
 * @brief Calculates bend points for edge paths using A* pathfinding.
 * 
 * This class encapsulates the logic for calculating orthogonal edge paths
 * between nodes, including self-loop handling and A* pathfinding with retry.
 */
class PathCalculator {
public:
    /**
     * @brief Construct a PathCalculator with a pathfinder reference.
     * @param pathFinder Reference to the pathfinder to use for A* calculations.
     */
    explicit PathCalculator(IPathFinder& pathFinder);

    /// Destructor (defined in .cpp for unique_ptr with forward-declared type)
    ~PathCalculator();

    /**
     * @brief Recalculate bend points for an edge layout using A* pathfinding.
     * 
     * For self-loops, generates an orthogonal path that wraps around the node corner.
     * For regular edges, uses A* pathfinding with multiple snap position attempts.
     * 
     * @param layout The edge layout to update (modified in place).
     * @param nodeLayouts All node layouts for obstacle detection.
     * @param gridSize Grid size for snapping (0 uses default).
     * @param otherEdges Optional other edges to avoid overlapping with.
     * @param movedNodes Set of nodes that were moved (for soft constraint on endpoint modification).
     */
    void recalculateBendPoints(
        EdgeLayout& layout,
        const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
        float gridSize,
        const std::unordered_map<EdgeId, EdgeLayout>* otherEdges = nullptr,
        const std::unordered_set<NodeId>* movedNodes = nullptr);

    /**
     * @brief Check if an EdgeLayout has fresh (non-stale) bendPoints.
     * 
     * Stale bendPoints occur when sourcePoint was updated (e.g., due to node drag)
     * but bendPoints still reflect the old position, creating a diagonal path.
     * 
     * @param layout The edge layout to check.
     * @param gridSize Grid size for diagonal detection threshold.
     * @return true if bendPoints are fresh and consistent with sourcePoint.
     */
    static bool hasFreshBendPoints(const EdgeLayout& layout, float gridSize);

private:
    /**
     * @brief Handle self-loop edge routing.
     * 
     * Generates an orthogonal path that exits the source edge, goes around
     * a corner of the node, and enters the target edge.
     * 
     * @param layout The self-loop edge layout to update.
     * @param srcNode The node the self-loop is attached to.
     * @param effectiveGridSize Grid size for snapping.
     */
    void handleSelfLoop(
        EdgeLayout& layout,
        const NodeLayout& srcNode,
        float effectiveGridSize);

    /**
     * @brief Try A* pathfinding with multiple snap position attempts.
     *
     * @param layout The edge layout to update.
     * @param nodeLayouts All node layouts.
     * @param effectiveGridSize Grid size for calculations.
     * @param otherEdges Other edges to avoid (may be modified by CooperativeRerouter).
     * @param movedNodes Set of nodes that were moved (for soft constraint).
     * @return true if a valid path was found.
     */
    bool tryAStarPathfinding(
        EdgeLayout& layout,
        const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
        float effectiveGridSize,
        std::unordered_map<EdgeId, EdgeLayout>& otherEdges,
        const std::unordered_set<NodeId>* movedNodes = nullptr);

    /// Initialize or return the unified retry chain
    UnifiedRetryChain& getRetryChain(float gridSize);

    IPathFinder& pathFinder_;
    std::unique_ptr<UnifiedRetryChain> retryChain_;
    float lastGridSize_ = 0.0f;
};

} // namespace arborvia
