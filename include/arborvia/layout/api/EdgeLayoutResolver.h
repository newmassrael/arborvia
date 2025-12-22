#pragma once

#include "arborvia/core/Types.h"
#include "arborvia/layout/config/LayoutResult.h"
#include <memory>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <string>

namespace arborvia {

// Forward declarations
class IPathFinder;

/**
 * @brief SSOT (Single Source of Truth) resolver for EdgeLayout.
 *
 * This class is the ONLY way to update derived fields in EdgeLayout:
 * - sourcePoint (derived from sourceSnapIndex)
 * - targetPoint (derived from targetSnapIndex)
 * - bendPoints (computed via A* pathfinding)
 * - labelPosition (computed from path midpoint)
 *
 * ## Architecture Invariant
 *
 * EdgeLayout stores two types of data:
 * 1. CORE DATA (source of truth): id, from, to, sourceEdge, targetEdge,
 *    sourceSnapIndex, targetSnapIndex, channelY, usedGridSize
 * 2. DERIVED DATA (computed by this resolver): sourcePoint, targetPoint,
 *    bendPoints, labelPosition
 *
 * The "orphaned bend points" bug occurred when SnapPositionUpdater changed
 * sourcePoint/targetPoint without updating bendPoints. This resolver ensures
 * ALL derived fields are computed atomically from snapIndex values.
 *
 * ## Usage Pattern
 *
 * ```cpp
 * // When snapIndex changes, ALWAYS call resolve():
 * layout.sourceSnapIndex = newIndex;
 * EdgeLayoutResolver::resolve(layout, nodeLayouts, pathFinder);
 *
 * // Or use batch resolution:
 * EdgeLayoutResolver::resolveAll(edgeLayouts, nodeLayouts, pathFinder);
 * ```
 *
 * ## Deleted Methods (DO NOT USE)
 *
 * The following EdgeLayout methods are DELETED as part of this architecture:
 * - setSourceSnap() - caused orphaned bendPoints
 * - setTargetSnap() - caused orphaned bendPoints
 * - copySnapStateFrom() - caused orphaned bendPoints
 *
 * Always use EdgeLayoutResolver::resolve() instead.
 */
class EdgeLayoutResolver {
public:
    /// Result of a resolve operation
    struct Result {
        bool success = false;
        std::string error;

        static Result ok() { return {true, ""}; }
        static Result fail(const std::string& msg) { return {false, msg}; }
    };

    /**
     * @brief Resolve all derived fields for a single edge.
     *
     * This method:
     * 1. Computes sourcePoint from sourceSnapIndex + sourceEdge + sourceNode
     * 2. Computes targetPoint from targetSnapIndex + targetEdge + targetNode
     * 3. Computes bendPoints via A* pathfinding (if pathFinder provided)
     * 4. Computes labelPosition from path midpoint
     *
     * @param layout EdgeLayout to update (modified in-place)
     * @param nodeLayouts All node layouts (to look up source/target nodes)
     * @param gridSize Grid size for snap point calculation
     * @param pathFinder Optional path finder for computing bendPoints.
     *                   If nullptr, bendPoints are cleared.
     * @return Result indicating success or failure with error message
     */
    static Result resolve(
        EdgeLayout& layout,
        const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
        float gridSize,
        std::shared_ptr<IPathFinder> pathFinder = nullptr);

    /**
     * @brief Resolve snap points only (without bendPoints).
     *
     * Use this when you only need to update sourcePoint/targetPoint
     * and will compute bendPoints separately (e.g., via optimizer).
     *
     * @param layout EdgeLayout to update (modifies sourcePoint, targetPoint only)
     * @param nodeLayouts All node layouts
     * @param gridSize Grid size for snap point calculation
     * @param movedNodes Optional set of moved nodes. If provided, only updates
     *                   the side connected to a moved node.
     * @return Result indicating success or failure
     */
    static Result resolveSnapPointsOnly(
        EdgeLayout& layout,
        const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
        float gridSize,
        const std::unordered_set<NodeId>* movedNodes = nullptr);

    /**
     * @brief Resolve all edges in a map.
     *
     * @param layouts Edge layouts to update (modified in-place)
     * @param nodeLayouts All node layouts
     * @param gridSize Grid size for calculations
     * @param pathFinder Optional path finder for computing bendPoints
     * @return Vector of edge IDs that failed to resolve
     */
    static std::vector<EdgeId> resolveAll(
        std::unordered_map<EdgeId, EdgeLayout>& layouts,
        const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
        float gridSize,
        std::shared_ptr<IPathFinder> pathFinder = nullptr);

    /**
     * @brief Resolve specific edges from a map.
     *
     * @param layouts Edge layouts (modified in-place for specified edges)
     * @param edgeIds Edges to resolve
     * @param nodeLayouts All node layouts
     * @param gridSize Grid size for calculations
     * @param pathFinder Optional path finder for computing bendPoints
     * @return Vector of edge IDs that failed to resolve
     */
    static std::vector<EdgeId> resolveEdges(
        std::unordered_map<EdgeId, EdgeLayout>& layouts,
        const std::vector<EdgeId>& edgeIds,
        const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
        float gridSize,
        std::shared_ptr<IPathFinder> pathFinder = nullptr);

    /**
     * @brief Validate that derived fields are consistent with core data.
     *
     * Use this for debugging to detect if sourcePoint/targetPoint have
     * drifted from their expected values based on snapIndex.
     *
     * @param layout EdgeLayout to validate
     * @param nodeLayouts All node layouts
     * @param gridSize Grid size for calculations
     * @param tolerance Position tolerance for comparison (default 0.1)
     * @return True if derived fields match expected values
     */
    static bool validate(
        const EdgeLayout& layout,
        const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
        float gridSize,
        float tolerance = 0.1f);

    /**
     * @brief Resolve all derived fields with full retry chain support.
     *
     * This is the RECOMMENDED method for atomic edge resolution. It:
     * 1. Computes snapPoint from snapIndex (respecting movedNodes constraint)
     * 2. Computes bendPoints via UnifiedRetryChain (with retry logic)
     * 3. Ensures orthogonality even if A* fails
     * 4. Computes labelPosition from path midpoint
     *
     * Unlike resolve(), this method:
     * - Uses UnifiedRetryChain for robust pathfinding with retries
     * - Respects movedNodes constraint (only updates moved side)
     * - Guarantees consistent state even on A* failure
     *
     * @param edgeId ID of the edge being resolved
     * @param layout EdgeLayout to update (modified in-place)
     * @param otherEdges Other edges for overlap detection (may be modified by cooperative rerouter)
     * @param nodeLayouts All node layouts
     * @param gridSize Grid size for calculations
     * @param pathFinder Path finder for A* computation
     * @param movedNodes Set of moved nodes (nullptr = all nodes moved)
     * @return Result indicating success or failure
     */
    static Result resolveWithRetry(
        EdgeId edgeId,
        EdgeLayout& layout,
        std::unordered_map<EdgeId, EdgeLayout>& otherEdges,
        const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
        float gridSize,
        std::shared_ptr<IPathFinder> pathFinder,
        const std::unordered_set<NodeId>* movedNodes = nullptr);

    /**
     * @brief Batch resolve edges with retry chain support.
     *
     * Resolves multiple edges atomically, respecting movedNodes constraint.
     * Each edge gets snapPoint + bendPoints + labelPosition computed together.
     *
     * ## Processing Order Dependency
     *
     * Edges are processed sequentially in the order specified by edgeIds.
     * Each resolved edge is added to the "otherEdges" set before processing
     * the next edge, which affects overlap detection and cooperative rerouting.
     *
     * This means:
     * - Earlier edges may influence later edges' paths
     * - Different edgeIds ordering may produce different (but valid) results
     * - All results satisfy constraints (no overlaps, orthogonal paths)
     *
     * @param layouts All edge layouts (modified in-place for specified edges)
     * @param edgeIds Edges to resolve (processing order matters)
     * @param nodeLayouts All node layouts
     * @param gridSize Grid size for calculations
     * @param pathFinder Path finder for A* computation
     * @param movedNodes Set of moved nodes (nullptr = all nodes moved)
     * @return Vector of edge IDs that failed to resolve
     */
    static std::vector<EdgeId> resolveEdgesWithRetry(
        std::unordered_map<EdgeId, EdgeLayout>& layouts,
        const std::vector<EdgeId>& edgeIds,
        const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
        float gridSize,
        std::shared_ptr<IPathFinder> pathFinder,
        const std::unordered_set<NodeId>* movedNodes = nullptr);

    /**
     * @brief Check if a node should be updated based on movedNodes constraint.
     *
     * A node is considered "moved" (and thus should be updated) if:
     * - movedNodes is nullptr (legacy behavior: all nodes can be updated)
     * - movedNodes is empty (all nodes can be updated)
     * - nodeId is in movedNodes set
     *
     * @param movedNodes Set of moved nodes (nullptr = all nodes moved)
     * @param nodeId Node to check
     * @return True if the node should be updated
     */
    static bool isNodeMoved(const std::unordered_set<NodeId>* movedNodes, NodeId nodeId) {
        return !movedNodes || movedNodes->empty() || movedNodes->count(nodeId) > 0;
    }

private:
    /**
     * @brief Compute bendPoints via A* pathfinding.
     *
     * @param layout EdgeLayout with sourcePoint/targetPoint already set
     * @param nodeLayouts All node layouts (for obstacle avoidance)
     * @param pathFinder Path finder for A* computation
     * @return True if bendPoints were successfully computed
     */
    static bool computeBendPoints(
        EdgeLayout& layout,
        const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
        std::shared_ptr<IPathFinder> pathFinder);

    /**
     * @brief Compute label position from path.
     *
     * @param layout EdgeLayout with all path points set
     */
    static void computeLabelPosition(EdgeLayout& layout);
};

}  // namespace arborvia
