#pragma once

#include "EdgeRoutingContext.h"
#include "arborvia/layout/config/LayoutTypes.h"
#include "../../sugiyama/routing/EdgeRoutingUtils.h"

#include <memory>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace arborvia {

// Forward declarations
class IPathFinder;
class CooperativeRerouter;

/// Handles detection and resolution of overlapping edges
///
/// Uses a multi-strategy approach:
/// 1. Cooperative pair rerouting (256 combination search)
/// 2. Path adjustment fallback
/// 3. CooperativeRerouter fallback
///
/// This class is designed for dependency injection via EdgeRoutingContext,
/// enabling clean separation from AStarEdgeOptimizer.
class OverlapResolver {
public:
    /// Result of cooperative rerouting for an edge pair
    struct EdgePairResult {
        EdgeLayout layoutA;
        EdgeLayout layoutB;
        int combinedScore;
        bool valid = false;
    };

    /// Detect overlapping edges in results
    /// @param layouts All edge layouts to check
    /// @return Pairs of overlapping edge IDs
    static std::vector<std::pair<EdgeId, EdgeId>> detectOverlaps(
        const std::unordered_map<EdgeId, EdgeLayout>& layouts);

    /// Resolve all overlapping pairs
    /// Uses resolveOverlappingPair + CooperativeRerouter fallback
    /// @param context Routing context with all dependencies
    /// @param overlappingPairs Pairs of overlapping edge IDs
    /// @param result Map to update with resolved layouts
    /// @param assignedLayouts All assigned layouts (updated with resolved layouts)
    /// @param resolvedEdges Optional set to track which edges were resolved
    /// @return true if any overlaps were resolved
    static bool resolveAllOverlaps(
        const EdgeRoutingContext& context,
        const std::vector<std::pair<EdgeId, EdgeId>>& overlappingPairs,
        std::unordered_map<EdgeId, EdgeLayout>& result,
        std::unordered_map<EdgeId, EdgeLayout>& assignedLayouts,
        std::unordered_set<EdgeId>* resolvedEdges = nullptr);

    /// Resolve overlapping edge pair by cooperative rerouting
    /// Removes both edges from obstacle map and evaluates 16×16 = 256 combinations
    /// to find the best non-overlapping (pathA, pathB) pair.
    /// @param context Routing context with all dependencies
    /// @param edgeIdA First edge
    /// @param edgeIdB Second edge
    /// @param layoutA Current layout for first edge
    /// @param layoutB Current layout for second edge
    /// @param assignedLayouts All currently assigned layouts
    /// @return Best non-overlapping pair result (valid=false if none found)
    static EdgePairResult resolveOverlappingPair(
        const EdgeRoutingContext& context,
        EdgeId edgeIdA, EdgeId edgeIdB,
        const EdgeLayout& layoutA, const EdgeLayout& layoutB,
        const std::unordered_map<EdgeId, EdgeLayout>& assignedLayouts);

    /// Fallback path adjustment when 256 combination search fails
    /// Tries various path adjustments to resolve overlap between two edges
    /// @param context Routing context with all dependencies
    /// @param edgeIdA First edge
    /// @param edgeIdB Second edge
    /// @param layoutA Layout for first edge (kept as-is or adjusted)
    /// @param layoutB Layout for second edge (adjusted to avoid overlap)
    /// @param otherLayouts Other assigned layouts (excluding A and B)
    /// @return Best adjusted pair result (valid=false if all attempts fail)
    static EdgePairResult tryPathAdjustmentFallback(
        const EdgeRoutingContext& context,
        EdgeId edgeIdA, EdgeId edgeIdB,
        const EdgeLayout& layoutA, const EdgeLayout& layoutB,
        const std::unordered_map<EdgeId, EdgeLayout>& otherLayouts);
};

}  // namespace arborvia
