#pragma once

#include "arborvia/core/Graph.h"
#include "arborvia/layout/config/LayoutOptions.h"
#include "arborvia/layout/config/LayoutResult.h"

#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <map>
#include <memory>

namespace arborvia {

// Forward declarations
class SnapIndexManager;
class IPathFinder;
class IEdgeOptimizer;
class PathRoutingCoordinator;
class EdgeValidator;
class ChannelRouter;
class PathCalculator;

/// Channel assignment info for an edge
struct ChannelAssignment {
    int channel = 0;         // Channel index (0-based)
    float yPosition = 0.0f;  // Computed position for channel segment (Y for vertical, X for horizontal layout)
    int sourceLayer = 0;     // Source node's layer
    int targetLayer = 0;     // Target node's layer
    bool isSelfLoop = false; // True if source == target
};

/// Channel region between two adjacent layers
struct ChannelRegion {
    int fromLayer = 0;       // Source layer index
    int toLayer = 0;         // Target layer index
    float regionStart = 0;   // Coordinate where region starts (Y for vertical, X for horizontal layout)
    float regionEnd = 0;     // Coordinate where region ends (Y for vertical, X for horizontal layout)
    std::vector<EdgeId> edges;  // Edges routed through this region
};

/// Routes edges with appropriate bend points
class EdgeRouting {
public:
    /// Constructor with dependency injection for pathfinder
    /// @param pathFinder Pathfinder to use for routing (uses AStarPathFinder by default)
    explicit EdgeRouting(std::shared_ptr<IPathFinder> pathFinder = nullptr);

    /// Constructor with routing coordinator for dual-algorithm support
    /// @param coordinator Routing coordinator managing drag/drop pathfinders (not owned)
    explicit EdgeRouting(PathRoutingCoordinator* coordinator);

    /// Destructor (defined in .cpp for unique_ptr to ChannelRouter)
    ~EdgeRouting();

    /// Result of edge routing operation
    struct Result {
        std::unordered_map<EdgeId, EdgeLayout> edgeLayouts;
        // Optional: channel assignments for debugging/visualization
        std::unordered_map<EdgeId, ChannelAssignment> channelAssignments;

        /// Get edge layout by ID, returns nullptr if not found
        const EdgeLayout* getEdgeLayout(EdgeId id) const {
            auto it = edgeLayouts.find(id);
            return it != edgeLayouts.end() ? &it->second : nullptr;
        }
    };

    /// Get the pathfinder instance (returns current pathfinder from coordinator if set)
    const IPathFinder& pathFinder() const;

    /// Set routing coordinator for dual-algorithm support
    /// @param coordinator Routing coordinator (not owned, can be null to disable)
    void setRoutingCoordinator(PathRoutingCoordinator* coordinator);

    /// Get routing coordinator (may be null)
    PathRoutingCoordinator* routingCoordinator() const;

    /// Set edge optimizer for snap point selection (optional)
    /// @param optimizer Edge optimizer implementing IEdgeOptimizer
    void setEdgeOptimizer(std::shared_ptr<IEdgeOptimizer> optimizer);

    /// Get edge optimizer (may be null)
    IEdgeOptimizer* edgeOptimizer() const;

    /// Route edges based on node positions
    /// @param graph The graph containing edges to route
    /// @param nodeLayouts Node positions
    /// @param reversedEdges Edges that need reversed routing
    /// @param options Layout options
    /// @param skipOptimization If true, skip optimizer (use when optimizer will be called later)
    Result route(const Graph& graph,
                const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
                const std::unordered_set<EdgeId>& reversedEdges,
                const LayoutOptions& options,
                bool skipOptimization = false);

    /// Optimize edge routing after snap distribution
    /// Runs optimizer on edges with their final snap positions to satisfy all constraints
    /// @param result The routing result to optimize (modified in place)
    /// @param nodeLayouts Node positions
    /// @param options Layout options
    void optimizeRouting(
        Result& result,
        const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
        const LayoutOptions& options);

    /// Distribute snap points evenly for edges connecting to same node edge (Auto mode)
    /// @param result The routing result to modify
    /// @param nodeLayouts Node positions for snap point calculation
    /// @param gridSize Grid size for coordinate snapping (0 = disabled)
    /// @param sortSnapPoints If true, sort snap points by other node position to minimize crossings
    void distributeAutoSnapPoints(
        Result& result,
        const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
        float gridSize = 0.0f,
        bool sortSnapPoints = true);

    /// Result of snap position update operation
    /// Provides transparency about which edges were actually processed
    struct SnapUpdateResult {
        std::unordered_set<EdgeId> processedEdges;      ///< All edges that had bendPoints recalculated
        std::unordered_set<EdgeId> redistributedEdges;  ///< Edges affected by snap redistribution (may include edges not in affectedEdges)
        bool needsFullReroute = false;                  ///< True if retry required full re-routing
        std::vector<EdgeId> edgesNeedingReroute;        ///< Edges that need full re-routing

        /// Check if any edges outside affectedEdges were processed due to redistribution
        bool hasIndirectUpdates(const std::vector<EdgeId>& affectedEdges) const {
            for (EdgeId edgeId : redistributedEdges) {
                if (std::find(affectedEdges.begin(), affectedEdges.end(), edgeId) == affectedEdges.end()) {
                    return true;
                }
            }
            return false;
        }
    };

    /// Update snap point positions when nodes move
    /// Preserves edge routing (sourceEdge, targetEdge) but recalculates snap positions
    /// Use this for simple position updates without edge re-routing
    ///
    /// NOTE: When snap redistribution occurs on a node-edge, ALL edges on that node-edge
    /// will have their snap positions updated, even if not in affectedEdges.
    /// The returned SnapUpdateResult shows exactly which edges were processed.
    ///
    /// @param edgeLayouts The edge layouts to update (modified in place)
    /// @param nodeLayouts Current node positions (after move)
    /// @param oldNodeLayouts Previous node positions (before move). Used to correctly
    ///                       compute candidateIdx from old snap positions. If empty,
    ///                       nodeLayouts is used (backward compatible behavior).
    /// @param affectedEdges Edges that need updating (connected to moved nodes)
    /// @param movedNodes Optional set of nodes that actually moved. If provided, only endpoints
    ///                   on these nodes will be recalculated. If empty, all endpoints are updated.
    /// @param gridSize Grid cell size for coordinate snapping (0 = disabled)
    /// @param skipBendPointRecalc If true, skip bend point recalculation (use existing bendPoints)
    /// @return SnapUpdateResult showing which edges were actually processed
    SnapUpdateResult updateSnapPositions(
        std::unordered_map<EdgeId, EdgeLayout>& edgeLayouts,
        const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
        const std::unordered_map<NodeId, NodeLayout>& oldNodeLayouts,
        const std::vector<EdgeId>& affectedEdges,
        const std::unordered_set<NodeId>& movedNodes = {},
        float gridSize = 0.0f,
        bool skipBendPointRecalc = false);

    /// @brief Convenience overload that uses nodeLayouts as oldNodeLayouts (backward compatible)
    SnapUpdateResult updateSnapPositions(
        std::unordered_map<EdgeId, EdgeLayout>& edgeLayouts,
        const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
        const std::vector<EdgeId>& affectedEdges,
        const std::unordered_set<NodeId>& movedNodes = {},
        float gridSize = 0.0f,
        bool skipBendPointRecalc = false);

    /// Update edge routing with optimization (for interactive drag)
    /// Runs edge optimizer to potentially change sourceEdge/targetEdge,
    /// then recalculates snap positions
    /// @param edgeLayouts The edge layouts to update (modified in place)
    /// @param nodeLayouts Current node positions
    /// @param affectedEdges Edges that need updating
    /// @param options Layout options including optimization settings
    /// @param movedNodes Optional set of nodes that moved
    void updateEdgeRoutingWithOptimization(
        std::unordered_map<EdgeId, EdgeLayout>& edgeLayouts,
        const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
        const std::vector<EdgeId>& affectedEdges,
        const LayoutOptions& options,
        const std::unordered_set<NodeId>& movedNodes = {});

    /// Regenerate bend points only, preserving source/target endpoints
    /// Use this when user explicitly moved a snap point and the position must be preserved.
    /// Unlike updateEdgeRoutingWithOptimization(), this does NOT change sourceEdge/targetEdge
    /// or sourcePoint/targetPoint - it only recalculates bendPoints.
    /// @param edgeLayouts The edge layouts to update (modified in place)
    /// @param nodeLayouts Current node positions
    /// @param affectedEdges Edges that need bend point recalculation
    /// @param options Layout options
    void regenerateBendPointsOnly(
        std::unordered_map<EdgeId, EdgeLayout>& edgeLayouts,
        const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
        const std::vector<EdgeId>& affectedEdges,
        const LayoutOptions& options);

private:
    // === Helper functions for snap point calculation ===

    /// Calculate snap point position on a node edge
    /// @param node The node layout
    /// @param edge Which edge of the node (Top, Bottom, Left, Right)
    /// @param position Relative position along the edge (0.0 to 1.0)
    /// @return Absolute point coordinates
    static Point calculateSnapPosition(const NodeLayout& node, NodeEdge edge, float position);

    /// Convert unified snap index to local index within a group
    /// This function converts to local index [0, count) for position calculation
    /// @param unifiedIdx The unified snap index from EdgeLayout
    /// @param offset The offset for this group (0 for incoming, inCount for outgoing)
    /// @param count The count of items in this group
    /// @return Local index clamped to [0, count)
    static int unifiedToLocalIndex(int unifiedIdx, int offset, int count);

    /// Recalculate bend points for an edge based on source and target points
    /// @param layout The edge layout to recalculate bend points for
    /// @param nodeLayouts All node layouts for intersection checking
    /// @param gridSize Grid cell size for coordinate snapping (0 = disabled)
    /// @param otherEdges Other edge layouts to avoid overlapping with (optional)
    /// @param movedNodes Set of nodes that were moved (for soft constraint on endpoint modification)
    void recalculateBendPoints(
        EdgeLayout& layout,
        const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
        float gridSize = 0.0f,
        const std::unordered_map<EdgeId, EdgeLayout>* otherEdges = nullptr,
        const std::unordered_set<NodeId>* movedNodes = nullptr);

    /// Count total connections on a node edge from all edge layouts
    /// @param edgeLayouts All edge layouts to count from
    /// @param nodeId The node to count connections for
    /// @param nodeEdge Which edge of the node
    /// @return Pair of (incoming count, outgoing count)
    static std::pair<int, int> countConnectionsOnNodeEdge(
        const std::unordered_map<EdgeId, EdgeLayout>& edgeLayouts,
        NodeId nodeId,
        NodeEdge nodeEdge);

    // === updateSnapPositions Helper Methods ===
    // These private helpers decompose the large updateSnapPositions method

    /// Detect diagonal segments and attempt swap retry to fix them
    /// @param edgeId The edge to check and fix
    /// @param edgeLayouts All edge layouts (may be modified during swap)
    /// @param nodeLayouts All node layouts
    /// @param movedNodes Set of nodes that have moved
    /// @param effectiveGridSize Grid size for calculations
    /// @param otherEdges Other edges for obstacle detection
    /// @return true if diagonal was detected and fixed (or no diagonal), false if unfixable
    bool detectAndFixDiagonals(
        EdgeId edgeId,
        std::unordered_map<EdgeId, EdgeLayout>& edgeLayouts,
        const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
        const std::unordered_set<NodeId>& movedNodes,
        float effectiveGridSize,
        std::unordered_map<EdgeId, EdgeLayout>& otherEdges);

    /// Validate and fix direction constraints for an edge
    /// Inserts clearance segments if first/last segment violates direction constraint
    /// @param layout The edge layout to validate and fix (modified in place)
    /// @param effectiveGridSize Grid size for calculations
    /// @return true if any direction violation was fixed (caller should run overlap avoidance)
    bool validateAndFixDirectionConstraints(
        EdgeLayout& layout,
        float effectiveGridSize);

    // === Channel-based routing methods ===

    /// Compute channel regions between adjacent layers
    std::vector<ChannelRegion> computeChannelRegions(
        const Graph& graph,
        const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
        const std::unordered_set<EdgeId>& reversedEdges,
        Direction direction);

    /// Allocate edges to channels
    std::unordered_map<EdgeId, ChannelAssignment> allocateChannels(
        const Graph& graph,
        const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
        const std::unordered_set<EdgeId>& reversedEdges,
        const LayoutOptions& options);

    /// Compute channel position (Y for vertical, X for horizontal layout)
    /// When gridSize > 0, returns grid-aligned value using quantized arithmetic
    float computeChannelY(const ChannelRegion& region,
                         int channelIndex,
                         const ChannelRoutingOptions& opts,
                         float gridSize = 0.0f);

    /// Build edge lookup map for bidirectional detection from Graph
    /// Returns map: (from, to) -> edgeId (skips self-loops)
    template<typename Hash>
    static std::unordered_map<std::pair<NodeId, NodeId>, EdgeId, Hash>
    buildEdgeMapFromGraph(const Graph& graph) {
        std::unordered_map<std::pair<NodeId, NodeId>, EdgeId, Hash> edgeMap;
        for (EdgeId edgeId : graph.edges()) {
            const EdgeData edge = graph.tryGetEdge(edgeId).value();
            if (edge.from != edge.to) {  // Skip self-loops
                edgeMap[{edge.from, edge.to}] = edgeId;
            }
        }
        return edgeMap;
    }

    /// Build edge lookup map for bidirectional detection from EdgeLayouts
    /// Returns map: (from, to) -> edgeId (skips self-loops)
    template<typename Hash>
    static std::unordered_map<std::pair<NodeId, NodeId>, EdgeId, Hash>
    buildEdgeMapFromLayouts(const std::unordered_map<EdgeId, EdgeLayout>& edgeLayouts) {
        std::unordered_map<std::pair<NodeId, NodeId>, EdgeId, Hash> edgeMap;
        for (const auto& [edgeId, layout] : edgeLayouts) {
            if (layout.from != layout.to) {  // Skip self-loops
                edgeMap[{layout.from, layout.to}] = edgeId;
            }
        }
        return edgeMap;
    }

    /// Route edge using channel assignment
    /// @param edge The edge data
    /// @param fromLayout Source node layout
    /// @param toLayout Target node layout
    /// @param isReversed Whether edge is reversed
    /// @param channel Channel assignment info
    /// @param options Layout options
    /// @param allNodeLayouts Optional: all node layouts for segment-node intersection checking
    EdgeLayout routeChannelOrthogonal(
        const EdgeData& edge,
        const NodeLayout& fromLayout,
        const NodeLayout& toLayout,
        bool isReversed,
        const ChannelAssignment& channel,
        const LayoutOptions& options,
        const std::unordered_map<NodeId, NodeLayout>* allNodeLayouts = nullptr);

    /// Route self-loop edge
    EdgeLayout routeSelfLoop(
        const EdgeData& edge,
        const NodeLayout& nodeLayout,
        int loopIndex,
        const LayoutOptions& options);

    /// Pathfinder instance (injected via constructor, used when no coordinator)
    std::shared_ptr<IPathFinder> pathFinder_;

    /// Channel router for channel-based edge routing
    std::unique_ptr<ChannelRouter> channelRouter_;

    /// Edge optimizer for snap point selection (optional)
    std::shared_ptr<IEdgeOptimizer> edgeOptimizer_;

    /// Routing coordinator for dual-algorithm support (optional, not owned)
    PathRoutingCoordinator* coordinator_ = nullptr;

    /// Get the active pathfinder (from coordinator if set, otherwise pathFinder_)
    IPathFinder& activePathFinder() const;
};


}  // namespace arborvia
