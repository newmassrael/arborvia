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
class IPathFinder;

/// Channel assignment info for an edge (moved from EdgeRouting.h)
/// Re-exported here for convenience, actual definition in EdgeRouting.h
struct ChannelAssignment;
struct ChannelRegion;

/**
 * @brief Handles channel-based edge routing between layers.
 *
 * This class encapsulates the channel routing algorithm which:
 * 1. Computes channel regions between adjacent layers
 * 2. Allocates edges to channels based on crossing minimization
 * 3. Routes edges through their assigned channels
 *
 * Channel routing is used during initial layout to create orthogonal
 * edge paths that minimize crossings and avoid node intersections.
 */
class ChannelRouter {
public:
    /// Constructor with pathfinder for bend point calculation
    /// @param pathFinder Pathfinder for A* routing (can be null for channel-only mode)
    explicit ChannelRouter(IPathFinder* pathFinder = nullptr);

    // =========================================================================
    // Channel Region Computation
    // =========================================================================

    /**
     * @brief Compute channel regions between adjacent layers.
     *
     * For each pair of adjacent layers, creates a ChannelRegion that
     * describes the available space for routing edges.
     *
     * @param graph The graph containing edges
     * @param nodeLayouts Node positions
     * @param reversedEdges Edges with reversed direction
     * @param direction Layout direction (affects region orientation)
     * @return Vector of channel regions ordered by layer
     */
    std::vector<ChannelRegion> computeChannelRegions(
        const Graph& graph,
        const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
        const std::unordered_set<EdgeId>& reversedEdges,
        Direction direction);

    // =========================================================================
    // Channel Allocation
    // =========================================================================

    /**
     * @brief Allocate edges to channels within regions.
     *
     * Assigns each edge to a channel number based on:
     * - Source node X position (for crossing minimization)
     * - Bidirectional edge handling (offset channels)
     * - Self-loop detection (special handling)
     *
     * @param graph The graph containing edges
     * @param nodeLayouts Node positions
     * @param reversedEdges Edges with reversed direction
     * @param options Layout options including channel routing config
     * @return Map from edge ID to channel assignment
     */
    std::unordered_map<EdgeId, ChannelAssignment> allocateChannels(
        const Graph& graph,
        const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
        const std::unordered_set<EdgeId>& reversedEdges,
        const LayoutOptions& options);

    // =========================================================================
    // Channel Position Calculation
    // =========================================================================

    /**
     * @brief Compute Y position for a channel.
     *
     * Calculates the exact Y (or X for horizontal layout) coordinate
     * for a channel within a region. Supports grid quantization.
     *
     * @param region The channel region
     * @param channelIndex 0-based channel index
     * @param opts Channel routing options
     * @param gridSize Grid size for quantization (0 = disabled)
     * @return Channel position in pixel coordinates
     */
    float computeChannelY(
        const ChannelRegion& region,
        int channelIndex,
        const ChannelRoutingOptions& opts,
        float gridSize = 0.0f);

    // =========================================================================
    // Edge Routing
    // =========================================================================

    /**
     * @brief Route a single edge using its channel assignment.
     *
     * Creates an EdgeLayout with orthogonal bend points that route
     * through the assigned channel. Handles:
     * - Source/target edge selection
     * - Channel-based intermediate points
     * - Node intersection avoidance
     * - Grid alignment
     * - Avoidance of already-routed edges (prevents overlaps)
     *
     * @param edge The edge data
     * @param fromLayout Source node layout
     * @param toLayout Target node layout
     * @param isReversed Whether edge direction is reversed
     * @param channel Channel assignment info
     * @param options Layout options
     * @param allNodeLayouts All node layouts for intersection checking
     * @param alreadyRoutedEdges Previously routed edges to treat as obstacles
     * @return Complete EdgeLayout with bend points
     */
    EdgeLayout routeChannelOrthogonal(
        const EdgeData& edge,
        const NodeLayout& fromLayout,
        const NodeLayout& toLayout,
        bool isReversed,
        const ChannelAssignment& channel,
        const LayoutOptions& options,
        const std::unordered_map<NodeId, NodeLayout>* allNodeLayouts = nullptr,
        const std::unordered_map<EdgeId, EdgeLayout>* alreadyRoutedEdges = nullptr);

    /**
     * @brief Route a self-loop edge.
     *
     * Delegates to SelfLoopRouter for consistent self-loop handling.
     *
     * @param edge The edge data (from == to)
     * @param nodeLayout Node layout
     * @param loopIndex Index for multiple self-loops on same node
     * @param options Layout options
     * @return EdgeLayout for self-loop
     */
    EdgeLayout routeSelfLoop(
        const EdgeData& edge,
        const NodeLayout& nodeLayout,
        int loopIndex,
        const LayoutOptions& options);

private:
    // =========================================================================
    // Bend Point Calculation
    // =========================================================================

    /**
     * @brief Calculate bend points for an edge using pathfinding.
     *
     * Uses A* pathfinding if pathfinder is available, otherwise
     * creates a simple orthogonal path. Already-routed edges are
     * added to the obstacle map to prevent overlapping paths.
     *
     * @param layout Edge layout with source/target set
     * @param nodeLayouts All node layouts for obstacle detection
     * @param gridSize Grid size for alignment
     * @param alreadyRoutedEdges Previously routed edges to treat as obstacles
     */
    void calculateBendPoints(
        EdgeLayout& layout,
        const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
        float gridSize,
        const std::unordered_map<EdgeId, EdgeLayout>* alreadyRoutedEdges = nullptr);

    /**
     * @brief Create bypass path around blocking nodes.
     *
     * When direct routing fails, creates a path that goes around
     * all intermediate nodes.
     *
     * @param layout Edge layout to modify
     * @param blockingNode Node blocking the path
     * @param allNodeLayouts All node layouts
     * @param gridSize Grid size for alignment
     */
    void createBypassPath(
        EdgeLayout& layout,
        const NodeLayout& blockingNode,
        const std::unordered_map<NodeId, NodeLayout>& allNodeLayouts,
        float gridSize);

    /**
     * @brief Ensure minimum clearance from source node.
     *
     * @param layout Edge layout to check/fix
     * @param gridSize Grid size for minimum clearance
     */
    void ensureSourceClearance(EdgeLayout& layout, float gridSize);

    // =========================================================================
    // Utility Methods
    // =========================================================================

    /// Hash function for pair keys
    struct PairHash {
        template <typename T1, typename T2>
        std::size_t operator()(const std::pair<T1, T2>& p) const {
            auto h1 = std::hash<T1>{}(p.first);
            auto h2 = std::hash<T2>{}(p.second);
            return h1 ^ (h2 << 1);
        }
    };

    /// Build edge lookup map for bidirectional detection
    template<typename Hash>
    static std::unordered_map<std::pair<NodeId, NodeId>, EdgeId, Hash>
    buildEdgeMap(const Graph& graph) {
        std::unordered_map<std::pair<NodeId, NodeId>, EdgeId, Hash> edgeMap;
        for (EdgeId edgeId : graph.edges()) {
            const EdgeData edge = graph.tryGetEdge(edgeId).value();
            if (edge.from != edge.to) {  // Skip self-loops
                edgeMap[{edge.from, edge.to}] = edgeId;
            }
        }
        return edgeMap;
    }

    /// Pathfinder for A* routing (not owned)
    IPathFinder* pathFinder_;
};

}  // namespace arborvia
