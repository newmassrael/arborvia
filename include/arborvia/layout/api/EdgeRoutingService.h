#pragma once

#include "arborvia/layout/config/LayoutResult.h"
#include "arborvia/layout/config/LayoutOptions.h"
#include "arborvia/layout/constraints/ValidatedEdgeLayout.h"
#include "arborvia/layout/api/IPathFinder.h"

#include <memory>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace arborvia {

// Forward declarations
class Graph;
class EdgeRouting;
class ConstraintGateway;

/// Unified service for all edge routing operations
///
/// This class provides a SINGLE ENTRY POINT for edge layout modifications,
/// ensuring all paths (initial layout, drag/drop, snap point changes) use
/// the same validation and constraint enforcement.
///
/// Key guarantees:
/// - All returned layouts have passed constraint validation
/// - Invalid layouts are rejected and reported in rejectedEdges
/// - Label positions are always recalculated
///
/// Usage:
/// ```cpp
/// EdgeRoutingService service(pathFinder);
/// auto result = service.routeInitial(graph, nodeLayouts, reversedEdges, options);
/// for (auto& [id, validated] : result.validatedLayouts) {
///     layoutResult.setEdgeLayout(id, std::move(validated));
/// }
/// ```
class EdgeRoutingService {
public:
    /// Result of routing operations
    struct RoutingResult {
        /// Edge layouts that passed validation
        std::unordered_map<EdgeId, ValidatedEdgeLayout> validatedLayouts;

        /// Edge IDs that failed hard constraint validation
        std::vector<EdgeId> rejectedEdges;

        /// Number of edges that passed validation
        size_t validCount() const { return validatedLayouts.size(); }

        /// Number of edges that failed validation
        size_t rejectedCount() const { return rejectedEdges.size(); }
    };

    /// Create service with optional path finder
    /// @param pathFinder Path finder for A* optimization (nullptr for geometric only)
    explicit EdgeRoutingService(std::shared_ptr<IPathFinder> pathFinder = nullptr);

    ~EdgeRoutingService();

    // Non-copyable
    EdgeRoutingService(const EdgeRoutingService&) = delete;
    EdgeRoutingService& operator=(const EdgeRoutingService&) = delete;

    // Movable
    EdgeRoutingService(EdgeRoutingService&&) noexcept;
    EdgeRoutingService& operator=(EdgeRoutingService&&) noexcept;

    /// Route edges for initial layout
    ///
    /// Full pipeline: channel allocation -> snap distribution -> optimization -> validation
    ///
    /// @param graph The graph containing edges
    /// @param nodeLayouts Node layouts for routing
    /// @param reversedEdges Edges that should be reversed during layout
    /// @param options Layout options
    /// @return Result with validated layouts and rejected edges
    RoutingResult routeInitial(
        const Graph& graph,
        const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
        const std::unordered_set<EdgeId>& reversedEdges,
        const LayoutOptions& options);

    /// Update edges after node move (drag/drop)
    ///
    /// Updates affected edges and validates results.
    ///
    /// @param currentLayouts Current edge layouts (will be copied, not modified)
    /// @param nodeLayouts Updated node layouts
    /// @param affectedEdges Edges that need recalculation
    /// @param options Layout options
    /// @param movedNodes Nodes that were moved
    /// @return Result with validated layouts and rejected edges
    RoutingResult updateAfterNodeMove(
        const std::unordered_map<EdgeId, EdgeLayout>& currentLayouts,
        const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
        const std::vector<EdgeId>& affectedEdges,
        const LayoutOptions& options,
        const std::unordered_set<NodeId>& movedNodes);

    /// Update edges after snap point move
    ///
    /// @param currentLayouts Current edge layouts
    /// @param nodeLayouts Node layouts
    /// @param affectedEdges Edges that need recalculation
    /// @param options Layout options
    /// @return Result with validated layouts and rejected edges
    RoutingResult updateAfterSnapMove(
        const std::unordered_map<EdgeId, EdgeLayout>& currentLayouts,
        const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
        const std::vector<EdgeId>& affectedEdges,
        const LayoutOptions& options);

private:
    /// Validate layouts and wrap in ValidatedEdgeLayout
    /// @param rawLayouts Layouts to validate (modified in place for label position)
    /// @param nodeLayouts Node layouts for context
    /// @param gridSize Grid size for context
    /// @return Result with validated and rejected edges
    RoutingResult validateAndWrap(
        std::unordered_map<EdgeId, EdgeLayout>& rawLayouts,
        const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
        float gridSize);

    /// Ensure label positions are calculated for all layouts
    void ensureLabelPositions(std::unordered_map<EdgeId, EdgeLayout>& layouts);

    std::shared_ptr<IPathFinder> pathFinder_;
    std::unique_ptr<EdgeRouting> edgeRouting_;
    std::unique_ptr<ConstraintGateway> constraintGateway_;
};

}  // namespace arborvia
