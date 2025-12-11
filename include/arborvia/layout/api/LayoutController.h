#pragma once

#include "arborvia/core/Types.h"
#include "arborvia/core/Graph.h"
#include "arborvia/layout/config/LayoutResult.h"
#include "arborvia/layout/config/LayoutOptions.h"
#include "arborvia/layout/config/ConstraintConfig.h"

#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <string>
#include <memory>

namespace arborvia {

// Forward declarations
class IEdgeOptimizer;
class ConstraintManager;
class ConstraintSolver;
struct FinalStateValidationResult;

/// Result of a node move operation
struct NodeMoveResult {
    bool success = false;
    Point actualPosition{0, 0};
    Point requestedPosition{0, 0};
    std::vector<EdgeId> affectedEdges;
    std::string reason;

    static NodeMoveResult ok(Point pos, const std::vector<EdgeId>& edges) {
        return {true, pos, pos, edges, ""};
    }

    static NodeMoveResult fail(Point requested, const std::string& reason) {
        return {false, {0, 0}, requested, {}, reason};
    }
};

/// Result of edge routing operation
struct EdgeRouteResult {
    bool success = false;
    std::vector<EdgeId> failedEdges;
    std::vector<std::pair<EdgeId, EdgeId>> remainingOverlaps;
    std::string reason;
};

/// Centralized controller for layout modifications
///
/// This is the ONLY way to modify node positions and edge routes.
/// Direct access to nodeLayouts/edgeLayouts is read-only.
/// All modifications go through validated APIs that enforce constraints.
///
/// Usage:
///   LayoutController controller(graph, options);
///   controller.initializeFrom(layoutResult);
///
///   // Read-only access
///   const auto& nodes = controller.nodeLayouts();
///
///   // Modifications through validated API
///   auto result = controller.moveNode(nodeId, newPosition);
///   if (!result.success) { /* handle failure */ }
///
class LayoutController {
public:
    /// Constructor
    /// @param graph Graph for connectivity info
    /// @param options Layout options (grid size, algorithms, etc.)
    explicit LayoutController(const Graph& graph, const LayoutOptions& options = LayoutOptions{});

    ~LayoutController();

    // =========================================================================
    // Initialization
    // =========================================================================

    /// Initialize from a LayoutResult (e.g., from SugiyamaLayout)
    void initializeFrom(const LayoutResult& result);

    /// Initialize from raw layouts
    void initializeFrom(
        const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
        const std::unordered_map<EdgeId, EdgeLayout>& edgeLayouts);

    // =========================================================================
    // Read-only Access (const only - no direct modification!)
    // =========================================================================

    /// Get all node layouts (read-only)
    const std::unordered_map<NodeId, NodeLayout>& nodeLayouts() const { return nodeLayouts_; }

    /// Get all edge layouts (read-only)
    const std::unordered_map<EdgeId, EdgeLayout>& edgeLayouts() const { return edgeLayouts_; }

    /// Get a specific node layout (read-only)
    const NodeLayout* getNode(NodeId id) const;

    /// Get a specific edge layout (read-only)
    const EdgeLayout* getEdge(EdgeId id) const;

    /// Get the graph
    const Graph& graph() const { return graph_; }

    /// Get layout options
    const LayoutOptions& options() const { return options_; }

    // =========================================================================
    // Validated Modification APIs (the ONLY way to modify)
    // =========================================================================

    /// Move a node to a new position with full constraint validation
    ///
    /// This method:
    /// 1. Validates against all registered constraints (via ConstraintManager)
    /// 2. Updates edge routing
    /// 3. Validates final state
    /// 4. Rolls back everything if any constraint fails
    ///
    /// @param nodeId Node to move
    /// @param newPosition Desired position
    /// @return Result with success status, actual position, and affected edges
    NodeMoveResult moveNode(NodeId nodeId, Point newPosition);

    /// Re-route specific edges
    /// @param edges Edges to re-route
    /// @return Result with success status and any remaining issues
    EdgeRouteResult rerouteEdges(const std::vector<EdgeId>& edges);

    /// Re-route all edges
    EdgeRouteResult rerouteAllEdges();

    // =========================================================================
    // Drag Support (visual feedback during drag, validation on drop)
    // =========================================================================

    /// Begin a drag operation - returns a temporary position for visual feedback
    /// Does NOT modify the actual layout
    struct DragPreview {
        Point visualPosition;  // Position for visual feedback
        bool isValid;          // Whether this would be a valid final position
        std::string reason;    // If invalid, why
    };

    /// Get preview position during drag (does not modify state)
    DragPreview getDragPreview(NodeId nodeId, Point proposedPosition) const;

    /// Complete a drag operation - validates and applies the move
    /// @param nodeId Node being dragged
    /// @param finalPosition Final drop position
    /// @return MoveResult with success/failure
    NodeMoveResult completeDrag(NodeId nodeId, Point finalPosition);

    // =========================================================================
    // Constraint Validation
    // =========================================================================

    /// Validate current state against all constraints
    FinalStateValidationResult validateAll() const;

    /// Check if a position would be valid for a node
    bool canMoveNodeTo(NodeId nodeId, Point position) const;

private:
    // Private data - no direct access from outside
    std::unordered_map<NodeId, NodeLayout> nodeLayouts_;
    std::unordered_map<EdgeId, EdgeLayout> edgeLayouts_;

    const Graph& graph_;
    LayoutOptions options_;

    std::unique_ptr<ConstraintManager> constraintManager_;
    std::unique_ptr<ConstraintSolver> constraintSolver_;

    // Internal helpers
    void updateEdgeRouting(const std::vector<EdgeId>& affectedEdges,
                          const std::unordered_set<NodeId>& movedNodes);

    std::vector<EdgeId> getConnectedEdges(NodeId nodeId) const;
    std::vector<EdgeId> getExpandedAffectedEdges(NodeId nodeId) const;
};

}  // namespace arborvia
