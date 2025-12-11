#pragma once

#include "arborvia/core/Types.h"
#include "arborvia/core/Graph.h"
#include "arborvia/layout/config/LayoutResult.h"

#include <unordered_map>
#include <optional>
#include <string>
#include <vector>

namespace arborvia {

/// Result of constraint-satisfying node placement
struct ConstraintPlacementResult {
    bool success = false;               ///< True if placement succeeded
    Point finalPosition{0, 0};          ///< Actual position after placement
    Point requestedPosition{0, 0};      ///< Originally requested position
    bool positionAdjusted = false;      ///< True if position was auto-adjusted
    std::string reason;                 ///< Failure reason or adjustment explanation
    
    /// Factory methods
    static ConstraintPlacementResult ok(Point pos) {
        return {true, pos, pos, false, ""};
    }
    
    static ConstraintPlacementResult adjusted(Point requested, Point actual, const std::string& reason) {
        return {true, actual, requested, true, reason};
    }
    
    static ConstraintPlacementResult fail(Point requested, const std::string& reason) {
        return {false, {0, 0}, requested, false, reason};
    }
};

/// Configuration for constraint solving
struct PositionFinderConfig {
    float gridSize = 10.0f;         ///< Grid size for A* pathfinding
    float maxSearchDistance = 200.0f; ///< Max distance to search for valid position
    float searchStepSize = 10.0f;   ///< Step size for position search
    int maxIterations = 1000;       ///< Max iterations for position search
};

/// Finds valid node positions that satisfy layout constraints
/// Ensures all edges have valid A* paths after node placement
///
/// NOTE: Global state validation has been moved to ConstraintManager::validateFinalState().
/// This class now focuses ONLY on position-finding operations.
class PositionFinder {
public:
    using Config = PositionFinderConfig;

    explicit PositionFinder(const Config& config = Config{});

    /// Place a node at a position that satisfies all constraints
    /// If the desired position violates constraints, finds nearest valid position
    /// @param nodeId Node to place
    /// @param desiredPosition Desired position (may be adjusted)
    /// @param nodeSize Size of the node
    /// @param nodeLayouts Current node layouts (will be modified on success)
    /// @param edgeLayouts Edge layouts for A* path validation
    /// @param graph Graph for connectivity info
    /// @return PlacementResult with final position or failure reason
    ConstraintPlacementResult placeNode(
        NodeId nodeId,
        Point desiredPosition,
        Size nodeSize,
        std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
        const std::unordered_map<EdgeId, EdgeLayout>& edgeLayouts,
        const Graph& graph);

    /// Validate that all edges connected to a node have valid A* paths
    /// @param nodeId Node to check
    /// @param nodePosition Position of the node
    /// @param nodeSize Size of the node
    /// @param nodeLayouts All node layouts
    /// @param edgeLayouts All edge layouts
    /// @param graph Graph for connectivity
    /// @return true if all connected edges have valid A* paths
    bool validateAllEdgePaths(
        NodeId nodeId,
        Point nodePosition,
        Size nodeSize,
        const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
        const std::unordered_map<EdgeId, EdgeLayout>& edgeLayouts,
        const Graph& graph) const;

    /// Check if a node position overlaps with other nodes
    /// @param nodeId Node to check (excluded from collision check)
    /// @param position Position to check
    /// @param size Size of the node
    /// @param nodeLayouts All node layouts
    /// @param margin Minimum gap between nodes
    /// @return true if no overlap
    bool checkNoOverlap(
        NodeId nodeId,
        Point position,
        Size size,
        const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
        float margin = 0.0f) const;

private:
    /// Find the nearest position that satisfies all constraints
    /// Uses BFS-like search starting from the desired position
    std::optional<Point> findNearestValidPosition(
        NodeId nodeId,
        Point center,
        Size nodeSize,
        const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
        const std::unordered_map<EdgeId, EdgeLayout>& edgeLayouts,
        const Graph& graph) const;

    /// Calculate snap point position on a node edge
    Point calculateSnapPointOnEdge(
        const NodeLayout& node,
        NodeEdge edge,
        int snapIndex) const;

    Config config_;
};

}  // namespace arborvia
