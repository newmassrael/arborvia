#pragma once

#include "arborvia/core/Types.h"
#include "arborvia/core/Graph.h"
#include "arborvia/layout/config/LayoutResult.h"

#include <unordered_map>
#include <optional>
#include <string>
#include <vector>

namespace arborvia {

/// Result of full constraint validation
struct ConstraintValidationResult {
    bool satisfied = true;
    std::vector<NodeId> overlappingNodes;           ///< Nodes that overlap
    std::vector<EdgeId> invalidPathEdges;           ///< Edges without valid A* path
    std::vector<EdgeId> diagonalEdges;              ///< Edges with diagonal segments
    std::vector<std::pair<EdgeId, EdgeId>> overlappingEdgePairs; ///< Edge pairs that overlap
    
    bool hasNodeOverlap() const { return !overlappingNodes.empty(); }
    bool hasInvalidPaths() const { return !invalidPathEdges.empty(); }
    bool hasDiagonals() const { return !diagonalEdges.empty(); }
    bool hasEdgeOverlap() const { return !overlappingEdgePairs.empty(); }
    
    std::string summary() const;
};

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

/// Solves layout constraints by finding valid node positions
/// Ensures all edges have valid A* paths after node placement
/// Configuration for constraint solving
struct ConstraintSolverConfig {
    float gridSize = 10.0f;         ///< Grid size for A* pathfinding
    float maxSearchDistance = 200.0f; ///< Max distance to search for valid position
    float searchStepSize = 10.0f;   ///< Step size for position search
    int maxIterations = 1000;       ///< Max iterations for position search
};

class ConstraintSolver {
public:
    using Config = ConstraintSolverConfig;

    explicit ConstraintSolver(const Config& config = Config{});

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

    // =========================================================================
    // Unified Constraint Validation (Single Entry Point)
    // =========================================================================

    /// Validate all constraints on the current layout state
    /// This is the SINGLE entry point for all constraint checking
    /// @param nodeLayouts All node layouts
    /// @param edgeLayouts All edge layouts
    /// @param graph Graph for connectivity
    /// @return Detailed validation result
    ConstraintValidationResult validateAll(
        const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
        const std::unordered_map<EdgeId, EdgeLayout>& edgeLayouts,
        const Graph& graph) const;

    /// Check for edge overlap (segment overlap between edge pairs)
    /// @param edgeLayouts All edge layouts
    /// @return Vector of overlapping edge pairs
    std::vector<std::pair<EdgeId, EdgeId>> findOverlappingEdgePairs(
        const std::unordered_map<EdgeId, EdgeLayout>& edgeLayouts) const;

    /// Check for diagonal segments in edges
    /// @param edgeLayouts All edge layouts
    /// @return Vector of edge IDs with diagonal segments
    std::vector<EdgeId> findDiagonalEdges(
        const std::unordered_map<EdgeId, EdgeLayout>& edgeLayouts) const;

    /// Check for node overlaps
    /// @param nodeLayouts All node layouts
    /// @return Vector of overlapping node IDs
    std::vector<NodeId> findOverlappingNodes(
        const std::unordered_map<NodeId, NodeLayout>& nodeLayouts) const;

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
