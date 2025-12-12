#pragma once

#include "arborvia/layout/api/IEdgeOptimizer.h"
#include "arborvia/layout/constraints/SegmentObstacleProvider.h"
#include "../../sugiyama/routing/PathIntersection.h"
#include "../../sugiyama/routing/EdgeRoutingUtils.h"

#include <unordered_map>
#include <vector>

namespace arborvia {

/// Fast edge optimizer using geometric path prediction (no A* pathfinding).
/// Evaluates 16 edge combinations using estimated path lengths and angles.
/// Ideal for real-time feedback during drag operations.
class GeometricEdgeOptimizer : public IEdgeOptimizer {
public:
    GeometricEdgeOptimizer() = default;

    std::unordered_map<EdgeId, EdgeLayout> optimize(
        const std::vector<EdgeId>& edges,
        const std::unordered_map<EdgeId, EdgeLayout>& currentLayouts,
        const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
        float gridSize,
        const std::unordered_set<NodeId>& movedNodes = {}) override;

    const char* algorithmName() const override { return "Geometric"; }

    /// Regenerate bendPoints using geometric path with obstacle avoidance
    /// Preserves existing sourceEdge/targetEdge, only recreates path geometry
    void regenerateBendPoints(
        const std::vector<EdgeId>& edges,
        std::unordered_map<EdgeId, EdgeLayout>& edgeLayouts,
        const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
        float gridSize) override;

private:
    /// Result of evaluating a single edge combination (uses shared type)
    using CombinationResult = EdgeCombinationResult;

    /// Evaluate edge combinations geometrically
    /// When preserveDirections() is true, only evaluates the existing combination.
    /// Otherwise, evaluates all 16 combinations (4 source × 4 target edges).
    std::vector<CombinationResult> evaluateCombinations(
        EdgeId edgeId,
        const EdgeLayout& baseLayout,
        const std::unordered_map<EdgeId, EdgeLayout>& assignedLayouts,
        const std::unordered_map<NodeId, NodeLayout>& nodeLayouts);

    /// Evaluate self-loop combinations (4 directions)
    /// Uses penalty system to select the best direction that avoids segment overlap
    std::vector<CombinationResult> evaluateSelfLoopCombinations(
        EdgeId edgeId,
        const EdgeLayout& baseLayout,
        const std::unordered_map<EdgeId, EdgeLayout>& assignedLayouts,
        const std::unordered_map<NodeId, NodeLayout>& nodeLayouts);

    /// Create candidate layout with geometric path prediction and overlap avoidance
    EdgeLayout createCandidateLayout(
        const EdgeLayout& base,
        NodeEdge sourceEdge,
        NodeEdge targetEdge,
        const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
        const std::unordered_map<EdgeId, EdgeLayout>& assignedLayouts);

    /// Calculate edge center point for given node edge (grid-aligned using SnapPointCalculator)
    /// @deprecated Use calculateSnapPositionWithContext for proper distribution
    Point calculateEdgeCenter(
        const NodeLayout& node,
        NodeEdge edge) const;

    /// Calculate snap position considering existing connections on the same node edge
    /// Uses GridSnapCalculator (SSOT) to distribute connections evenly
    Point calculateSnapPositionWithContext(
        const NodeLayout& node,
        NodeEdge nodeEdge,
        const EdgeLayout& currentEdge,
        const std::unordered_map<EdgeId, EdgeLayout>& assignedLayouts,
        bool isSource) const;

    /// Predict simple orthogonal path (L-shaped or straight)
    static std::vector<BendPoint> predictOrthogonalPath(
        const Point& source,
        const Point& target,
        NodeEdge sourceEdge,
        NodeEdge targetEdge);

    /// Create path with obstacle avoidance
    /// @param source Source point
    /// @param target Target point
    /// @param sourceEdge Source edge direction
    /// @param targetEdge Target edge direction
    /// @param nodeLayouts All node layouts for collision detection
    /// @param excludeNodes Nodes to exclude from collision check (source/target)
    std::vector<BendPoint> createPathWithObstacleAvoidance(
        const Point& source,
        const Point& target,
        NodeEdge sourceEdge,
        NodeEdge targetEdge,
        const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
        NodeId sourceNodeId,
        NodeId targetNodeId);

    /// Find nodes that collide with a path segment
    /// @param segmentPos Position of segment in path (First/Middle/Last)
    ///        - First: excludes source node only
    ///        - Middle: excludes no nodes (checks all including src/tgt)
    ///        - Last: excludes target node only
    std::vector<std::pair<NodeId, NodeLayout>> findCollidingNodes(
        const Point& p1,
        const Point& p2,
        const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
        NodeId sourceNode,
        NodeId targetNode,
        SegmentPosition segmentPos = SegmentPosition::Middle);

    /// Create detour path around an obstacle
    /// @param source Start point of segment
    /// @param target End point of segment  
    /// @param obstacle The node to route around
    /// @param margin Clearance margin around obstacle
    /// @return Bend points for the detour (excluding source/target)
    static std::vector<Point> createDetourAroundNode(
        const Point& source,
        const Point& target,
        const NodeLayout& obstacle,
        float margin = 10.0f);

    /// Check if a segment intersects a node (with margin)
    static bool segmentIntersectsNode(
        const Point& p1,
        const Point& p2,
        const NodeLayout& node,
        float margin = 0.0f);

    /// Score the geometric path quality (lower is better)
    int scoreGeometricPath(
        const EdgeLayout& candidate,
        const std::unordered_map<EdgeId, EdgeLayout>& assignedLayouts,
        const std::unordered_map<NodeId, NodeLayout>& nodeLayouts);

    /// Adjust path to avoid overlap with assigned edges
    /// @param candidate Candidate layout to adjust
    /// @param assignedLayouts Already assigned edges
    /// @return Adjusted bend points that avoid overlap
    std::vector<BendPoint> adjustPathToAvoidOverlap(
        const EdgeLayout& candidate,
        const std::unordered_map<EdgeId, EdgeLayout>& assignedLayouts);

    /// Find alternative X coordinate that doesn't overlap with existing vertical segments
    /// @param originalX The X coordinate that causes overlap
    /// @param yMin Minimum Y of the segment
    /// @param yMax Maximum Y of the segment
    /// @param assignedLayouts Already assigned edges
    /// @param gridSpacing Grid spacing for offset calculation
    /// @return Alternative X coordinate
    float findAlternativeX(
        float originalX,
        float yMin,
        float yMax,
        const std::unordered_map<EdgeId, EdgeLayout>& assignedLayouts,
        float gridSpacing = 20.0f);

    /// Find alternative Y coordinate that doesn't overlap with existing horizontal segments
    float findAlternativeY(
        float originalY,
        float xMin,
        float xMax,
        const std::unordered_map<EdgeId, EdgeLayout>& assignedLayouts,
        float gridSpacing = 20.0f);

    /// Create self-loop path with collision avoidance against other nodes
    /// Tries different offsets if collision is detected
    std::vector<BendPoint> createSelfLoopPath(
        const EdgeLayout& layout,
        const NodeLayout& selfNode,
        const std::unordered_map<NodeId, NodeLayout>& nodeLayouts);

    /// Check if a self-loop path has collision with other nodes
    bool selfLoopHasCollision(
        const std::vector<BendPoint>& bends,
        const Point& source,
        const Point& target,
        NodeId selfNodeId,
        const std::unordered_map<NodeId, NodeLayout>& nodeLayouts);

    /// Try alternative mid-point strategies for path with collision
    /// Returns the best path based on penalty scoring
    std::vector<BendPoint> tryAlternativeMidPoints(
        const Point& source,
        const Point& target,
        NodeEdge sourceEdge,
        NodeEdge targetEdge,
        const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
        const std::unordered_map<EdgeId, EdgeLayout>& edgeLayouts,
        NodeId sourceNodeId,
        NodeId targetNodeId);

    /// Grid size set during optimize() call - used by helper methods
    float gridSize_ = 20.0f;
    
    // Constraint state inherited from IEdgeOptimizer:
    // - originalLayouts_, movedNodes_, isNodeFixed(), setConstraintState()
    // - createPenaltyContext() for consistent PenaltyContext creation
};

}  // namespace arborvia
