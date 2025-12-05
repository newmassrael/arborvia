#pragma once

#include "arborvia/layout/IEdgeOptimizer.h"
#include "EdgeScorer.h"

#include <unordered_map>
#include <vector>

namespace arborvia {

/// Fast edge optimizer using geometric path prediction (no A* pathfinding).
/// Evaluates 16 edge combinations using estimated path lengths and angles.
/// Ideal for real-time feedback during drag operations.
class GeometricEdgeOptimizer : public IEdgeOptimizer {
public:
    explicit GeometricEdgeOptimizer(const ScoringWeights& weights = {});

    void setWeights(const ScoringWeights& weights);
    [[nodiscard]] const ScoringWeights& weights() const;

    std::unordered_map<EdgeId, EdgeLayout> optimize(
        const std::vector<EdgeId>& edges,
        const std::unordered_map<EdgeId, EdgeLayout>& currentLayouts,
        const std::unordered_map<NodeId, NodeLayout>& nodeLayouts) override;

    const char* algorithmName() const override { return "Geometric"; }

private:
    struct CombinationResult {
        NodeEdge sourceEdge;
        NodeEdge targetEdge;
        int score;
        EdgeLayout layout;
    };

    /// Evaluate all 16 edge combinations geometrically
    std::vector<CombinationResult> evaluateAllCombinations(
        EdgeId edgeId,
        const EdgeLayout& baseLayout,
        const std::unordered_map<EdgeId, EdgeLayout>& assignedLayouts,
        const std::unordered_map<NodeId, NodeLayout>& nodeLayouts);

    /// Create candidate layout with geometric path prediction
    EdgeLayout createCandidateLayout(
        const EdgeLayout& base,
        NodeEdge sourceEdge,
        NodeEdge targetEdge,
        const std::unordered_map<NodeId, NodeLayout>& nodeLayouts);

    /// Calculate edge center point for given node edge
    static Point calculateEdgeCenter(
        const NodeLayout& node,
        NodeEdge edge);

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
    std::vector<std::pair<NodeId, NodeLayout>> findCollidingNodes(
        const Point& p1,
        const Point& p2,
        const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
        NodeId excludeSource,
        NodeId excludeTarget);

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

    EdgeScorer scorer_;
};

}  // namespace arborvia
