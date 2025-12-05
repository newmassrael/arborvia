#pragma once

#include "arborvia/core/Types.h"
#include "arborvia/layout/LayoutOptions.h"
#include "arborvia/layout/LayoutResult.h"

#include <unordered_map>

namespace arborvia {


// Use ScoringWeights from LayoutOptions.h (arborvia namespace)
using ::arborvia::ScoringWeights;

/// Scores edge routing combinations for optimization
/// Used by AStarEdgeOptimizer to evaluate and compare routing options
/// 
/// This is SEPARATE from the constraint system:
/// - Constraints: Boolean validation (can we do this?)
/// - Scorer: Numeric optimization (which option is best?)
class EdgeScorer {
public:
    explicit EdgeScorer(const ScoringWeights& weights = {});

    /// Calculate total score for an edge layout
    /// Lower scores are better
    /// @param candidate The edge layout to score
    /// @param assignedLayouts Other edges already assigned (for intersection counting)
    /// @param nodeLayouts Node positions (for collision detection)
    /// @return Total weighted score
    int calculateScore(
        const EdgeLayout& candidate,
        const std::unordered_map<EdgeId, EdgeLayout>& assignedLayouts,
        const std::unordered_map<NodeId, NodeLayout>& nodeLayouts) const;

    // === Individual Scoring Components ===

    /// Check if snap points (source/target) are too close together
    /// @param layout Edge layout to check
    /// @return 1 if too close, 0 otherwise
    int checkSnapPointProximity(const EdgeLayout& layout) const;

    /// Check for self-overlap (MIN_SEGMENT violation)
    /// Detects when path would need to backtrack
    /// @param layout Edge layout to check
    /// @return 1 if self-overlap detected, 0 otherwise
    int checkSelfOverlap(const EdgeLayout& layout) const;

    /// Count node collisions (path passing through nodes)
    /// @param layout Edge layout to check
    /// @param nodeLayouts All node layouts
    /// @return Number of nodes the path passes through
    int countNodeCollisions(
        const EdgeLayout& layout,
        const std::unordered_map<NodeId, NodeLayout>& nodeLayouts) const;

    /// Count intersections with other edge paths
    /// @param layout Edge layout to check
    /// @param otherLayouts Other edge layouts
    /// @return Number of intersections
    int countPathIntersections(
        const EdgeLayout& layout,
        const std::unordered_map<EdgeId, EdgeLayout>& otherLayouts) const;

    /// Calculate distance component (Euclidean distance)
    /// @param layout Edge layout
    /// @return Distance between source and target points
    float calculateDistance(const EdgeLayout& layout) const;

    /// Get current weights
    const ScoringWeights& weights() const { return weights_; }

    /// Set new weights
    void setWeights(const ScoringWeights& weights) { weights_ = weights; }

private:
    ScoringWeights weights_;

    /// Check if segment intersects node (wraps EdgeRouting::segmentIntersectsNode)
    static bool segmentIntersectsNode(
        const Point& p1, const Point& p2,
        const NodeLayout& node, float margin = 0.0f);
};


}  // namespace arborvia
