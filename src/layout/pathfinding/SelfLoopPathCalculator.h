#pragma once

#include "arborvia/layout/api/IEdgePathCalculator.h"

namespace arborvia {

/// Path calculator for self-loop edges (from == to)
///
/// Self-loops use geometric L-shaped routing through a corner point
/// outside the node. This calculator handles:
/// - Extension point calculation (perpendicular to each edge)
/// - Corner point selection (outside node bounds)
/// - Grid snapping for consistent rendering
///
/// Valid self-loop combinations are adjacent edges only:
/// - Top ↔ Left, Top ↔ Right
/// - Bottom ↔ Left, Bottom ↔ Right
///
/// This centralizes self-loop path logic that was previously duplicated in:
/// - SnapPointController::calculatePreviewPath()
/// - AStarEdgeOptimizer::regenerateBendPoints()
/// - GeometricEdgeOptimizer::regenerateBendPoints()
class SelfLoopPathCalculator : public IEdgePathCalculator {
public:
    SelfLoopPathCalculator() = default;
    ~SelfLoopPathCalculator() override = default;

    /// Calculate L-shaped path for self-loop
    EdgePathResult calculatePath(
        const EdgeLayout& layout,
        const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
        const PathConfig& config) override;

    /// Returns true if layout.from == layout.to
    bool canHandle(const EdgeLayout& layout) const override;

    const char* name() const override { return "SelfLoopPathCalculator"; }

private:
    /// Calculate extension point perpendicular to node edge
    static Point calculateExtensionPoint(
        const Point& snapPoint,
        NodeEdge edge,
        float offset);

    /// Select corner point that is outside node bounds
    static Point selectCornerOutsideNode(
        const Point& srcExt,
        const Point& tgtExt,
        const NodeLayout& node);
};

}  // namespace arborvia
