#pragma once

#include "arborvia/layout/api/IEdgePathCalculator.h"

namespace arborvia {

/// Path calculator for self-loop edges (from == to)
///
/// DELEGATION PATTERN: This calculator delegates to SelfLoopRouter when
/// context is set, ensuring consistent routing with loopIndex-based stacking.
///
/// When context is set (via setContext):
/// - Uses SelfLoopRouter::route() for path calculation
/// - Calculates loopIndex from edgeLayouts for proper stacking
/// - Always produces 3 bend points (consistent with initial routing)
///
/// When context is NOT set (fallback mode):
/// - Uses simple geometric L-shape routing
/// - May produce 2-3 bend points
/// - Does NOT support proper stacking of multiple self-loops
///
/// Valid self-loop combinations are adjacent edges only:
/// - Top ↔ Left, Top ↔ Right
/// - Bottom ↔ Left, Bottom ↔ Right
class SelfLoopPathCalculator : public IEdgePathCalculator {
public:
    SelfLoopPathCalculator() = default;
    ~SelfLoopPathCalculator() override = default;

    /// Calculate path for self-loop
    /// If context is set, delegates to SelfLoopRouter for consistent stacking
    EdgePathResult calculatePath(
        const EdgeLayout& layout,
        const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
        const PathConfig& config) override;

    /// Returns true if layout.from == layout.to
    bool canHandle(const EdgeLayout& layout) const override;

    const char* name() const override { return "SelfLoopPathCalculator"; }

    /// Set context for proper self-loop stacking (loopIndex calculation)
    /// @param ctx Must contain edgeLayouts; options is optional (uses gridSize if null)
    void setContext(const PathCalculatorContext& ctx) override;

private:
    /// Calculate path using SelfLoopRouter (when context is available)
    EdgePathResult calculatePathWithRouter(
        const EdgeLayout& layout,
        const std::unordered_map<NodeId, NodeLayout>& nodeLayouts) const;

    /// Fallback: Calculate extension point perpendicular to node edge
    static Point calculateExtensionPoint(
        const Point& snapPoint,
        NodeEdge edge,
        float offset);

    /// Fallback: Select corner point that is outside node bounds
    static Point selectCornerOutsideNode(
        const Point& srcExt,
        const Point& tgtExt,
        const NodeLayout& node);

    /// Context for delegation to SelfLoopRouter
    PathCalculatorContext context_;
};

}  // namespace arborvia
