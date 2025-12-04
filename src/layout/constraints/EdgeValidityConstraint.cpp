#include "arborvia/layout/EdgeValidityConstraint.h"
#include "arborvia/core/GeometryUtils.h"
#include "../sugiyama/EdgeRouting.h"

#include <unordered_set>

namespace arborvia {

ConstraintResult EdgeValidityConstraint::check(const ConstraintContext& ctx) const {
    auto nodeIt = ctx.nodeLayouts.find(ctx.nodeId);
    if (nodeIt == ctx.nodeLayouts.end()) {
        return ConstraintResult::ok();  // Node not found, allow move
    }

    const Size& nodeSize = nodeIt->second.size;

    // Collect affected edges:
    // 1. Edges connected to the moved node
    // 2. Edges whose path intersects the new node position
    std::vector<EdgeId> affectedEdges;
    affectedEdges.reserve(ctx.edgeLayouts.size() / 10);

    for (const auto& [edgeId, layout] : ctx.edgeLayouts) {
        // Type 1: Connected to moved node
        if (layout.from == ctx.nodeId || layout.to == ctx.nodeId) {
            affectedEdges.push_back(edgeId);
            continue;
        }
        // Type 2: Path intersects new node position
        if (geometry::edgePathIntersectsAABB(layout, ctx.newPosition, nodeSize)) {
            affectedEdges.push_back(edgeId);
        }
    }

    // If no edges affected, validation passes
    if (affectedEdges.empty()) {
        return ConstraintResult::ok();
    }

    // Create temporary layouts with the proposed position
    auto tempNodeLayouts = ctx.nodeLayouts;
    auto tempNodeIt = tempNodeLayouts.find(ctx.nodeId);
    tempNodeIt->second.position = ctx.newPosition;

    auto tempEdgeLayouts = ctx.edgeLayouts;

    // Update affected edge positions
    std::unordered_set<NodeId> movedNodes = {ctx.nodeId};
    algorithms::EdgeRouting routing;
    routing.updateEdgePositions(
        tempEdgeLayouts, tempNodeLayouts, affectedEdges,
        movedNodes, ctx.gridSize);

    // Validate affected edges
    std::vector<EdgeId> invalidEdges;
    for (EdgeId edgeId : affectedEdges) {
        const auto& layout = tempEdgeLayouts.at(edgeId);
        auto validation = algorithms::EdgeRouting::validateEdgeLayout(layout, tempNodeLayouts);
        if (!validation.valid) {
            invalidEdges.push_back(edgeId);
        }
    }

    if (!invalidEdges.empty()) {
        return ConstraintResult::failWithEdges(
            "Edge routing would become invalid",
            std::move(invalidEdges));
    }

    return ConstraintResult::ok();
}

}  // namespace arborvia
