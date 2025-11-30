#include "arborvia/layout/LayoutUtils.h"
#include "sugiyama/EdgeRouting.h"

namespace arborvia {

void LayoutUtils::updateEdgePositions(
    std::unordered_map<EdgeId, EdgeLayout>& edgeLayouts,
    const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
    const std::vector<EdgeId>& affectedEdges,
    SnapDistribution distribution,
    const std::unordered_set<NodeId>& movedNodes) {
    
    algorithms::EdgeRouting::updateEdgePositions(
        edgeLayouts, nodeLayouts, affectedEdges, distribution, movedNodes);
}

}  // namespace arborvia
