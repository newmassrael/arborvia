#pragma once

#include "LayoutEnums.h"
#include <unordered_map>

namespace arborvia {

/// Complete manual layout state for Auto/Manual mode switching
struct ManualLayoutState {
    /// Node positions (user-dragged positions)
    std::unordered_map<NodeId, Point> nodePositions;

    /// Edge routing configurations
    std::unordered_map<EdgeId, EdgeRoutingConfig> edgeRoutings;

    /// Snap point configurations per node
    std::unordered_map<NodeId, SnapPointConfig> snapConfigs;

    bool hasNodePosition(NodeId id) const {
        return nodePositions.find(id) != nodePositions.end();
    }

    bool hasEdgeRouting(EdgeId id) const {
        return edgeRoutings.find(id) != edgeRoutings.end();
    }

    bool hasSnapConfig(NodeId id) const {
        return snapConfigs.find(id) != snapConfigs.end();
    }

    void clear() {
        nodePositions.clear();
        edgeRoutings.clear();
        snapConfigs.clear();
    }

    bool isEmpty() const {
        return nodePositions.empty() && edgeRoutings.empty() && snapConfigs.empty();
    }
};

}  // namespace arborvia
