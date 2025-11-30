#pragma once
#include "../core/Types.h"
#include <unordered_map>
#include <string>

namespace arborvia {

// Layout mode selection
enum class LayoutMode {
    Auto,    // Algorithm determines all positions and snap points
    Manual   // User controls node positions and edge routing
};

// Which edge of a node to use for connections
enum class NodeEdge {
    Top,
    Bottom,
    Left,
    Right
};

// Snap point count configuration per node edge
struct SnapPointConfig {
    int topCount = 1;
    int bottomCount = 1;
    int leftCount = 1;
    int rightCount = 1;

    int getCount(NodeEdge edge) const {
        switch (edge) {
            case NodeEdge::Top: return topCount;
            case NodeEdge::Bottom: return bottomCount;
            case NodeEdge::Left: return leftCount;
            case NodeEdge::Right: return rightCount;
        }
        return 1;
    }

    void setCount(NodeEdge edge, int count) {
        switch (edge) {
            case NodeEdge::Top: topCount = count; break;
            case NodeEdge::Bottom: bottomCount = count; break;
            case NodeEdge::Left: leftCount = count; break;
            case NodeEdge::Right: rightCount = count; break;
        }
    }
};

// Edge routing configuration - which edge and snap point to use
struct EdgeRoutingConfig {
    NodeEdge sourceEdge = NodeEdge::Bottom;  // Which edge of source node
    NodeEdge targetEdge = NodeEdge::Top;     // Which edge of target node
    int sourceSnapIndex = 0;                  // Which snap point on source edge
    int targetSnapIndex = 0;                  // Which snap point on target edge
};

// Complete manual layout state
struct ManualLayoutState {
    // Node positions (user-dragged positions)
    std::unordered_map<NodeId, Point> nodePositions;

    // Edge routing configurations
    std::unordered_map<EdgeId, EdgeRoutingConfig> edgeRoutings;

    // Snap point configurations per node
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
