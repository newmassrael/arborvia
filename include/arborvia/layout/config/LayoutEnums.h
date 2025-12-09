#pragma once

#include "../../core/Types.h"
#include <vector>

namespace arborvia {

/// A bend point in an edge route
struct BendPoint {
    Point position;
    bool isControlPoint = false;  ///< For spline curves
};

/// Which edge of a node to use for connections
enum class NodeEdge {
    Top,
    Bottom,
    Left,
    Right
};

/// Snap point count configuration per node edge
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

/// Edge routing configuration - which edge and snap point to use
struct EdgeRoutingConfig {
    NodeEdge sourceEdge = NodeEdge::Bottom;  ///< Which edge of source node
    NodeEdge targetEdge = NodeEdge::Top;     ///< Which edge of target node
    int sourceSnapIndex = 0;                  ///< Which snap point on source edge
    int targetSnapIndex = 0;                  ///< Which snap point on target edge

    /// Manual bend points (empty = use auto routing)
    std::vector<BendPoint> manualBendPoints;

    bool hasManualBendPoints() const {
        return !manualBendPoints.empty();
    }
};

}  // namespace arborvia
