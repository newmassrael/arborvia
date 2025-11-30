#pragma once

#include "LayoutTypes.h"
#include "LayoutResult.h"
#include "../core/Graph.h"
#include <string>

namespace arborvia {

/// Manages manual layout state for Auto/Manual mode switching
class ManualLayoutManager {
public:
    ManualLayoutManager() = default;

    // Mode management
    LayoutMode getMode() const { return mode_; }
    void setMode(LayoutMode mode) { mode_ = mode; }

    // Manual state access
    ManualLayoutState& getManualState() { return manualState_; }
    const ManualLayoutState& getManualState() const { return manualState_; }

    // Node position management
    void setNodePosition(NodeId id, const Point& position);
    Point getNodePosition(NodeId id) const;
    bool hasNodePosition(NodeId id) const;

    // Snap point configuration
    void setSnapPointCount(NodeId id, NodeEdge edge, int count);
    int getSnapPointCount(NodeId id, NodeEdge edge) const;
    void setSnapConfig(NodeId id, const SnapPointConfig& config);
    SnapPointConfig getSnapConfig(NodeId id) const;

    // Edge routing configuration
    void setEdgeRouting(EdgeId id, const EdgeRoutingConfig& config);
    void setEdgeSourceEdge(EdgeId id, NodeEdge edge, int snapIndex = 0);
    void setEdgeTargetEdge(EdgeId id, NodeEdge edge, int snapIndex = 0);
    EdgeRoutingConfig getEdgeRouting(EdgeId id) const;
    bool hasEdgeRouting(EdgeId id) const;

    // Bend point management
    void addBendPoint(EdgeId edgeId, size_t index, const Point& position);
    void appendBendPoint(EdgeId edgeId, const Point& position);
    void removeBendPoint(EdgeId edgeId, size_t index);
    void moveBendPoint(EdgeId edgeId, size_t index, const Point& position);
    const std::vector<BendPoint>& getBendPoints(EdgeId edgeId) const;
    bool hasManualBendPoints(EdgeId edgeId) const;
    void clearBendPoints(EdgeId edgeId);
    void setBendPoints(EdgeId edgeId, const std::vector<BendPoint>& points);

    /// Result of orthogonal drag constraint calculation
    struct OrthogonalDragResult {
        Point newCurrentPos{0, 0};      ///< Constrained position for the dragged bend point
        Point adjustedNextPos{0, 0};    ///< Adjusted position for the next bend point (if applicable)
        bool nextAdjusted = false;      ///< True if next bend point position was adjusted
    };

    /// Calculate constrained drag position to maintain orthogonality.
    /// This is the core logic for orthogonal bend point dragging.
    /// @param prevPoint The previous point in the path (source or previous bend)
    /// @param currentPos Current position of the bend point being dragged
    /// @param nextPoint The next point in the path (next bend or target)
    /// @param dragTarget Where the user is trying to drag to
    /// @param hasNextBend True if there is a next bend point that can be adjusted
    /// @param isLastBend True if this is the last bend point (next is target)
    /// @return Constrained positions that maintain orthogonality
    static OrthogonalDragResult calculateOrthogonalDrag(
        const Point& prevPoint,
        const Point& currentPos,
        const Point& nextPoint,
        const Point& dragTarget,
        bool hasNextBend,
        bool isLastBend);

    // Apply manual state to layout result
    void applyManualState(LayoutResult& result, const Graph& graph) const;

    // Capture current layout as manual state
    void captureFromResult(const LayoutResult& result);

    // Snap point calculation
    static Point calculateSnapPoint(
        const NodeLayout& node,
        NodeEdge edge,
        int snapIndex,
        int totalSnapPoints);

    // JSON serialization
    std::string toJson() const;
    bool fromJson(const std::string& json);
    bool saveToFile(const std::string& path) const;
    bool loadFromFile(const std::string& path);

    // Clear manual state
    void clearManualState();

private:
    LayoutMode mode_ = LayoutMode::Auto;
    ManualLayoutState manualState_;

    // Helper: get default snap config
    static SnapPointConfig defaultSnapConfig();
};

}  // namespace arborvia
