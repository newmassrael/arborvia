#pragma once

#include "../config/LayoutTypes.h"
#include "../config/LayoutResult.h"
#include "../../core/Graph.h"
#include <string>

namespace arborvia {

/// Manages manual layout state for Auto/Manual mode switching
class ManualLayoutManager {
public:
    ManualLayoutManager() = default;

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

    /// Clear all edge routing configurations (snap edges and indices).
    /// Use this before re-routing to allow fresh routing from algorithm.
    void clearAllEdgeRoutings();

    // Apply manual state to layout result (node positions + edge routings)
    void applyManualState(LayoutResult& result, const Graph& graph) const;

    // Apply only manual node positions (call before edge routing)
    void applyManualNodePositions(LayoutResult& result) const;

    // Apply only manual edge routings (call after edge routing)
    void applyManualEdgeRoutings(LayoutResult& result) const;

    // Capture current layout as manual state
    void captureFromResult(const LayoutResult& result);

    /// Sync snap point configurations from edge layouts.
    /// Updates snap counts to accommodate all edge connections.
    /// @param edgeLayouts The edge layouts to sync from
    void syncSnapConfigsFromEdgeLayouts(
        const std::unordered_map<EdgeId, EdgeLayout>& edgeLayouts);

    // JSON serialization
    std::string toJson() const;
    bool fromJson(const std::string& json);
    bool saveToFile(const std::string& path) const;
    bool loadFromFile(const std::string& path);

    // Clear manual state
    void clearManualState();

private:
    ManualLayoutState manualState_;

    // Helper: get default snap config
    static SnapPointConfig defaultSnapConfig();
};

}  // namespace arborvia
