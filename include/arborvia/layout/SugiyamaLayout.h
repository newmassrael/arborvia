#pragma once

#include "../core/CompoundGraph.h"
#include "ILayout.h"
#include "LayoutOptions.h"
#include "LayoutResult.h"
#include "ManualLayoutManager.h"

#include <memory>
#include <vector>

namespace arborvia {

// Forward declarations for algorithm phases
namespace algorithms {
    class CycleRemoval;
    class LayerAssignment;
    class CrossingMinimization;
    class CoordinateAssignment;
    class EdgeRouting;
}

/// Sugiyama-style hierarchical graph layout algorithm
///
/// Implements the classic layered graph drawing approach:
/// 1. Cycle Removal - Make graph acyclic by reversing edges
/// 2. Layer Assignment - Assign nodes to horizontal layers
/// 3. Crossing Minimization - Reduce edge crossings
/// 4. Coordinate Assignment - Compute final x,y positions
/// 5. Edge Routing - Route edges with bend points
class SugiyamaLayout : public ILayout {
public:
    SugiyamaLayout();
    explicit SugiyamaLayout(const LayoutOptions& options);
    ~SugiyamaLayout();
    
    // Non-copyable, movable
    SugiyamaLayout(const SugiyamaLayout&) = delete;
    SugiyamaLayout& operator=(const SugiyamaLayout&) = delete;
    SugiyamaLayout(SugiyamaLayout&&) noexcept;
    SugiyamaLayout& operator=(SugiyamaLayout&&) noexcept;
    
    /// Set layout options
    void setOptions(const LayoutOptions& options) override;
    const LayoutOptions& options() const override { return options_; }

    /// Set manual layout manager for Auto/Manual mode support
    void setManualLayoutManager(ManualLayoutManager* manager) { manualManager_ = manager; }
    ManualLayoutManager* manualLayoutManager() const { return manualManager_; }

    /// Perform layout on a simple graph
    LayoutResult layout(const Graph& graph) override;

    /// Perform layout on a compound graph (with hierarchy)
    LayoutResult layout(const CompoundGraph& graph) override;

    /// Perform incremental layout (reuses cached result if graph unchanged)
    /// Returns true if a new layout was computed, false if cached result was used
    std::pair<LayoutResult, bool> layoutIncremental(Graph& graph);
    std::pair<LayoutResult, bool> layoutIncremental(CompoundGraph& graph);

    /// Check if cached result is valid for the given graph version
    bool hasCachedResult(uint64_t graphVersion) const { return cachedVersion_ == graphVersion && cachedVersion_ > 0; }
    
    /// Get statistics from last layout
    struct LayoutStats {
        int layerCount = 0;
        int maxLayerWidth = 0;
        int edgeCrossings = 0;
        int reversedEdges = 0;
        float totalEdgeLength = 0.0f;
    };
    const LayoutStats& lastStats() const { return stats_; }

private:
    LayoutOptions options_;
    LayoutStats stats_;
    ManualLayoutManager* manualManager_ = nullptr;

    // Incremental layout caching
    uint64_t cachedVersion_ = 0;
    LayoutResult cachedResult_;
    
    // Internal layout state
    struct LayoutState;
    std::unique_ptr<LayoutState> state_;
    
    // Layout phases for simple graphs
    void removeCycles();
    void assignLayers();
    void minimizeCrossings();
    void assignCoordinates();
    void routeEdges();
    void updateEdgePositionsAfterManualState();
    
    // Compound graph specific
    void layoutCompoundNode(NodeId id, const CompoundGraph& graph);
    void layoutParallelRegions(NodeId id, const CompoundGraph& graph);
    void updateCompoundBounds(NodeId id, const CompoundGraph& graph);
};

}  // namespace arborvia
