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
class CycleRemoval;
class LongestPathLayerAssignment;
class BarycenterCrossingMinimization;
class SimpleCoordinateAssignment;
class EdgeRouting;

// Interfaces
class ICycleRemoval;
class ILayerAssignment;
class ICrossingMinimization;
class ICoordinateAssignment;
class IPathFinder;

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
    void setManualLayoutManager(std::shared_ptr<ManualLayoutManager> manager) { manualManager_ = manager; }
    std::shared_ptr<ManualLayoutManager> manualLayoutManager() const { return manualManager_; }

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

    /// Algorithm injection (for swapping implementations)
    /// If not set, defaults are used
    void setCycleRemoval(std::shared_ptr<ICycleRemoval> impl);
    void setLayerAssignment(std::shared_ptr<ILayerAssignment> impl);
    void setCrossingMinimization(std::shared_ptr<ICrossingMinimization> impl);
    void setCoordinateAssignment(std::shared_ptr<ICoordinateAssignment> impl);
    void setPathFinder(std::shared_ptr<IPathFinder> impl);

private:
    LayoutOptions options_;
    LayoutStats stats_;
    std::shared_ptr<ManualLayoutManager> manualManager_;

    // Algorithm implementations (nullptr = use defaults)
    std::shared_ptr<ICycleRemoval> cycleRemoval_;
    std::shared_ptr<ILayerAssignment> layerAssignment_;
    std::shared_ptr<ICrossingMinimization> crossingMinimization_;
    std::shared_ptr<ICoordinateAssignment> coordinateAssignment_;
    std::shared_ptr<IPathFinder> pathFinder_;

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

    /// Final cleanup: remove duplicate bend points and fix bends inside nodes
    void applyFinalCleanup();

    // Compound graph specific
    void layoutCompoundNode(NodeId id, const CompoundGraph& graph);
    void layoutParallelRegions(NodeId id, const CompoundGraph& graph);
    void updateCompoundBounds(NodeId id, const CompoundGraph& graph);
};

}  // namespace arborvia
