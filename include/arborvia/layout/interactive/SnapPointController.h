#pragma once

#include "arborvia/core/Types.h"
#include "arborvia/layout/config/LayoutOptions.h"
#include "arborvia/layout/config/LayoutEnums.h"
#include "arborvia/layout/config/LayoutResult.h"
#include "arborvia/layout/api/IPathFinder.h"

#include <vector>
#include <unordered_map>
#include <memory>
#include <functional>

namespace arborvia {

/// Snap point candidate on a node edge
struct SnapCandidate {
    NodeEdge edge;
    int candidateIndex;
    Point position;
};

/// Controller for snap point drag operations
/// Extracts snap point interaction logic from UI layer to library layer
class SnapPointController {
public:
    /// Minimum distance (in grid cells) for valid direct path without bend points
    static constexpr float kMinDirectPathGridCells = 2.0f;
    /// Result of starting a drag operation
    struct DragStartResult {
        bool success = false;
        std::string reason;
        std::vector<SnapCandidate> candidates;  ///< All valid snap candidates
        Point originalPosition;                  ///< Original snap point position
        NodeEdge originalEdge;                   ///< Original node edge
        int originalSnapIndex = -1;              ///< Original snap index
    };

    /// Result of updating drag position
    struct DragUpdateResult {
        bool hasValidPreview = false;
        int snappedCandidateIndex = -1;         ///< Index into candidates vector
        EdgeLayout previewLayout;                ///< Preview edge layout with A* path
    };

    /// Result of completing drag (drop)
    struct DropResult {
        bool success = false;
        std::string reason;
        EdgeId swapEdgeId = INVALID_EDGE;       ///< Edge that was swapped (if any)
        bool swapIsSource = false;               ///< Whether swap was on source side
        std::vector<EdgeId> affectedEdges;       ///< All edges needing reroute
        Point actualPosition;                    ///< Final snapped position
        NodeEdge actualEdge;                     ///< Final node edge
        int actualSnapIndex = -1;                ///< Final snap index
    };

    /// Callback type for bend point regeneration (decouples from EdgeRouting)
    using BendPointRegenerator = std::function<void(
        std::unordered_map<EdgeId, EdgeLayout>& edgeLayouts,
        const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
        const std::vector<EdgeId>& affectedEdges,
        const LayoutOptions& options)>;

    SnapPointController();
    explicit SnapPointController(std::shared_ptr<IPathFinder> pathFinder);
    ~SnapPointController();

    /// Start drag operation - calculates all candidates
    /// @param edgeId Edge being dragged
    /// @param isSource true if dragging source snap, false if target
    /// @param nodeLayouts Current node layouts
    /// @param edgeLayouts Current edge layouts
    /// @param gridSize Grid cell size for snap calculations
    DragStartResult startDrag(
        EdgeId edgeId,
        bool isSource,
        const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
        const std::unordered_map<EdgeId, EdgeLayout>& edgeLayouts,
        float gridSize);

    /// Update drag position - finds nearest candidate and generates preview
    /// @param mousePosition Current mouse position (graph coordinates)
    /// @param nodeLayouts Current node layouts (used for A* obstacle avoidance)
    /// @param gridSize Grid cell size
    DragUpdateResult updateDrag(
        Point mousePosition,
        const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
        float gridSize);

    /// Complete drag operation - applies changes and handles swap
    /// @param snappedCandidateIndex Index from DragUpdateResult (or -1 for mouse position)
    /// @param mousePosition Fallback position if snappedCandidateIndex is invalid
    /// @param nodeLayouts Current node layouts
    /// @param edgeLayouts Edge layouts to modify
    /// @param options Layout options
    /// @param regenerator Callback for regenerating bend points (optional)
    DropResult completeDrag(
        int snappedCandidateIndex,
        Point mousePosition,
        const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
        std::unordered_map<EdgeId, EdgeLayout>& edgeLayouts,
        const LayoutOptions& options,
        const BendPointRegenerator& regenerator = nullptr);

    /// Cancel drag operation - clears internal state
    void cancelDrag();

    /// Check if drag is in progress
    bool isDragging() const { return isDragging_; }

    /// Get current dragged edge ID
    EdgeId getDraggedEdgeId() const { return draggedEdgeId_; }

    /// Get whether dragging source (true) or target (false)
    bool isDraggingSource() const { return isDraggingSource_; }

    /// Get current candidates (valid after startDrag)
    const std::vector<SnapCandidate>& getCandidates() const { return candidates_; }

    /// Get current preview layout (valid after updateDrag with hasValidPreview=true)
    const EdgeLayout& getPreviewLayout() const { return previewLayout_; }

private:
    /// Find edge occupying a specific snap position
    struct OccupyingEdge {
        EdgeId edgeId = INVALID_EDGE;
        bool isSource = false;
    };

    OccupyingEdge findOccupyingEdge(
        NodeId nodeId,
        NodeEdge edge,
        int snapIndex,
        EdgeId excludeEdgeId,
        const std::unordered_map<EdgeId, EdgeLayout>& edgeLayouts) const;

    /// Calculate A* preview path
    bool calculatePreviewPath(
        EdgeLayout& layout,
        const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
        float gridSize);

    // Drag state
    bool isDragging_ = false;
    EdgeId draggedEdgeId_ = INVALID_EDGE;
    bool isDraggingSource_ = true;
    NodeId dragNodeId_ = INVALID_NODE;

    // Original state (for swap)
    Point originalPosition_;
    NodeEdge originalEdge_ = NodeEdge::Top;
    int originalSnapIndex_ = -1;
    EdgeLayout originalLayout_;

    // Candidates and preview
    std::vector<SnapCandidate> candidates_;
    EdgeLayout previewLayout_;
    int currentCandidateIndex_ = -1;

    // Pathfinder for preview
    std::shared_ptr<IPathFinder> pathFinder_;
};

}  // namespace arborvia
