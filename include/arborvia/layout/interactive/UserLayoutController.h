#pragma once

#include "../config/LayoutTypes.h"
#include "../config/LayoutResult.h"
#include "../../core/Graph.h"
#include <string>
#include <vector>

namespace arborvia {

/// Constants for orthogonal routing calculations
namespace OrthogonalRouting {
    constexpr float EPSILON = 0.001f;           ///< Tolerance for floating-point comparison
    constexpr float COLLINEAR_THRESHOLD = 1.0f; ///< Threshold for collinear point detection
}

/// Result of bend point pair calculation for orthogonal insertion
struct BendPointPairResult {
    Point first{0, 0};      ///< First bend point to insert
    Point second{0, 0};     ///< Second bend point to insert
    size_t insertIndex = 0; ///< Index at which to insert (first goes here, second at insertIndex+1)
};

/// Result of orthogonal drag constraint calculation
struct OrthogonalDragResult {
    Point newCurrentPos{0, 0};      ///< Constrained position for the dragged bend point
    Point adjustedNextPos{0, 0};    ///< Adjusted position for the next bend point (if applicable)
    bool nextAdjusted = false;      ///< True if next bend point position was adjusted
};

/// Controller for user layout modifications.
/// Manages manual layout state and provides orthogonal routing calculations.
class UserLayoutController {
public:
    UserLayoutController() = default;

    // =========================================================================
    // Manual State Access
    // =========================================================================

    ManualLayoutState& getManualState() { return manualState_; }
    const ManualLayoutState& getManualState() const { return manualState_; }

    // =========================================================================
    // Node Position Management
    // =========================================================================

    void setNodePosition(NodeId id, const Point& position);
    Point getNodePosition(NodeId id) const;
    bool hasNodePosition(NodeId id) const;

    // =========================================================================
    // Snap Point Configuration
    // =========================================================================

    void setSnapPointCount(NodeId id, NodeEdge edge, int count);
    int getSnapPointCount(NodeId id, NodeEdge edge) const;
    void setSnapConfig(NodeId id, const SnapPointConfig& config);
    SnapPointConfig getSnapConfig(NodeId id) const;

    // =========================================================================
    // Edge Routing Configuration
    // =========================================================================

    void setEdgeRouting(EdgeId id, const EdgeRoutingConfig& config);
    void setEdgeSourceEdge(EdgeId id, NodeEdge edge, int snapIndex = 0);
    void setEdgeTargetEdge(EdgeId id, NodeEdge edge, int snapIndex = 0);
    EdgeRoutingConfig getEdgeRouting(EdgeId id) const;
    bool hasEdgeRouting(EdgeId id) const;

    // =========================================================================
    // Bend Point Management (GridPoint API - ensures grid alignment)
    // =========================================================================

    /// Add a bend point at the specified index
    /// @param gridSize Required for converting GridPoint to pixel coordinates
    void addBendPoint(EdgeId edgeId, size_t index, const GridPoint& gridPosition, float gridSize);

    /// Append a bend point at the end
    void appendBendPoint(EdgeId edgeId, const GridPoint& gridPosition, float gridSize);

    /// Remove a bend point at the specified index
    void removeBendPoint(EdgeId edgeId, size_t index);

    /// Move a bend point to a new position
    void moveBendPoint(EdgeId edgeId, size_t index, const GridPoint& gridPosition, float gridSize);

    /// Get all bend points for an edge
    const std::vector<BendPoint>& getBendPoints(EdgeId edgeId) const;

    /// Check if edge has manual bend points
    bool hasManualBendPoints(EdgeId edgeId) const;

    /// Clear all bend points for an edge
    void clearBendPoints(EdgeId edgeId);

    /// Set all bend points for an edge
    void setBendPoints(EdgeId edgeId, const std::vector<GridPoint>& gridPositions, float gridSize);

    /// Clear all edge routing configurations (snap edges and indices).
    /// Use this before re-routing to allow fresh routing from algorithm.
    void clearAllEdgeRoutings();

    // =========================================================================
    // Orthogonal Routing Calculations (Static - Pure Functions)
    // =========================================================================

    /// Calculate a pair of bend points for orthogonal routing at click position.
    /// Creates two bend points that maintain orthogonality when inserted into the path.
    /// @param edgeLayout The edge layout (provides source/target points)
    /// @param existingBendPoints Current manual bend points (may be empty)
    /// @param clickPosition Where the user clicked to insert
    /// @param segmentIndex Which segment was clicked (from EdgeHitResult)
    /// @return BendPointPairResult with calculated positions and insert index
    static BendPointPairResult calculateBendPointPair(
        const EdgeLayout& edgeLayout,
        const std::vector<BendPoint>& existingBendPoints,
        const Point& clickPosition,
        int segmentIndex);

    /// Calculate constrained drag position to maintain orthogonality.
    /// Core logic for orthogonal bend point dragging.
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

    // =========================================================================
    // Apply/Capture Manual State
    // =========================================================================

    /// Apply manual state to layout result (node positions + edge routings)
    /// @param gridSize Grid cell size for snap point quantization (must be > 0)
    void applyManualState(LayoutResult& result, const Graph& graph, float gridSize) const;

    /// Apply only manual node positions (call before edge routing)
    void applyManualNodePositions(LayoutResult& result) const;

    /// Apply only manual edge routings (call after edge routing)
    /// @param gridSize Grid cell size for snap point quantization (must be > 0)
    void applyManualEdgeRoutings(LayoutResult& result, float gridSize) const;

    /// Capture current layout as manual state
    void captureFromResult(const LayoutResult& result);

    // =========================================================================
    // Serialization
    // =========================================================================

    std::string toJson() const;
    bool fromJson(const std::string& json);
    bool saveToFile(const std::string& path) const;
    bool loadFromFile(const std::string& path);

    /// Clear manual state
    void clearManualState();

private:
    ManualLayoutState manualState_;

    /// Helper: get default snap config
    static SnapPointConfig defaultSnapConfig();
};

// Backward compatibility alias (deprecated - will be removed in future version)
using ManualLayoutManager = UserLayoutController;

}  // namespace arborvia
