#pragma once

#include "../../core/Types.h"
#include "../config/LayoutResult.h"
#include "../config/LayoutEnums.h"

#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

namespace arborvia {

// Forward declarations
struct LayoutOptions;

/// Context for path calculators that need additional information
/// beyond the basic calculatePath parameters (e.g., for self-loop stacking)
struct PathCalculatorContext {
    const std::unordered_map<EdgeId, EdgeLayout>* edgeLayouts = nullptr;
    const LayoutOptions* options = nullptr;  ///< Optional - if null, default options with gridSize are used
    float gridSize = 20.0f;  ///< Grid size for creating default options when options is null
};

/// Configuration for edge path calculation
struct PathConfig {
    float gridSize = 20.0f;
    int extensionCells = 2;    ///< Extension point distance in grid cells (not pixels)
    bool snapToGrid = true;
    
    /// Get extension offset in pixels
    float extensionOffset() const { return static_cast<float>(extensionCells) * gridSize; }
};

/// Result of single edge path calculation
struct EdgePathResult {
    bool success = false;
    std::vector<BendPoint> bendPoints;
    std::string failureReason;

    /// Optional: New snap state if path calculation required snap point adjustment
    /// (e.g., self-loop stacking with loopIndex)
    /// When set, caller should update the layout's snap state to maintain orthogonality
    /// IMPORTANT: snapIndex and Point must always be updated together (SSOT)
    std::optional<int> newSourceSnapIndex;
    std::optional<Point> newSourcePoint;
    std::optional<int> newTargetSnapIndex;
    std::optional<Point> newTargetPoint;
};

/// Interface for single-edge path calculation
///
/// This interface sits between IEdgeOptimizer (multi-edge optimization)
/// and IPathFinder (grid-level pathfinding), providing a unified API
/// for calculating paths for individual edges.
///
/// Layer hierarchy:
///   IEdgeOptimizer (multi-edge batch optimization)
///        ↓ uses
///   IEdgePathCalculator (single edge path - THIS INTERFACE)
///        ↓ may use
///   IPathFinder (grid-level A* pathfinding)
///
/// Implementations:
/// - SelfLoopPathCalculator: Geometric L-shape for self-loops
/// - AStarPathCalculator: A* pathfinding for regular edges
/// - CompositePathCalculator: Auto-routes to appropriate calculator
class IEdgePathCalculator {
public:
    virtual ~IEdgePathCalculator() = default;

    /// Calculate path for a single edge
    /// @param layout Edge layout with source/target positions and edges set
    /// @param nodeLayouts All node layouts (for obstacle avoidance)
    /// @param config Path calculation configuration
    /// @return EdgePathResult with bendPoints on success
    virtual EdgePathResult calculatePath(
        const EdgeLayout& layout,
        const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
        const PathConfig& config) = 0;

    /// Check if this calculator can handle the given edge type
    /// @param layout Edge layout to check
    /// @return true if this calculator should handle this edge
    virtual bool canHandle(const EdgeLayout& layout) const = 0;

    /// Get calculator name for debugging/logging
    virtual const char* name() const = 0;

    /// Set context for calculators that need additional information
    /// (e.g., edgeLayouts for self-loop stacking with loopIndex)
    /// Default implementation does nothing - override if needed
    virtual void setContext(const PathCalculatorContext& /*ctx*/) {}
};

}  // namespace arborvia
