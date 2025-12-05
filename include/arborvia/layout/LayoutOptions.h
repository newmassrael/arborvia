#pragma once

#include "../core/Types.h"
#include "LayoutTypes.h"
#include "ConstraintConfig.h"

namespace arborvia {

/// Direction of hierarchical layout
enum class Direction {
    TopToBottom,   // Root at top, leaves at bottom
    BottomToTop,   // Root at bottom, leaves at top
    LeftToRight,   // Root at left, leaves at right
    RightToLeft    // Root at right, leaves at left
};

/// Self-loop routing direction
enum class SelfLoopDirection {
    Right,   // Loop exits to the right
    Left,    // Loop exits to the left
    Top,     // Loop exits upward
    Bottom,  // Loop exits downward
    Auto     // Automatically choose best direction
};

/// Configuration for self-loop routing
struct SelfLoopConfig {
    SelfLoopDirection preferredDirection = SelfLoopDirection::Auto;
    float loopOffset = 20.0f;       // Distance from node edge
    float stackSpacing = 15.0f;     // Spacing between multiple self-loops
};

/// Configuration for channel-based edge routing
struct ChannelRoutingOptions {
    float channelSpacing = 15.0f;     // Spacing between parallel channels
    float channelOffset = 25.0f;      // Minimum offset from layer boundary
    bool centerSingleEdge = true;     // Center edge when only one in channel region
    int maxChannelsPerRegion = 10;    // Maximum channels per region
    SelfLoopConfig selfLoop;          // Self-loop configuration
};

/// Configuration for grid-based coordinate snapping
struct GridConfig {
    float cellSize = 0.0f;  // Grid cell size (0 = disabled, >0 = snap to multiples)

    /// Check if grid snapping is enabled
    [[nodiscard]] bool isEnabled() const noexcept { return cellSize > 0.0f; }

    /// Returns cellSize if enabled, 0.0f otherwise
    /// Caller can skip snapping when return value is 0
    [[nodiscard]] float effectiveCellSize() const noexcept {
        return isEnabled() ? cellSize : 0.0f;
    }

    /// Snap value to grid (returns original if grid disabled)
    [[nodiscard]] float snap(float value) const noexcept {
        return isEnabled()
            ? std::round(value / cellSize) * cellSize
            : value;
    }

    /// Snap point to grid (returns original if grid disabled)
    [[nodiscard]] Point snap(const Point& p) const noexcept {
        return {snap(p.x), snap(p.y)};
    }

    /// Convert pixel coordinate to grid unit (round to nearest)
    int toGrid(float pixel) const {
        return static_cast<int>(std::round(pixel / cellSize));
    }

    /// Convert pixel coordinate to grid unit (floor)
    int toGridFloor(float pixel) const {
        return static_cast<int>(std::floor(pixel / cellSize));
    }

    /// Convert pixel coordinate to grid unit (ceil)
    int toGridCeil(float pixel) const {
        return static_cast<int>(std::ceil(pixel / cellSize));
    }

    /// Convert grid unit to pixel coordinate
    float toPixel(int grid) const {
        return grid * cellSize;
    }

    /// Convert Point to GridPoint
    GridPoint toGridPoint(const Point& p) const {
        return GridPoint::fromPixel(p, cellSize);
    }

    /// Convert GridPoint to Point
    Point toPixelPoint(const GridPoint& g) const {
        return g.toPixel(cellSize);
    }
};

/// Node alignment within a layer
enum class NodeAlignment {
    TopLeft,
    Center,
    BottomRight
};

/// Scoring weights for edge routing optimization
/// Higher scores indicate worse routing choices (lower is better)
/// Used by AStarEdgeOptimizer to evaluate edge combinations
struct ScoringWeights {
    int tooCloseSnap = 100000;      ///< Snap points too close together
    int selfOverlap = 50000;        ///< Path overlaps itself (MIN_SEGMENT violation)
    int nodeCollision = 10000;      ///< Path passes through a node
    int pathIntersection = 1000;    ///< Path intersects another edge
    int detourBonus = -5000;        ///< Bonus for detour paths (negative = good)
    float minSnapDistance = 60.0f;  ///< Minimum distance between snap points (pixels)
    float minSegmentLength = 20.0f; ///< Minimum segment length before turn
};

/// Algorithm selection for real-time drag feedback
/// Used when nodes are being dragged for immediate visual feedback
enum class DragAlgorithm {
    None,       ///< No optimization during drag (use existing routing)
    Geometric   ///< Fast geometric path prediction (no A* pathfinding)
};

/// Algorithm selection for post-drag optimization
/// Applied after drag operation completes (typically with debounce)
enum class PostDragAlgorithm {
    None,   ///< No post-drag optimization
    AStar   ///< A* pathfinding for optimal obstacle-avoiding paths
};

/// Options for edge routing optimization
/// This is SEPARATE from the constraint system:
/// - Constraints: Boolean validation ("Can we do this?")
/// - Optimization: Numeric scoring ("Which option is best?")
struct OptimizationOptions {
    /// Algorithm to use during drag operations (real-time feedback)
    /// Geometric is fast but produces approximate paths
    DragAlgorithm dragAlgorithm = DragAlgorithm::Geometric;

    /// Algorithm to use after drag completes (final optimization)
    /// AStar produces optimal paths but is slower
    PostDragAlgorithm postDragAlgorithm = PostDragAlgorithm::AStar;

    /// Enable snap point sorting by other node position
    /// Minimizes edge crossings by ordering snap points based on
    /// the positions of nodes they connect to
    bool sortSnapPoints = true;

    /// Scoring weights for optimization algorithms
    ScoringWeights scoringWeights;
};

/// Crossing minimization strategy
enum class CrossingMinimization {
    None,               // No optimization
    BarycenterHeuristic // Fast, good results
};

/// Layer assignment strategy
/// Options for controlling layout behavior
struct LayoutOptions {
    // General layout direction
    Direction direction = Direction::TopToBottom;

    // Spacing
    float nodeSpacingHorizontal = 100.0f;  // Space between nodes in same layer (5 grid units @ 20px)
    float nodeSpacingVertical = 100.0f;    // Space between layers (5 grid units @ 20px)

    // Compound node settings
    float compoundPadding = 20.0f;        // Padding inside compound nodes
    float parallelSpacing = 30.0f;        // Space between parallel regions

    // Edge routing (channel-based orthogonal routing)
    float edgeBendRadius = 5.0f;          // Radius for rounded bends
    ChannelRoutingOptions channelRouting; // Channel routing options

    // Algorithm settings
    NodeAlignment nodeAlignment = NodeAlignment::Center;
    CrossingMinimization crossingMinimization = CrossingMinimization::BarycenterHeuristic;

    // Crossing minimization iterations
    int crossingMinimizationPasses = 4;

    // Default node size when not specified
    Size defaultNodeSize = {100.0f, 50.0f};

    // Auto mode: dynamically generate snap points based on connection count
    bool autoSnapPoints = true;

    // Default snap point count per edge (used in auto mode)
    int defaultSnapPointCount = 2;

    // Grid configuration (all coordinates snap to grid)
    GridConfig gridConfig;

    // Drag constraint configuration
    // If empty, default constraints (MinDistance + EdgeValidity) are used
    ConstraintConfig constraintConfig;

    // Edge routing optimization options
    // Separate from constraints - optimization runs after constraint validation passes
    OptimizationOptions optimizationOptions;

    // Builder pattern for convenient configuration
    LayoutOptions& setDirection(Direction d) { direction = d; return *this; }
    LayoutOptions& setNodeSpacing(float h, float v) {
        nodeSpacingHorizontal = h;
        nodeSpacingVertical = v;
        return *this;
    }
    LayoutOptions& setCompoundPadding(float p) { compoundPadding = p; return *this; }
    LayoutOptions& setChannelSpacing(float spacing) {
        channelRouting.channelSpacing = spacing;
        return *this;
    }
    LayoutOptions& setChannelOffset(float offset) {
        channelRouting.channelOffset = offset;
        return *this;
    }
    LayoutOptions& setSelfLoopDirection(SelfLoopDirection dir) {
        channelRouting.selfLoop.preferredDirection = dir;
        return *this;
    }
    LayoutOptions& setCrossingMinimization(CrossingMinimization c) {
        crossingMinimization = c;
        return *this;
    }
    LayoutOptions& setAutoSnapPoints(bool enabled) { autoSnapPoints = enabled; return *this; }
    LayoutOptions& setDefaultSnapPointCount(int count) { defaultSnapPointCount = count; return *this; }
    LayoutOptions& setGridCellSize(float size) { gridConfig.cellSize = size; return *this; }
    LayoutOptions& setConstraintConfig(const ConstraintConfig& config) { constraintConfig = config; return *this; }
    LayoutOptions& setMinNodeDistance(float gridUnits) {
        constraintConfig = ConstraintConfig::createEmpty().addMinDistance(gridUnits);
        return *this;
    }
    /// @deprecated Use setDragAlgorithm() and setPostDragAlgorithm() instead
    LayoutOptions& setUseGreedyOptimizer(bool enabled) {
        if (enabled) {
            optimizationOptions.dragAlgorithm = DragAlgorithm::Geometric;
            optimizationOptions.postDragAlgorithm = PostDragAlgorithm::AStar;
        } else {
            optimizationOptions.dragAlgorithm = DragAlgorithm::None;
            optimizationOptions.postDragAlgorithm = PostDragAlgorithm::None;
        }
        return *this;
    }
    LayoutOptions& setDragAlgorithm(DragAlgorithm algo) {
        optimizationOptions.dragAlgorithm = algo;
        return *this;
    }
    LayoutOptions& setPostDragAlgorithm(PostDragAlgorithm algo) {
        optimizationOptions.postDragAlgorithm = algo;
        return *this;
    }
    LayoutOptions& setSortSnapPoints(bool enabled) {
        optimizationOptions.sortSnapPoints = enabled;
        return *this;
    }
    LayoutOptions& setScoringWeights(const ScoringWeights& weights) {
        optimizationOptions.scoringWeights = weights;
        return *this;
    }
    LayoutOptions& setOptimizationOptions(const OptimizationOptions& opts) {
        optimizationOptions = opts;
        return *this;
    }
};

}  // namespace arborvia
