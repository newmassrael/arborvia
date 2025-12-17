#pragma once

#include "../../core/Types.h"
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

/// Configuration for self-loop routing (grid-based)
struct SelfLoopConfig {
    SelfLoopDirection preferredDirection = SelfLoopDirection::Auto;
    int loopOffsetGrids = 1;       // Distance from node edge (grid units)
    int stackSpacingGrids = 1;     // Spacing between multiple self-loops (grid units)
};

/// Configuration for channel-based edge routing (grid-based)
struct ChannelRoutingOptions {
    int channelSpacingGrids = 1;      // Spacing between parallel channels (grid units)
    int channelOffsetGrids = 1;       // Minimum offset from layer boundary (grid units)
    bool centerSingleEdge = true;     // Center edge when only one in channel region
    int maxChannelsPerRegion = 10;    // Maximum channels per region
    SelfLoopConfig selfLoop;          // Self-loop configuration
    
    // Helper methods to get pixel values (for backward compatibility)
    float channelSpacing(float gridSize) const { return channelSpacingGrids * gridSize; }
    float channelOffset(float gridSize) const { return channelOffsetGrids * gridSize; }
};

/// Configuration for grid-based coordinates
/// Internal calculations use grid coordinates (integers), pixels are for output only
struct GridConfig {
    float cellSize = 20.0f;  // Grid cell size in pixels

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

/// Scoring weights for edge routing optimization (grid-based)
/// Higher scores indicate worse routing choices (lower is better)
/// Used by AStarEdgeOptimizer to evaluate edge combinations
struct ScoringWeights {
    int constraintViolation = 200000; ///< Path violates forbidden zone constraints (highest penalty)
    int tooCloseSnap = 100000;      ///< Snap points too close together
    int selfOverlap = 50000;        ///< Path overlaps itself (MIN_SEGMENT violation)
    int nodeCollision = 10000;      ///< Path passes through a node
    int pathIntersection = 1000;    ///< Path intersects another edge
    int detourBonus = -5000;        ///< Bonus for detour paths (negative = good)
    int minSnapDistanceGrids = 3;   ///< Minimum distance between snap points (grid units)
    int minSegmentLengthGrids = 1;  ///< Minimum segment length before turn (grid units)
};

/// Algorithm selection for real-time drag feedback
/// Used when nodes are being dragged for immediate visual feedback
enum class DragAlgorithm {
    None,           ///< No optimization during drag (use existing routing)
    Geometric,      ///< Fast geometric path prediction (no A* pathfinding)
    AStar,          ///< A* pathfinding for optimal obstacle-avoiding paths
    HideUntilDrop   ///< Hide edges during drag, calculate with A* on drop
};

/// Traits for DragAlgorithm - Single Source of Truth for algorithm characteristics
/// Add new algorithm behaviors here when extending DragAlgorithm enum
namespace DragAlgorithmTraits {
    /// Returns true if the algorithm requires A* path validation during drag
    /// Only AStar mode validates paths during drag; others skip for performance
    inline bool requiresPathValidationDuringDrag(DragAlgorithm algo) {
        return algo == DragAlgorithm::AStar;
    }

    /// Returns true if edges should be hidden during drag
    inline bool hidesEdgesDuringDrag(DragAlgorithm algo) {
        return algo == DragAlgorithm::HideUntilDrop;
    }

    /// Returns true if the algorithm recalculates paths during drag
    inline bool recalculatesPathsDuringDrag(DragAlgorithm algo) {
        return algo == DragAlgorithm::Geometric || algo == DragAlgorithm::AStar;
    }
}

/// Algorithm selection for post-drag optimization
/// Applied after drag operation completes (typically with debounce)
enum class PostDragAlgorithm {
    None,       ///< No post-drag optimization
    AStar,      ///< A* pathfinding for optimal obstacle-avoiding paths
    Geometric   ///< Fast geometric path prediction (greedy algorithm)
};

/// Options for edge routing optimization
/// This is SEPARATE from the constraint system:
/// - Constraints: Boolean validation ("Can we do this?")
/// - Optimization: Numeric scoring ("Which option is best?")
struct OptimizationOptions {
    /// Algorithm to use during drag operations (real-time feedback)
    /// HideUntilDrop hides edges during drag for best UX, calculates on drop
    DragAlgorithm dragAlgorithm = DragAlgorithm::HideUntilDrop;

    /// Algorithm to use after drag completes (final optimization)
    /// AStar produces optimal obstacle-avoiding paths
    /// Geometric is faster but may not avoid all obstacles
    PostDragAlgorithm postDragAlgorithm = PostDragAlgorithm::AStar;

    /// Enable snap point sorting by other node position
    /// Minimizes edge crossings by ordering snap points based on
    /// the positions of nodes they connect to
    bool sortSnapPoints = true;

    /// Scoring weights for optimization algorithms
    ScoringWeights scoringWeights;

    /// Enable Rip-up and Reroute for overlap elimination
    /// When true, edges that overlap after initial routing are re-routed
    bool enableRipUpAndReroute = true;

    /// Maximum iterations for Rip-up and Reroute
    int maxRipUpIterations = 3;
};

/// Crossing minimization strategy
enum class CrossingMinimization {
    None,               // No optimization
    BarycenterHeuristic // Fast, good results
};

/// Options for controlling layout behavior (grid-based)
/// All spacing/sizing values are in grid units (integers)
/// Actual pixel values = gridUnits * gridConfig.cellSize
struct LayoutOptions {
    // General layout direction
    Direction direction = Direction::TopToBottom;

    // Spacing (grid units)
    int nodeSpacingHorizontalGrids = 5;  // Space between nodes in same layer
    int nodeSpacingVerticalGrids = 5;    // Space between layers

    // Compound node settings (grid units)
    int compoundPaddingGrids = 1;        // Padding inside compound nodes
    int parallelSpacingGrids = 2;        // Space between parallel regions

    // Edge routing (channel-based orthogonal routing)
    float edgeBendRadius = 5.0f;          // Radius for rounded bends (pixels, for visual smoothness)
    ChannelRoutingOptions channelRouting; // Channel routing options

    // Algorithm settings
    NodeAlignment nodeAlignment = NodeAlignment::Center;
    CrossingMinimization crossingMinimization = CrossingMinimization::BarycenterHeuristic;

    // Crossing minimization iterations
    int crossingMinimizationPasses = 4;

    // Default node size (grid units)
    int defaultNodeWidthGrids = 5;   // 5 * 20 = 100px
    int defaultNodeHeightGrids = 3;  // 3 * 20 = 60px

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

    // Helper methods to get pixel values
    float nodeSpacingHorizontal() const { return nodeSpacingHorizontalGrids * gridConfig.cellSize; }
    float nodeSpacingVertical() const { return nodeSpacingVerticalGrids * gridConfig.cellSize; }
    float compoundPadding() const { return compoundPaddingGrids * gridConfig.cellSize; }
    float parallelSpacing() const { return parallelSpacingGrids * gridConfig.cellSize; }
    Size defaultNodeSize() const { 
        return {defaultNodeWidthGrids * gridConfig.cellSize, 
                defaultNodeHeightGrids * gridConfig.cellSize}; 
    }
    float channelSpacing() const { return channelRouting.channelSpacingGrids * gridConfig.cellSize; }
    float channelOffset() const { return channelRouting.channelOffsetGrids * gridConfig.cellSize; }
    float minSnapDistance() const { return optimizationOptions.scoringWeights.minSnapDistanceGrids * gridConfig.cellSize; }
    float minSegmentLength() const { return optimizationOptions.scoringWeights.minSegmentLengthGrids * gridConfig.cellSize; }

    // Builder pattern for convenient configuration (grid units)
    LayoutOptions& setDirection(Direction d) { direction = d; return *this; }
    LayoutOptions& setNodeSpacingGrids(int h, int v) {
        nodeSpacingHorizontalGrids = h;
        nodeSpacingVerticalGrids = v;
        return *this;
    }
    LayoutOptions& setCompoundPaddingGrids(int p) { compoundPaddingGrids = p; return *this; }
    LayoutOptions& setChannelSpacingGrids(int spacing) {
        channelRouting.channelSpacingGrids = spacing;
        return *this;
    }
    LayoutOptions& setChannelOffsetGrids(int offset) {
        channelRouting.channelOffsetGrids = offset;
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
        constraintConfig = ConstraintConfig::createEmpty().addDirectionAwareMargin(gridUnits);
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
    LayoutOptions& setEnableRipUpAndReroute(bool enabled) {
        optimizationOptions.enableRipUpAndReroute = enabled;
        return *this;
    }
    LayoutOptions& setMaxRipUpIterations(int iterations) {
        optimizationOptions.maxRipUpIterations = iterations;
        return *this;
    }
};

}  // namespace arborvia
