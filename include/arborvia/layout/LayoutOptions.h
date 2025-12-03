#pragma once

#include "../core/Types.h"
#include "LayoutTypes.h"

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
    bool isEnabled() const { return cellSize > 0.0f; }

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

/// Crossing minimization strategy
enum class CrossingMinimization {
    None,          // No optimization
    BarycenterHeuristic,  // Fast, good results
    MedianHeuristic       // Alternative heuristic
};

/// Layer assignment strategy
enum class LayerAssignment {
    LongestPath,   // Minimize total edge length
    NetworkSimplex // Better layer balance (future)
};

/// Snap point distribution mode for Auto layout
enum class SnapDistribution {
    Unified,       // All connections (incoming + outgoing) distributed evenly together
    Separated      // Incoming on left half, outgoing on right half of each edge
};

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
    LayerAssignment layerAssignment = LayerAssignment::LongestPath;
    
    // Crossing minimization iterations
    int crossingMinimizationPasses = 4;
    
    // Default node size when not specified
    Size defaultNodeSize = {100.0f, 50.0f};
    
    // Layout mode (Auto or Manual)
    LayoutMode mode = LayoutMode::Auto;
    
    // Auto mode: dynamically generate snap points based on connection count
    bool autoSnapPoints = true;
    
    // Default snap point count per edge (used in auto mode)
    int defaultSnapPointCount = 2;
    
    // Snap distribution mode (how incoming/outgoing edges share node edges)
    SnapDistribution snapDistribution = SnapDistribution::Separated;

    // Grid configuration (all coordinates snap to grid)
    GridConfig gridConfig;

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
    LayoutOptions& setLayoutMode(LayoutMode m) { mode = m; return *this; }
    LayoutOptions& setAutoSnapPoints(bool enabled) { autoSnapPoints = enabled; return *this; }
    LayoutOptions& setDefaultSnapPointCount(int count) { defaultSnapPointCount = count; return *this; }
    LayoutOptions& setSnapDistribution(SnapDistribution sd) { snapDistribution = sd; return *this; }
    LayoutOptions& setGridCellSize(float size) { gridConfig.cellSize = size; return *this; }
};

}  // namespace arborvia
