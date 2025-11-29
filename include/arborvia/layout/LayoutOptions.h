#pragma once

#include "../core/Types.h"

namespace arborvia {

/// Direction of hierarchical layout
enum class Direction {
    TopToBottom,   // Root at top, leaves at bottom
    BottomToTop,   // Root at bottom, leaves at top
    LeftToRight,   // Root at left, leaves at right
    RightToLeft    // Root at right, leaves at left
};

/// Edge routing style
enum class EdgeRouting {
    Orthogonal,    // Right-angle bends only
    Polyline,      // Straight line segments
    Splines        // Smooth curves (future)
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

/// Options for controlling layout behavior
struct LayoutOptions {
    // General layout direction
    Direction direction = Direction::TopToBottom;
    
    // Spacing
    float nodeSpacingHorizontal = 50.0f;  // Space between nodes in same layer
    float nodeSpacingVertical = 75.0f;    // Space between layers
    
    // Compound node settings
    float compoundPadding = 20.0f;        // Padding inside compound nodes
    float parallelSpacing = 30.0f;        // Space between parallel regions
    
    // Edge routing
    EdgeRouting edgeRouting = EdgeRouting::Orthogonal;
    float edgeBendRadius = 5.0f;          // Radius for rounded bends
    
    // Algorithm settings
    NodeAlignment nodeAlignment = NodeAlignment::Center;
    CrossingMinimization crossingMinimization = CrossingMinimization::BarycenterHeuristic;
    LayerAssignment layerAssignment = LayerAssignment::LongestPath;
    
    // Crossing minimization iterations
    int crossingMinimizationPasses = 4;
    
    // Default node size when not specified
    Size defaultNodeSize = {100.0f, 50.0f};
    
    // Builder pattern for convenient configuration
    LayoutOptions& setDirection(Direction d) { direction = d; return *this; }
    LayoutOptions& setNodeSpacing(float h, float v) { 
        nodeSpacingHorizontal = h; 
        nodeSpacingVertical = v; 
        return *this; 
    }
    LayoutOptions& setCompoundPadding(float p) { compoundPadding = p; return *this; }
    LayoutOptions& setEdgeRouting(EdgeRouting r) { edgeRouting = r; return *this; }
    LayoutOptions& setCrossingMinimization(CrossingMinimization c) { 
        crossingMinimization = c; 
        return *this; 
    }
};

}  // namespace arborvia
