#pragma once

#include "arborvia/core/Graph.h"
#include "arborvia/layout/LayoutOptions.h"
#include "arborvia/layout/LayoutResult.h"

#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace arborvia {
namespace algorithms {

/// Assigns final x,y coordinates to nodes
class CoordinateAssignment {
public:
    struct Result {
        std::unordered_map<NodeId, Point> positions;  // Top-left positions
    };
    
    /// Assign coordinates based on layer ordering
    Result assign(const Graph& graph,
                 const std::vector<std::vector<NodeId>>& layers,
                 const LayoutOptions& options);
    
    /// Assign coordinates considering node sizes
    Result assignWithSizes(const Graph& graph,
                          const std::vector<std::vector<NodeId>>& layers,
                          const std::unordered_map<NodeId, Size>& nodeSizes,
                          const LayoutOptions& options);

private:
    // Simple left-to-right assignment with centering
    void simpleAssignment(const Graph& graph,
                         const std::vector<std::vector<NodeId>>& layers,
                         const std::unordered_map<NodeId, Size>& nodeSizes,
                         const LayoutOptions& options,
                         Result& result);
    
    // Brandes-Köpf style coordinate assignment (better edge straightness)
    void brandesKopfAssignment(const Graph& graph,
                              const std::vector<std::vector<NodeId>>& layers,
                              const std::unordered_map<NodeId, Size>& nodeSizes,
                              const LayoutOptions& options,
                              Result& result);
    
    float computeLayerY(int layer,
                       const std::vector<std::vector<NodeId>>& layers,
                       const std::unordered_map<NodeId, Size>& nodeSizes,
                       const LayoutOptions& options);
};

}  // namespace algorithms
}  // namespace arborvia
