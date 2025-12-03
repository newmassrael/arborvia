#pragma once

#include "../core/Types.h"
#include "../core/Graph.h"
#include "LayoutOptions.h"

#include <vector>
#include <unordered_map>

namespace arborvia {
namespace algorithms {

/// Result of coordinate assignment operation
struct CoordinateAssignmentResult {
    std::unordered_map<NodeId, Point> positions;  ///< Top-left positions for each node
};

/// Abstract interface for coordinate assignment algorithms
/// 
/// Implementations can provide different strategies:
/// - Simple (default, left-to-right with centering)
/// - Brandes-Köpf (better edge straightness)
/// - Priority Layout (minimize edge length)
/// 
/// Use this interface to swap coordinate assignment algorithms without
/// modifying SugiyamaLayout or other dependent code.
class ICoordinateAssignment {
public:
    virtual ~ICoordinateAssignment() = default;

    /// Assign coordinates based on layer ordering
    /// @param graph The input graph
    /// @param layers Ordered layers from crossing minimization
    /// @param options Layout options (spacing, etc.)
    /// @return Result containing node positions
    virtual CoordinateAssignmentResult assign(
        const Graph& graph,
        const std::vector<std::vector<NodeId>>& layers,
        const LayoutOptions& options) const = 0;

    /// Assign coordinates considering node sizes
    /// @param graph The input graph
    /// @param layers Ordered layers from crossing minimization
    /// @param nodeSizes Size of each node
    /// @param options Layout options
    /// @return Result containing node positions
    virtual CoordinateAssignmentResult assignWithSizes(
        const Graph& graph,
        const std::vector<std::vector<NodeId>>& layers,
        const std::unordered_map<NodeId, Size>& nodeSizes,
        const LayoutOptions& options) const = 0;

    /// Get algorithm name for debugging/logging
    virtual const char* algorithmName() const = 0;
};

}  // namespace algorithms
}  // namespace arborvia
