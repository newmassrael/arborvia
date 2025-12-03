#pragma once

#include "arborvia/layout/ICoordinateAssignment.h"
#include "arborvia/layout/LayoutResult.h"

#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace arborvia {
namespace algorithms {

/// Simple coordinate assignment algorithm
/// 
/// This is the default implementation of ICoordinateAssignment using
/// simple left-to-right assignment with centering, plus optional
/// Brandes-Köpf style assignment for better edge straightness.
class SimpleCoordinateAssignment : public ICoordinateAssignment {
public:
    /// Type alias for backward compatibility
    using Result = CoordinateAssignmentResult;

    SimpleCoordinateAssignment() = default;

    /// Get algorithm name
    const char* algorithmName() const override { return "Simple"; }

    /// Assign coordinates based on layer ordering
    CoordinateAssignmentResult assign(
        const Graph& graph,
        const std::vector<std::vector<NodeId>>& layers,
        const LayoutOptions& options) const override;

    /// Assign coordinates considering node sizes
    CoordinateAssignmentResult assignWithSizes(
        const Graph& graph,
        const std::vector<std::vector<NodeId>>& layers,
        const std::unordered_map<NodeId, Size>& nodeSizes,
        const LayoutOptions& options) const override;

private:
    // Simple left-to-right assignment with centering
    void simpleAssignment(const Graph& graph,
                         const std::vector<std::vector<NodeId>>& layers,
                         const std::unordered_map<NodeId, Size>& nodeSizes,
                         const LayoutOptions& options,
                         CoordinateAssignmentResult& result) const;

    // Brandes-Köpf style coordinate assignment (better edge straightness)
    void brandesKopfAssignment(const Graph& graph,
                              const std::vector<std::vector<NodeId>>& layers,
                              const std::unordered_map<NodeId, Size>& nodeSizes,
                              const LayoutOptions& options,
                              CoordinateAssignmentResult& result) const;

    float computeLayerY(int layer,
                       const std::vector<std::vector<NodeId>>& layers,
                       const std::unordered_map<NodeId, Size>& nodeSizes,
                       const LayoutOptions& options) const;
};

/// Backward compatibility alias
using CoordinateAssignment = SimpleCoordinateAssignment;

}  // namespace algorithms
}  // namespace arborvia
