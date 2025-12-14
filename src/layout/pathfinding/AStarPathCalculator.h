#pragma once

#include "arborvia/layout/api/IEdgePathCalculator.h"
#include "arborvia/layout/api/IPathFinder.h"

#include <memory>

namespace arborvia {

/// Path calculator for regular edges using A* pathfinding
///
/// This calculator wraps IPathFinder to provide A*-based path calculation
/// for edges where from != to. It handles:
/// - ObstacleMap construction from node layouts
/// - Grid coordinate conversion
/// - Path to bend points conversion
///
/// For self-loops (from == to), use SelfLoopPathCalculator instead.
class AStarPathCalculator : public IEdgePathCalculator {
public:
    /// Create calculator with pathfinder
    /// @param pathFinder A* pathfinder implementation
    explicit AStarPathCalculator(std::shared_ptr<IPathFinder> pathFinder);
    ~AStarPathCalculator() override = default;

    /// Calculate A* path for regular edge
    EdgePathResult calculatePath(
        const EdgeLayout& layout,
        const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
        const PathConfig& config) override;

    /// Returns true if layout.from != layout.to
    bool canHandle(const EdgeLayout& layout) const override;

    const char* name() const override { return "AStarPathCalculator"; }

private:
    std::shared_ptr<IPathFinder> pathFinder_;
};

}  // namespace arborvia
