#pragma once

#include "arborvia/layout/api/IEdgePathCalculator.h"
#include "arborvia/layout/api/IPathFinder.h"

#include <memory>
#include <vector>

namespace arborvia {

/// Composite path calculator that routes to appropriate calculator
///
/// This calculator automatically selects the right path calculator
/// based on edge type:
/// - Self-loops (from == to) → SelfLoopPathCalculator
/// - Regular edges (from != to) → AStarPathCalculator
///
/// Usage:
/// @code
/// auto calculator = CompositePathCalculator::createDefault(pathFinder);
/// auto result = calculator->calculatePath(layout, nodeLayouts, config);
/// @endcode
class CompositePathCalculator : public IEdgePathCalculator {
public:
    CompositePathCalculator() = default;
    ~CompositePathCalculator() override = default;

    /// Add a calculator to the chain
    /// Calculators are checked in order added
    void addCalculator(std::shared_ptr<IEdgePathCalculator> calculator);

    /// Calculate path using first matching calculator
    EdgePathResult calculatePath(
        const EdgeLayout& layout,
        const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
        const PathConfig& config) override;

    /// Always returns true (composite handles all edges)
    bool canHandle(const EdgeLayout& layout) const override;

    const char* name() const override { return "CompositePathCalculator"; }

    /// Create default calculator with SelfLoop and AStar
    /// @param pathFinder A* pathfinder for regular edges
    /// @return Configured composite calculator
    static std::shared_ptr<CompositePathCalculator> createDefault(
        std::shared_ptr<IPathFinder> pathFinder);

private:
    std::vector<std::shared_ptr<IEdgePathCalculator>> calculators_;
};

}  // namespace arborvia
