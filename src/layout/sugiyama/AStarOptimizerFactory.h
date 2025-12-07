#pragma once

#include "arborvia/layout/IEdgeOptimizerFactory.h"

namespace arborvia {

/// Factory for creating A* based edge optimizers
/// 
/// Creates AStarEdgeOptimizer instances configured with:
/// - Penalty system for scoring edge layouts
/// - Grid size for A* pathfinding
/// - Direction preservation settings
///
/// Usage:
/// @code
/// AStarOptimizerFactory factory;
/// auto config = OptimizerConfig::balanced();
/// auto optimizer = factory.create(config);
/// @endcode
class AStarOptimizerFactory : public IEdgeOptimizerFactory {
public:
    ~AStarOptimizerFactory() override = default;

    /// Create an A* optimizer with the given configuration
    /// @param config Optimizer configuration (penalty system, grid size, etc.)
    /// @return New AStarEdgeOptimizer instance
    std::unique_ptr<IEdgeOptimizer> create(const OptimizerConfig& config) override;

    /// Factory name for registration
    std::string name() const override { return "AStar"; }
};

}  // namespace arborvia
