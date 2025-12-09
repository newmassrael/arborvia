#pragma once

#include "arborvia/layout/api/IEdgeOptimizerFactory.h"

namespace arborvia {

/// Factory for creating Geometric edge optimizers
/// 
/// Creates GeometricEdgeOptimizer instances configured with:
/// - Penalty system for scoring edge layouts
/// - Direction preservation settings
///
/// Geometric optimizer uses fast path prediction instead of A* pathfinding,
/// making it ideal for real-time drag feedback.
///
/// Usage:
/// @code
/// GeometricOptimizerFactory factory;
/// auto config = OptimizerConfig::aggressive();
/// auto optimizer = factory.create(config);
/// @endcode
class GeometricOptimizerFactory : public IEdgeOptimizerFactory {
public:
    ~GeometricOptimizerFactory() override = default;

    /// Create a Geometric optimizer with the given configuration
    /// @param config Optimizer configuration (penalty system, etc.)
    /// @return New GeometricEdgeOptimizer instance
    std::unique_ptr<IEdgeOptimizer> create(const OptimizerConfig& config) override;

    /// Factory name for registration
    std::string name() const override { return "Geometric"; }
};

}  // namespace arborvia
