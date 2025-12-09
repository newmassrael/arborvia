#pragma once

#include "../config/OptimizerConfig.h"

#include <memory>
#include <string>

namespace arborvia {

// Forward declaration
class IEdgeOptimizer;

/// Factory interface for creating edge optimizers
///
/// Allows decoupled creation of optimizer instances with consistent
/// configuration. Register factories with OptimizerRegistry for
/// name-based optimizer creation.
///
/// Built-in factories:
/// - AStarOptimizerFactory: A* pathfinding (high quality, slower)
/// - GeometricOptimizerFactory: Geometric prediction (fast, approximate)
///
/// Usage:
/// @code
/// class MyOptimizerFactory : public IEdgeOptimizerFactory {
/// public:
///     std::unique_ptr<IEdgeOptimizer> create(const OptimizerConfig& config) override {
///         return std::make_unique<MyOptimizer>(config);
///     }
///     std::string name() const override { return "MyOptimizer"; }
/// };
/// @endcode
class IEdgeOptimizerFactory {
public:
    virtual ~IEdgeOptimizerFactory() = default;

    /// Create optimizer with given configuration
    /// @param config Configuration for the optimizer
    /// @return New optimizer instance (ownership transferred to caller)
    virtual std::unique_ptr<IEdgeOptimizer> create(const OptimizerConfig& config) = 0;

    /// Get the factory name for registration
    /// @return Unique identifier for this factory (e.g., "AStar", "Geometric")
    virtual std::string name() const = 0;
};

}  // namespace arborvia
