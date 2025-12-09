#pragma once

#include "arborvia/layout/api/IEdgeOptimizerFactory.h"

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace arborvia {

/// Central registry for optimizer factories
///
/// Provides singleton access to registered optimizer factories,
/// enabling name-based optimizer creation with consistent configuration.
///
/// Built-in factories are registered automatically:
/// - "AStar": AStarOptimizerFactory
/// - "Geometric": GeometricOptimizerFactory
///
/// Usage:
/// @code
/// // Get registry instance
/// auto& registry = OptimizerRegistry::instance();
///
/// // Create optimizer by name
/// auto config = OptimizerConfig::balanced();
/// auto optimizer = registry.create("AStar", config);
///
/// // Register custom factory
/// registry.registerFactory(std::make_unique<MyOptimizerFactory>());
/// @endcode
class OptimizerRegistry {
public:
    /// Get singleton instance
    /// Built-in factories are registered on first access
    static OptimizerRegistry& instance();

    // Non-copyable, non-movable (singleton)
    OptimizerRegistry(const OptimizerRegistry&) = delete;
    OptimizerRegistry& operator=(const OptimizerRegistry&) = delete;

    /// Register a factory
    /// @param factory Factory to register (ownership transferred)
    /// @throws std::runtime_error if factory with same name already exists
    void registerFactory(std::unique_ptr<IEdgeOptimizerFactory> factory);

    /// Unregister a factory by name
    /// @param name Name of factory to unregister
    /// @return true if factory was found and removed
    bool unregisterFactory(const std::string& name);

    /// Create optimizer by factory name
    /// @param name Name of factory to use
    /// @param config Configuration for the optimizer
    /// @return New optimizer instance, or nullptr if factory not found
    std::unique_ptr<IEdgeOptimizer> create(
        const std::string& name,
        const OptimizerConfig& config);

    /// Check if a factory is registered
    /// @param name Name of factory to check
    /// @return true if factory is registered
    bool hasFactory(const std::string& name) const;

    /// Get list of available optimizer names
    /// @return Vector of registered factory names
    std::vector<std::string> availableOptimizers() const;

    /// Get factory by name
    /// @param name Name of factory to get
    /// @return Pointer to factory, or nullptr if not found
    const IEdgeOptimizerFactory* getFactory(const std::string& name) const;

private:
    OptimizerRegistry();
    ~OptimizerRegistry() = default;

    /// Register built-in factories
    void registerBuiltinFactories();

    std::unordered_map<std::string, std::unique_ptr<IEdgeOptimizerFactory>> factories_;
    bool builtinsRegistered_ = false;
};

}  // namespace arborvia
