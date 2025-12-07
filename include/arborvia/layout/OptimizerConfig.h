#pragma once

#include "EdgePenaltySystem.h"
#include "IPathFinder.h"

#include <memory>
#include <string>

namespace arborvia {

/// Configuration for edge optimizer creation
///
/// Contains all settings needed to create and configure an edge optimizer.
/// Includes penalty system, grid settings, and behavior flags.
///
/// Presets:
/// - aggressive(): Lower penalties, faster convergence
/// - balanced(): Default settings
/// - conservative(): Stricter constraints, higher quality
///
/// Usage:
/// @code
/// auto config = OptimizerConfig::balanced();
/// config.gridSize = 15.0f;  // Override specific settings
///
/// auto optimizer = factory.create(config);
/// @endcode
struct OptimizerConfig {
    /// Penalty system (shared across optimizer lifetime)
    /// If nullptr, optimizer will use default penalty system
    std::shared_ptr<EdgePenaltySystem> penaltySystem;

    /// Path finder for A* optimizer (optional)
    /// If nullptr, optimizer will create its own AStarPathFinder
    std::shared_ptr<IPathFinder> pathFinder;

    /// Grid size in pixels for pathfinding
    /// Larger values = faster but less precise
    /// Typical range: 10-30 pixels
    float gridSize = 20.0f;

    /// Preserve existing source/target edge directions during optimization
    /// When true, optimizer only optimizes path routing, not edge selection
    /// Used during drag operations to maintain user intent
    bool preserveDirections = false;

    /// Maximum iterations for A* pathfinding
    /// Higher values allow finding more complex paths
    int maxIterations = 50000;

    /// Margin around nodes in grid units
    /// Paths must stay this far from node boundaries
    int nodeMargin = 1;

    // === Named Presets ===

    /// Create aggressive configuration
    /// Lower penalties, larger grid, faster convergence
    /// Best for: Real-time drag feedback
    static OptimizerConfig aggressive();

    /// Create balanced configuration (default)
    /// Standard penalties, medium grid
    /// Best for: General use
    static OptimizerConfig balanced();

    /// Create conservative configuration
    /// Stricter constraints, smaller grid, higher quality
    /// Best for: Final layout, export
    static OptimizerConfig conservative();

    // === Convenience Methods ===

    /// Set penalty system
    OptimizerConfig& withPenaltySystem(std::shared_ptr<EdgePenaltySystem> system) {
        penaltySystem = std::move(system);
        return *this;
    }

    /// Set grid size
    OptimizerConfig& withGridSize(float size) {
        gridSize = size;
        return *this;
    }

    /// Set preserve directions flag
    OptimizerConfig& withPreserveDirections(bool preserve) {
        preserveDirections = preserve;
        return *this;
    }

    /// Set max iterations
    OptimizerConfig& withMaxIterations(int iterations) {
        maxIterations = iterations;
        return *this;
    }

    /// Set path finder
    OptimizerConfig& withPathFinder(std::shared_ptr<IPathFinder> finder) {
        pathFinder = std::move(finder);
        return *this;
    }
};

}  // namespace arborvia
