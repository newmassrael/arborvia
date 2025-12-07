#include "arborvia/layout/OptimizerConfig.h"

namespace arborvia {

OptimizerConfig OptimizerConfig::aggressive() {
    OptimizerConfig config;
    config.penaltySystem = EdgePenaltySystem::createMinimal();
    config.gridSize = 30.0f;         // Larger grid = faster search
    config.preserveDirections = false;
    config.maxIterations = 30000;    // Lower iteration limit
    config.nodeMargin = 1;
    return config;
}

OptimizerConfig OptimizerConfig::balanced() {
    OptimizerConfig config;
    config.penaltySystem = EdgePenaltySystem::createDefault();
    config.gridSize = 20.0f;
    config.preserveDirections = false;
    config.maxIterations = 50000;
    config.nodeMargin = 1;
    return config;
}

OptimizerConfig OptimizerConfig::conservative() {
    OptimizerConfig config;
    config.penaltySystem = EdgePenaltySystem::createStrict();
    config.gridSize = 10.0f;         // Smaller grid = more precise
    config.preserveDirections = false;
    config.maxIterations = 100000;   // Higher iteration limit
    config.nodeMargin = 2;           // Larger margin
    return config;
}

}  // namespace arborvia
