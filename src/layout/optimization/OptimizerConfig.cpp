#include "arborvia/layout/config/OptimizerConfig.h"

namespace arborvia {

OptimizerConfig OptimizerConfig::aggressive() {
    OptimizerConfig config;
    config.penaltySystem = EdgePenaltySystem::createMinimal();
    // Note: gridSize now comes from LayoutOptions.gridConfig.cellSize
    config.preserveDirections = false;
    config.maxIterations = 30000;    // Lower iteration limit
    config.nodeMargin = 1;
    return config;
}

OptimizerConfig OptimizerConfig::balanced() {
    OptimizerConfig config;
    config.penaltySystem = EdgePenaltySystem::createDefault();
    // Note: gridSize now comes from LayoutOptions.gridConfig.cellSize
    config.preserveDirections = false;
    config.maxIterations = 50000;
    config.nodeMargin = 1;
    return config;
}

OptimizerConfig OptimizerConfig::conservative() {
    OptimizerConfig config;
    config.penaltySystem = EdgePenaltySystem::createStrict();
    // Note: gridSize now comes from LayoutOptions.gridConfig.cellSize
    config.preserveDirections = false;
    config.maxIterations = 100000;   // Higher iteration limit
    config.nodeMargin = 2;           // Larger margin
    return config;
}

}  // namespace arborvia
