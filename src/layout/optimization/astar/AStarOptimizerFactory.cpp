#include "AStarOptimizerFactory.h"
#include "AStarEdgeOptimizer.h"

namespace arborvia {

std::unique_ptr<IEdgeOptimizer> AStarOptimizerFactory::create(const OptimizerConfig& config) {
    // Create optimizer (gridSize is now passed to optimize() method, not constructor)
    auto optimizer = std::make_unique<AStarEdgeOptimizer>(
        config.pathFinder);  // May be nullptr (uses default AStarPathFinder)

    // Configure penalty system
    if (config.penaltySystem) {
        optimizer->setPenaltySystem(config.penaltySystem);
    }

    // Configure direction preservation
    optimizer->setPreserveDirections(config.preserveDirections);

    return optimizer;
}

}  // namespace arborvia
