#include "GeometricOptimizerFactory.h"
#include "GeometricEdgeOptimizer.h"

namespace arborvia {

std::unique_ptr<IEdgeOptimizer> GeometricOptimizerFactory::create(const OptimizerConfig& config) {
    // Create optimizer (gridSize is now passed to optimize() method, not constructor)
    auto optimizer = std::make_unique<GeometricEdgeOptimizer>();

    // Configure penalty system
    if (config.penaltySystem) {
        optimizer->setPenaltySystem(config.penaltySystem);
    }

    // Configure direction preservation
    optimizer->setPreserveDirections(config.preserveDirections);

    return optimizer;
}

}  // namespace arborvia
