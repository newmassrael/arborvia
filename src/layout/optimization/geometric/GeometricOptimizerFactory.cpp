#include "GeometricOptimizerFactory.h"
#include "GeometricEdgeOptimizer.h"

namespace arborvia {

std::unique_ptr<IEdgeOptimizer> GeometricOptimizerFactory::create(const OptimizerConfig& config) {
    // Create optimizer with grid size
    auto optimizer = std::make_unique<GeometricEdgeOptimizer>(config.gridSize);

    // Configure penalty system
    if (config.penaltySystem) {
        optimizer->setPenaltySystem(config.penaltySystem);
    }

    // Configure direction preservation
    optimizer->setPreserveDirections(config.preserveDirections);

    return optimizer;
}

}  // namespace arborvia
