#include "CompositePathCalculator.h"
#include "SelfLoopPathCalculator.h"
#include "AStarPathCalculator.h"

namespace arborvia {

void CompositePathCalculator::addCalculator(std::shared_ptr<IEdgePathCalculator> calculator) {
    if (calculator) {
        calculators_.push_back(std::move(calculator));
    }
}

bool CompositePathCalculator::canHandle(const EdgeLayout& /*layout*/) const {
    // Composite can handle any edge if it has calculators
    return !calculators_.empty();
}

EdgePathResult CompositePathCalculator::calculatePath(
    const EdgeLayout& layout,
    const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
    const PathConfig& config) {

    // Try each calculator in order
    for (const auto& calculator : calculators_) {
        if (calculator->canHandle(layout)) {
            return calculator->calculatePath(layout, nodeLayouts, config);
        }
    }

    // No calculator could handle this edge
    EdgePathResult result;
    result.failureReason = "No calculator can handle this edge type";
    return result;
}

std::shared_ptr<CompositePathCalculator> CompositePathCalculator::createDefault(
    std::shared_ptr<IPathFinder> pathFinder) {

    auto composite = std::make_shared<CompositePathCalculator>();

    // Add self-loop calculator (handles from == to)
    composite->addCalculator(std::make_shared<SelfLoopPathCalculator>());

    // Add A* calculator for regular edges (handles from != to)
    if (pathFinder) {
        composite->addCalculator(std::make_shared<AStarPathCalculator>(std::move(pathFinder)));
    }

    return composite;
}

}  // namespace arborvia
