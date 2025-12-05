#include "arborvia/layout/MinDistanceConstraint.h"
#include "arborvia/layout/ValidRegionCalculator.h"

namespace arborvia {

MinDistanceConstraint::MinDistanceConstraint(float minGridDistance)
    : minGridDistance_(minGridDistance) {
    // Note: minGridDistance_ is kept for API compatibility but ValidRegionCalculator
    // uses constants::MIN_NODE_GRID_DISTANCE internally for direction-aware margins
}

ConstraintResult MinDistanceConstraint::check(const ConstraintContext& ctx) const {
    auto nodeIt = ctx.nodeLayouts.find(ctx.nodeId);
    if (nodeIt == ctx.nodeLayouts.end()) {
        return ConstraintResult::ok();
    }

    // Delegate to ValidRegionCalculator for direction-aware margin calculation
    auto zones = ValidRegionCalculator::calculate(
        ctx.nodeId, ctx.nodeLayouts, ctx.edgeLayouts, ctx.gridSize);

    bool valid = ValidRegionCalculator::isValid(
        ctx.newPosition, nodeIt->second.size, zones);

    return valid ? ConstraintResult::ok()
                 : ConstraintResult::fail("Too close to another node");
}

std::vector<Rect> MinDistanceConstraint::getBlockedRegions(const ConstraintContext& ctx) const {
    // Delegate to ValidRegionCalculator for consistent visualization
    auto zones = ValidRegionCalculator::calculate(
        ctx.nodeId, ctx.nodeLayouts, ctx.edgeLayouts, ctx.gridSize);

    std::vector<Rect> regions;
    regions.reserve(zones.size());
    for (const auto& zone : zones) {
        regions.push_back(zone.bounds);
    }
    return regions;
}

}  // namespace arborvia
