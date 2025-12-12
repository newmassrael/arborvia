#include "arborvia/layout/api/EdgePenaltySystem.h"
#include "arborvia/layout/constraints/ConstraintPenaltyAdapter.h"
#include "arborvia/layout/constraints/builtins/OrthogonalityConstraint.h"
#include "arborvia/layout/constraints/builtins/DirectionalPenetrationConstraint.h"
#include "layout/optimization/BuiltinPenalties.h"

#include <stdexcept>

namespace arborvia {

std::shared_ptr<EdgePenaltySystem> EdgePenaltySystem::createDefault() {
    auto system = std::make_shared<EdgePenaltySystem>();

    // Hard constraints (200,000 weight)
    system->addPenalty(std::make_unique<SegmentOverlapPenalty>());
    system->addPenalty(std::make_unique<DirectionPenalty>());
    system->addPenalty(std::make_unique<NodeCollisionPenalty>());
    system->addPenalty(std::make_unique<TooCloseSnapPenalty>(60.0f));
    system->addPenalty(std::make_unique<SnapPointOverlapPenalty>());
    system->addPenalty(std::make_unique<SelfOverlapPenalty>(20.0f));
    system->addPenalty(std::make_unique<ForbiddenZonePenalty>());
    system->addPenalty(std::make_unique<FixedEndpointPenalty>());

    // Unified constraints via adapter (single source of truth)
    system->addPenalty(std::make_unique<ConstraintPenaltyAdapter>(
        std::make_shared<OrthogonalityConstraint>()));
    system->addPenalty(std::make_unique<ConstraintPenaltyAdapter>(
        std::make_shared<DirectionalSourcePenetrationConstraint>()));
    system->addPenalty(std::make_unique<ConstraintPenaltyAdapter>(
        std::make_shared<DirectionalTargetPenetrationConstraint>()));

    // Soft penalties
    system->addPenalty(std::make_unique<PathIntersectionPenalty>(1000));

    return system;
}

std::shared_ptr<EdgePenaltySystem> EdgePenaltySystem::createMinimal() {
    auto system = std::make_shared<EdgePenaltySystem>();

    // Only critical hard constraints for fast evaluation
    system->addPenalty(std::make_unique<SegmentOverlapPenalty>());
    system->addPenalty(std::make_unique<NodeCollisionPenalty>());
    system->addPenalty(std::make_unique<DirectionPenalty>());
    system->addPenalty(std::make_unique<SnapPointOverlapPenalty>());

    // Unified constraints via adapter
    system->addPenalty(std::make_unique<ConstraintPenaltyAdapter>(
        std::make_shared<OrthogonalityConstraint>()));
    system->addPenalty(std::make_unique<ConstraintPenaltyAdapter>(
        std::make_shared<DirectionalSourcePenetrationConstraint>()));
    system->addPenalty(std::make_unique<ConstraintPenaltyAdapter>(
        std::make_shared<DirectionalTargetPenetrationConstraint>()));

    return system;
}

std::shared_ptr<EdgePenaltySystem> EdgePenaltySystem::createStrict() {
    auto system = std::make_shared<EdgePenaltySystem>();

    // All hard constraints
    system->addPenalty(std::make_unique<SegmentOverlapPenalty>());
    system->addPenalty(std::make_unique<DirectionPenalty>());
    system->addPenalty(std::make_unique<NodeCollisionPenalty>());
    system->addPenalty(std::make_unique<TooCloseSnapPenalty>(80.0f));  // Stricter distance
    system->addPenalty(std::make_unique<SnapPointOverlapPenalty>());
    system->addPenalty(std::make_unique<SelfOverlapPenalty>(30.0f));   // Stricter segment
    system->addPenalty(std::make_unique<ForbiddenZonePenalty>());

    // Unified constraints via adapter
    system->addPenalty(std::make_unique<ConstraintPenaltyAdapter>(
        std::make_shared<OrthogonalityConstraint>()));
    system->addPenalty(std::make_unique<ConstraintPenaltyAdapter>(
        std::make_shared<DirectionalSourcePenetrationConstraint>()));
    system->addPenalty(std::make_unique<ConstraintPenaltyAdapter>(
        std::make_shared<DirectionalTargetPenetrationConstraint>()));

    // Higher weight for soft penalties
    system->addPenalty(std::make_unique<PathIntersectionPenalty>(2000));

    return system;
}

void EdgePenaltySystem::addPenalty(std::unique_ptr<IEdgePenalty> penalty) {
    if (!penalty) {
        throw std::runtime_error("Cannot add null penalty");
    }

    const std::string name = penalty->name();
    if (nameToIndex_.find(name) != nameToIndex_.end()) {
        throw std::runtime_error("Penalty with name '" + name + "' already exists");
    }

    nameToIndex_[name] = penalties_.size();
    penalties_.push_back(std::move(penalty));
}

bool EdgePenaltySystem::removePenalty(const std::string& name) {
    auto it = nameToIndex_.find(name);
    if (it == nameToIndex_.end()) {
        return false;
    }

    size_t indexToRemove = it->second;

    // Remove from vector (swap with last, then pop)
    if (indexToRemove < penalties_.size() - 1) {
        std::swap(penalties_[indexToRemove], penalties_.back());
        // Update index of swapped element
        nameToIndex_[penalties_[indexToRemove]->name()] = indexToRemove;
    }
    penalties_.pop_back();

    // Remove from index map
    nameToIndex_.erase(it);

    return true;
}

const IEdgePenalty* EdgePenaltySystem::getPenalty(const std::string& name) const {
    auto it = nameToIndex_.find(name);
    if (it == nameToIndex_.end()) {
        return nullptr;
    }
    return penalties_[it->second].get();
}

bool EdgePenaltySystem::hasPenalty(const std::string& name) const {
    return nameToIndex_.find(name) != nameToIndex_.end();
}

std::vector<std::string> EdgePenaltySystem::penaltyNames() const {
    std::vector<std::string> names;
    names.reserve(penalties_.size());
    for (const auto& penalty : penalties_) {
        names.push_back(penalty->name());
    }
    return names;
}

int EdgePenaltySystem::calculateTotalPenalty(
    const EdgeLayout& candidate,
    const PenaltyContext& context) const {

    int totalScore = 0;
    for (const auto& penalty : penalties_) {
        totalScore += penalty->calculatePenalty(candidate, context);
    }
    return totalScore;
}

std::unordered_map<std::string, int> EdgePenaltySystem::calculatePenaltyBreakdown(
    const EdgeLayout& candidate,
    const PenaltyContext& context) const {

    std::unordered_map<std::string, int> breakdown;
    for (const auto& penalty : penalties_) {
        breakdown[penalty->name()] = penalty->calculatePenalty(candidate, context);
    }
    return breakdown;
}

bool EdgePenaltySystem::passesHardConstraints(
    const EdgeLayout& candidate,
    const PenaltyContext& context) const {

    for (const auto& penalty : penalties_) {
        if (penalty->isHardConstraint()) {
            int score = penalty->calculatePenalty(candidate, context);
            if (score > 0) {
                return false;  // Hard constraint violated
            }
        }
    }
    return true;
}

}  // namespace arborvia
