#include "arborvia/layout/ConstraintManager.h"
#include "arborvia/layout/MinDistanceConstraint.h"
#include "arborvia/layout/EdgeValidityConstraint.h"

#include <algorithm>
#include <optional>

namespace arborvia {

void ConstraintManager::addConstraint(std::unique_ptr<IDragConstraint> constraint) {
    if (!constraint) return;

    switch (constraint->tier()) {
        case ConstraintTier::Fast:
            fastConstraints_.push_back(std::move(constraint));
            break;
        case ConstraintTier::Medium:
            mediumConstraints_.push_back(std::move(constraint));
            break;
        case ConstraintTier::Expensive:
            expensiveConstraints_.push_back(std::move(constraint));
            break;
    }
}

bool ConstraintManager::removeConstraint(const std::string& name) {
    if (removeFromTier(fastConstraints_, name)) return true;
    if (removeFromTier(mediumConstraints_, name)) return true;
    if (removeFromTier(expensiveConstraints_, name)) return true;
    return false;
}

bool ConstraintManager::removeFromTier(std::vector<std::unique_ptr<IDragConstraint>>& tier,
                                        const std::string& name) {
    auto it = std::find_if(tier.begin(), tier.end(),
        [&name](const std::unique_ptr<IDragConstraint>& c) {
            return c && c->name() == name;
        });

    if (it != tier.end()) {
        tier.erase(it);
        return true;
    }
    return false;
}

bool ConstraintManager::hasConstraint(const std::string& name) const {
    auto hasName = [&name](const std::unique_ptr<IDragConstraint>& c) {
        return c && c->name() == name;
    };

    if (std::any_of(fastConstraints_.begin(), fastConstraints_.end(), hasName)) return true;
    if (std::any_of(mediumConstraints_.begin(), mediumConstraints_.end(), hasName)) return true;
    if (std::any_of(expensiveConstraints_.begin(), expensiveConstraints_.end(), hasName)) return true;
    return false;
}

size_t ConstraintManager::constraintCount() const {
    return fastConstraints_.size() + mediumConstraints_.size() + expensiveConstraints_.size();
}

DragValidationResult ConstraintManager::validate(const ConstraintContext& ctx) const {
    // Check constraints in tier order: Fast -> Medium -> Expensive
    // Stop on first failure for efficiency

    auto checkTier = [&ctx](const std::vector<std::unique_ptr<IDragConstraint>>& tier)
        -> std::optional<DragValidationResult> {
        for (const auto& constraint : tier) {
            if (!constraint) continue;

            auto result = constraint->check(ctx);
            if (!result.satisfied) {
                return DragValidationResult{
                    false,
                    constraint->name(),
                    result.reason,
                    std::move(result.invalidEdges)
                };
            }
        }
        return std::nullopt;
    };

    // Check fast constraints first
    if (auto result = checkTier(fastConstraints_)) {
        return *result;
    }

    // Check medium constraints
    if (auto result = checkTier(mediumConstraints_)) {
        return *result;
    }

    // Check expensive constraints last
    if (auto result = checkTier(expensiveConstraints_)) {
        return *result;
    }

    return DragValidationResult::ok();
}

std::vector<Rect> ConstraintManager::getAllBlockedRegions(const ConstraintContext& ctx) const {
    std::vector<Rect> regions;

    auto collectRegions = [&ctx, &regions](const std::vector<std::unique_ptr<IDragConstraint>>& tier) {
        for (const auto& constraint : tier) {
            if (constraint && constraint->hasVisualization()) {
                auto constraintRegions = constraint->getBlockedRegions(ctx);
                regions.insert(regions.end(), 
                               constraintRegions.begin(), 
                               constraintRegions.end());
            }
        }
    };

    collectRegions(fastConstraints_);
    collectRegions(mediumConstraints_);
    collectRegions(expensiveConstraints_);

    return regions;
}

void ConstraintManager::clear() {
    fastConstraints_.clear();
    mediumConstraints_.clear();
    expensiveConstraints_.clear();
}

ConstraintManager ConstraintManager::createDefault(float minGridDistance) {
    ConstraintManager manager;
    manager.addConstraint(std::make_unique<MinDistanceConstraint>(minGridDistance));
    manager.addConstraint(std::make_unique<EdgeValidityConstraint>());
    return manager;
}

}  // namespace arborvia
