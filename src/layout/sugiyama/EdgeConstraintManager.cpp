#include "arborvia/layout/EdgeConstraintManager.h"
#include "SegmentOverlapConstraint.h"

#include <algorithm>

namespace arborvia {

void EdgeConstraintManager::addConstraint(std::unique_ptr<IEdgeConstraint> constraint) {
    if (constraint) {
        constraints_.push_back(std::move(constraint));
    }
}

bool EdgeConstraintManager::removeConstraint(const std::string& name) {
    auto it = std::remove_if(constraints_.begin(), constraints_.end(),
        [&name](const std::unique_ptr<IEdgeConstraint>& c) {
            return c && c->name() == name;
        });

    if (it != constraints_.end()) {
        constraints_.erase(it, constraints_.end());
        return true;
    }
    return false;
}

bool EdgeConstraintManager::hasConstraint(const std::string& name) const {
    for (const auto& constraint : constraints_) {
        if (constraint && constraint->name() == name) {
            return true;
        }
    }
    return false;
}

bool EdgeConstraintManager::isValid(
    const EdgeLayout& candidate,
    const std::unordered_map<EdgeId, EdgeLayout>& assignedLayouts) const {

    for (const auto& constraint : constraints_) {
        if (constraint && !constraint->isValid(candidate, assignedLayouts)) {
            return false;
        }
    }
    return true;
}

std::string EdgeConstraintManager::getFirstViolation(
    const EdgeLayout& candidate,
    const std::unordered_map<EdgeId, EdgeLayout>& assignedLayouts) const {

    for (const auto& constraint : constraints_) {
        if (constraint && !constraint->isValid(candidate, assignedLayouts)) {
            return constraint->name();
        }
    }
    return {};
}

EdgeConstraintManager EdgeConstraintManager::createDefault() {
    EdgeConstraintManager manager;
    manager.addConstraint(std::make_unique<SegmentOverlapConstraint>());
    return manager;
}

}  // namespace arborvia
