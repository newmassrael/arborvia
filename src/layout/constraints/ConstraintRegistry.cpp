#include "arborvia/layout/constraints/ConstraintRegistry.h"
#include "arborvia/layout/constraints/builtins/OrthogonalityConstraint.h"
#include "arborvia/layout/constraints/builtins/NodePenetrationConstraint.h"
#include "arborvia/layout/constraints/builtins/DirectionalPenetrationConstraint.h"
#include "arborvia/layout/constraints/builtins/SegmentOverlapConstraint.h"
#include <algorithm>

namespace arborvia {

ConstraintRegistry& ConstraintRegistry::instance() {
    static ConstraintRegistry instance;
    return instance;
}

ConstraintRegistry::ConstraintRegistry() {
    registerDefaultConstraints();
}

void ConstraintRegistry::registerConstraint(std::unique_ptr<IEdgeConstraint> constraint) {
    if (!constraint) return;
    
    const std::string name = constraint->name();
    if (hasConstraint(name)) {
        // Replace existing constraint with same name
        unregisterConstraint(name);
    }
    
    nameToIndex_[name] = constraints_.size();
    constraints_.push_back(std::move(constraint));
}

bool ConstraintRegistry::unregisterConstraint(const std::string& name) {
    auto it = nameToIndex_.find(name);
    if (it == nameToIndex_.end()) {
        return false;
    }
    
    size_t index = it->second;
    
    // Remove from vector
    constraints_.erase(constraints_.begin() + static_cast<ptrdiff_t>(index));
    
    // Rebuild name index (indices shifted)
    nameToIndex_.clear();
    for (size_t i = 0; i < constraints_.size(); ++i) {
        nameToIndex_[constraints_[i]->name()] = i;
    }
    
    return true;
}

const IEdgeConstraint* ConstraintRegistry::getConstraint(const std::string& name) const {
    auto it = nameToIndex_.find(name);
    if (it == nameToIndex_.end()) {
        return nullptr;
    }
    return constraints_[it->second].get();
}

bool ConstraintRegistry::hasConstraint(const std::string& name) const {
    return nameToIndex_.find(name) != nameToIndex_.end();
}

std::vector<const IEdgeConstraint*> ConstraintRegistry::hardConstraints() const {
    std::vector<const IEdgeConstraint*> result;
    for (const auto& c : constraints_) {
        if (c->isHardConstraint()) {
            result.push_back(c.get());
        }
    }
    return result;
}

std::vector<const IEdgeConstraint*> ConstraintRegistry::softConstraints() const {
    std::vector<const IEdgeConstraint*> result;
    for (const auto& c : constraints_) {
        if (!c->isHardConstraint()) {
            result.push_back(c.get());
        }
    }
    return result;
}

std::vector<const IEdgeConstraint*> ConstraintRegistry::allConstraints() const {
    std::vector<const IEdgeConstraint*> result;
    result.reserve(constraints_.size());
    for (const auto& c : constraints_) {
        result.push_back(c.get());
    }
    return result;
}

std::vector<std::string> ConstraintRegistry::constraintNames() const {
    std::vector<std::string> names;
    names.reserve(constraints_.size());
    for (const auto& c : constraints_) {
        names.push_back(c->name());
    }
    return names;
}

void ConstraintRegistry::registerDefaultConstraints() {
    // Hard constraints (must be satisfied)
    registerConstraint(std::make_unique<OrthogonalityConstraint>());
    registerConstraint(std::make_unique<NodePenetrationConstraint>());
    registerConstraint(std::make_unique<DirectionalSourcePenetrationConstraint>());
    registerConstraint(std::make_unique<DirectionalTargetPenetrationConstraint>());
    
    // Soft constraints (penalized but allowed)
    registerConstraint(std::make_unique<SegmentOverlapConstraint>());
}

} // namespace arborvia
