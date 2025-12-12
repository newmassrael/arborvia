#pragma once

#include "IEdgeConstraint.h"
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace arborvia {

/// Singleton registry holding all edge routing constraints
///
/// This is the SINGLE SOURCE OF TRUTH for constraint definitions.
/// All constraint checking goes through this registry.
class ConstraintRegistry {
public:
    /// Get the singleton instance with default constraints registered
    static ConstraintRegistry& instance();

    /// Register a new constraint
    /// @param constraint The constraint to register (takes ownership)
    void registerConstraint(std::unique_ptr<IEdgeConstraint> constraint);

    /// Unregister a constraint by name
    /// @return true if constraint was found and removed
    bool unregisterConstraint(const std::string& name);

    /// Get constraint by name
    /// @return Pointer to constraint, or nullptr if not found
    const IEdgeConstraint* getConstraint(const std::string& name) const;

    /// Check if a constraint is registered
    bool hasConstraint(const std::string& name) const;

    /// Get all hard constraints
    std::vector<const IEdgeConstraint*> hardConstraints() const;

    /// Get all soft constraints
    std::vector<const IEdgeConstraint*> softConstraints() const;

    /// Get all constraints
    std::vector<const IEdgeConstraint*> allConstraints() const;

    /// Get constraint names
    std::vector<std::string> constraintNames() const;

private:
    ConstraintRegistry();
    ~ConstraintRegistry() = default;

    // Non-copyable, non-movable singleton
    ConstraintRegistry(const ConstraintRegistry&) = delete;
    ConstraintRegistry& operator=(const ConstraintRegistry&) = delete;

    /// Register default constraints (called by constructor)
    void registerDefaultConstraints();

    std::vector<std::unique_ptr<IEdgeConstraint>> constraints_;
    std::unordered_map<std::string, size_t> nameToIndex_;
};

} // namespace arborvia
