#pragma once

#include "IEdgeConstraint.h"

#include <memory>
#include <string>
#include <vector>

namespace arborvia {

/// Manages edge routing constraints for all optimizers
/// 
/// When a constraint is added here, it automatically applies to
/// ALL edge optimizers (AStarEdgeOptimizer, GeometricEdgeOptimizer, etc.)
///
/// Usage:
/// @code
/// EdgeConstraintManager manager;
/// manager.addConstraint(std::make_unique<SegmentOverlapConstraint>());
/// 
/// // Pass to optimizer
/// optimizer.setConstraintManager(&manager);
/// 
/// // Or check manually
/// if (manager.isValid(candidate, assignedLayouts)) {
///     // candidate passes all constraints
/// }
/// @endcode
class EdgeConstraintManager {
public:
    EdgeConstraintManager() = default;
    ~EdgeConstraintManager() = default;

    // Non-copyable but movable
    EdgeConstraintManager(const EdgeConstraintManager&) = delete;
    EdgeConstraintManager& operator=(const EdgeConstraintManager&) = delete;
    EdgeConstraintManager(EdgeConstraintManager&&) = default;
    EdgeConstraintManager& operator=(EdgeConstraintManager&&) = default;

    /// Add a constraint to the manager
    /// @param constraint The constraint to add (ownership transferred)
    void addConstraint(std::unique_ptr<IEdgeConstraint> constraint);

    /// Remove a constraint by name
    /// @param name The name of the constraint to remove
    /// @return true if constraint was found and removed
    bool removeConstraint(const std::string& name);

    /// Check if a constraint with the given name exists
    /// @param name The constraint name to check
    /// @return true if constraint exists
    bool hasConstraint(const std::string& name) const;

    /// Get the number of registered constraints
    size_t constraintCount() const { return constraints_.size(); }

    /// Check if a candidate satisfies ALL constraints
    /// @param candidate The edge layout to validate
    /// @param assignedLayouts Other edges already assigned
    /// @return true if all constraints are satisfied
    bool isValid(
        const EdgeLayout& candidate,
        const std::unordered_map<EdgeId, EdgeLayout>& assignedLayouts) const;

    /// Get the name of the first violated constraint (for debugging)
    /// @param candidate The edge layout to validate
    /// @param assignedLayouts Other edges already assigned
    /// @return Name of first violated constraint, or empty string if all pass
    std::string getFirstViolation(
        const EdgeLayout& candidate,
        const std::unordered_map<EdgeId, EdgeLayout>& assignedLayouts) const;

    /// Clear all constraints
    void clear() { constraints_.clear(); }

    /// Create a default constraint manager with standard constraints
    /// Includes: SegmentOverlapConstraint
    /// @return Configured EdgeConstraintManager
    static EdgeConstraintManager createDefault();

private:
    std::vector<std::unique_ptr<IEdgeConstraint>> constraints_;
};

}  // namespace arborvia
