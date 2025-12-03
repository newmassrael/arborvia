#pragma once

#include "IDragConstraint.h"

#include <memory>
#include <string>
#include <vector>

namespace arborvia {

/// Result of drag validation through ConstraintManager
struct DragValidationResult {
    bool valid = true;                      ///< True if all constraints are satisfied
    std::string failedConstraint;           ///< Name of first failed constraint (empty if valid)
    std::string reason;                     ///< Reason for failure (empty if valid)
    std::vector<EdgeId> invalidEdges;       ///< Edges that would be invalid

    /// Create a valid result
    static DragValidationResult ok() { return {true, "", "", {}}; }
};

/// Manager for drag constraints with tiered execution
///
/// Constraints are organized by performance tier and executed in order:
/// Fast -> Medium -> Expensive. Execution stops on first failure.
///
/// Example usage:
/// @code
/// ConstraintManager manager;
/// manager.addConstraint(std::make_unique<MinDistanceConstraint>(5.0f));
/// manager.addConstraint(std::make_unique<EdgeValidityConstraint>());
///
/// ConstraintContext ctx{nodeId, newPos, nodeLayouts, edgeLayouts, gridSize};
/// auto result = manager.validate(ctx);
/// if (!result.valid) {
///     std::cout << "Failed: " << result.failedConstraint << ": " << result.reason << "\n";
/// }
/// @endcode
class ConstraintManager {
public:
    ConstraintManager() = default;
    ~ConstraintManager() = default;

    // Non-copyable but movable
    ConstraintManager(const ConstraintManager&) = delete;
    ConstraintManager& operator=(const ConstraintManager&) = delete;
    ConstraintManager(ConstraintManager&&) = default;
    ConstraintManager& operator=(ConstraintManager&&) = default;

    /// Add a constraint to the manager
    /// The constraint is automatically sorted by its tier
    /// @param constraint The constraint to add (ownership transferred)
    void addConstraint(std::unique_ptr<IDragConstraint> constraint);

    /// Remove a constraint by name
    /// @param name The name of the constraint to remove
    /// @return true if constraint was found and removed
    bool removeConstraint(const std::string& name);

    /// Check if a constraint with the given name exists
    /// @param name The constraint name to check
    /// @return true if constraint exists
    bool hasConstraint(const std::string& name) const;

    /// Get the number of registered constraints
    size_t constraintCount() const;

    /// Validate a drag operation against all constraints
    /// Constraints are checked in tier order (Fast -> Medium -> Expensive)
    /// Validation stops on first failure for efficiency
    /// @param ctx The constraint context
    /// @return DragValidationResult with validation status
    DragValidationResult validate(const ConstraintContext& ctx) const;

    /// Get all blocked regions from constraints with visualization support
    /// @param ctx The constraint context
    /// @return Combined blocked regions from all constraints
    std::vector<Rect> getAllBlockedRegions(const ConstraintContext& ctx) const;

    /// Clear all constraints
    void clear();

    /// Create a default constraint manager with standard constraints
    /// Includes: MinDistanceConstraint, EdgeValidityConstraint
    /// @param minGridDistance Minimum distance between nodes in grid units (default 5.0)
    /// @return Configured ConstraintManager
    static ConstraintManager createDefault(float minGridDistance = 5.0f);

private:
    /// Find and remove constraint from a specific tier vector
    bool removeFromTier(std::vector<std::unique_ptr<IDragConstraint>>& tier, 
                        const std::string& name);

    std::vector<std::unique_ptr<IDragConstraint>> fastConstraints_;
    std::vector<std::unique_ptr<IDragConstraint>> mediumConstraints_;
    std::vector<std::unique_ptr<IDragConstraint>> expensiveConstraints_;
};

}  // namespace arborvia
