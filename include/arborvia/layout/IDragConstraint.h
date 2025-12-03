#pragma once

#include "../core/Types.h"
#include "LayoutResult.h"

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace arborvia {

/// Context information passed to drag constraints
struct ConstraintContext {
    NodeId nodeId;                                                      ///< Node being dragged
    Point newPosition;                                                  ///< Proposed new position
    const std::unordered_map<NodeId, NodeLayout>& nodeLayouts;          ///< Current node layouts
    const std::unordered_map<EdgeId, EdgeLayout>& edgeLayouts;          ///< Current edge layouts
    float gridSize;                                                     ///< Grid size for calculations
};

/// Result of a constraint check
struct ConstraintResult {
    bool satisfied = true;          ///< True if constraint is satisfied
    std::string reason;             ///< Reason for failure (empty if satisfied)
    std::vector<EdgeId> invalidEdges;  ///< Edges that would be invalid (for edge-related constraints)

    /// Create a satisfied result
    static ConstraintResult ok() { return {true, "", {}}; }

    /// Create a failed result with reason
    static ConstraintResult fail(const std::string& reason) {
        return {false, reason, {}};
    }

    /// Create a failed result with invalid edges
    static ConstraintResult failWithEdges(const std::string& reason, std::vector<EdgeId> edges) {
        return {false, reason, std::move(edges)};
    }
};

/// Performance tier for constraint execution ordering
enum class ConstraintTier {
    Fast,       ///< O(1) or simple O(n) checks (e.g., distance, bounds)
    Medium,     ///< O(n) with computation (e.g., snap point calculation)
    Expensive   ///< O(n*m) or pathfinding (e.g., edge validity)
};

/// Interface for drag constraints
/// 
/// Implement this interface to create custom drag constraints.
/// Constraints are checked in order of their tier (Fast -> Medium -> Expensive)
/// to allow early failure for cheap checks.
class IDragConstraint {
public:
    virtual ~IDragConstraint() = default;

    /// Check if the constraint is satisfied for the given context
    /// @param ctx The constraint context with node position and layouts
    /// @return ConstraintResult indicating satisfaction status
    virtual ConstraintResult check(const ConstraintContext& ctx) const = 0;

    /// Get the performance tier of this constraint
    /// @return ConstraintTier for execution ordering
    virtual ConstraintTier tier() const = 0;

    /// Get the name of this constraint (for debugging/logging)
    /// @return Unique constraint name
    virtual std::string name() const = 0;

    /// Check if this constraint supports visualization
    /// @return true if getBlockedRegions() provides meaningful data
    virtual bool hasVisualization() const { return false; }

    /// Get regions that would be blocked by this constraint
    /// Used for interactive feedback during drag operations
    /// @param ctx The constraint context
    /// @return Vector of blocked rectangular regions
    virtual std::vector<Rect> getBlockedRegions(const ConstraintContext& ctx) const {
        (void)ctx;  // Suppress unused parameter warning
        return {};
    }
};

}  // namespace arborvia
