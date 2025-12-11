#pragma once

#include "arborvia/layout/api/IDragConstraint.h"
#include "arborvia/layout/config/LayoutOptions.h"

#include <memory>
#include <set>
#include <string>
#include <unordered_map>
#include <vector>

namespace arborvia {

/// Result of final state validation (post-routing)
struct FinalStateValidationResult {
    bool satisfied = true;
    std::vector<NodeId> overlappingNodes;           ///< Nodes that overlap
    std::vector<EdgeId> diagonalEdges;              ///< Edges with diagonal segments
    std::vector<std::pair<EdgeId, EdgeId>> overlappingEdgePairs; ///< Edge pairs that overlap
    
    bool hasNodeOverlap() const { return !overlappingNodes.empty(); }
    bool hasDiagonals() const { return !diagonalEdges.empty(); }
    bool hasEdgeOverlap() const { return !overlappingEdgePairs.empty(); }
    
    std::string summary() const;
};

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
/// manager.addConstraint(std::make_unique<DirectionAwareMarginConstraint>(5.0f));
/// manager.addConstraint(std::make_unique<EdgePathValidityConstraint>(10.0f));
///
/// ConstraintContext ctx{nodeId, newPos, nodeLayouts, edgeLayouts, &graph, gridSize};
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

    // =========================================================================
    // Pre-Move Validation (Single Node)
    // =========================================================================

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

    // =========================================================================
    // Post-Routing Validation (Global State)
    // =========================================================================

    /// Validate the final layout state after routing
    /// Checks: node overlaps, diagonal edges, edge overlaps
    /// @param nodeLayouts All node layouts
    /// @param edgeLayouts All edge layouts
    /// @return FinalStateValidationResult with detailed issues
    FinalStateValidationResult validateFinalState(
        const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
        const std::unordered_map<EdgeId, EdgeLayout>& edgeLayouts) const;

    /// Find nodes that overlap with each other
    /// @param nodeLayouts All node layouts
    /// @return Vector of overlapping node IDs
    std::vector<NodeId> findOverlappingNodes(
        const std::unordered_map<NodeId, NodeLayout>& nodeLayouts) const;

    /// Find edges with diagonal segments (non-orthogonal)
    /// @param edgeLayouts All edge layouts
    /// @return Vector of edge IDs with diagonal segments
    std::vector<EdgeId> findDiagonalEdges(
        const std::unordered_map<EdgeId, EdgeLayout>& edgeLayouts) const;

    /// Find pairs of edges that overlap
    /// @param edgeLayouts All edge layouts
    /// @return Vector of overlapping edge pairs
    std::vector<std::pair<EdgeId, EdgeId>> findOverlappingEdgePairs(
        const std::unordered_map<EdgeId, EdgeLayout>& edgeLayouts) const;

    /// Clear all constraints
    void clear();

    /// Create a default constraint manager with standard constraints
    /// Includes: DirectionAwareMarginConstraint, EdgePathValidityConstraint
    /// @param minGridDistance Minimum distance between nodes in grid units (default 5.0)
    /// @param options Layout options (used to determine which constraints to enable)
    /// @return Configured ConstraintManager
    /// @note EdgePathValidityConstraint is skipped when DragAlgorithm::HideUntilDrop is used
    ///       because UnifiedRetryChain will handle path finding after drop with full retry logic
    static ConstraintManager createDefault(float minGridDistance = 5.0f,
                                           const LayoutOptions& options = LayoutOptions{});

private:
    /// Find and remove constraint from a specific tier vector
    bool removeFromTier(std::vector<std::unique_ptr<IDragConstraint>>& tier, 
                        const std::string& name);

    std::vector<std::unique_ptr<IDragConstraint>> fastConstraints_;
    std::vector<std::unique_ptr<IDragConstraint>> mediumConstraints_;
    std::vector<std::unique_ptr<IDragConstraint>> expensiveConstraints_;
};

}  // namespace arborvia
