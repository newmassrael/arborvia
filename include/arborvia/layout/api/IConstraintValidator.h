#pragma once

#include "../config/LayoutResult.h"
#include "../../core/Types.h"

#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace arborvia {

// Forward declarations
class Graph;
struct ConstraintContext;

/// Result of final state validation (post-routing)
///
/// Used to report all constraint violations found in a layout after routing.
/// Contains separate lists for different violation types:
/// - Node overlaps: nodes that physically intersect
/// - Diagonal edges: edges with non-orthogonal segments
/// - Edge overlaps: pairs of edges that share the same path segment
struct FinalStateValidationResult {
    bool satisfied = true;                                ///< True if all constraints satisfied
    std::vector<NodeId> overlappingNodes;                 ///< Nodes that overlap
    std::vector<EdgeId> diagonalEdges;                    ///< Edges with diagonal segments
    std::vector<std::pair<EdgeId, EdgeId>> overlappingEdgePairs; ///< Edge pairs that overlap

    bool hasNodeOverlap() const { return !overlappingNodes.empty(); }
    bool hasDiagonals() const { return !diagonalEdges.empty(); }
    bool hasEdgeOverlap() const { return !overlappingEdgePairs.empty(); }

    std::string summary() const;
};

/// Result of drag validation (pre-move)
///
/// Used to report whether a proposed node move violates any constraints.
/// If invalid, includes the name of the failed constraint and reason.
struct DragValidationResult {
    bool valid = true;                      ///< True if all constraints satisfied
    std::string failedConstraint;           ///< Name of first failed constraint (empty if valid)
    std::string reason;                     ///< Reason for failure (empty if valid)
    std::vector<EdgeId> invalidEdges;       ///< Edges that would be invalid

    /// Create a valid result
    static DragValidationResult ok() { return {true, "", "", {}}; }
};

/// Interface for constraint validation
///
/// Provides validation for drag operations (pre-move) and final layout state (post-routing).
/// This interface allows for different validation implementations and enables
/// testing with mock validators.
///
/// Implementations should check:
/// - Pre-move: Node overlaps, minimum margins, path validity
/// - Post-routing: Node overlaps, diagonal edges, edge overlaps
///
/// @see ConstraintManager for the default implementation
class IConstraintValidator {
public:
    virtual ~IConstraintValidator() = default;

    // =========================================================================
    // Pre-Move Validation (Single Node)
    // =========================================================================

    /// Validate a drag operation against all constraints
    ///
    /// Checks whether moving a node to the proposed position would violate
    /// any constraints. Used during drag operations to provide real-time feedback.
    ///
    /// @param ctx The constraint context containing node, position, and layout state
    /// @return DragValidationResult with validation status and failure details
    virtual DragValidationResult validate(const ConstraintContext& ctx) const = 0;

    /// Get all blocked regions from constraints with visualization support
    ///
    /// Returns regions where the node cannot be placed due to constraint violations.
    /// Used for visual feedback during drag operations.
    ///
    /// @param ctx The constraint context
    /// @return Combined blocked regions from all constraints
    virtual std::vector<Rect> getAllBlockedRegions(const ConstraintContext& ctx) const = 0;

    // =========================================================================
    // Post-Routing Validation (Global State)
    // =========================================================================

    /// Validate the final layout state after routing
    ///
    /// Performs comprehensive validation of the entire layout including:
    /// - Node overlaps (excluding Point nodes which have no area)
    /// - Diagonal edges (non-orthogonal segments)
    /// - Edge overlaps (edges sharing the same path segment)
    ///
    /// @param nodeLayouts All node layouts
    /// @param edgeLayouts All edge layouts
    /// @return FinalStateValidationResult with detailed issues
    virtual FinalStateValidationResult validateFinalState(
        const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
        const std::unordered_map<EdgeId, EdgeLayout>& edgeLayouts) const = 0;
};

}  // namespace arborvia
