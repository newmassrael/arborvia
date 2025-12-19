#pragma once

#include "ConstraintRegistry.h"
#include "ConstraintViolation.h"
#include "IEdgeConstraint.h"
#include "ValidatedEdgeLayout.h"
#include <optional>
#include <unordered_map>
#include <vector>

namespace arborvia {

/// Result of constraint validation
struct ValidationResult {
    /// True if all hard constraints are satisfied
    bool passesHardConstraints = true;

    /// All violations found (both hard and soft)
    std::vector<ConstraintViolation> violations;

    /// Check if validation passed completely (no violations)
    bool isClean() const { return violations.empty(); }
};

/// Gateway for constraint enforcement
///
/// This class is the SINGLE ENTRY POINT for constraint validation.
/// All code that modifies edge layouts should use this gateway
/// to ensure constraint compliance.
class ConstraintGateway {
public:
    /// Create gateway using the global registry
    ConstraintGateway();

    /// Create gateway with specific registry (for testing)
    explicit ConstraintGateway(const ConstraintRegistry& registry);

    /// Validate a single edge layout against all constraints
    /// @param layout The edge layout to validate
    /// @param ctx Context with other edges and nodes
    /// @return Validation result with all violations
    ValidationResult validate(
        const EdgeLayout& layout,
        const EdgeConstraintContext& ctx) const;

    /// Validate multiple edge layouts
    /// @param layouts Edge layouts to validate
    /// @param nodeLayouts Node layouts for context
    /// @param gridSize Grid size for context
    /// @return Map of edge ID to validation result
    std::unordered_map<EdgeId, ValidationResult> validateAll(
        const std::unordered_map<EdgeId, EdgeLayout>& layouts,
        const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
        float gridSize = 20.0f) const;

    /// Check only hard constraints (faster for common case)
    /// @return true if all hard constraints pass
    bool passesHardConstraints(
        const EdgeLayout& layout,
        const EdgeConstraintContext& ctx) const;

    // =========================================================================
    // Type-Safe Validation API (Compile-Time Enforcement)
    // =========================================================================

    /// Validate and wrap in ValidatedEdgeLayout
    /// 
    /// This is the PREFERRED way to validate edge layouts. It returns
    /// ValidatedEdgeLayout which is required by exit points like
    /// LayoutResult::setEdgeLayout().
    /// 
    /// @param layout The edge layout to validate
    /// @param ctx Context with other edges and nodes
    /// @return ValidatedEdgeLayout if validation passes, std::nullopt otherwise
    std::optional<ValidatedEdgeLayout> validateAndWrap(
        const EdgeLayout& layout,
        const EdgeConstraintContext& ctx) const;

    /// Validate and wrap, returning layout even if soft constraints fail
    /// 
    /// This method validates but still wraps the layout if only soft
    /// constraints fail (hard constraints must pass).
    /// 
    /// @param layout The edge layout to validate
    /// @param ctx Context with other edges and nodes
    /// @param outViolations Optional output for soft constraint violations
    /// @return ValidatedEdgeLayout if hard constraints pass, std::nullopt otherwise
    std::optional<ValidatedEdgeLayout> validateAndWrapRelaxed(
        const EdgeLayout& layout,
        const EdgeConstraintContext& ctx,
        std::vector<ConstraintViolation>* outViolations = nullptr) const;

    /// Batch validate and wrap multiple layouts
    /// 
    /// @param layouts Edge layouts to validate
    /// @param nodeLayouts Node layouts for context
    /// @param gridSize Grid size for context
    /// @return Map of edge ID to ValidatedEdgeLayout (only includes valid layouts)
    std::unordered_map<EdgeId, ValidatedEdgeLayout> validateAllAndWrap(
        const std::unordered_map<EdgeId, EdgeLayout>& layouts,
        const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
        float gridSize = 20.0f) const;

    /// Validate and return PathValidationResult compatible structure
    /// This is for compatibility with existing code during migration
    struct PathValidationResult {
        bool valid = true;
        bool hasOverlap = false;
        bool hasDiagonal = false;
        bool hasNodePenetration = false;
        bool hasSourcePenetration = false;
        bool hasTargetPenetration = false;
        std::vector<EdgeId> overlappingEdges;
    };

    PathValidationResult validatePathResult(
        EdgeId edgeId,
        const EdgeLayout& layout,
        const std::unordered_map<EdgeId, EdgeLayout>& otherEdges,
        const std::unordered_map<NodeId, NodeLayout>& nodeLayouts) const;

private:
    const ConstraintRegistry& registry_;
};

} // namespace arborvia
