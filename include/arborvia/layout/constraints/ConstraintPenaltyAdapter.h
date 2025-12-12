#pragma once

#include "IEdgeConstraint.h"
#include "../api/IEdgePenalty.h"
#include <memory>

namespace arborvia {

/// Adapter that wraps an IEdgeConstraint to implement IEdgePenalty interface
///
/// This enables a single constraint definition to be used in both:
/// - ConstraintGateway (for validation with detailed violation info)
/// - EdgePenaltySystem (for optimization scoring)
///
/// Example usage:
/// @code
/// auto constraint = std::make_shared<OrthogonalityConstraint>();
/// penaltySystem->addPenalty(std::make_unique<ConstraintPenaltyAdapter>(constraint));
/// @endcode
class ConstraintPenaltyAdapter : public IEdgePenalty {
public:
    /// Create adapter wrapping a constraint
    /// @param constraint The constraint to wrap (shared ownership)
    /// @param tolerance Tolerance for constraint checking (default 1.0f)
    explicit ConstraintPenaltyAdapter(
        std::shared_ptr<IEdgeConstraint> constraint,
        float tolerance = 1.0f)
        : constraint_(std::move(constraint))
        , tolerance_(tolerance) {}

    /// Calculate penalty by checking constraint violations
    /// @return HARD_CONSTRAINT_PENALTY if hard constraint violated,
    ///         violation count * 1000 for soft constraints,
    ///         0 if no violations
    int calculatePenalty(
        const EdgeLayout& candidate,
        const PenaltyContext& context) const override {
        
        // Convert PenaltyContext to EdgeConstraintContext
        EdgeConstraintContext ctx{
            context.assignedLayouts,
            context.nodeLayouts,
            context.gridSize,
            tolerance_
        };
        
        auto violations = constraint_->check(candidate, ctx);
        
        if (violations.empty()) {
            return 0;
        }
        
        if (constraint_->isHardConstraint()) {
            return HARD_CONSTRAINT_PENALTY;
        }
        
        // Soft constraint: return weighted violation count
        return static_cast<int>(violations.size()) * defaultWeight();
    }

    std::string name() const override {
        return constraint_->name();
    }

    int defaultWeight() const override {
        return constraint_->isHardConstraint() ? HARD_CONSTRAINT_PENALTY : 1000;
    }

    bool isHardConstraint() const override {
        return constraint_->isHardConstraint();
    }

    /// Get the underlying constraint (for testing/debugging)
    const IEdgeConstraint* constraint() const {
        return constraint_.get();
    }

private:
    std::shared_ptr<IEdgeConstraint> constraint_;
    float tolerance_;
};

}  // namespace arborvia
