#include "arborvia/layout/constraints/ConstraintGateway.h"

namespace arborvia {

ConstraintGateway::ConstraintGateway()
    : registry_(ConstraintRegistry::instance()) {
}

ConstraintGateway::ConstraintGateway(const ConstraintRegistry& registry)
    : registry_(registry) {
}

ValidationResult ConstraintGateway::validate(
    const EdgeLayout& layout,
    const EdgeConstraintContext& ctx) const {
    
    ValidationResult result;
    
    // Check all constraints
    for (const auto* constraint : registry_.allConstraints()) {
        auto violations = constraint->check(layout, ctx);
        
        for (auto& v : violations) {
            if (constraint->isHardConstraint()) {
                result.passesHardConstraints = false;
            }
            result.violations.push_back(std::move(v));
        }
    }
    
    return result;
}

std::unordered_map<EdgeId, ValidationResult> ConstraintGateway::validateAll(
    const std::unordered_map<EdgeId, EdgeLayout>& layouts,
    const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
    float gridSize) const {
    
    std::unordered_map<EdgeId, ValidationResult> results;
    
    EdgeConstraintContext ctx{layouts, nodeLayouts, gridSize, 1.0f};
    
    for (const auto& [edgeId, layout] : layouts) {
        results[edgeId] = validate(layout, ctx);
    }
    
    return results;
}

bool ConstraintGateway::passesHardConstraints(
    const EdgeLayout& layout,
    const EdgeConstraintContext& ctx) const {
    
    // Only check hard constraints for efficiency
    for (const auto* constraint : registry_.hardConstraints()) {
        auto violations = constraint->check(layout, ctx);
        if (!violations.empty()) {
            return false;
        }
    }
    
    return true;
}

ConstraintGateway::PathValidationResult ConstraintGateway::validatePathResult(
    EdgeId edgeId,
    const EdgeLayout& layout,
    const std::unordered_map<EdgeId, EdgeLayout>& otherEdges,
    const std::unordered_map<NodeId, NodeLayout>& nodeLayouts) const {
    
    PathValidationResult result;
    
    // Build context including this edge
    std::unordered_map<EdgeId, EdgeLayout> allEdges = otherEdges;
    allEdges[edgeId] = layout;
    
    EdgeConstraintContext ctx{allEdges, nodeLayouts, 20.0f, 1.0f};
    
    auto validation = validate(layout, ctx);
    
    result.valid = validation.passesHardConstraints;
    
    for (const auto& v : validation.violations) {
        switch (v.type) {
            case ConstraintViolationType::Orthogonality:
                result.hasDiagonal = true;
                result.valid = false;
                break;
            case ConstraintViolationType::NodePenetration:
                result.hasNodePenetration = true;
                result.valid = false;
                break;
            case ConstraintViolationType::DirectionalSourcePenetration:
                result.hasSourcePenetration = true;
                result.valid = false;
                break;
            case ConstraintViolationType::DirectionalTargetPenetration:
                result.hasTargetPenetration = true;
                result.valid = false;
                break;
            case ConstraintViolationType::SegmentOverlap:
                result.hasOverlap = true;
                if (v.otherEdgeId.has_value()) {
                    result.overlappingEdges.push_back(v.otherEdgeId.value());
                }
                break;
        }
    }
    
    return result;
}

} // namespace arborvia
