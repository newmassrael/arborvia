#include "layout/interactive/ConstraintManager.h"
#include "layout/constraints/DirectionAwareMarginConstraint.h"
#include "layout/constraints/EdgePathValidityConstraint.h"
#include "sugiyama/routing/PathIntersection.h"
#include "arborvia/common/Logger.h"

#include <algorithm>
#include <cmath>
#include <optional>
#include <sstream>

namespace arborvia {

// ============================================================================
// FinalStateValidationResult
// ============================================================================

std::string FinalStateValidationResult::summary() const {
    if (satisfied) return "All constraints satisfied";
    
    std::ostringstream ss;
    ss << "Violations found: ";
    std::vector<std::string> issues;
    
    if (!overlappingNodes.empty()) {
        issues.push_back(std::to_string(overlappingNodes.size()) + " overlapping nodes");
    }
    if (!diagonalEdges.empty()) {
        issues.push_back(std::to_string(diagonalEdges.size()) + " diagonal edges");
    }
    if (!overlappingEdgePairs.empty()) {
        issues.push_back(std::to_string(overlappingEdgePairs.size()) + " overlapping edge pairs");
    }
    
    for (size_t i = 0; i < issues.size(); ++i) {
        if (i > 0) ss << ", ";
        ss << issues[i];
    }
    
    return ss.str();
}

void ConstraintManager::addConstraint(std::unique_ptr<IDragConstraint> constraint) {
    if (!constraint) return;

    switch (constraint->tier()) {
        case ConstraintTier::Fast:
            fastConstraints_.push_back(std::move(constraint));
            break;
        case ConstraintTier::Medium:
            mediumConstraints_.push_back(std::move(constraint));
            break;
        case ConstraintTier::Expensive:
            expensiveConstraints_.push_back(std::move(constraint));
            break;
    }
}

bool ConstraintManager::removeConstraint(const std::string& name) {
    if (removeFromTier(fastConstraints_, name)) return true;
    if (removeFromTier(mediumConstraints_, name)) return true;
    if (removeFromTier(expensiveConstraints_, name)) return true;
    return false;
}

bool ConstraintManager::removeFromTier(std::vector<std::unique_ptr<IDragConstraint>>& tier,
                                        const std::string& name) {
    auto it = std::find_if(tier.begin(), tier.end(),
        [&name](const std::unique_ptr<IDragConstraint>& c) {
            return c && c->name() == name;
        });

    if (it != tier.end()) {
        tier.erase(it);
        return true;
    }
    return false;
}

bool ConstraintManager::hasConstraint(const std::string& name) const {
    auto hasName = [&name](const std::unique_ptr<IDragConstraint>& c) {
        return c && c->name() == name;
    };

    if (std::any_of(fastConstraints_.begin(), fastConstraints_.end(), hasName)) return true;
    if (std::any_of(mediumConstraints_.begin(), mediumConstraints_.end(), hasName)) return true;
    if (std::any_of(expensiveConstraints_.begin(), expensiveConstraints_.end(), hasName)) return true;
    return false;
}

size_t ConstraintManager::constraintCount() const {
    return fastConstraints_.size() + mediumConstraints_.size() + expensiveConstraints_.size();
}

DragValidationResult ConstraintManager::validate(const ConstraintContext& ctx) const {
    // Check constraints in tier order: Fast -> Medium -> Expensive
    // Stop on first failure for efficiency

    LOG_DEBUG("[ConstraintManager::validate] nodeId={} pos=({},{}) fastConstraints={} mediumConstraints={} expensiveConstraints={}",
              ctx.nodeId, ctx.newPosition.x, ctx.newPosition.y,
              fastConstraints_.size(), mediumConstraints_.size(), expensiveConstraints_.size());

    auto checkTier = [&ctx](const std::vector<std::unique_ptr<IDragConstraint>>& tier, const char* tierName)
        -> std::optional<DragValidationResult> {
        for (const auto& constraint : tier) {
            if (!constraint) continue;

            LOG_DEBUG("[ConstraintManager::validate] Checking {} constraint: {}", tierName, constraint->name());
            auto result = constraint->check(ctx);
            LOG_DEBUG("[ConstraintManager::validate] {} constraint {} result: satisfied={} reason={}",
                      tierName, constraint->name(), result.satisfied, result.reason);
            if (!result.satisfied) {
                return DragValidationResult{
                    false,
                    constraint->name(),
                    result.reason,
                    std::move(result.invalidEdges)
                };
            }
        }
        return std::nullopt;
    };

    // Check fast constraints first
    if (auto result = checkTier(fastConstraints_, "Fast")) {
        return *result;
    }

    // Check medium constraints
    if (auto result = checkTier(mediumConstraints_, "Medium")) {
        return *result;
    }

    // Check expensive constraints last
    if (auto result = checkTier(expensiveConstraints_, "Expensive")) {
        return *result;
    }

    return DragValidationResult::ok();
}

std::vector<Rect> ConstraintManager::getAllBlockedRegions(const ConstraintContext& ctx) const {
    std::vector<Rect> regions;

    auto collectRegions = [&ctx, &regions](const std::vector<std::unique_ptr<IDragConstraint>>& tier) {
        for (const auto& constraint : tier) {
            if (constraint && constraint->hasVisualization()) {
                auto constraintRegions = constraint->getBlockedRegions(ctx);
                regions.insert(regions.end(), 
                               constraintRegions.begin(), 
                               constraintRegions.end());
            }
        }
    };

    collectRegions(fastConstraints_);
    collectRegions(mediumConstraints_);
    collectRegions(expensiveConstraints_);

    return regions;
}

void ConstraintManager::clear() {
    fastConstraints_.clear();
    mediumConstraints_.clear();
    expensiveConstraints_.clear();
}

ConstraintManager ConstraintManager::createDefault(float minGridDistance, const LayoutOptions& options) {
    ConstraintManager manager;
    // Use DirectionAwareMarginConstraint for direction-aware margin calculation
    manager.addConstraint(std::make_unique<DirectionAwareMarginConstraint>(minGridDistance));

    // EdgePathValidityConstraint: A* path validation during drag
    // Use DragAlgorithmTraits (Single Source of Truth) to determine if validation is needed
    if (DragAlgorithmTraits::requiresPathValidationDuringDrag(options.optimizationOptions.dragAlgorithm)) {
        manager.addConstraint(std::make_unique<EdgePathValidityConstraint>(10.0f));
    }

    return manager;
}

// ============================================================================
// Final State Validation (Post-Routing)
// ============================================================================

FinalStateValidationResult ConstraintManager::validateFinalState(
    const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
    const std::unordered_map<EdgeId, EdgeLayout>& edgeLayouts) const {
    
    FinalStateValidationResult result;
    
    // 1. Check node overlaps
    result.overlappingNodes = findOverlappingNodes(nodeLayouts);
    
    // 2. Check diagonal edges
    result.diagonalEdges = findDiagonalEdges(edgeLayouts);
    
    // 3. Check edge overlaps (uses PathIntersection)
    result.overlappingEdgePairs = findOverlappingEdgePairs(edgeLayouts);
    
    // 4. Determine overall status
    result.satisfied = result.overlappingNodes.empty() &&
                       result.diagonalEdges.empty() &&
                       result.overlappingEdgePairs.empty();
    
    return result;
}

std::vector<NodeId> ConstraintManager::findOverlappingNodes(
    const std::unordered_map<NodeId, NodeLayout>& nodeLayouts) const {
    
    std::set<NodeId> overlapping;
    
    // Convert to vector for indexed access
    std::vector<std::pair<NodeId, const NodeLayout*>> nodes;
    nodes.reserve(nodeLayouts.size());
    for (const auto& [id, layout] : nodeLayouts) {
        nodes.emplace_back(id, &layout);
    }
    
    // Check all pairs
    for (size_t i = 0; i < nodes.size(); ++i) {
        for (size_t j = i + 1; j < nodes.size(); ++j) {
            const auto& a = *nodes[i].second;
            const auto& b = *nodes[j].second;

            // Skip nodes that don't participate in overlap detection (e.g., Point nodes)
            if (!a.participatesInOverlapDetection() || !b.participatesInOverlapDetection()) {
                continue;
            }

            // AABB intersection check
            if (a.position.x < b.position.x + b.size.width &&
                a.position.x + a.size.width > b.position.x &&
                a.position.y < b.position.y + b.size.height &&
                a.position.y + a.size.height > b.position.y) {
                overlapping.insert(nodes[i].first);
                overlapping.insert(nodes[j].first);
            }
        }
    }
    
    return std::vector<NodeId>(overlapping.begin(), overlapping.end());
}

std::vector<EdgeId> ConstraintManager::findDiagonalEdges(
    const std::unordered_map<EdgeId, EdgeLayout>& edgeLayouts) const {
    
    std::vector<EdgeId> diagonals;
    
    for (const auto& [edgeId, edge] : edgeLayouts) {
        auto points = edge.allPoints();
        if (points.size() < 2) continue;
        
        for (size_t i = 0; i + 1 < points.size(); ++i) {
            float dx = std::abs(points[i + 1].x - points[i].x);
            float dy = std::abs(points[i + 1].y - points[i].y);
            if (dx > 1.0f && dy > 1.0f) {
                diagonals.push_back(edgeId);
                break;  // Only count each edge once
            }
        }
    }
    
    return diagonals;
}

std::vector<std::pair<EdgeId, EdgeId>> ConstraintManager::findOverlappingEdgePairs(
    const std::unordered_map<EdgeId, EdgeLayout>& edgeLayouts) const {
    // Delegate to PathIntersection (Single Source of Truth)
    return PathIntersection::findAllOverlappingPairs(edgeLayouts);
}

}  // namespace arborvia
