#pragma once

#include "IEdgePenalty.h"

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace arborvia {

/// Unified penalty system for edge routing optimization
///
/// Manages a collection of IEdgePenalty instances and calculates
/// total penalty scores for candidate edge layouts. This replaces
/// the previous split between EdgeConstraintManager (boolean) and
/// EdgeScorer (scoring).
///
/// All penalties are applied consistently across A* and Geometric
/// optimizers, ensuring uniform constraint enforcement.
///
/// Built-in penalties (via createDefault()):
/// - SegmentOverlapPenalty (200,000) - segment overlap detection
/// - DirectionPenalty (200,000) - entry/exit direction validation
/// - NodeCollisionPenalty (200,000) - node collision detection
/// - TooCloseSnapPenalty (200,000) - snap point spacing
/// - SelfOverlapPenalty (200,000) - self-intersection detection
/// - ForbiddenZonePenalty (200,000) - forbidden zone violation
/// - PathIntersectionPenalty (1,000) - edge crossing (soft)
///
/// Usage:
/// @code
/// auto system = EdgePenaltySystem::createDefault();
/// system->addPenalty(std::make_unique<CustomPenalty>());
///
/// PenaltyContext ctx{assignedLayouts, nodeLayouts, forbiddenZones, 20.0f};
/// int score = system->calculateTotalPenalty(candidate, ctx);
/// @endcode
class EdgePenaltySystem {
public:
    EdgePenaltySystem() = default;
    ~EdgePenaltySystem() = default;

    // Non-copyable, movable
    EdgePenaltySystem(const EdgePenaltySystem&) = delete;
    EdgePenaltySystem& operator=(const EdgePenaltySystem&) = delete;
    EdgePenaltySystem(EdgePenaltySystem&&) = default;
    EdgePenaltySystem& operator=(EdgePenaltySystem&&) = default;

    /// Create system with all default penalties
    /// Includes all 7 built-in penalties
    static std::shared_ptr<EdgePenaltySystem> createDefault();

    /// Create minimal system (fewer penalties, faster)
    /// Only includes critical hard constraints
    static std::shared_ptr<EdgePenaltySystem> createMinimal();

    /// Create strict system (all penalties with higher weights)
    /// For conservative/high-quality routing
    static std::shared_ptr<EdgePenaltySystem> createStrict();

    /// Add a penalty to the system
    /// @param penalty Penalty to add (ownership transferred)
    /// @throws std::runtime_error if penalty with same name already exists
    void addPenalty(std::unique_ptr<IEdgePenalty> penalty);

    /// Remove a penalty by name
    /// @param name Name of penalty to remove
    /// @return true if penalty was found and removed
    bool removePenalty(const std::string& name);

    /// Get a penalty by name
    /// @param name Name of penalty to find
    /// @return Pointer to penalty, or nullptr if not found
    const IEdgePenalty* getPenalty(const std::string& name) const;

    /// Check if a penalty exists
    /// @param name Name of penalty to check
    /// @return true if penalty exists
    bool hasPenalty(const std::string& name) const;

    /// Get all penalty names
    /// @return Vector of penalty names
    std::vector<std::string> penaltyNames() const;

    /// Get count of penalties
    /// @return Number of penalties in system
    size_t penaltyCount() const { return penalties_.size(); }

    /// Calculate total penalty score for a candidate layout
    /// @param candidate The edge layout to evaluate
    /// @param context Context with node layouts, other edges, etc.
    /// @return Sum of all penalty scores
    int calculateTotalPenalty(
        const EdgeLayout& candidate,
        const PenaltyContext& context) const;

    /// Calculate penalty breakdown for debugging
    /// @param candidate The edge layout to evaluate
    /// @param context Context with node layouts, other edges, etc.
    /// @return Map of penalty name to score
    std::unordered_map<std::string, int> calculatePenaltyBreakdown(
        const EdgeLayout& candidate,
        const PenaltyContext& context) const;

    /// Check if candidate passes all hard constraints
    /// @param candidate The edge layout to evaluate
    /// @param context Context with node layouts, other edges, etc.
    /// @return true if no hard constraint is violated
    bool passesHardConstraints(
        const EdgeLayout& candidate,
        const PenaltyContext& context) const;

private:
    std::vector<std::unique_ptr<IEdgePenalty>> penalties_;
    std::unordered_map<std::string, size_t> nameToIndex_;
};

}  // namespace arborvia
