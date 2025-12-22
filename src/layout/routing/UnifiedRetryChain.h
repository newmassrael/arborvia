#pragma once

#include "arborvia/layout/api/IPathFinder.h"
#include "arborvia/layout/config/LayoutResult.h"
#include "CooperativeRerouter.h"

#include <memory>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <string>

namespace arborvia {

/// Unified retry chain for A* pathfinding with CooperativeRerouter fallback
///
/// Retry sequence:
/// 1. A* attempt with current snap points
/// 2. CooperativeRerouter (reroute blocking edges)
/// 3. Snap point variation + A* + rerouter (for each snap position)
/// 4. NodeEdge switch + rerouter
/// 5. Final exhaustive A*
///
/// This class consolidates retry logic from PathCalculator and EdgePathFixer
/// into a single, unified retry chain. It is designed to be used by the library
/// API to automatically handle A* pathfinding failures.
class UnifiedRetryChain {
public:
    /// Configuration for retry behavior
    struct RetryConfig {
        int maxSnapRetries = 9;           ///< Number of snap point variations to try
        int maxNodeEdgeCombinations = 16; ///< Maximum NodeEdge combinations (4x4)
        bool enableCooperativeReroute = true; ///< Enable CooperativeRerouter fallback
        float gridSize = 10.0f;           ///< Grid size for snap calculations
        
        /// Nodes that were moved (for soft constraint on endpoint modification)
        /// If set, only endpoints connected to moved nodes can be modified.
        /// Endpoints connected to unmoved nodes will be preserved.
        /// If nullptr, all endpoints can be modified (legacy behavior).
        const std::unordered_set<NodeId>* movedNodes = nullptr;
    };

    /// Detailed overlap information for logging
    struct OverlapDetail {
        EdgeId otherEdgeId;                            ///< ID of the other edge
        int thisSegmentIndex;                          ///< Index of overlapping segment in this edge
        int otherSegmentIndex;                         ///< Index of overlapping segment in other edge
        Point thisSegStart, thisSegEnd;                ///< This edge's segment coordinates
        Point otherSegStart, otherSegEnd;              ///< Other edge's segment coordinates
    };

    /// Result of post-pathfinding constraint validation
    struct PathValidationResult {
        bool valid = true;                             ///< Whether path meets all constraints
        bool hasOverlap = false;                       ///< Path overlaps with other edges
        bool hasDiagonal = false;                      ///< Path contains diagonal (non-orthogonal) segments
        bool hasNodePenetration = false;               ///< Path penetrates a non-source/target node
        bool hasSourcePenetration = false;             ///< Intermediate segment penetrates source node
        bool hasTargetPenetration = false;             ///< Intermediate segment penetrates target node
        std::vector<EdgeId> overlappingEdges;          ///< IDs of edges this path overlaps with
        std::vector<OverlapDetail> overlapDetails;     ///< Detailed overlap information
    };

    /// Result of retry chain execution
    struct RetryResult {
        bool success = false;                          ///< Whether path was found
        EdgeLayout layout;                             ///< Final edge layout (with bend points)
        std::vector<EdgeLayout> reroutedEdges;         ///< Edges rerouted by CooperativeRerouter
        int astarAttempts = 0;                         ///< Number of A* attempts made
        int cooperativeAttempts = 0;                   ///< Number of CooperativeRerouter attempts
        std::string failureReason;                     ///< Reason for failure (if any)
        PathValidationResult validation;               ///< Post-pathfinding constraint validation result
    };

    /// Constructor
    /// @param pathFinder A* pathfinder implementation
    /// @param gridSize Grid size for snap calculations
    UnifiedRetryChain(
        std::shared_ptr<IPathFinder> pathFinder,
        float gridSize);

    ~UnifiedRetryChain();

    /// Main entry point: Calculate path with full retry chain
    ///
    /// Executes the following retry sequence:
    /// 1. Basic A* attempt with current snap points
    /// 2. CooperativeRerouter (if A* fails)
    /// 3. For each snap position: A* → CooperativeRerouter → A*
    /// 4. NodeEdge switch with rerouter attempts
    ///
    /// @param edgeId ID of the edge being routed
    /// @param layout Current edge layout (snap points fixed)
    /// @param otherEdges Other edges in the layout (may be modified by rerouter)
    /// @param nodeLayouts Node positions
    /// @param config Retry configuration
    /// @return Result containing success status, layout, and rerouted edges
    RetryResult calculatePath(
        EdgeId edgeId,
        const EdgeLayout& layout,
        std::unordered_map<EdgeId, EdgeLayout>& otherEdges,
        const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
        const RetryConfig& config);

    /// @overload Convenience overload with default config
    RetryResult calculatePath(
        EdgeId edgeId,
        const EdgeLayout& layout,
        std::unordered_map<EdgeId, EdgeLayout>& otherEdges,
        const std::unordered_map<NodeId, NodeLayout>& nodeLayouts);

    /// Update grid size
    void setGridSize(float gridSize);

    /// Ensure orthogonality at source and target endpoints
    /// The grid path may not align perfectly with sourcePoint/targetPoint
    /// (e.g., sourcePoint.y=50 while grid bend is at y=60 for gridSize=20)
    /// This inserts alignment points to ensure the first and last segments are orthogonal
    /// @param layout Edge layout to fix (modified in place)
    void ensureEndpointOrthogonality(EdgeLayout& layout) const;

private:
    std::shared_ptr<IPathFinder> pathFinder_;
    std::unique_ptr<CooperativeRerouter> cooperativeRerouter_;
    float gridSize_;

    /// Step 1: Basic A* attempt
    /// @return true if path found, false otherwise
    bool tryAStarPath(
        EdgeLayout& layout,
        const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
        const std::unordered_map<EdgeId, EdgeLayout>& otherEdges);

    /// Step 2: CooperativeRerouter attempt
    CooperativeRerouter::RerouteResult tryCooperativeReroute(
        EdgeId edgeId,
        const EdgeLayout& layout,
        std::unordered_map<EdgeId, EdgeLayout>& otherEdges,
        const std::unordered_map<NodeId, NodeLayout>& nodeLayouts);

    /// Step 3: Snap point variation with A* + rerouter
    /// For each snap ratio: A* → CooperativeRerouter → A*
    RetryResult trySnapPointVariations(
        EdgeId edgeId,
        const EdgeLayout& originalLayout,
        std::unordered_map<EdgeId, EdgeLayout>& otherEdges,
        const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
        const RetryConfig& config);

    /// Step 4: NodeEdge switch with rerouter attempts
    /// Tries all 16 NodeEdge combinations with A* + CooperativeRerouter
    RetryResult tryNodeEdgeSwitch(
        EdgeId edgeId,
        const EdgeLayout& originalLayout,
        std::unordered_map<EdgeId, EdgeLayout>& otherEdges,
        const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
        const RetryConfig& config);

    /// Calculate snap point position for a given ratio
    Point calculateSnapPointForRatio(
        const NodeLayout& node,
        NodeEdge edge,
        float ratio,
        int* outCandidateIndex = nullptr) const;

    /// Get effective grid size (handles <= 0 case)
    float effectiveGridSize() const;

    /// Validate path result against constraints
    /// Checks for overlaps, diagonal segments, and node penetration
    /// @param edgeId ID of the edge being validated
    /// @param layout Edge layout to validate
    /// @param otherEdges Other edges to check against
    /// @param nodeLayouts Node positions for penetration check
    /// @return Validation result with details of any violations
    PathValidationResult validatePathResult(
        EdgeId edgeId,
        const EdgeLayout& layout,
        const std::unordered_map<EdgeId, EdgeLayout>& otherEdges,
        const std::unordered_map<NodeId, NodeLayout>& nodeLayouts) const;

};

}  // namespace arborvia
