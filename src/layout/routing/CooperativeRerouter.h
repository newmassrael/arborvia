#pragma once

#include "arborvia/layout/api/IPathFinder.h"
#include "arborvia/layout/config/LayoutResult.h"
#include "../sugiyama/routing/EdgeRoutingUtils.h"

#include <memory>
#include <unordered_map>
#include <vector>
#include <set>

namespace arborvia {

// Forward declarations
class ObstacleMap;

/// Cooperative rerouting utility for edge pathfinding
///
/// Problem: A transition is blocking B's start/end point.
/// Solution: Reroute A so it doesn't block B's path.
///
/// Algorithm:
/// 1. B's expected path (sourcePoint → targetPoint) is reserved in obstacle map
/// 2. Find edges (A) that overlap with B's expected path
/// 3. Reroute A using obstacle map → A avoids B's reserved area
/// 4. Now B can calculate its actual path
///
/// Cross vs Overlap:
/// - Cross (allowed): Two edges intersect at a single point (X shape)
/// - Overlap (forbidden): Two edges share the same segment (parallel overlap)
class CooperativeRerouter {
public:
    explicit CooperativeRerouter(
        std::shared_ptr<IPathFinder> pathFinder = nullptr,
        float gridSize = 0.0f);

    ~CooperativeRerouter();

    /// Result of cooperative rerouting
    struct RerouteResult {
        bool success = false;
        EdgeLayout layout;                     // B's final layout
        std::vector<EdgeLayout> reroutedEdges; // Blocking edges (A) that were rerouted
        std::vector<EdgeId> failedReroutes;    // Blocking edges that could not be rerouted
        int attempts = 0;
        std::string failureReason;
    };

    /// Main entry point: Make room for B by rerouting blocking edges (A)
    ///
    /// Algorithm:
    /// 1. Calculate B's expected path (simple geometric path)
    /// 2. Add B's expected path to shared obstacle map (reserved area)
    /// 3. Find edges (A) that overlap with B's expected path
    /// 4. Reroute A using obstacle map → A must avoid B's reserved area
    /// 5. Calculate B's actual path (should succeed now)
    ///
    /// @param edgeId B's edge ID
    /// @param currentLayout B's layout (snap points fixed)
    /// @param otherLayouts Other edges including A (modified if rerouted)
    /// @param nodeLayouts Node positions
    /// @return Result with B's layout and rerouted blocking edges
    RerouteResult rerouteWithCooperation(
        EdgeId edgeId,
        const EdgeLayout& currentLayout,
        std::unordered_map<EdgeId, EdgeLayout>& otherLayouts,
        const std::unordered_map<NodeId, NodeLayout>& nodeLayouts);

    void setPathFinder(std::shared_ptr<IPathFinder> pathFinder);
    void setGridSize(float gridSize);
    float effectiveGridSize() const;

    /// Check if two edges have segment overlap (not cross)
    /// Utility method exposing PathIntersection::hasSegmentOverlap
    /// for external validation and testing purposes.
    /// Note: Internal rerouting uses cell-based overlap detection.
    static bool hasSegmentOverlap(const EdgeLayout& a, const EdgeLayout& b);

private:
    std::shared_ptr<IPathFinder> pathFinder_;
    float gridSize_ = 0.0f;
    static constexpr int MAX_REROUTE_ATTEMPTS = 10;

    /// Calculate B's expected path (simple geometric path from source to target)
    /// This is used to reserve B's area before rerouting blocking edges
    std::vector<GridPoint> calculateExpectedPath(
        const EdgeLayout& layout,
        ObstacleMap& obstacles) const;

    /// Find edges that overlap with B's expected path
    std::vector<EdgeId> findBlockingEdges(
        const EdgeLayout& myLayout,
        const std::vector<GridPoint>& myExpectedPath,
        const std::unordered_map<EdgeId, EdgeLayout>& otherLayouts,
        const ObstacleMap& obstacles) const;

    /// Calculate A* path for an edge
    EdgeLayout calculatePath(
        const EdgeLayout& layout,
        ObstacleMap& obstacles,
        bool& pathFound);

    /// Add path to obstacle map as reserved area (high cost)
    void reservePathInObstacles(
        ObstacleMap& obstacles,
        EdgeId edgeId,
        const std::vector<GridPoint>& path) const;
};

}  // namespace arborvia
