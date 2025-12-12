#include "layout/optimization/BuiltinPenalties.h"
#include "arborvia/layout/util/LayoutUtils.h"
#include "arborvia/layout/config/MoveDirection.h"
#include "sugiyama/routing/PathIntersection.h"
#include "arborvia/core/GeometryUtils.h"

#include <cmath>
#include <algorithm>
#include "arborvia/common/Logger.h"

#ifndef EDGE_ROUTING_DEBUG
#define EDGE_ROUTING_DEBUG 0
#endif

namespace arborvia {

using constants::EPSILON;

// === Helper Functions ===

namespace {

bool segmentIntersectsNode(
    const Point& p1, const Point& p2,
    const NodeLayout& node, float margin = 0.0f) {

    float nodeXmin = node.position.x - margin;
    float nodeXmax = node.position.x + node.size.width + margin;
    float nodeYmin = node.position.y - margin;
    float nodeYmax = node.position.y + node.size.height + margin;

    bool isHorizontal = (std::abs(p1.y - p2.y) < EPSILON);
    bool isVertical = (std::abs(p1.x - p2.x) < EPSILON);

    if (isHorizontal) {
        float y = p1.y;
        float xMin = std::min(p1.x, p2.x);
        float xMax = std::max(p1.x, p2.x);
        return (y > nodeYmin && y < nodeYmax && xMin < nodeXmax && xMax > nodeXmin);
    }
    else if (isVertical) {
        float x = p1.x;
        float yMin = std::min(p1.y, p2.y);
        float yMax = std::max(p1.y, p2.y);
        return (x > nodeXmin && x < nodeXmax && yMin < nodeYmax && yMax > nodeYmin);
    }

    // Non-orthogonal segment: use general AABB intersection
    float xMin = std::min(p1.x, p2.x);
    float xMax = std::max(p1.x, p2.x);
    float yMin = std::min(p1.y, p2.y);
    float yMax = std::max(p1.y, p2.y);

    return !(xMax < nodeXmin || xMin > nodeXmax || yMax < nodeYmin || yMin > nodeYmax);
}

bool isPointInZone(const Point& p, const ForbiddenZone& zone) {
    return p.x >= zone.bounds.x &&
           p.x < zone.bounds.x + zone.bounds.width &&
           p.y >= zone.bounds.y &&
           p.y < zone.bounds.y + zone.bounds.height;
}

bool segmentIntersectsZone(
    const Point& p1, const Point& p2,
    const ForbiddenZone& zone) {

    float zoneXmin = zone.bounds.x;
    float zoneXmax = zone.bounds.x + zone.bounds.width;
    float zoneYmin = zone.bounds.y;
    float zoneYmax = zone.bounds.y + zone.bounds.height;

    bool isHorizontal = (std::abs(p1.y - p2.y) < EPSILON);
    bool isVertical = (std::abs(p1.x - p2.x) < EPSILON);

    if (isHorizontal) {
        float y = p1.y;
        float xMin = std::min(p1.x, p2.x);
        float xMax = std::max(p1.x, p2.x);
        return (y > zoneYmin && y < zoneYmax && xMin < zoneXmax && xMax > zoneXmin);
    }
    else if (isVertical) {
        float x = p1.x;
        float yMin = std::min(p1.y, p2.y);
        float yMax = std::max(p1.y, p2.y);
        return (x > zoneXmin && x < zoneXmax && yMin < zoneYmax && yMax > zoneYmin);
    }

    // Non-orthogonal: AABB intersection
    float xMin = std::min(p1.x, p2.x);
    float xMax = std::max(p1.x, p2.x);
    float yMin = std::min(p1.y, p2.y);
    float yMax = std::max(p1.y, p2.y);

    return !(xMax < zoneXmin || xMin > zoneXmax || yMax < zoneYmin || yMin > zoneYmax);
}

}  // anonymous namespace

// === SegmentOverlapPenalty ===

int SegmentOverlapPenalty::calculatePenalty(
    const EdgeLayout& candidate,
    const PenaltyContext& context) const {

    if (PathIntersection::hasOverlapWithOthers(candidate, context.assignedLayouts, candidate.id)) {
#if EDGE_ROUTING_DEBUG
        LOG_DEBUG("[SegmentOverlapPenalty] Edge {} OVERLAPS with others, returning {}",
                  candidate.id, defaultWeight());
#endif
        return defaultWeight();
    }
    return 0;
}

// === DirectionPenalty ===

int DirectionPenalty::calculatePenalty(
    const EdgeLayout& candidate,
    const PenaltyContext& context) const {

    (void)context;  // Not needed for direction check

    int violations = 0;

    // Get required directions based on source/target edges
    MoveDirection requiredSrcDir = LayoutUtils::getRequiredSourceDirection(candidate.sourceEdge);
    MoveDirection requiredTgtDir = LayoutUtils::getRequiredTargetArrivalDirection(candidate.targetEdge);

    // Check first segment direction
    if (requiredSrcDir != MoveDirection::None) {
        MoveDirection actualFirstDir = LayoutUtils::getFirstSegmentDirection(candidate);
        if (actualFirstDir != MoveDirection::None && actualFirstDir != requiredSrcDir) {
            violations++;
        }
    }

    // Check last segment direction
    if (requiredTgtDir != MoveDirection::None) {
        MoveDirection actualLastDir = LayoutUtils::getLastSegmentDirection(candidate);
        if (actualLastDir != MoveDirection::None && actualLastDir != requiredTgtDir) {
            violations++;
        }
    }

    return violations * defaultWeight();
}

// === NodeCollisionPenalty ===

int NodeCollisionPenalty::calculatePenalty(
    const EdgeLayout& candidate,
    const PenaltyContext& context) const {

    int collisions = 0;

    // Collect all segments
    std::vector<std::pair<Point, Point>> segments;
    candidate.forEachSegment([&](const Point& p1, const Point& p2) {
        segments.emplace_back(p1, p2);
    });

    // Check each segment against each node
    for (const auto& [nodeId, node] : context.nodeLayouts) {
        // Skip source and target nodes
        if (nodeId == candidate.from || nodeId == candidate.to) {
            continue;
        }

        for (size_t i = 0; i < segments.size(); ++i) {
            const auto& [p1, p2] = segments[i];

            if (segmentIntersectsNode(p1, p2, node)) {
                ++collisions;
                break;  // Count each node at most once
            }
        }
    }

    // Also check collisions with source/target nodes (skip first/last segment)
    if (segments.size() > 1) {
        auto srcIt = context.nodeLayouts.find(candidate.from);
        if (srcIt != context.nodeLayouts.end()) {
            for (size_t i = 1; i < segments.size(); ++i) {
                const auto& [p1, p2] = segments[i];
                if (segmentIntersectsNode(p1, p2, srcIt->second)) {
                    ++collisions;
                    break;
                }
            }
        }

        auto tgtIt = context.nodeLayouts.find(candidate.to);
        if (tgtIt != context.nodeLayouts.end()) {
            for (size_t i = 0; i + 1 < segments.size(); ++i) {
                const auto& [p1, p2] = segments[i];
                if (segmentIntersectsNode(p1, p2, tgtIt->second)) {
                    ++collisions;
                    break;
                }
            }
        }
    }

    return collisions * defaultWeight();
}

// === TooCloseSnapPenalty ===

int TooCloseSnapPenalty::calculatePenalty(
    const EdgeLayout& candidate,
    const PenaltyContext& context) const {

    (void)context;  // Not needed

    const float dx = std::abs(candidate.sourcePoint.x - candidate.targetPoint.x);
    const float dy = std::abs(candidate.sourcePoint.y - candidate.targetPoint.y);

    const bool sourceIsVertical = (candidate.sourceEdge == NodeEdge::Top ||
                                    candidate.sourceEdge == NodeEdge::Bottom);
    const bool targetIsVertical = (candidate.targetEdge == NodeEdge::Top ||
                                    candidate.targetEdge == NodeEdge::Bottom);

    // Only check when both edges have same orientation
    if (!sourceIsVertical && !targetIsVertical) {
        // Both horizontal (Left/Right)
        if (dx < minDistance_ && dy < minDistance_) {
            return defaultWeight();
        }
    } else if (sourceIsVertical && targetIsVertical) {
        // Both vertical (Top/Bottom)
        if (dy < minDistance_ && dx < minDistance_) {
            return defaultWeight();
        }
    }

    return 0;
}

// === SnapPointOverlapPenalty ===

int SnapPointOverlapPenalty::calculatePenalty(
    const EdgeLayout& candidate,
    const PenaltyContext& context) const {

    const float TOL = tolerance_;

    for (const auto& [edgeId, other] : context.assignedLayouts) {
        if (edgeId == candidate.id) continue;

        // Check candidate's SOURCE against other's SOURCE on same NodeEdge
        if (other.from == candidate.from &&
            other.sourceEdge == candidate.sourceEdge) {
            float dx = std::abs(other.sourcePoint.x - candidate.sourcePoint.x);
            float dy = std::abs(other.sourcePoint.y - candidate.sourcePoint.y);
            if (dx < TOL && dy < TOL) {
                return defaultWeight();
            }
        }

        // Check candidate's SOURCE against other's TARGET on same NodeEdge
        if (other.to == candidate.from &&
            other.targetEdge == candidate.sourceEdge) {
            float dx = std::abs(other.targetPoint.x - candidate.sourcePoint.x);
            float dy = std::abs(other.targetPoint.y - candidate.sourcePoint.y);
            if (dx < TOL && dy < TOL) {
                return defaultWeight();
            }
        }

        // Check candidate's TARGET against other's SOURCE on same NodeEdge
        if (other.from == candidate.to &&
            other.sourceEdge == candidate.targetEdge) {
            float dx = std::abs(other.sourcePoint.x - candidate.targetPoint.x);
            float dy = std::abs(other.sourcePoint.y - candidate.targetPoint.y);
            if (dx < TOL && dy < TOL) {
                return defaultWeight();
            }
        }

        // Check candidate's TARGET against other's TARGET on same NodeEdge
        if (other.to == candidate.to &&
            other.targetEdge == candidate.targetEdge) {
            float dx = std::abs(other.targetPoint.x - candidate.targetPoint.x);
            float dy = std::abs(other.targetPoint.y - candidate.targetPoint.y);
            if (dx < TOL && dy < TOL) {
                return defaultWeight();
            }
        }
    }

    return 0;
}

// === SelfOverlapPenalty ===

int SelfOverlapPenalty::calculatePenalty(
    const EdgeLayout& candidate,
    const PenaltyContext& context) const {

    (void)context;  // Not needed

    const float MIN_SEGMENT = minSegmentLength_;

    const bool sourceIsVertical = (candidate.sourceEdge == NodeEdge::Top ||
                                    candidate.sourceEdge == NodeEdge::Bottom);
    const bool targetIsVertical = (candidate.targetEdge == NodeEdge::Top ||
                                    candidate.targetEdge == NodeEdge::Bottom);

    // Case 1: Both horizontal edges (H-V-H path)
    if (!sourceIsVertical && !targetIsVertical) {
        float x1 = (candidate.sourceEdge == NodeEdge::Right)
            ? candidate.sourcePoint.x + MIN_SEGMENT
            : candidate.sourcePoint.x - MIN_SEGMENT;

        float x2 = (candidate.targetEdge == NodeEdge::Right)
            ? candidate.targetPoint.x + MIN_SEGMENT
            : candidate.targetPoint.x - MIN_SEGMENT;

        if (candidate.targetEdge == NodeEdge::Left) {
            if (x1 > x2) return defaultWeight();
        } else {
            if (x1 < x2) return defaultWeight();
        }
    }

    // Case 2: Both vertical edges (V-H-V path)
    if (sourceIsVertical && targetIsVertical) {
        float y1 = (candidate.sourceEdge == NodeEdge::Top)
            ? candidate.sourcePoint.y - MIN_SEGMENT
            : candidate.sourcePoint.y + MIN_SEGMENT;

        float y2 = (candidate.targetEdge == NodeEdge::Top)
            ? candidate.targetPoint.y - MIN_SEGMENT
            : candidate.targetPoint.y + MIN_SEGMENT;

        if (candidate.targetEdge == NodeEdge::Top) {
            if (y1 > y2) return defaultWeight();
        } else {
            if (y1 < y2) return defaultWeight();
        }
    }

    return 0;
}

// === ForbiddenZonePenalty ===

int ForbiddenZonePenalty::calculatePenalty(
    const EdgeLayout& candidate,
    const PenaltyContext& context) const {

    if (context.forbiddenZones.empty()) {
        return 0;
    }

    int violations = 0;

    for (const auto& zone : context.forbiddenZones) {
        // Skip zones created by source/target nodes
        if (zone.blockedBy == candidate.from || zone.blockedBy == candidate.to) {
            continue;
        }

        // Check snap points
        if (isPointInZone(candidate.sourcePoint, zone)) {
            violations++;
        }
        if (isPointInZone(candidate.targetPoint, zone)) {
            violations++;
        }

        // Check bendPoints
        for (const auto& bend : candidate.bendPoints) {
            if (isPointInZone(bend.position, zone)) {
                violations++;
            }
        }

        // Check path segments
        std::vector<std::pair<Point, Point>> segments;
        candidate.forEachSegment([&](const Point& p1, const Point& p2) {
            segments.emplace_back(p1, p2);
        });

        for (const auto& [p1, p2] : segments) {
            if (segmentIntersectsZone(p1, p2, zone)) {
                violations++;
                break;
            }
        }
    }

    return violations * defaultWeight();
}

// === PathIntersectionPenalty ===

int PathIntersectionPenalty::calculatePenalty(
    const EdgeLayout& candidate,
    const PenaltyContext& context) const {

    int totalIntersections = 0;

    for (const auto& [edgeId, other] : context.assignedLayouts) {
        totalIntersections += PathIntersection::countPathIntersections(candidate, other);
    }

    return totalIntersections * defaultWeight();
}

// === OrthogonalityPenalty ===

int OrthogonalityPenalty::calculatePenalty(
    const EdgeLayout& candidate,
    const PenaltyContext& /*context*/) const {

    constexpr float DIRECTION_TOLERANCE = 1.0f;

    // Build path points: source → bendPoints → target
    std::vector<Point> path;
    path.push_back(candidate.sourcePoint);
    for (const auto& bp : candidate.bendPoints) {
        path.push_back(bp.position);
    }
    path.push_back(candidate.targetPoint);

    // Need at least 2 points
    if (path.size() < 2) {
        return defaultWeight();  // Invalid path is non-orthogonal
    }

    // Check each segment for orthogonality
    for (size_t i = 0; i + 1 < path.size(); ++i) {
        const Point& p1 = path[i];
        const Point& p2 = path[i + 1];

        float dx = std::abs(p2.x - p1.x);
        float dy = std::abs(p2.y - p1.y);

        bool isHorizontal = (dy < DIRECTION_TOLERANCE);
        bool isVertical = (dx < DIRECTION_TOLERANCE);

        if (!isHorizontal && !isVertical) {
            return defaultWeight();  // Non-orthogonal segment found
        }
    }

    return 0;  // All segments are orthogonal
}

// === FixedEndpointPenalty ===

int FixedEndpointPenalty::calculatePenalty(
    const EdgeLayout& candidate,
    const PenaltyContext& context) const {
    
    // If no constraints are set, no penalty
    if (context.movedNodes.empty() || context.originalLayouts == nullptr) {
        return 0;
    }
    
    // Find original layout for this edge
    auto originalIt = context.originalLayouts->find(candidate.id);
    if (originalIt == context.originalLayouts->end()) {
        return 0;  // No original layout to compare against
    }
    const EdgeLayout& original = originalIt->second;
    
    // Single violation flag - any violation returns HARD_CONSTRAINT_PENALTY
    // This is the SINGLE SOURCE OF TRUTH for endpoint constraint checking
    
    // Check source endpoint - if source node is fixed, edge AND position must match
    if (context.isEndpointFixed(candidate.from)) {
        // Edge direction must match
        if (candidate.sourceEdge != original.sourceEdge) {
            return defaultWeight();
        }
        // Position must match within tolerance
        float srcDx = std::abs(candidate.sourcePoint.x - original.sourcePoint.x);
        float srcDy = std::abs(candidate.sourcePoint.y - original.sourcePoint.y);
        if (srcDx > tolerance_ || srcDy > tolerance_) {
            return defaultWeight();
        }
    }
    
    // Check target endpoint - if target node is fixed, edge AND position must match
    if (context.isEndpointFixed(candidate.to)) {
        // Edge direction must match
        if (candidate.targetEdge != original.targetEdge) {
            return defaultWeight();
        }
        // Position must match within tolerance
        float tgtDx = std::abs(candidate.targetPoint.x - original.targetPoint.x);
        float tgtDy = std::abs(candidate.targetPoint.y - original.targetPoint.y);
        if (tgtDx > tolerance_ || tgtDy > tolerance_) {
            return defaultWeight();
        }
    }
    
    return 0;
}

// === DirectionalPenetrationPenalty ===

bool DirectionalPenetrationPenalty::segmentPenetratesNode(
    const Point& p1, const Point& p2,
    const NodeLayout& node) const {
    
    // Node AABB bounds (shrunk by tolerance to allow edge touching)
    float nodeLeft = node.position.x + tolerance_;
    float nodeRight = node.position.x + node.size.width - tolerance_;
    float nodeTop = node.position.y + tolerance_;
    float nodeBottom = node.position.y + node.size.height - tolerance_;
    
    // Segment bounding box
    float segLeft = std::min(p1.x, p2.x);
    float segRight = std::max(p1.x, p2.x);
    float segTop = std::min(p1.y, p2.y);
    float segBottom = std::max(p1.y, p2.y);
    
    // Quick AABB rejection test
    if (segRight < nodeLeft || segLeft > nodeRight ||
        segBottom < nodeTop || segTop > nodeBottom) {
        return false;
    }
    
    // For orthogonal segments, AABB overlap means intersection
    bool isHorizontal = std::abs(p2.y - p1.y) < tolerance_;
    bool isVertical = std::abs(p2.x - p1.x) < tolerance_;
    
    if (isHorizontal) {
        return p1.y > nodeTop && p1.y < nodeBottom &&
               segRight > nodeLeft && segLeft < nodeRight;
    }
    
    if (isVertical) {
        return p1.x > nodeLeft && p1.x < nodeRight &&
               segBottom > nodeTop && segTop < nodeBottom;
    }
    
    // Diagonal segment - assume penetration
    return true;
}

int DirectionalPenetrationPenalty::calculatePenalty(
    const EdgeLayout& candidate,
    const PenaltyContext& context) const {
    
    // Build path: source -> bendPoints -> target
    std::vector<Point> path;
    path.push_back(candidate.sourcePoint);
    for (const auto& bp : candidate.bendPoints) {
        path.push_back(bp.position);
    }
    path.push_back(candidate.targetPoint);
    
    // Find source and target node layouts
    auto srcIt = context.nodeLayouts.find(candidate.from);
    auto tgtIt = context.nodeLayouts.find(candidate.to);
    
    // Check source penetration (skip first segment i=0)
    if (srcIt != context.nodeLayouts.end()) {
        const NodeLayout& srcNode = srcIt->second;
        for (size_t i = 1; i + 1 < path.size(); ++i) {
            if (segmentPenetratesNode(path[i], path[i + 1], srcNode)) {
                return defaultWeight();  // Intermediate segment penetrates source
            }
        }
    }
    
    // Check target penetration (skip last segment)
    if (tgtIt != context.nodeLayouts.end()) {
        const NodeLayout& tgtNode = tgtIt->second;
        size_t lastSegmentIndex = path.size() >= 2 ? path.size() - 2 : 0;
        for (size_t i = 0; i < lastSegmentIndex; ++i) {
            if (segmentPenetratesNode(path[i], path[i + 1], tgtNode)) {
                return defaultWeight();  // Intermediate segment penetrates target
            }
        }
    }
    
    return 0;  // No directional penetration
}

}  // namespace arborvia
