#include "EdgeScorer.h"
#include "PathIntersection.h"
#include "arborvia/core/GeometryUtils.h"

#include <cmath>
#include <algorithm>

namespace arborvia {

using constants::EPSILON;

EdgeScorer::EdgeScorer(const ScoringWeights& weights)
    : weights_(weights) {}

int EdgeScorer::calculateScore(
    const EdgeLayout& candidate,
    const std::unordered_map<EdgeId, EdgeLayout>& assignedLayouts,
    const std::unordered_map<NodeId, NodeLayout>& nodeLayouts) const {

    int score = 0;

    // tooCloseSnap (100,000)
    score += checkSnapPointProximity(candidate) * weights_.tooCloseSnap;

    // selfOverlap (50,000)
    score += checkSelfOverlap(candidate) * weights_.selfOverlap;

    // nodeCollisions (10,000)
    score += countNodeCollisions(candidate, nodeLayouts) * weights_.nodeCollision;

    // pathIntersections (1,000)
    score += countPathIntersections(candidate, assignedLayouts) * weights_.pathIntersection;

    // distance (1)
    score += static_cast<int>(calculateDistance(candidate));

    return score;
}

int EdgeScorer::checkSnapPointProximity(const EdgeLayout& layout) const {
    // Check if source and target snap points are too close
    // This applies when both edges exit in similar directions
    
    const float dx = std::abs(layout.sourcePoint.x - layout.targetPoint.x);
    const float dy = std::abs(layout.sourcePoint.y - layout.targetPoint.y);

    // Determine edge orientations
    const bool sourceIsVertical = (layout.sourceEdge == NodeEdge::Top || 
                                    layout.sourceEdge == NodeEdge::Bottom);
    const bool targetIsVertical = (layout.targetEdge == NodeEdge::Top || 
                                    layout.targetEdge == NodeEdge::Bottom);

    // Only check when both edges have same orientation
    // (both horizontal or both vertical)
    if (!sourceIsVertical && !targetIsVertical) {
        // Both horizontal (Left/Right): check if too close
        if (dx < weights_.minSnapDistance && dy < weights_.minSnapDistance) {
            return 1;
        }
    } else if (sourceIsVertical && targetIsVertical) {
        // Both vertical (Top/Bottom): check if too close
        if (dy < weights_.minSnapDistance && dx < weights_.minSnapDistance) {
            return 1;
        }
    }

    return 0;
}

int EdgeScorer::checkSelfOverlap(const EdgeLayout& layout) const {
    // Check for MIN_SEGMENT self-overlap
    // This happens when the path would need to backtrack
    
    const float MIN_SEGMENT = weights_.minSegmentLength;

    const bool sourceIsVertical = (layout.sourceEdge == NodeEdge::Top || 
                                    layout.sourceEdge == NodeEdge::Bottom);
    const bool targetIsVertical = (layout.targetEdge == NodeEdge::Top || 
                                    layout.targetEdge == NodeEdge::Bottom);

    // Case 1: Both horizontal edges (H-V-H path)
    if (!sourceIsVertical && !targetIsVertical) {
        // Calculate intermediate points after MIN_SEGMENT
        float x1; // After source MIN_SEGMENT
        if (layout.sourceEdge == NodeEdge::Right) {
            x1 = layout.sourcePoint.x + MIN_SEGMENT;
        } else { // Left
            x1 = layout.sourcePoint.x - MIN_SEGMENT;
        }

        float x2; // Before target MIN_SEGMENT
        if (layout.targetEdge == NodeEdge::Right) {
            x2 = layout.targetPoint.x + MIN_SEGMENT;
        } else { // Left
            x2 = layout.targetPoint.x - MIN_SEGMENT;
        }

        // Detect overlap
        if (layout.targetEdge == NodeEdge::Left) {
            // Target MIN_SEGMENT goes left: x1 must be left of x2
            if (x1 > x2) return 1;
        } else { // Right
            // Target MIN_SEGMENT goes right: x1 must be right of x2
            if (x1 < x2) return 1;
        }
    }

    // Case 2: Both vertical edges (V-H-V path)
    if (sourceIsVertical && targetIsVertical) {
        float y1; // After source MIN_SEGMENT
        if (layout.sourceEdge == NodeEdge::Top) {
            y1 = layout.sourcePoint.y - MIN_SEGMENT;
        } else { // Bottom
            y1 = layout.sourcePoint.y + MIN_SEGMENT;
        }

        float y2; // Before target MIN_SEGMENT
        if (layout.targetEdge == NodeEdge::Top) {
            y2 = layout.targetPoint.y - MIN_SEGMENT;
        } else { // Bottom
            y2 = layout.targetPoint.y + MIN_SEGMENT;
        }

        // Detect overlap
        if (layout.targetEdge == NodeEdge::Top) {
            // Target MIN_SEGMENT goes up: y1 must be above y2
            if (y1 > y2) return 1;
        } else { // Bottom
            // Target MIN_SEGMENT goes down: y1 must be below y2
            if (y1 < y2) return 1;
        }
    }

    // Case 3: Mixed edges (H-V or V-H) - no self-overlap possible
    return 0;
}

int EdgeScorer::countNodeCollisions(
    const EdgeLayout& layout,
    const std::unordered_map<NodeId, NodeLayout>& nodeLayouts) const {

    int collisions = 0;

    // Collect all segments
    std::vector<std::pair<Point, Point>> segments;
    layout.forEachSegment([&](const Point& p1, const Point& p2) {
        segments.emplace_back(p1, p2);
    });

    // Check each segment against each node
    for (const auto& [nodeId, node] : nodeLayouts) {
        // Skip source and target nodes
        if (nodeId == layout.from || nodeId == layout.to) {
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
        // Check source node (skip first segment)
        auto srcIt = nodeLayouts.find(layout.from);
        if (srcIt != nodeLayouts.end()) {
            for (size_t i = 1; i < segments.size(); ++i) {
                const auto& [p1, p2] = segments[i];
                if (segmentIntersectsNode(p1, p2, srcIt->second)) {
                    ++collisions;
                    break;
                }
            }
        }

        // Check target node (skip last segment)
        auto tgtIt = nodeLayouts.find(layout.to);
        if (tgtIt != nodeLayouts.end()) {
            for (size_t i = 0; i + 1 < segments.size(); ++i) {
                const auto& [p1, p2] = segments[i];
                if (segmentIntersectsNode(p1, p2, tgtIt->second)) {
                    ++collisions;
                    break;
                }
            }
        }
    }

    return collisions;
}

int EdgeScorer::countPathIntersections(
    const EdgeLayout& layout,
    const std::unordered_map<EdgeId, EdgeLayout>& otherLayouts) const {

    int totalIntersections = 0;

    for (const auto& [edgeId, other] : otherLayouts) {
        totalIntersections += PathIntersection::countPathIntersections(layout, other);
    }

    return totalIntersections;
}

float EdgeScorer::calculateDistance(const EdgeLayout& layout) const {
    return layout.sourcePoint.distanceTo(layout.targetPoint);
}

bool EdgeScorer::segmentIntersectsNode(
    const Point& p1, const Point& p2,
    const NodeLayout& node, float margin) {

    // Expand node bounds by margin
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


}  // namespace arborvia
