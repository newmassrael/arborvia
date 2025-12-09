#include "layout/routing/EdgeNudger.h"

#include <algorithm>
#include <cmath>
#include <unordered_map>

namespace arborvia {

namespace {
    constexpr float COORDINATE_TOLERANCE = 1.0f;  // Tolerance for same coordinate
}

EdgeNudger::EdgeNudger()
    : config_() {}

EdgeNudger::EdgeNudger(const Config& config)
    : config_(config) {}

std::vector<EdgeNudger::Segment> EdgeNudger::buildSegments(
    const std::unordered_map<EdgeId, EdgeLayout>& edgeLayouts) const {

    std::vector<Segment> segments;

    for (const auto& [edgeId, layout] : edgeLayouts) {
        // Build path points: source -> bendPoints -> target
        std::vector<Point> path;
        path.push_back(layout.sourcePoint);
        for (const auto& bp : layout.bendPoints) {
            path.push_back(bp.position);
        }
        path.push_back(layout.targetPoint);

        // Extract segments
        for (size_t i = 0; i + 1 < path.size(); ++i) {
            const Point& p1 = path[i];
            const Point& p2 = path[i + 1];

            // Calculate segment length
            float dx = std::abs(p2.x - p1.x);
            float dy = std::abs(p2.y - p1.y);
            float length = dx + dy;  // Manhattan length for orthogonal

            // Skip very short segments
            if (length < config_.minSegmentLength) {
                continue;
            }

            Segment seg;
            seg.edgeId = edgeId;
            seg.segmentIndex = i;
            seg.p1 = p1;
            seg.p2 = p2;
            seg.isVertical = (dx < COORDINATE_TOLERANCE);
            seg.coordinate = seg.isVertical ? p1.x : p1.y;

            segments.push_back(seg);
        }
    }

    return segments;
}

bool EdgeNudger::segmentsOverlap(const Segment& a, const Segment& b) const {
    // Must be same orientation
    if (a.isVertical != b.isVertical) {
        return false;
    }

    // Must be same coordinate (within tolerance)
    if (std::abs(a.coordinate - b.coordinate) > COORDINATE_TOLERANCE) {
        return false;
    }

    // Check range overlap
    if (a.isVertical) {
        // Vertical segments: check Y range overlap
        float aMinY = std::min(a.p1.y, a.p2.y);
        float aMaxY = std::max(a.p1.y, a.p2.y);
        float bMinY = std::min(b.p1.y, b.p2.y);
        float bMaxY = std::max(b.p1.y, b.p2.y);

        // Overlap if ranges intersect (not just touch)
        return (aMinY < bMaxY - COORDINATE_TOLERANCE) &&
               (bMinY < aMaxY - COORDINATE_TOLERANCE);
    } else {
        // Horizontal segments: check X range overlap
        float aMinX = std::min(a.p1.x, a.p2.x);
        float aMaxX = std::max(a.p1.x, a.p2.x);
        float bMinX = std::min(b.p1.x, b.p2.x);
        float bMaxX = std::max(b.p1.x, b.p2.x);

        return (aMinX < bMaxX - COORDINATE_TOLERANCE) &&
               (bMinX < aMaxX - COORDINATE_TOLERANCE);
    }
}

std::vector<EdgeNudger::OverlapGroup> EdgeNudger::groupByCoordinate(
    const std::vector<Segment>& segments) const {

    std::vector<OverlapGroup> groups;

    // Simple O(n^2) grouping - acceptable for typical edge counts
    std::vector<bool> processed(segments.size(), false);

    for (size_t i = 0; i < segments.size(); ++i) {
        if (processed[i]) {
            continue;
        }

        const Segment& base = segments[i];

        // Find all segments that overlap with base
        OverlapGroup group;
        group.isVertical = base.isVertical;
        group.coordinate = base.coordinate;
        group.segments.push_back(base);
        processed[i] = true;

        for (size_t j = i + 1; j < segments.size(); ++j) {
            if (processed[j]) {
                continue;
            }

            // Check if this segment overlaps with any segment in the group
            bool overlapsWithGroup = false;
            for (const Segment& groupSeg : group.segments) {
                if (segmentsOverlap(groupSeg, segments[j])) {
                    overlapsWithGroup = true;
                    break;
                }
            }

            if (overlapsWithGroup) {
                group.segments.push_back(segments[j]);
                processed[j] = true;
            }
        }

        // Only keep groups with 2+ segments (actual overlaps)
        if (group.segments.size() > 1) {
            groups.push_back(std::move(group));
        }
    }

    return groups;
}

float EdgeNudger::calculateOffset(size_t indexInGroup, size_t groupSize) const {
    if (groupSize <= 1) {
        return 0.0f;
    }

    // Distribute segments evenly around the center
    // For 2 segments: -offset/2, +offset/2
    // For 3 segments: -offset, 0, +offset
    // etc.
    float totalWidth = config_.nudgeOffset * (groupSize - 1);
    float startOffset = -totalWidth / 2.0f;

    return startOffset + indexInGroup * config_.nudgeOffset;
}

std::vector<EdgeNudger::OverlapGroup> EdgeNudger::detectOverlaps(
    const std::unordered_map<EdgeId, EdgeLayout>& edgeLayouts) const {

    auto segments = buildSegments(edgeLayouts);
    return groupByCoordinate(segments);
}

EdgeNudger::Result EdgeNudger::applyNudging(
    const std::unordered_map<EdgeId, EdgeLayout>& edgeLayouts) const {

    Result result;

    if (!config_.enabled) {
        return result;
    }

    // Detect overlaps
    auto segments = buildSegments(edgeLayouts);
    result.overlapGroups = groupByCoordinate(segments);
    result.totalOverlaps = static_cast<int>(result.overlapGroups.size());

    // Apply nudging to each group
    for (const auto& group : result.overlapGroups) {
        for (size_t i = 0; i < group.segments.size(); ++i) {
            const Segment& seg = group.segments[i];
            float offset = calculateOffset(i, group.segments.size());

            NudgedSegment nudged;
            nudged.edgeId = seg.edgeId;
            nudged.segmentIndex = seg.segmentIndex;

            if (group.isVertical) {
                // Vertical segment: nudge X coordinate
                nudged.p1 = {seg.p1.x + offset, seg.p1.y};
                nudged.p2 = {seg.p2.x + offset, seg.p2.y};
            } else {
                // Horizontal segment: nudge Y coordinate
                nudged.p1 = {seg.p1.x, seg.p1.y + offset};
                nudged.p2 = {seg.p2.x, seg.p2.y + offset};
            }

            result.nudgedSegments.push_back(nudged);
        }
    }

    return result;
}

}  // namespace arborvia
