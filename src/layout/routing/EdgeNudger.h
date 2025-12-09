#pragma once

#include "arborvia/core/Types.h"
#include "arborvia/layout/config/LayoutResult.h"

#include <unordered_map>
#include <vector>

namespace arborvia {

/// Visual separation of overlapping edge segments
/// 
/// EdgeNudger provides a post-processing step that visually separates
/// segments that share the same coordinate. This is the final fallback
/// when Rip-up and Reroute cannot eliminate all overlaps.
///
/// Key characteristics:
/// - Does NOT change logical path (bendPoints remain unchanged)
/// - Only affects rendering positions (visual offset)
/// - Applied after all routing optimization is complete
class EdgeNudger {
public:
    /// Configuration for nudging
    struct Config {
        float nudgeOffset = 2.0f;      ///< Pixel offset between overlapping segments
        float minSegmentLength = 10.0f; ///< Minimum segment length to nudge
        bool enabled = true;            ///< Enable/disable nudging
        
        /// Default constructor
        Config() = default;
    };

    /// Segment representation for overlap detection
    struct Segment {
        EdgeId edgeId = INVALID_EDGE;
        size_t segmentIndex = 0;  ///< Index within the edge's path
        Point p1;
        Point p2;
        bool isVertical = false;
        float coordinate = 0.0f;     ///< x for vertical, y for horizontal
    };

    /// Group of overlapping segments
    struct OverlapGroup {
        bool isVertical = false;
        float coordinate = 0.0f;
        std::vector<Segment> segments;
    };

    /// Nudged segment result (for rendering)
    struct NudgedSegment {
        EdgeId edgeId = INVALID_EDGE;
        size_t segmentIndex = 0;
        Point p1;  ///< Nudged start point
        Point p2;  ///< Nudged end point
    };

    /// Result of nudging operation
    struct Result {
        std::vector<OverlapGroup> overlapGroups;  ///< Detected overlap groups
        std::vector<NudgedSegment> nudgedSegments; ///< Segments with visual offset
        int totalOverlaps = 0;                     ///< Number of overlap groups found
    };

    /// Default constructor
    EdgeNudger();
    
    /// Constructor with configuration
    explicit EdgeNudger(const Config& config);

    /// Detect overlapping segments without modifying layouts
    /// @param edgeLayouts Edge layouts to analyze
    /// @return Vector of overlap groups
    std::vector<OverlapGroup> detectOverlaps(
        const std::unordered_map<EdgeId, EdgeLayout>& edgeLayouts) const;

    /// Apply nudging and return visual positions
    /// Does NOT modify the original layouts, returns nudged positions for rendering
    /// @param edgeLayouts Edge layouts to process
    /// @return Result with overlap groups and nudged segment positions
    Result applyNudging(
        const std::unordered_map<EdgeId, EdgeLayout>& edgeLayouts) const;

    /// Get configuration
    const Config& config() const { return config_; }

    /// Set configuration
    void setConfig(const Config& config) { config_ = config; }

private:
    Config config_;

    /// Build all segments from edge layouts
    std::vector<Segment> buildSegments(
        const std::unordered_map<EdgeId, EdgeLayout>& edgeLayouts) const;

    /// Group segments by coordinate (find overlaps)
    std::vector<OverlapGroup> groupByCoordinate(
        const std::vector<Segment>& segments) const;

    /// Check if two segments overlap (share coordinate range)
    bool segmentsOverlap(const Segment& a, const Segment& b) const;

    /// Calculate nudge offset for a segment in a group
    float calculateOffset(size_t indexInGroup, size_t groupSize) const;
};

}  // namespace arborvia
