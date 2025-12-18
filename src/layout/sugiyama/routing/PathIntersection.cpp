#include "PathIntersection.h"
#include "arborvia/core/GeometryUtils.h"

#include <cmath>
#include <algorithm>

namespace arborvia {

using constants::EPSILON;

namespace PathIntersection {

namespace {
    /// Counter-Clockwise orientation test
    /// @return >0 if CCW (left turn), <0 if CW (right turn), =0 if collinear
    float ccw(const Point& a, const Point& b, const Point& c) {
        // Cross product of vectors (b-a) and (c-a)
        // Positive = counter-clockwise, Negative = clockwise, Zero = collinear
        return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
    }

    /// Check if point c lies on segment ab (when collinear)
    bool onSegment(const Point& a, const Point& b, const Point& c) {
        // Check if point c lies on segment ab (assumes collinearity)
        return c.x >= std::min(a.x, b.x) - EPSILON &&
               c.x <= std::max(a.x, b.x) + EPSILON &&
               c.y >= std::min(a.y, b.y) - EPSILON &&
               c.y <= std::max(a.y, b.y) + EPSILON;
    }
}  // anonymous namespace

bool segmentsIntersect(const Point& a1, const Point& a2,
                       const Point& b1, const Point& b2) {
    // Skip if segments share an endpoint (adjacent segments in path)
    if ((a1 == b1) || (a1 == b2) || (a2 == b1) || (a2 == b2)) {
        return false;
    }

    float d1 = ccw(b1, b2, a1);
    float d2 = ccw(b1, b2, a2);
    float d3 = ccw(a1, a2, b1);
    float d4 = ccw(a1, a2, b2);

    // General case: segments straddle each other
    if (((d1 > EPSILON && d2 < -EPSILON) || (d1 < -EPSILON && d2 > EPSILON)) &&
        ((d3 > EPSILON && d4 < -EPSILON) || (d3 < -EPSILON && d4 > EPSILON))) {
        return true;
    }

    // Collinear cases: check if endpoints lie on the other segment
    if (std::abs(d1) <= EPSILON && onSegment(b1, b2, a1)) return true;
    if (std::abs(d2) <= EPSILON && onSegment(b1, b2, a2)) return true;
    if (std::abs(d3) <= EPSILON && onSegment(a1, a2, b1)) return true;
    if (std::abs(d4) <= EPSILON && onSegment(a1, a2, b2)) return true;

    return false;
}

int countPathIntersections(const EdgeLayout& e1, const EdgeLayout& e2) {
    int count = 0;

    // Build segment lists for both edges
    std::vector<std::pair<Point, Point>> segments1, segments2;

    // Collect segments from edge 1
    e1.forEachSegment([&](const Point& p1, const Point& p2) {
        segments1.emplace_back(p1, p2);
    });

    // Collect segments from edge 2
    e2.forEachSegment([&](const Point& p1, const Point& p2) {
        segments2.emplace_back(p1, p2);
    });

    // Compare all segment pairs
    for (const auto& [a1, a2] : segments1) {
        for (const auto& [b1, b2] : segments2) {
            if (segmentsIntersect(a1, a2, b1, b2)) {
                ++count;
            }
        }
    }

    return count;
}

int countIntersectionsWithOthers(
    const EdgeLayout& edge,
    const std::unordered_map<EdgeId, EdgeLayout>& otherEdges,
    EdgeId excludeEdgeId) {

    int total = 0;

    for (const auto& [edgeId, otherLayout] : otherEdges) {
        if (edgeId == excludeEdgeId) {
            continue;
        }
        total += countPathIntersections(edge, otherLayout);
    }

    return total;
}

// =============================================================================
// Segment Overlap Detection
// =============================================================================

bool segmentsOverlap(const Point& a1, const Point& a2,
                     const Point& b1, const Point& b2) {
    // Check if segments are collinear (on the same line)
    float d1 = ccw(a1, a2, b1);
    float d2 = ccw(a1, a2, b2);

    // If not collinear, no overlap possible
    if (std::abs(d1) > EPSILON || std::abs(d2) > EPSILON) {
        return false;
    }

    // Segments are collinear - check if they overlap
    // For horizontal segments (same Y): check X range overlap
    bool isHorizontal = std::abs(a1.y - a2.y) < EPSILON;
    // For vertical segments (same X): check Y range overlap
    bool isVertical = std::abs(a1.x - a2.x) < EPSILON;

    if (isHorizontal) {
        // Both segments horizontal on same Y
        if (std::abs(a1.y - b1.y) > EPSILON) {
            return false;  // Different Y lines
        }
        // Check X range overlap
        float aMinX = std::min(a1.x, a2.x);
        float aMaxX = std::max(a1.x, a2.x);
        float bMinX = std::min(b1.x, b2.x);
        float bMaxX = std::max(b1.x, b2.x);

        // Overlap exists if ranges intersect (more than just touching at endpoint)
        return (aMinX < bMaxX - EPSILON) && (bMinX < aMaxX - EPSILON);
    }

    if (isVertical) {
        // Both segments vertical on same X
        if (std::abs(a1.x - b1.x) > EPSILON) {
            return false;  // Different X lines
        }
        // Check Y range overlap
        float aMinY = std::min(a1.y, a2.y);
        float aMaxY = std::max(a1.y, a2.y);
        float bMinY = std::min(b1.y, b2.y);
        float bMaxY = std::max(b1.y, b2.y);

        // Overlap exists if ranges intersect (more than just touching at endpoint)
        return (aMinY < bMaxY - EPSILON) && (bMinY < aMaxY - EPSILON);
    }

    // Diagonal segments (non-orthogonal) - check parametric overlap
    // Project both segments onto their shared line and check overlap
    float dx = a2.x - a1.x;
    float dy = a2.y - a1.y;
    float len = std::sqrt(dx * dx + dy * dy);
    if (len < EPSILON) return false;

    // Project b1 and b2 onto line (a1, a2)
    auto project = [&](const Point& p) -> float {
        return ((p.x - a1.x) * dx + (p.y - a1.y) * dy) / (len * len);
    };

    float t1 = 0.0f;  // a1 is at t=0
    float t2 = 1.0f;  // a2 is at t=1
    float t3 = project(b1);
    float t4 = project(b2);

    float aMin = std::min(t1, t2);
    float aMax = std::max(t1, t2);
    float bMin = std::min(t3, t4);
    float bMax = std::max(t3, t4);

    return (aMin < bMax - EPSILON) && (bMin < aMax - EPSILON);
}

bool hasSegmentOverlap(const EdgeLayout& e1, const EdgeLayout& e2) {
    // Build segment lists for both edges
    std::vector<std::pair<Point, Point>> segments1, segments2;

    e1.forEachSegment([&](const Point& p1, const Point& p2) {
        segments1.emplace_back(p1, p2);
    });

    e2.forEachSegment([&](const Point& p1, const Point& p2) {
        segments2.emplace_back(p1, p2);
    });

    // Check all segment pairs for overlap
    for (const auto& [a1, a2] : segments1) {
        for (const auto& [b1, b2] : segments2) {
            if (segmentsOverlap(a1, a2, b1, b2)) {
                return true;
            }
        }
    }

    return false;
}

bool hasSegmentOverlapExcludingLast(const EdgeLayout& e1, const EdgeLayout& e2, int excludeLastN) {
    // Build segment lists for both edges
    std::vector<std::pair<Point, Point>> segments1, segments2;

    e1.forEachSegment([&](const Point& p1, const Point& p2) {
        segments1.emplace_back(p1, p2);
    });

    e2.forEachSegment([&](const Point& p1, const Point& p2) {
        segments2.emplace_back(p1, p2);
    });

    // Determine how many segments to check (excluding last N from each)
    int check1 = std::max(0, static_cast<int>(segments1.size()) - excludeLastN);
    int check2 = std::max(0, static_cast<int>(segments2.size()) - excludeLastN);

    // Check segment pairs, excluding last N segments from each edge
    for (int i = 0; i < check1; ++i) {
        for (int j = 0; j < check2; ++j) {
            if (segmentsOverlap(segments1[i].first, segments1[i].second,
                                segments2[j].first, segments2[j].second)) {
                return true;
            }
        }
    }

    return false;
}

bool hasOverlapWithOthers(
    const EdgeLayout& edge,
    const std::unordered_map<EdgeId, EdgeLayout>& otherEdges,
    EdgeId excludeEdgeId) {

    for (const auto& [edgeId, otherLayout] : otherEdges) {
        if (edgeId == excludeEdgeId || edgeId == edge.id) {
            continue;
        }
        if (hasSegmentOverlap(edge, otherLayout)) {
            return true;
        }
    }

    return false;
}

// =============================================================================
// Overlap Avoidance Helpers
// =============================================================================

namespace {
    /// Get detailed overlap between two segments (returns grid coordinates)
    /// @param gridSize Grid cell size for coordinate conversion (if 0, uses 20.0f as fallback)
    /// Returns overlap info if segments overlap on same line
    OverlapInfo getSegmentOverlapInfoGrid(const Point& a1, const Point& a2,
                                           const Point& b1, const Point& b2,
                                           float gridSize) {
        OverlapInfo info;

        // Use 20.0f as fallback when grid is disabled (gridSize == 0)
        float effectiveGridSize = constants::effectiveGridSize(gridSize);

        bool aHorizontal = std::abs(a1.y - a2.y) < EPSILON;
        bool aVertical = std::abs(a1.x - a2.x) < EPSILON;
        bool bHorizontal = std::abs(b1.y - b2.y) < EPSILON;
        bool bVertical = std::abs(b1.x - b2.x) < EPSILON;

        // Both vertical on same X
        if (aVertical && bVertical && std::abs(a1.x - b1.x) < EPSILON) {
            float aMinY = std::min(a1.y, a2.y);
            float aMaxY = std::max(a1.y, a2.y);
            float bMinY = std::min(b1.y, b2.y);
            float bMaxY = std::max(b1.y, b2.y);

            float overlapStart = std::max(aMinY, bMinY);
            float overlapEnd = std::min(aMaxY, bMaxY);

            if (overlapStart < overlapEnd - EPSILON) {
                info.found = true;
                info.isVertical = true;
                // Convert to grid coordinates
                info.sharedGridCoord = static_cast<int>(std::round(a1.x / effectiveGridSize));
                info.overlapStartGrid = static_cast<int>(std::floor(overlapStart / effectiveGridSize));
                info.overlapEndGrid = static_cast<int>(std::ceil(overlapEnd / effectiveGridSize));
            }
        }
        // Both horizontal on same Y
        else if (aHorizontal && bHorizontal && std::abs(a1.y - b1.y) < EPSILON) {
            float aMinX = std::min(a1.x, a2.x);
            float aMaxX = std::max(a1.x, a2.x);
            float bMinX = std::min(b1.x, b2.x);
            float bMaxX = std::max(b1.x, b2.x);

            float overlapStart = std::max(aMinX, bMinX);
            float overlapEnd = std::min(aMaxX, bMaxX);

            if (overlapStart < overlapEnd - EPSILON) {
                info.found = true;
                info.isVertical = false;
                // Convert to grid coordinates
                info.sharedGridCoord = static_cast<int>(std::round(a1.y / effectiveGridSize));
                info.overlapStartGrid = static_cast<int>(std::floor(overlapStart / effectiveGridSize));
                info.overlapEndGrid = static_cast<int>(std::ceil(overlapEnd / effectiveGridSize));
            }
        }

        return info;
    }
}  // anonymous namespace

OverlapInfo findOverlapInfo(
    const EdgeLayout& candidate,
    const std::unordered_map<EdgeId, EdgeLayout>& assignedLayouts,
    EdgeId excludeEdgeId,
    float gridSize) {

    std::vector<std::pair<Point, Point>> candidateSegments;
    candidate.forEachSegment([&](const Point& p1, const Point& p2) {
        candidateSegments.emplace_back(p1, p2);
    });

    for (const auto& [edgeId, otherLayout] : assignedLayouts) {
        if (edgeId == excludeEdgeId || edgeId == candidate.id) {
            continue;
        }

        std::vector<std::pair<Point, Point>> otherSegments;
        otherLayout.forEachSegment([&](const Point& p1, const Point& p2) {
            otherSegments.emplace_back(p1, p2);
        });

        for (const auto& [a1, a2] : candidateSegments) {
            for (const auto& [b1, b2] : otherSegments) {
                auto info = getSegmentOverlapInfoGrid(a1, a2, b1, b2, gridSize);
                if (info.found) {
                    return info;
                }
            }
        }
    }

    return OverlapInfo{};
}

std::unordered_set<int> findUsedVerticalGridX(
    const std::unordered_map<EdgeId, EdgeLayout>& assignedLayouts,
    int yMinGrid, int yMaxGrid,
    float gridSize) {

    std::unordered_set<int> usedGridX;

    // Use 20.0f as fallback when grid is disabled (gridSize == 0)
    float effectiveGridSize = constants::effectiveGridSize(gridSize);

    for (const auto& [edgeId, layout] : assignedLayouts) {
        layout.forEachSegment([&](const Point& p1, const Point& p2) {
            // Check if vertical segment
            if (std::abs(p1.x - p2.x) < EPSILON) {
                // Convert segment Y range to grid
                int segMinGridY = static_cast<int>(std::floor(std::min(p1.y, p2.y) / effectiveGridSize));
                int segMaxGridY = static_cast<int>(std::ceil(std::max(p1.y, p2.y) / effectiveGridSize));

                // Check if Y ranges overlap
                if (segMinGridY < yMaxGrid && segMaxGridY > yMinGrid) {
                    // Convert X to grid and add (set automatically handles duplicates)
                    int gridX = static_cast<int>(std::round(p1.x / effectiveGridSize));
                    usedGridX.insert(gridX);
                }
            }
        });
    }

    return usedGridX;
}

std::unordered_set<int> findUsedHorizontalGridY(
    const std::unordered_map<EdgeId, EdgeLayout>& assignedLayouts,
    int xMinGrid, int xMaxGrid,
    float gridSize) {

    std::unordered_set<int> usedGridY;

    // Use 20.0f as fallback when grid is disabled (gridSize == 0)
    float effectiveGridSize = constants::effectiveGridSize(gridSize);

    for (const auto& [edgeId, layout] : assignedLayouts) {
        layout.forEachSegment([&](const Point& p1, const Point& p2) {
            // Check if horizontal segment
            if (std::abs(p1.y - p2.y) < EPSILON) {
                // Convert segment X range to grid
                int segMinGridX = static_cast<int>(std::floor(std::min(p1.x, p2.x) / effectiveGridSize));
                int segMaxGridX = static_cast<int>(std::ceil(std::max(p1.x, p2.x) / effectiveGridSize));

                // Check if X ranges overlap
                if (segMinGridX < xMaxGrid && segMaxGridX > xMinGrid) {
                    // Convert Y to grid and add (set automatically handles duplicates)
                    int gridY = static_cast<int>(std::round(p1.y / effectiveGridSize));
                    usedGridY.insert(gridY);
                }
            }
        });
    }

    return usedGridY;
}

// =============================================================================
// Overlap Adjustment
// =============================================================================

std::vector<BendPoint> adjustPathToAvoidOverlap(
    const EdgeLayout& candidate,
    const std::unordered_map<EdgeId, EdgeLayout>& assignedLayouts,
    float gridSpacing) {

    // Use 20.0f as default when grid is disabled (gridSpacing == 0)
    // This provides a reasonable offset for overlap avoidance
    float effectiveGridSize = constants::effectiveGridSize(gridSpacing);

    // Check if there's any overlap (returns grid coordinates)
    auto overlapInfo = findOverlapInfo(candidate, assignedLayouts, candidate.id, effectiveGridSize);
    if (!overlapInfo.found) {
        return candidate.bendPoints;  // No overlap, return as-is
    }

    // Build the full path including source/target
    std::vector<Point> pathPoints;
    pathPoints.push_back(candidate.sourcePoint);
    for (const auto& bp : candidate.bendPoints) {
        pathPoints.push_back(bp.position);
    }
    pathPoints.push_back(candidate.targetPoint);

    // Find and adjust the overlapping segment
    std::vector<BendPoint> adjustedBends;

    // Convert shared coordinate to pixel for comparison
    float sharedPixel = overlapInfo.sharedGridCoord * effectiveGridSize;

    if (overlapInfo.isVertical) {
        // Vertical segment overlap - need to change X coordinate
        // Use grid-based function to find alternative (guaranteed grid-aligned when grid enabled)
        int newGridX = findAlternativeGridX(
            overlapInfo.sharedGridCoord,
            overlapInfo.overlapStartGrid,
            overlapInfo.overlapEndGrid,
            assignedLayouts,
            effectiveGridSize);
        
        // Convert to pixel only at output boundary
        float newX = newGridX * effectiveGridSize;

        // Rebuild bend points with new X
        for (size_t i = 0; i < pathPoints.size() - 1; ++i) {
            const Point& p1 = pathPoints[i];
            const Point& p2 = pathPoints[i + 1];

            // Check if this is a vertical segment at the overlapping X
            if (std::abs(p1.x - sharedPixel) < EPSILON &&
                std::abs(p2.x - sharedPixel) < EPSILON) {
                // This segment needs adjustment - insert offset bend points
                if (i > 0) {
                    // Not starting from source, add transition to new X
                    adjustedBends.push_back({{newX, p1.y}});
                }
                adjustedBends.push_back({{newX, p2.y}});
            } else if (i > 0 && i < pathPoints.size() - 2) {
                // Regular intermediate point
                adjustedBends.push_back({{p2.x, p2.y}});
            }
        }
    } else {
        // Horizontal segment overlap - need to change Y coordinate
        // Use grid-based function to find alternative (guaranteed grid-aligned when grid enabled)
        int newGridY = findAlternativeGridY(
            overlapInfo.sharedGridCoord,
            overlapInfo.overlapStartGrid,
            overlapInfo.overlapEndGrid,
            assignedLayouts,
            effectiveGridSize);
        
        // Convert to pixel only at output boundary
        float newY = newGridY * effectiveGridSize;

        // Rebuild bend points with new Y
        for (size_t i = 0; i < pathPoints.size() - 1; ++i) {
            const Point& p1 = pathPoints[i];
            const Point& p2 = pathPoints[i + 1];

            // Check if this is a horizontal segment at the overlapping Y
            if (std::abs(p1.y - sharedPixel) < EPSILON &&
                std::abs(p2.y - sharedPixel) < EPSILON) {
                // This segment needs adjustment
                if (i > 0) {
                    adjustedBends.push_back({{p1.x, newY}});
                }
                adjustedBends.push_back({{p2.x, newY}});
            } else if (i > 0 && i < pathPoints.size() - 2) {
                adjustedBends.push_back({{p2.x, p2.y}});
            }
        }
    }

    // If adjustment produced valid bends, return them
    if (!adjustedBends.empty()) {
        return adjustedBends;
    }

    // Fallback: return original bends
    return candidate.bendPoints;
}

int findAlternativeGridX(
    int originalGridX,
    int yMinGrid,
    int yMaxGrid,
    const std::unordered_map<EdgeId, EdgeLayout>& assignedLayouts,
    float gridSize) {

    // Find all grid X coordinates used by vertical segments in the Y range
    auto usedGridX = findUsedVerticalGridX(assignedLayouts, yMinGrid, yMaxGrid, gridSize);

    // Try offsets in both directions to find unused grid X
    for (int offset = 1; offset <= 5; ++offset) {
        // Try positive offset first
        int candidateGridX = originalGridX + offset;
        if (usedGridX.find(candidateGridX) == usedGridX.end()) {
            return candidateGridX;
        }

        // Try negative offset
        candidateGridX = originalGridX - offset;
        if (usedGridX.find(candidateGridX) == usedGridX.end()) {
            return candidateGridX;
        }
    }

    // Fallback: use +1 offset
    return originalGridX + 1;
}

int findAlternativeGridY(
    int originalGridY,
    int xMinGrid,
    int xMaxGrid,
    const std::unordered_map<EdgeId, EdgeLayout>& assignedLayouts,
    float gridSize) {

    // Find all grid Y coordinates used by horizontal segments in the X range
    auto usedGridY = findUsedHorizontalGridY(assignedLayouts, xMinGrid, xMaxGrid, gridSize);

    // Try offsets in both directions to find unused grid Y
    for (int offset = 1; offset <= 5; ++offset) {
        // Try positive offset first
        int candidateGridY = originalGridY + offset;
        if (usedGridY.find(candidateGridY) == usedGridY.end()) {
            return candidateGridY;
        }

        // Try negative offset
        candidateGridY = originalGridY - offset;
        if (usedGridY.find(candidateGridY) == usedGridY.end()) {
            return candidateGridY;
        }
    }

    // Fallback: use +1 offset
    return originalGridY + 1;
}

// =========================================================================
// Bulk Overlap Detection (with bounding box optimization)
// =========================================================================

std::vector<std::pair<EdgeId, EdgeId>> findAllOverlappingPairs(
    const std::unordered_map<EdgeId, EdgeLayout>& edgeLayouts) {

    std::vector<std::pair<EdgeId, EdgeId>> overlappingPairs;

    // Pre-calculate bounding boxes for early rejection
    struct EdgeBounds {
        EdgeId id;
        const EdgeLayout* layout;
        float minX, minY, maxX, maxY;
    };

    std::vector<EdgeBounds> edgeBounds;
    edgeBounds.reserve(edgeLayouts.size());

    for (const auto& [id, layout] : edgeLayouts) {
        EdgeBounds bounds;
        bounds.id = id;
        bounds.layout = &layout;

        // Calculate bounding box from all points
        bounds.minX = bounds.maxX = layout.sourcePoint.x;
        bounds.minY = bounds.maxY = layout.sourcePoint.y;

        auto updateBounds = [&](const Point& p) {
            bounds.minX = std::min(bounds.minX, p.x);
            bounds.minY = std::min(bounds.minY, p.y);
            bounds.maxX = std::max(bounds.maxX, p.x);
            bounds.maxY = std::max(bounds.maxY, p.y);
        };

        for (const auto& bp : layout.bendPoints) {
            updateBounds(bp.position);
        }
        updateBounds(layout.targetPoint);

        edgeBounds.push_back(bounds);
    }

    // Check all pairs with bounding box pre-filter
    for (size_t i = 0; i < edgeBounds.size(); ++i) {
        for (size_t j = i + 1; j < edgeBounds.size(); ++j) {
            const EdgeBounds& a = edgeBounds[i];
            const EdgeBounds& b = edgeBounds[j];

            // Early rejection: bounding boxes don't overlap
            if (a.maxX < b.minX || b.maxX < a.minX ||
                a.maxY < b.minY || b.maxY < a.minY) {
                continue;
            }

            // Bounding boxes overlap - do actual segment overlap check
            if (hasSegmentOverlap(*a.layout, *b.layout)) {
                overlappingPairs.emplace_back(a.id, b.id);
            }
        }
    }

    return overlappingPairs;
}

}  // namespace PathIntersection


}  // namespace arborvia
