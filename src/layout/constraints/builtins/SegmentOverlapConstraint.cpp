#include "arborvia/layout/constraints/builtins/SegmentOverlapConstraint.h"
#include <algorithm>
#include <cmath>
#include <sstream>

namespace arborvia {

bool SegmentOverlapConstraint::segmentsOverlap(
    const Point& a1, const Point& a2,
    const Point& b1, const Point& b2,
    float tolerance) const {
    
    // Check if both segments are collinear (on same line)
    bool aHorizontal = std::abs(a2.y - a1.y) < tolerance;
    bool aVertical = std::abs(a2.x - a1.x) < tolerance;
    bool bHorizontal = std::abs(b2.y - b1.y) < tolerance;
    bool bVertical = std::abs(b2.x - b1.x) < tolerance;
    
    // Both must be same orientation
    if (aHorizontal && bHorizontal) {
        // Both horizontal - check if on same y and x ranges overlap
        if (std::abs(a1.y - b1.y) > tolerance) {
            return false;  // Different y levels
        }
        
        float aLeft = std::min(a1.x, a2.x);
        float aRight = std::max(a1.x, a2.x);
        float bLeft = std::min(b1.x, b2.x);
        float bRight = std::max(b1.x, b2.x);
        
        // Check for overlap in x range (more than just touching at endpoint)
        float overlapStart = std::max(aLeft, bLeft);
        float overlapEnd = std::min(aRight, bRight);
        
        return overlapEnd - overlapStart > tolerance;
    }
    
    if (aVertical && bVertical) {
        // Both vertical - check if on same x and y ranges overlap
        if (std::abs(a1.x - b1.x) > tolerance) {
            return false;  // Different x levels
        }
        
        float aTop = std::min(a1.y, a2.y);
        float aBottom = std::max(a1.y, a2.y);
        float bTop = std::min(b1.y, b2.y);
        float bBottom = std::max(b1.y, b2.y);
        
        // Check for overlap in y range
        float overlapStart = std::max(aTop, bTop);
        float overlapEnd = std::min(aBottom, bBottom);
        
        return overlapEnd - overlapStart > tolerance;
    }
    
    return false;  // Different orientations or diagonal - no collinear overlap
}

std::vector<ConstraintViolation> SegmentOverlapConstraint::check(
    const EdgeLayout& layout,
    const EdgeConstraintContext& ctx) const {
    
    std::vector<ConstraintViolation> violations;
    
    // Build this edge's path
    std::vector<Point> path;
    path.push_back(layout.sourcePoint);
    for (const auto& bp : layout.bendPoints) {
        path.push_back(bp.position);
    }
    path.push_back(layout.targetPoint);
    
    // Compare with all other edges
    for (const auto& [otherEdgeId, otherLayout] : ctx.allEdgeLayouts) {
        if (otherEdgeId == layout.id) {
            continue;  // Skip self
        }
        
        // Build other edge's path
        std::vector<Point> otherPath;
        otherPath.push_back(otherLayout.sourcePoint);
        for (const auto& bp : otherLayout.bendPoints) {
            otherPath.push_back(bp.position);
        }
        otherPath.push_back(otherLayout.targetPoint);
        
        // Check each segment pair
        for (size_t i = 0; i + 1 < path.size(); ++i) {
            for (size_t j = 0; j + 1 < otherPath.size(); ++j) {
                if (segmentsOverlap(path[i], path[i + 1], 
                                    otherPath[j], otherPath[j + 1],
                                    ctx.tolerance)) {
                    ConstraintViolation v;
                    v.type = ConstraintViolationType::SegmentOverlap;
                    v.edgeId = layout.id;
                    v.otherEdgeId = otherEdgeId;
                    v.segmentIndex = static_cast<int>(i);
                    
                    std::ostringstream oss;
                    oss << "Segment " << i << " overlaps with edge " << otherEdgeId 
                        << " segment " << j;
                    v.message = oss.str();
                    
                    violations.push_back(std::move(v));
                }
            }
        }
    }
    
    return violations;
}

} // namespace arborvia
