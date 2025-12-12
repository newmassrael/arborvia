#include "arborvia/layout/constraints/builtins/DirectionalPenetrationConstraint.h"
#include <algorithm>
#include <cmath>
#include <sstream>

namespace arborvia {

// Helper function used by both constraints
static bool segmentPenetratesNodeImpl(
    const Point& p1, const Point& p2,
    const NodeLayout& node,
    float tolerance) {
    
    // Node AABB bounds
    float nodeLeft = node.position.x;
    float nodeRight = node.position.x + node.size.width;
    float nodeTop = node.position.y;
    float nodeBottom = node.position.y + node.size.height;
    
    // Shrink by tolerance to allow edge touching
    nodeLeft += tolerance;
    nodeRight -= tolerance;
    nodeTop += tolerance;
    nodeBottom -= tolerance;
    
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
    bool isHorizontal = std::abs(p2.y - p1.y) < tolerance;
    bool isVertical = std::abs(p2.x - p1.x) < tolerance;
    
    if (isHorizontal) {
        // Horizontal segment intersects if it passes through node's interior
        return p1.y > nodeTop && p1.y < nodeBottom &&
               segRight > nodeLeft && segLeft < nodeRight;
    }
    
    if (isVertical) {
        // Vertical segment intersects if it passes through node's interior
        return p1.x > nodeLeft && p1.x < nodeRight &&
               segBottom > nodeTop && segTop < nodeBottom;
    }
    
    // Diagonal segment - assume penetration (should be caught by OrthogonalityConstraint)
    return true;
}

bool DirectionalSourcePenetrationConstraint::segmentPenetratesNode(
    const Point& p1, const Point& p2,
    const NodeLayout& node,
    float tolerance) const {
    return segmentPenetratesNodeImpl(p1, p2, node, tolerance);
}

std::vector<ConstraintViolation> DirectionalSourcePenetrationConstraint::check(
    const EdgeLayout& layout,
    const EdgeConstraintContext& ctx) const {
    
    std::vector<ConstraintViolation> violations;
    
    // Find source node layout
    auto srcIt = ctx.nodeLayouts.find(layout.from);
    if (srcIt == ctx.nodeLayouts.end()) {
        return violations;  // Can't check without source node info
    }
    const NodeLayout& srcNode = srcIt->second;
    
    // Build path: source -> bendPoints -> target
    std::vector<Point> path;
    path.push_back(layout.sourcePoint);
    for (const auto& bp : layout.bendPoints) {
        path.push_back(bp.position);
    }
    path.push_back(layout.targetPoint);
    
    // Skip first segment (i=0): sourcePoint -> first point is allowed to touch source
    // Check segments starting from i=1
    for (size_t i = 1; i + 1 < path.size(); ++i) {
        const auto& p1 = path[i];
        const auto& p2 = path[i + 1];
        
        if (segmentPenetratesNode(p1, p2, srcNode, ctx.tolerance)) {
            ConstraintViolation v;
            v.type = ConstraintViolationType::DirectionalSourcePenetration;
            v.edgeId = layout.id;
            v.nodeId = layout.from;
            v.segmentIndex = static_cast<int>(i);
            
            std::ostringstream oss;
            oss << "Intermediate segment " << i << " penetrates source node " << layout.from
                << ": (" << p1.x << "," << p1.y << ") -> (" << p2.x << "," << p2.y << ")";
            v.message = oss.str();
            
            violations.push_back(std::move(v));
        }
    }
    
    return violations;
}

bool DirectionalTargetPenetrationConstraint::segmentPenetratesNode(
    const Point& p1, const Point& p2,
    const NodeLayout& node,
    float tolerance) const {
    return segmentPenetratesNodeImpl(p1, p2, node, tolerance);
}

std::vector<ConstraintViolation> DirectionalTargetPenetrationConstraint::check(
    const EdgeLayout& layout,
    const EdgeConstraintContext& ctx) const {
    
    std::vector<ConstraintViolation> violations;
    
    // Find target node layout
    auto tgtIt = ctx.nodeLayouts.find(layout.to);
    if (tgtIt == ctx.nodeLayouts.end()) {
        return violations;  // Can't check without target node info
    }
    const NodeLayout& tgtNode = tgtIt->second;
    
    // Build path: source -> bendPoints -> target
    std::vector<Point> path;
    path.push_back(layout.sourcePoint);
    for (const auto& bp : layout.bendPoints) {
        path.push_back(bp.position);
    }
    path.push_back(layout.targetPoint);
    
    // Skip last segment: last point -> targetPoint is allowed to touch target
    // Check segments from i=0 to i < path.size() - 2 (excluding last segment)
    size_t lastSegmentIndex = path.size() >= 2 ? path.size() - 2 : 0;
    
    for (size_t i = 0; i < lastSegmentIndex; ++i) {
        const auto& p1 = path[i];
        const auto& p2 = path[i + 1];
        
        if (segmentPenetratesNode(p1, p2, tgtNode, ctx.tolerance)) {
            ConstraintViolation v;
            v.type = ConstraintViolationType::DirectionalTargetPenetration;
            v.edgeId = layout.id;
            v.nodeId = layout.to;
            v.segmentIndex = static_cast<int>(i);
            
            std::ostringstream oss;
            oss << "Intermediate segment " << i << " penetrates target node " << layout.to
                << ": (" << p1.x << "," << p1.y << ") -> (" << p2.x << "," << p2.y << ")";
            v.message = oss.str();
            
            violations.push_back(std::move(v));
        }
    }
    
    return violations;
}

} // namespace arborvia
