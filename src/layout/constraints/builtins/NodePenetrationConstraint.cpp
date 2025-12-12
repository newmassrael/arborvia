#include "arborvia/layout/constraints/builtins/NodePenetrationConstraint.h"
#include <algorithm>
#include <cmath>
#include <sstream>

namespace arborvia {

bool NodePenetrationConstraint::segmentIntersectsNode(
    const Point& p1, const Point& p2,
    const NodeLayout& node,
    float tolerance) const {
    
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
        // Horizontal segment intersects if it passes through node's vertical range
        return p1.y > nodeTop && p1.y < nodeBottom &&
               segRight > nodeLeft && segLeft < nodeRight;
    }
    
    if (isVertical) {
        // Vertical segment intersects if it passes through node's horizontal range
        return p1.x > nodeLeft && p1.x < nodeRight &&
               segBottom > nodeTop && segTop < nodeBottom;
    }
    
    // For diagonal segments, use line-AABB intersection
    // This is a simplified check - diagonal segments should be caught by OrthogonalityConstraint
    return true;
}

std::vector<ConstraintViolation> NodePenetrationConstraint::check(
    const EdgeLayout& layout,
    const EdgeConstraintContext& ctx) const {
    
    std::vector<ConstraintViolation> violations;
    
    // Build path: source -> bendPoints -> target
    std::vector<Point> path;
    path.push_back(layout.sourcePoint);
    for (const auto& bp : layout.bendPoints) {
        path.push_back(bp.position);
    }
    path.push_back(layout.targetPoint);
    
    // Check each segment against each node (except source/target)
    for (size_t i = 0; i + 1 < path.size(); ++i) {
        const auto& p1 = path[i];
        const auto& p2 = path[i + 1];
        
        for (const auto& [nodeId, node] : ctx.nodeLayouts) {
            // Skip source and target nodes - they have their own constraints
            if (nodeId == layout.from || nodeId == layout.to) {
                continue;
            }
            
            if (segmentIntersectsNode(p1, p2, node, ctx.tolerance)) {
                ConstraintViolation v;
                v.type = ConstraintViolationType::NodePenetration;
                v.edgeId = layout.id;
                v.nodeId = nodeId;
                v.segmentIndex = static_cast<int>(i);
                
                std::ostringstream oss;
                oss << "Segment " << i << " penetrates node " << nodeId;
                v.message = oss.str();
                
                violations.push_back(std::move(v));
            }
        }
    }
    
    return violations;
}

} // namespace arborvia
