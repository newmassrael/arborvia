#include "arborvia/layout/constraints/builtins/DirectionalPenetrationConstraint.h"
#include "arborvia/core/GeometryUtils.h"
#include <sstream>

namespace arborvia {

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
        
        if (geometry::segmentPenetratesNodeInterior(p1, p2, srcNode, ctx.tolerance)) {
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
        
        if (geometry::segmentPenetratesNodeInterior(p1, p2, tgtNode, ctx.tolerance)) {
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
