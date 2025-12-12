#include "arborvia/layout/constraints/builtins/OrthogonalityConstraint.h"
#include <cmath>
#include <sstream>

namespace arborvia {

std::vector<ConstraintViolation> OrthogonalityConstraint::check(
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
    
    // Check each segment for orthogonality
    for (size_t i = 0; i + 1 < path.size(); ++i) {
        const auto& p1 = path[i];
        const auto& p2 = path[i + 1];
        
        float dx = std::abs(p2.x - p1.x);
        float dy = std::abs(p2.y - p1.y);
        
        // Segment is orthogonal if one dimension is zero (within tolerance)
        bool isHorizontal = dy < ctx.tolerance;
        bool isVertical = dx < ctx.tolerance;
        
        if (!isHorizontal && !isVertical) {
            ConstraintViolation v;
            v.type = ConstraintViolationType::Orthogonality;
            v.edgeId = layout.id;
            v.segmentIndex = static_cast<int>(i);
            
            std::ostringstream oss;
            oss << "Segment " << i << " is diagonal: ("
                << p1.x << "," << p1.y << ") -> ("
                << p2.x << "," << p2.y << ")";
            v.message = oss.str();
            
            violations.push_back(std::move(v));
        }
    }
    
    return violations;
}

} // namespace arborvia
