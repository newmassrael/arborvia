#include "layout/constraints/BoundaryConstraint.h"

#include <algorithm>
#include <limits>

namespace arborvia {

BoundaryConstraint::BoundaryConstraint(float minX, float minY, float maxX, float maxY, float margin)
    : minX_(minX), minY_(minY), maxX_(maxX), maxY_(maxY), margin_(margin), autoDetect_(false) {}

BoundaryConstraint::BoundaryConstraint(float margin)
    : minX_(0), minY_(0), maxX_(0), maxY_(0), margin_(margin), autoDetect_(true) {}

ConstraintResult BoundaryConstraint::check(const ConstraintContext& ctx) const {
    // Get node being dragged
    auto it = ctx.nodeLayouts.find(ctx.nodeId);
    if (it == ctx.nodeLayouts.end()) {
        return ConstraintResult::ok();  // Node not found, allow
    }
    
    const auto& nodeLayout = it->second;
    float nodeWidth = nodeLayout.size.width;
    float nodeHeight = nodeLayout.size.height;
    
    // Calculate effective bounds
    float effectiveMinX, effectiveMinY, effectiveMaxX, effectiveMaxY;
    
    if (autoDetect_) {
        // Auto-detect bounds from other nodes (excluding the dragged node)
        calculateBounds(ctx.nodeLayouts, effectiveMinX, effectiveMinY, effectiveMaxX, effectiveMaxY, ctx.nodeId);
        
        // If no other nodes, use large default bounds
        if (effectiveMinX > effectiveMaxX) {
            return ConstraintResult::ok();  // No bounds detected
        }
        
        // Expand bounds by margin
        effectiveMinX -= margin_;
        effectiveMinY -= margin_;
        effectiveMaxX += margin_;
        effectiveMaxY += margin_;
    } else {
        effectiveMinX = minX_ + margin_;
        effectiveMinY = minY_ + margin_;
        effectiveMaxX = maxX_ - margin_;
        effectiveMaxY = maxY_ - margin_;
    }
    
    // Check if proposed position would be within bounds
    float newX = ctx.newPosition.x;
    float newY = ctx.newPosition.y;
    
    // Check left boundary
    if (newX < effectiveMinX) {
        return ConstraintResult::fail("Node would exceed left boundary");
    }
    
    // Check right boundary (account for node width)
    if (newX + nodeWidth > effectiveMaxX) {
        return ConstraintResult::fail("Node would exceed right boundary");
    }
    
    // Check top boundary
    if (newY < effectiveMinY) {
        return ConstraintResult::fail("Node would exceed top boundary");
    }
    
    // Check bottom boundary (account for node height)
    if (newY + nodeHeight > effectiveMaxY) {
        return ConstraintResult::fail("Node would exceed bottom boundary");
    }
    
    return ConstraintResult::ok();
}

std::vector<Rect> BoundaryConstraint::getBlockedRegions(const ConstraintContext& ctx) const {
    std::vector<Rect> regions;
    
    // Get node size
    auto it = ctx.nodeLayouts.find(ctx.nodeId);
    if (it == ctx.nodeLayouts.end()) {
        return regions;
    }
    
    const auto& nodeLayout = it->second;
    float nodeWidth = nodeLayout.size.width;
    float nodeHeight = nodeLayout.size.height;
    
    // Calculate effective bounds
    float effectiveMinX, effectiveMinY, effectiveMaxX, effectiveMaxY;
    
    if (autoDetect_) {
        calculateBounds(ctx.nodeLayouts, effectiveMinX, effectiveMinY, effectiveMaxX, effectiveMaxY, ctx.nodeId);
        if (effectiveMinX > effectiveMaxX) {
            return regions;  // No bounds
        }
        effectiveMinX -= margin_;
        effectiveMinY -= margin_;
        effectiveMaxX += margin_;
        effectiveMaxY += margin_;
    } else {
        effectiveMinX = minX_ + margin_;
        effectiveMinY = minY_ + margin_;
        effectiveMaxX = maxX_ - margin_;
        effectiveMaxY = maxY_ - margin_;
    }
    
    // Create blocked regions representing areas outside the boundary
    // These are simplified as thin rectangles along the edges
    
    constexpr float LARGE = 10000.0f;
    
    // Left blocked region (everything to the left of minX)
    regions.push_back(Rect{
        Point{effectiveMinX - LARGE, effectiveMinY - LARGE},
        Size{LARGE, effectiveMaxY - effectiveMinY + 2 * LARGE}
    });
    
    // Right blocked region
    regions.push_back(Rect{
        Point{effectiveMaxX - nodeWidth, effectiveMinY - LARGE},
        Size{LARGE, effectiveMaxY - effectiveMinY + 2 * LARGE}
    });
    
    // Top blocked region
    regions.push_back(Rect{
        Point{effectiveMinX - LARGE, effectiveMinY - LARGE},
        Size{effectiveMaxX - effectiveMinX + 2 * LARGE, LARGE}
    });
    
    // Bottom blocked region
    regions.push_back(Rect{
        Point{effectiveMinX - LARGE, effectiveMaxY - nodeHeight},
        Size{effectiveMaxX - effectiveMinX + 2 * LARGE, LARGE}
    });
    
    return regions;
}

void BoundaryConstraint::calculateBounds(
    const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
    float& minX, float& minY, float& maxX, float& maxY,
    NodeId excludeNodeId) const {
    
    minX = std::numeric_limits<float>::max();
    minY = std::numeric_limits<float>::max();
    maxX = std::numeric_limits<float>::lowest();
    maxY = std::numeric_limits<float>::lowest();
    
    for (const auto& [nodeId, layout] : nodeLayouts) {
        // Exclude the dragged node from bounds calculation
        if (nodeId == excludeNodeId) {
            continue;
        }
        minX = std::min(minX, layout.position.x);
        minY = std::min(minY, layout.position.y);
        maxX = std::max(maxX, layout.position.x + layout.size.width);
        maxY = std::max(maxY, layout.position.y + layout.size.height);
    }
}

}  // namespace arborvia
