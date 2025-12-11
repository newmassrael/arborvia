#pragma once

#include "arborvia/layout/api/IDragConstraint.h"

namespace arborvia {

/// Constraint that ensures nodes stay within layout boundaries
/// Tier: Medium - O(1) per check but requires bounds calculation
class BoundaryConstraint : public IDragConstraint {
public:
    /// Construct with boundary limits
    /// @param minX Minimum X coordinate
    /// @param minY Minimum Y coordinate  
    /// @param maxX Maximum X coordinate
    /// @param maxY Maximum Y coordinate
    /// @param margin Margin from boundary (default 0)
    BoundaryConstraint(float minX, float minY, float maxX, float maxY, float margin = 0.0f);
    
    /// Construct with automatic boundary detection from node layouts
    /// Will compute bounds from existing nodes and expand by margin
    /// @param margin Margin from detected boundary
    explicit BoundaryConstraint(float margin = 50.0f);

    ConstraintResult check(const ConstraintContext& ctx) const override;
    ConstraintTier tier() const override { return ConstraintTier::Medium; }
    std::string name() const override { return "BoundaryConstraint"; }
    
    bool hasVisualization() const override { return true; }
    std::vector<Rect> getBlockedRegions(const ConstraintContext& ctx) const override;

private:
    float minX_, minY_, maxX_, maxY_;
    float margin_;
    bool autoDetect_;
    
    /// Calculate bounds from node layouts, excluding the specified node
    /// @param excludeNodeId Node to exclude from bounds calculation (typically the dragged node)
    void calculateBounds(const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
                        float& minX, float& minY, float& maxX, float& maxY,
                        NodeId excludeNodeId) const;
};

}  // namespace arborvia
