#pragma once

#include "arborvia/core/Types.h"
#include "arborvia/layout/LayoutResult.h"

#include <unordered_map>
#include <string>

namespace arborvia {

/// Validates edge layouts against routing constraints
class EdgeValidator {
public:
    /// Validation result for an edge layout
    struct ValidationResult {
        bool valid = true;
        bool orthogonal = true;           ///< All segments are orthogonal
        bool noNodeIntersection = true;   ///< No segments pass through nodes
        bool sourceDirectionOk = true;    ///< First segment matches sourceEdge direction
        bool targetDirectionOk = true;    ///< Last segment matches targetEdge direction

        /// Get human-readable error description
        std::string getErrorDescription() const;
    };

    /// Validate an edge layout against all routing constraints
    /// Checks: orthogonality, node intersection, direction constraints
    /// @param layout The edge layout to validate
    /// @param nodeLayouts All node layouts for intersection checking
    /// @return ValidationResult with detailed constraint status
    static ValidationResult validate(
        const EdgeLayout& layout,
        const std::unordered_map<NodeId, NodeLayout>& nodeLayouts);

    /// Check if an orthogonal segment intersects a node's interior or margin zone
    /// @param p1 Start point of segment
    /// @param p2 End point of segment
    /// @param node The node to check intersection with
    /// @param margin Optional margin to expand node bounds (for proximity detection)
    /// @return True if segment passes through node interior or margin zone (excluding boundary when margin=0)
    static bool segmentIntersectsNode(
        const Point& p1,
        const Point& p2,
        const NodeLayout& node,
        float margin = 0.0f);

private:
    static constexpr float EPSILON = 1e-6f;
    static constexpr float DIRECTION_TOLERANCE = 0.1f;
    static constexpr float SEGMENT_TOLERANCE = 1.0f;
};

}  // namespace arborvia
