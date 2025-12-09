#include "EdgeValidator.h"

#include <cmath>
#include <vector>

namespace arborvia {

std::string EdgeValidator::ValidationResult::getErrorDescription() const {
    if (valid) return "Valid";

    std::string desc;
    if (!orthogonal) desc += "Non-orthogonal segments; ";
    if (!noNodeIntersection) desc += "Segments pass through nodes; ";
    if (!sourceDirectionOk) desc += "First segment direction violates sourceEdge; ";
    if (!targetDirectionOk) desc += "Last segment direction violates targetEdge; ";

    if (!desc.empty() && desc.back() == ' ') {
        desc.pop_back();
        desc.pop_back();  // Remove trailing "; "
    }
    return desc;
}

bool EdgeValidator::segmentIntersectsNode(
    const Point& p1,
    const Point& p2,
    const NodeLayout& node,
    float margin) {

    // Expand node bounds by margin for proximity detection
    float nodeXmin = node.position.x - margin;
    float nodeXmax = node.position.x + node.size.width + margin;
    float nodeYmin = node.position.y - margin;
    float nodeYmax = node.position.y + node.size.height + margin;

    bool isHorizontal = (std::abs(p1.y - p2.y) < EPSILON);
    bool isVertical = (std::abs(p1.x - p2.x) < EPSILON);

    if (isHorizontal) {
        // Horizontal segment: check if Y is strictly inside node Y-range AND X-ranges overlap
        float y = p1.y;
        float xMin = std::min(p1.x, p2.x);
        float xMax = std::max(p1.x, p2.x);

        // Y must be strictly inside (not on boundary) AND X-ranges must overlap
        return (y > nodeYmin && y < nodeYmax && xMin < nodeXmax && xMax > nodeXmin);
    }
    else if (isVertical) {
        // Vertical segment: check if X is strictly inside node X-range AND Y-ranges overlap
        float x = p1.x;
        float yMin = std::min(p1.y, p2.y);
        float yMax = std::max(p1.y, p2.y);

        // X must be strictly inside (not on boundary) AND Y-ranges must overlap
        return (x > nodeXmin && x < nodeXmax && yMin < nodeYmax && yMax > nodeYmin);
    }

    return false;  // No diagonal segments in orthogonal routing
}

EdgeValidator::ValidationResult EdgeValidator::validate(
    const EdgeLayout& layout,
    const std::unordered_map<NodeId, NodeLayout>& nodeLayouts) {

    ValidationResult result;

    // Build path points
    std::vector<Point> path;
    path.push_back(layout.sourcePoint);
    for (const auto& bp : layout.bendPoints) {
        path.push_back(bp.position);
    }
    path.push_back(layout.targetPoint);

    // Need at least 2 points
    if (path.size() < 2) {
        result.valid = false;
        result.orthogonal = false;
        return result;
    }

    // === Check 1: Source direction constraint ===
    // First segment must exit in the direction indicated by sourceEdge
    {
        float dx = path[1].x - path[0].x;
        float dy = path[1].y - path[0].y;

        switch (layout.sourceEdge) {
            case NodeEdge::Top:    result.sourceDirectionOk = (dy < -DIRECTION_TOLERANCE); break;
            case NodeEdge::Bottom: result.sourceDirectionOk = (dy > DIRECTION_TOLERANCE);  break;
            case NodeEdge::Left:   result.sourceDirectionOk = (dx < -DIRECTION_TOLERANCE); break;
            case NodeEdge::Right:  result.sourceDirectionOk = (dx > DIRECTION_TOLERANCE);  break;
        }
    }

    // === Check 2: Target direction constraint ===
    // Last segment must enter in the direction indicated by targetEdge
    {
        size_t last = path.size() - 1;
        float dx = path[last].x - path[last - 1].x;
        float dy = path[last].y - path[last - 1].y;

        switch (layout.targetEdge) {
            case NodeEdge::Top:    result.targetDirectionOk = (dy > DIRECTION_TOLERANCE);  break;  // Enter from above
            case NodeEdge::Bottom: result.targetDirectionOk = (dy < -DIRECTION_TOLERANCE); break;  // Enter from below
            case NodeEdge::Left:   result.targetDirectionOk = (dx > DIRECTION_TOLERANCE);  break;  // Enter from left
            case NodeEdge::Right:  result.targetDirectionOk = (dx < -DIRECTION_TOLERANCE); break;  // Enter from right
        }
    }

    // === Check 3: Orthogonality and node intersection ===
    for (size_t i = 0; i + 1 < path.size(); ++i) {
        const Point& p1 = path[i];
        const Point& p2 = path[i + 1];

        bool isHoriz = std::abs(p1.y - p2.y) < DIRECTION_TOLERANCE;
        bool isVert = std::abs(p1.x - p2.x) < DIRECTION_TOLERANCE;

        // Orthogonality check
        if (!isHoriz && !isVert) {
            result.orthogonal = false;
        }

        // Node intersection check
        bool isFirstSeg = (i == 0);
        bool isLastSeg = (i == path.size() - 2);

        for (const auto& [nodeId, node] : nodeLayouts) {
            // Skip source node at first segment start, target node at last segment end
            bool skipAtP1 = (isFirstSeg && nodeId == layout.from);
            bool skipAtP2 = (isLastSeg && nodeId == layout.to);

            // Check if segment passes through node interior
            float xmin = node.position.x;
            float xmax = node.position.x + node.size.width;
            float ymin = node.position.y;
            float ymax = node.position.y + node.size.height;

            bool intersects = false;

            if (isHoriz) {
                float y = p1.y;
                float segXmin = std::min(p1.x, p2.x);
                float segXmax = std::max(p1.x, p2.x);

                // Shrink segment if endpoints should be excluded
                if (skipAtP1) {
                    if (segXmin == p1.x) segXmin += SEGMENT_TOLERANCE;
                    else segXmax -= SEGMENT_TOLERANCE;
                }
                if (skipAtP2) {
                    if (segXmax == p2.x) segXmax -= SEGMENT_TOLERANCE;
                    else segXmin += SEGMENT_TOLERANCE;
                }

                // Check if y is strictly inside node and x ranges overlap
                intersects = (y > ymin && y < ymax && segXmin < xmax && segXmax > xmin);
            } else if (isVert) {
                float x = p1.x;
                float segYmin = std::min(p1.y, p2.y);
                float segYmax = std::max(p1.y, p2.y);

                // Shrink segment if endpoints should be excluded
                if (skipAtP1) {
                    if (segYmin == p1.y) segYmin += SEGMENT_TOLERANCE;
                    else segYmax -= SEGMENT_TOLERANCE;
                }
                if (skipAtP2) {
                    if (segYmax == p2.y) segYmax -= SEGMENT_TOLERANCE;
                    else segYmin += SEGMENT_TOLERANCE;
                }

                // Check if x is strictly inside node and y ranges overlap
                intersects = (x > xmin && x < xmax && segYmin < ymax && segYmax > ymin);
            }

            if (intersects) {
                result.noNodeIntersection = false;
            }
        }
    }

    // Final validity
    result.valid = result.orthogonal &&
                   result.noNodeIntersection &&
                   result.sourceDirectionOk &&
                   result.targetDirectionOk;

    return result;
}

}  // namespace arborvia
