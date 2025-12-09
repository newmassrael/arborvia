#include "PathCleanup.h"
#include <cmath>

namespace arborvia {

bool PathCleanup::isPointDuplicate(const Point& a, const Point& b) {
    return std::abs(a.x - b.x) < EPSILON && std::abs(a.y - b.y) < EPSILON;
}

bool PathCleanup::isSpike(const Point& a, const Point& b, const Point& c) {
    // Check vertical spike (all on same X)
    bool sameX = (std::abs(a.x - b.x) < EPSILON && std::abs(b.x - c.x) < EPSILON);
    if (sameX) {
        bool aToB_down = b.y > a.y;
        bool bToC_down = c.y > b.y;
        if (aToB_down != bToC_down) {
            return true;
        }
    }

    // Check horizontal spike (all on same Y)
    bool sameY = (std::abs(a.y - b.y) < EPSILON && std::abs(b.y - c.y) < EPSILON);
    if (sameY) {
        bool aToB_right = b.x > a.x;
        bool bToC_right = c.x > b.x;
        if (aToB_right != bToC_right) {
            return true;
        }
    }

    return false;
}

void PathCleanup::removeSpikesAndDuplicates(std::vector<Point>& points) {
    bool modified = true;
    while (modified && points.size() > 2) {
        modified = false;

        // Remove duplicate consecutive points
        for (size_t i = 0; i + 1 < points.size(); ++i) {
            if (isPointDuplicate(points[i], points[i + 1])) {
                points.erase(points.begin() + static_cast<long>(i + 1));
                modified = true;
                break;
            }
        }
        if (modified) continue;

        // Remove spike points (direction reversal on same line)
        // Priority 1: Remove boundary spikes (involving source/target points)
        // These are critical because they indicate a path going back on itself at the endpoints
        // A spike at the boundary is WORSE than a direction constraint violation
        
        // Check source boundary spike: points[0], points[1], points[2]
        if (points.size() >= 3 && isSpike(points[0], points[1], points[2])) {
            // Remove the first bend (points[1]) to fix the spike
            points.erase(points.begin() + 1);
            modified = true;
        }
        if (modified) continue;
        
        // Check target boundary spike: points[n-3], points[n-2], points[n-1]
        if (points.size() >= 3 && isSpike(points[points.size() - 3], points[points.size() - 2], points[points.size() - 1])) {
            // Remove the last bend (points[n-2]) to fix the spike
            points.erase(points.begin() + static_cast<long>(points.size() - 2));
            modified = true;
        }
        if (modified) continue;
        
        // Priority 2: Remove interior spikes (protected first/last bends)
        // BUT protect first bend (points[1]) and last bend (points[n-2]) for direction constraints
        // First bend determines if first segment goes in correct direction from sourceEdge
        // Last bend determines if last segment arrives in correct direction to targetEdge
        for (size_t i = 1; i + 3 < points.size(); ++i) {
            if (isSpike(points[i], points[i + 1], points[i + 2])) {
                points.erase(points.begin() + static_cast<long>(i + 1));
                modified = true;
                break;
            }
        }
        if (modified) continue;

        // Remove redundant collinear intermediate points
        // If three consecutive points are on the same line (vertical or horizontal),
        // the middle point can be removed as it doesn't affect the path
        // BUT protect first and last bends (required for direction constraints)
        for (size_t i = 1; i + 3 < points.size(); ++i) {
            const Point& a = points[i];
            const Point& b = points[i + 1];
            const Point& c = points[i + 2];

            // Check if all three are on same vertical line
            bool sameX = (std::abs(a.x - b.x) < EPSILON && std::abs(b.x - c.x) < EPSILON);
            // Check if all three are on same horizontal line
            bool sameY = (std::abs(a.y - b.y) < EPSILON && std::abs(b.y - c.y) < EPSILON);

            if (sameX || sameY) {
                // Middle point is redundant, remove it
                points.erase(points.begin() + static_cast<long>(i + 1));
                modified = true;
                break;
            }
        }
        if (modified) continue;

        // Remove non-consecutive duplicate points and the circular detour between them
        // This is safe because if two points are identical, the path between them
        // is a closed loop that accomplishes nothing
        for (size_t i = 0; i + 2 < points.size(); ++i) {
            for (size_t j = i + 2; j < points.size(); ++j) {
                if (isPointDuplicate(points[i], points[j])) {
                    // Found duplicate at i and j - remove all points from i+1 to j (inclusive)
                    // This eliminates the circular detour between them
                    points.erase(points.begin() + static_cast<long>(i + 1),
                                 points.begin() + static_cast<long>(j + 1));
                    modified = true;
                    break;
                }
            }
            if (modified) break;
        }
    }
}

void PathCleanup::removeEndpointDuplicates(EdgeLayout& layout) {
    if (layout.bendPoints.empty()) return;

    // Remove first bend if it duplicates sourcePoint
    while (!layout.bendPoints.empty() &&
           isPointDuplicate(layout.bendPoints.front().position, layout.sourcePoint)) {
        layout.bendPoints.erase(layout.bendPoints.begin());
    }

    // Remove last bend if it duplicates targetPoint, but ensure proper approach direction
    while (!layout.bendPoints.empty() &&
           isPointDuplicate(layout.bendPoints.back().position, layout.targetPoint)) {
        // Check if removing this bend would violate direction constraint
        // Get the previous point (either previous bend or sourcePoint)
        Point prevPoint = layout.bendPoints.size() > 1
            ? layout.bendPoints[layout.bendPoints.size() - 2].position
            : layout.sourcePoint;

        // Check if the segment from prevPoint to targetPoint has correct direction
        bool needsVerticalApproach = (layout.targetEdge == NodeEdge::Top ||
                                      layout.targetEdge == NodeEdge::Bottom);
        bool needsHorizontalApproach = (layout.targetEdge == NodeEdge::Left ||
                                        layout.targetEdge == NodeEdge::Right);

        float dx = layout.targetPoint.x - prevPoint.x;
        float dy = layout.targetPoint.y - prevPoint.y;
        bool isVertical = std::abs(dx) < EPSILON;
        bool isHorizontal = std::abs(dy) < EPSILON;

        if ((needsVerticalApproach && !isVertical) ||
            (needsHorizontalApproach && !isHorizontal)) {
            // Removing this bend would violate direction constraint
            // Instead of removing, adjust the bend to provide proper approach
            Point& lastBend = layout.bendPoints.back().position;
            if (needsVerticalApproach) {
                // Move bend to be directly above/below target
                lastBend.x = layout.targetPoint.x;
                // Keep Y at previous segment's Y (or offset from target)
                if (layout.targetEdge == NodeEdge::Top) {
                    lastBend.y = std::min(lastBend.y, layout.targetPoint.y - DEFAULT_MARGIN);
                } else {
                    lastBend.y = std::max(lastBend.y, layout.targetPoint.y + DEFAULT_MARGIN);
                }
            } else {
                // Move bend to be directly left/right of target
                lastBend.y = layout.targetPoint.y;
                // Keep X at previous segment's X (or offset from target)
                if (layout.targetEdge == NodeEdge::Left) {
                    lastBend.x = std::min(lastBend.x, layout.targetPoint.x - DEFAULT_MARGIN);
                } else {
                    lastBend.x = std::max(lastBend.x, layout.targetPoint.x + DEFAULT_MARGIN);
                }
            }
            break;  // Don't remove, we adjusted it
        }

        layout.bendPoints.pop_back();
    }

    // Remove consecutive duplicate bend points
    for (size_t i = 0; i + 1 < layout.bendPoints.size(); ) {
        if (isPointDuplicate(layout.bendPoints[i].position,
                            layout.bendPoints[i + 1].position)) {
            layout.bendPoints.erase(layout.bendPoints.begin() + static_cast<long>(i + 1));
        } else {
            ++i;
        }
    }
}

void PathCleanup::moveBendsOutsideNode(EdgeLayout& layout, const NodeLayout& targetNode, float margin) {
    if (layout.bendPoints.empty()) return;

    float nodeLeft = targetNode.position.x;
    float nodeRight = targetNode.position.x + targetNode.size.width;
    float nodeTop = targetNode.position.y;
    float nodeBottom = targetNode.position.y + targetNode.size.height;

    for (auto& bend : layout.bendPoints) {
        Point& p = bend.position;

        // Check if point is inside or on boundary of target node
        bool insideX = (p.x >= nodeLeft && p.x <= nodeRight);
        bool insideY = (p.y >= nodeTop && p.y <= nodeBottom);

        if (insideX && insideY) {
            // Point is inside node - move it outside based on target edge direction
            switch (layout.targetEdge) {
                case NodeEdge::Top:
                    // Move bend above the node
                    if (p.y >= nodeTop) {
                        p.y = nodeTop - margin;
                    }
                    break;
                case NodeEdge::Bottom:
                    // Move bend below the node
                    if (p.y <= nodeBottom) {
                        p.y = nodeBottom + margin;
                    }
                    break;
                case NodeEdge::Left:
                    // Move bend left of the node
                    if (p.x >= nodeLeft) {
                        p.x = nodeLeft - margin;
                    }
                    break;
                case NodeEdge::Right:
                    // Move bend right of the node
                    if (p.x <= nodeRight) {
                        p.x = nodeRight + margin;
                    }
                    break;
            }
        }
    }
}

}  // namespace arborvia
