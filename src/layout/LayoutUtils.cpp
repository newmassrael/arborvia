#include "arborvia/layout/LayoutUtils.h"
#include "sugiyama/EdgeRouting.h"
#include <cmath>
#include <algorithm>

namespace arborvia {

namespace {
    constexpr float EPSILON_LEN2 = 0.0001f;  // Minimum squared length for valid segment
}

void LayoutUtils::updateEdgePositions(
    std::unordered_map<EdgeId, EdgeLayout>& edgeLayouts,
    const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
    const std::vector<EdgeId>& affectedEdges,
    SnapDistribution distribution,
    const std::unordered_set<NodeId>& movedNodes) {
    
    algorithms::EdgeRouting::updateEdgePositions(
        edgeLayouts, nodeLayouts, affectedEdges, distribution, movedNodes);
}

float LayoutUtils::pointToSegmentDistance(
    const Point& point,
    const Point& segmentStart,
    const Point& segmentEnd,
    Point& outClosestPoint)
{
    float dx = segmentEnd.x - segmentStart.x;
    float dy = segmentEnd.y - segmentStart.y;
    float len2 = dx * dx + dy * dy;

    float t = 0.0f;
    if (len2 > EPSILON_LEN2) {
        t = std::max(0.0f, std::min(1.0f,
            ((point.x - segmentStart.x) * dx + (point.y - segmentStart.y) * dy) / len2));
    }

    outClosestPoint = {segmentStart.x + t * dx, segmentStart.y + t * dy};

    float distX = point.x - outClosestPoint.x;
    float distY = point.y - outClosestPoint.y;
    return std::sqrt(distX * distX + distY * distY);
}

LayoutUtils::EdgeHitResult LayoutUtils::hitTestEdge(
    const Point& point,
    const EdgeLayout& edge,
    float threshold)
{
    EdgeHitResult result;
    auto points = edge.allPoints();

    float minDist = threshold;
    for (size_t i = 1; i < points.size(); ++i) {
        Point closest;
        float dist = pointToSegmentDistance(point, points[i-1], points[i], closest);

        if (dist < minDist) {
            minDist = dist;
            result.hit = true;
            result.segmentIndex = static_cast<int>(i) - 1;
            result.closestPoint = closest;
            result.distance = dist;
        }
    }

    return result;
}

Point LayoutUtils::calculateSnapPointFromPosition(
    const NodeLayout& node,
    NodeEdge edge,
    float position)
{
    switch (edge) {
        case NodeEdge::Top:
            return {
                node.position.x + node.size.width * position,
                node.position.y
            };
        case NodeEdge::Bottom:
            return {
                node.position.x + node.size.width * position,
                node.position.y + node.size.height
            };
        case NodeEdge::Left:
            return {
                node.position.x,
                node.position.y + node.size.height * position
            };
        case NodeEdge::Right:
            return {
                node.position.x + node.size.width,
                node.position.y + node.size.height * position
            };
    }
    return node.center();
}

Point LayoutUtils::calculateSnapPoint(
    const NodeLayout& node,
    NodeEdge edge,
    int snapIndex,
    int totalSnapPoints)
{
    if (totalSnapPoints <= 0) totalSnapPoints = 1;
    if (snapIndex < 0) snapIndex = 0;
    if (snapIndex >= totalSnapPoints) snapIndex = totalSnapPoints - 1;
    float position = static_cast<float>(snapIndex + 1) / static_cast<float>(totalSnapPoints + 1);
    return calculateSnapPointFromPosition(node, edge, position);
}

Point LayoutUtils::calculateEdgeLabelPosition(const EdgeLayout& edge)
{
    const auto& bends = edge.bendPoints;

    // Case 1: 2+ bend points -> midpoint of first two bends (main segment)
    // In channel routing, bendPoint1 and bendPoint2 form the horizontal/vertical segment
    if (bends.size() >= 2) {
        return {
            (bends[0].position.x + bends[1].position.x) / 2.0f,
            (bends[0].position.y + bends[1].position.y) / 2.0f
        };
    }

    // Case 2: 1 bend point -> midpoint along the path
    if (bends.size() == 1) {
        float len1 = edge.sourcePoint.distanceTo(bends[0].position);
        float len2 = bends[0].position.distanceTo(edge.targetPoint);
        float total = len1 + len2;
        
        if (total < 0.001f) {
            return bends[0].position;
        }
        
        float half = total / 2.0f;

        if (half <= len1) {
            float t = half / len1;
            return edge.sourcePoint + (bends[0].position - edge.sourcePoint) * t;
        } else {
            float t = (half - len1) / len2;
            return bends[0].position + (edge.targetPoint - bends[0].position) * t;
        }
    }

    // Case 3: No bend points -> midpoint between source and target
    return {
        (edge.sourcePoint.x + edge.targetPoint.x) / 2.0f,
        (edge.sourcePoint.y + edge.targetPoint.y) / 2.0f
    };
}

}  // namespace arborvia
