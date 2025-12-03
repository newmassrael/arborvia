#include "arborvia/layout/LayoutUtils.h"
#include "sugiyama/EdgeRouting.h"
#include <cmath>
#include <algorithm>

namespace arborvia {

namespace {
    constexpr float EPSILON_LEN2 = 0.0001f;  // Minimum squared length for valid segment
    
    /// Check if a line segment intersects an axis-aligned bounding box
    /// Uses separating axis theorem for orthogonal segments
    bool segmentIntersectsAABB(
        const Point& p1,
        const Point& p2,
        const Point& boxPos,
        const Size& boxSize) {
        
        float boxRight = boxPos.x + boxSize.width;
        float boxBottom = boxPos.y + boxSize.height;
        
        // Get segment bounding box
        float segMinX = std::min(p1.x, p2.x);
        float segMaxX = std::max(p1.x, p2.x);
        float segMinY = std::min(p1.y, p2.y);
        float segMaxY = std::max(p1.y, p2.y);
        
        // Quick rejection: bounding boxes don't overlap
        if (segMaxX < boxPos.x || segMinX > boxRight ||
            segMaxY < boxPos.y || segMinY > boxBottom) {
            return false;
        }
        
        // For orthogonal segments (which edge paths typically are),
        // if bounding boxes overlap, segment intersects the box
        bool isVertical = std::abs(p1.x - p2.x) < 0.001f;
        bool isHorizontal = std::abs(p1.y - p2.y) < 0.001f;
        
        if (isVertical || isHorizontal) {
            // Bounding box overlap already confirmed above
            return true;
        }
        
        // For diagonal segments, do more precise check
        // Check if any corner of the box is on opposite sides of the line
        auto sign = [](float v) { return v > 0 ? 1 : (v < 0 ? -1 : 0); };
        
        float dx = p2.x - p1.x;
        float dy = p2.y - p1.y;
        
        // Cross product signs for each corner
        auto crossSign = [&](float cx, float cy) {
            return sign((cx - p1.x) * dy - (cy - p1.y) * dx);
        };
        
        int s1 = crossSign(boxPos.x, boxPos.y);
        int s2 = crossSign(boxRight, boxPos.y);
        int s3 = crossSign(boxRight, boxBottom);
        int s4 = crossSign(boxPos.x, boxBottom);
        
        // If all corners are on the same side, no intersection
        if (s1 == s2 && s2 == s3 && s3 == s4 && s1 != 0) {
            return false;
        }
        
        return true;
    }
    
    /// Check if any segment of an edge path intersects the given AABB
    bool edgePathIntersectsAABB(
        const EdgeLayout& layout,
        const Point& boxPos,
        const Size& boxSize) {
        
        auto points = layout.allPoints();
        for (size_t i = 1; i < points.size(); ++i) {
            if (segmentIntersectsAABB(points[i-1], points[i], boxPos, boxSize)) {
                return true;
            }
        }
        return false;
    }
}

void LayoutUtils::updateEdgePositions(
    std::unordered_map<EdgeId, EdgeLayout>& edgeLayouts,
    const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
    const std::vector<EdgeId>& affectedEdges,
    SnapDistribution distribution,
    const std::unordered_set<NodeId>& movedNodes,
    float gridSize) {

    algorithms::EdgeRouting::updateEdgePositions(
        edgeLayouts, nodeLayouts, affectedEdges, distribution, movedNodes, gridSize);
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

std::vector<EdgeId> LayoutUtils::getConnectedEdges(
    NodeId nodeId,
    const std::unordered_map<EdgeId, EdgeLayout>& edgeLayouts) {
    
    std::vector<EdgeId> result;
    for (const auto& [edgeId, layout] : edgeLayouts) {
        if (layout.from == nodeId || layout.to == nodeId) {
            result.push_back(edgeId);
        }
    }
    return result;
}

LayoutUtils::DragValidation LayoutUtils::canMoveNodeTo(
    NodeId nodeId,
    Point newPosition,
    const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
    const std::unordered_map<EdgeId, EdgeLayout>& edgeLayouts,
    float gridSize) {
    
    DragValidation result;
    result.valid = true;
    
    auto nodeIt = nodeLayouts.find(nodeId);
    if (nodeIt == nodeLayouts.end()) {
        return result;  // Node not found, allow move
    }
    
    // === Early return: Distance check (O(N), very fast) ===
    // Forbid move if within 5 grid units of any other node
    constexpr float MIN_GRID_DISTANCE = 5.0f;
    float minDist = MIN_GRID_DISTANCE * gridSize;
    const Size& nodeSize = nodeIt->second.size;
    
    for (const auto& [otherId, other] : nodeLayouts) {
        if (otherId == nodeId) continue;
        
        // Calculate AABB distance (gap between bounding boxes)
        float dx = std::max(0.0f, std::max(other.position.x - (newPosition.x + nodeSize.width),
                                           newPosition.x - (other.position.x + other.size.width)));
        float dy = std::max(0.0f, std::max(other.position.y - (newPosition.y + nodeSize.height),
                                           newPosition.y - (other.position.y + other.size.height)));
        
        if (dx < minDist && dy < minDist) {
            result.valid = false;
            return result;  // Too close to another node - early return
        }
    }
    
    // === Full validation (expensive, but node is far enough from others) ===
    
    // Create temporary node layouts with the proposed move
    auto tempNodeLayouts = nodeLayouts;
    auto tempNodeIt = tempNodeLayouts.find(nodeId);
    tempNodeIt->second.position = newPosition;
    
    // Collect only AFFECTED edges for validation (performance optimization)
    // Affected edges are:
    // 1. Edges directly connected to the moved node
    // 2. Edges whose current path intersects the node's new position
    std::vector<EdgeId> affectedEdges;
    affectedEdges.reserve(edgeLayouts.size() / 10);  // Typically ~10% of edges affected
    
    for (const auto& [edgeId, layout] : edgeLayouts) {
        // Type 1: Connected to moved node
        if (layout.from == nodeId || layout.to == nodeId) {
            affectedEdges.push_back(edgeId);
            continue;
        }
        // Type 2: Path intersects new node position
        if (edgePathIntersectsAABB(layout, newPosition, nodeSize)) {
            affectedEdges.push_back(edgeId);
        }
    }
    
    // Create temp edge layouts for validation
    auto tempEdgeLayouts = edgeLayouts;
    
    // Update only affected edge positions after the node move
    std::unordered_set<NodeId> movedNodes = {nodeId};
    algorithms::EdgeRouting::updateEdgePositions(tempEdgeLayouts, tempNodeLayouts, affectedEdges,
                                                  SnapDistribution::Separated, movedNodes, gridSize);
    
    // Check only affected edges for valid routing
    for (EdgeId edgeId : affectedEdges) {
        const auto& layout = tempEdgeLayouts.at(edgeId);
        auto validation = algorithms::EdgeRouting::validateEdgeLayout(layout, tempNodeLayouts);
        if (!validation.valid) {
            result.valid = false;
            result.invalidEdges.push_back(edgeId);
        }
    }
    
    return result;
}

bool LayoutUtils::canMoveNodeToFast(
    NodeId nodeId,
    Point newPosition,
    const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
    float margin) {
    
    auto it = nodeLayouts.find(nodeId);
    if (it == nodeLayouts.end()) {
        return true;  // Node not found, allow move
    }
    
    // Get moved node's new bounds
    const Size& size = it->second.size;
    float x1 = newPosition.x - margin;
    float y1 = newPosition.y - margin;
    float x2 = newPosition.x + size.width + margin;
    float y2 = newPosition.y + size.height + margin;
    
    // Check overlap with all other nodes
    for (const auto& [otherId, otherNode] : nodeLayouts) {
        if (otherId == nodeId) continue;
        
        float ox1 = otherNode.position.x;
        float oy1 = otherNode.position.y;
        float ox2 = ox1 + otherNode.size.width;
        float oy2 = oy1 + otherNode.size.height;
        
        // Check for overlap (AABB intersection)
        bool overlaps = (x1 < ox2 && x2 > ox1 && y1 < oy2 && y2 > oy1);
        if (overlaps) {
            return false;
        }
    }
    
    return true;
}

}  // namespace arborvia
