#include "arborvia/layout/LayoutUtils.h"
#include "arborvia/core/GeometryUtils.h"
#include "arborvia/layout/ConstraintManager.h"
#include "arborvia/layout/ConstraintConfig.h"
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
    const std::unordered_set<NodeId>& movedNodes,
    float gridSize) {

    algorithms::EdgeRouting routing;
    routing.updateEdgePositions(
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
    float minDist = threshold;
    int segmentIndex = 0;

    edge.forEachSegment([&](const Point& p1, const Point& p2) {
        Point closest;
        float dist = pointToSegmentDistance(point, p1, p2, closest);

        if (dist < minDist) {
            minDist = dist;
            result.hit = true;
            result.segmentIndex = segmentIndex;
            result.closestPoint = closest;
            result.distance = dist;
        }
        ++segmentIndex;
    });

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
    
    // Use ConstraintManager with default constraints
    // Default constraints: MinDistanceConstraint (5 grid units), EdgeValidityConstraint
    static auto constraintManager = ConstraintManager::createDefault(constants::MIN_NODE_GRID_DISTANCE);
    
    return canMoveNodeTo(nodeId, newPosition, nodeLayouts, edgeLayouts, constraintManager, gridSize);
}

LayoutUtils::DragValidation LayoutUtils::canMoveNodeTo(
    NodeId nodeId,
    Point newPosition,
    const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
    const std::unordered_map<EdgeId, EdgeLayout>& edgeLayouts,
    const ConstraintManager& constraintManager,
    float gridSize) {
    
    ConstraintContext ctx{nodeId, newPosition, nodeLayouts, edgeLayouts, gridSize};
    auto validationResult = constraintManager.validate(ctx);
    
    // Convert DragValidationResult to DragValidation for API compatibility
    DragValidation result;
    result.valid = validationResult.valid;
    result.invalidEdges = std::move(validationResult.invalidEdges);
    
    return result;
}

LayoutUtils::DragValidation LayoutUtils::canMoveNodeTo(
    NodeId nodeId,
    Point newPosition,
    const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
    const std::unordered_map<EdgeId, EdgeLayout>& edgeLayouts,
    const ConstraintConfig& config,
    float gridSize) {
    
    // Create manager from config (or use default if config is empty)
    auto manager = config.empty() 
        ? ConstraintFactory::create(ConstraintConfig::createDefault())
        : ConstraintFactory::create(config);
    
    return canMoveNodeTo(nodeId, newPosition, nodeLayouts, edgeLayouts, *manager, gridSize);
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
