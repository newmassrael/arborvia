#include "arborvia/layout/util/LayoutUtils.h"
#include "arborvia/core/GeometryUtils.h"
#include "arborvia/core/Graph.h"
#include "arborvia/layout/constraints/PositionFinder.h"
#include "layout/interactive/ConstraintManager.h"
#include "arborvia/layout/config/ConstraintConfig.h"
#include "arborvia/layout/interactive/PathRoutingCoordinator.h"
#include "sugiyama/routing/EdgeRouting.h"
#include "snap/GridSnapCalculator.h"
#include "snap/SnapPointCalculator.h"
#include "pathfinding/ObstacleMap.h"
#include "pathfinding/AStarPathFinder.h"
#include "sugiyama/routing/SelfLoopRouter.h"
#include <cmath>
#include "arborvia/common/Logger.h"
#include <algorithm>
#include <set>

namespace arborvia {

namespace {
    constexpr float EPSILON_LEN2 = 0.0001f;  // Minimum squared length for valid segment
    
    // PositionFinder configuration constants
    constexpr float CONSTRAINT_SEARCH_RADIUS = 400.0f;  // Max distance to search for valid position
    constexpr int CONSTRAINT_MAX_ITERATIONS = 2000;     // Max BFS iterations
    constexpr float NODE_AREA_MARGIN = 5.0f;            // Margin for indirect edge detection

    // Check if a line segment intersects a rectangle (AABB)
    bool segmentIntersectsRect(const Point& p1, const Point& p2,
                               float rx, float ry, float rw, float rh) {
        // Quick bounding box check
        float minX = std::min(p1.x, p2.x);
        float maxX = std::max(p1.x, p2.x);
        float minY = std::min(p1.y, p2.y);
        float maxY = std::max(p1.y, p2.y);

        // No overlap if segment's bounding box doesn't intersect rectangle
        if (maxX < rx || minX > rx + rw || maxY < ry || minY > ry + rh) {
            return false;
        }

        // Check if either endpoint is inside the rectangle
        auto pointInRect = [&](const Point& p) {
            return p.x >= rx && p.x <= rx + rw && p.y >= ry && p.y <= ry + rh;
        };

        if (pointInRect(p1) || pointInRect(p2)) {
            return true;
        }

        // Check if segment crosses any edge of the rectangle
        // Using line-segment vs line-segment intersection
        auto segmentsIntersect = [](const Point& a1, const Point& a2,
                                    const Point& b1, const Point& b2) -> bool {
            auto cross = [](const Point& o, const Point& a, const Point& b) {
                return (a.x - o.x) * (b.y - o.y) - (a.y - o.y) * (b.x - o.x);
            };

            float d1 = cross(b1, b2, a1);
            float d2 = cross(b1, b2, a2);
            float d3 = cross(a1, a2, b1);
            float d4 = cross(a1, a2, b2);

            if (((d1 > 0 && d2 < 0) || (d1 < 0 && d2 > 0)) &&
                ((d3 > 0 && d4 < 0) || (d3 < 0 && d4 > 0))) {
                return true;
            }

            return false;
        };

        // Rectangle edges
        Point topLeft{rx, ry};
        Point topRight{rx + rw, ry};
        Point bottomLeft{rx, ry + rh};
        Point bottomRight{rx + rw, ry + rh};

        // Check intersection with each rectangle edge
        if (segmentsIntersect(p1, p2, topLeft, topRight)) return true;      // Top
        if (segmentsIntersect(p1, p2, bottomLeft, bottomRight)) return true; // Bottom
        if (segmentsIntersect(p1, p2, topLeft, bottomLeft)) return true;     // Left
        if (segmentsIntersect(p1, p2, topRight, bottomRight)) return true;   // Right

        return false;
    }

}  // namespace

bool LayoutUtils::edgePassesThroughNode(const EdgeLayout& edge, const NodeLayout& node) {
    bool passes = false;
    edge.forEachSegment([&](const Point& p1, const Point& p2) {
        if (!passes && segmentIntersectsRect(p1, p2,
                node.position.x, node.position.y,
                node.size.width, node.size.height)) {
            passes = true;
        }
    });
    return passes;
}

// ========== Edge Position Updates ==========
// NOTE: For node moves, use LayoutController::moveNode() instead

void LayoutUtils::updateEdgePositions(
    std::unordered_map<EdgeId, EdgeLayout>& edgeLayouts,
    const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
    const std::unordered_map<NodeId, NodeLayout>& oldNodeLayouts,
    const std::vector<EdgeId>& affectedEdges,
    const std::unordered_set<NodeId>& movedNodes,
    float gridSize) {

    EdgeRouting routing;
    routing.updateSnapPositions(
        edgeLayouts, nodeLayouts, oldNodeLayouts, affectedEdges, movedNodes, gridSize);
}

void LayoutUtils::updateEdgePositions(
    std::unordered_map<EdgeId, EdgeLayout>& edgeLayouts,
    const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
    const std::vector<EdgeId>& affectedEdges,
    const std::unordered_set<NodeId>& movedNodes,
    float gridSize) {
    // Backward compatible: use nodeLayouts as oldNodeLayouts
    updateEdgePositions(edgeLayouts, nodeLayouts, nodeLayouts, affectedEdges, movedNodes, gridSize);
}

void LayoutUtils::updateEdgePositions(
    std::unordered_map<EdgeId, EdgeLayout>& edgeLayouts,
    const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
    const std::vector<EdgeId>& affectedEdges,
    PathRoutingCoordinator& coordinator,
    const std::unordered_set<NodeId>& movedNodes,
    float gridSize) {

    // Create EdgeRouting with raw pointer to coordinator (not owned)
    EdgeRouting routing(&coordinator);
    routing.updateSnapPositions(
        edgeLayouts, nodeLayouts, affectedEdges, movedNodes, gridSize);
}

void LayoutUtils::updateEdgePositions(
    std::unordered_map<EdgeId, EdgeLayout>& edgeLayouts,
    const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
    const std::vector<EdgeId>& affectedEdges,
    const LayoutOptions& options,
    const std::unordered_set<NodeId>& movedNodes) {

    EdgeRouting routing;
    
    // updateEdgePositions is called after drop, not during drag
    // Use postDragAlgorithm instead of dragAlgorithm (which may be HideUntilDrop)
    LayoutOptions postDropOptions = options;
    switch (options.optimizationOptions.postDragAlgorithm) {
        case PostDragAlgorithm::AStar:
            postDropOptions.optimizationOptions.dragAlgorithm = DragAlgorithm::AStar;
            break;
        case PostDragAlgorithm::Geometric:
            postDropOptions.optimizationOptions.dragAlgorithm = DragAlgorithm::Geometric;
            break;
        case PostDragAlgorithm::None:
            // Keep existing dragAlgorithm or fallback to Geometric
            postDropOptions.optimizationOptions.dragAlgorithm = DragAlgorithm::Geometric;
            break;
    }
    
    LOG_DEBUG("[LayoutUtils::updateEdgePositions] postDragAlgo={} -> dragAlgo={}, affectedEdges.size={}, movedNodes.size={}",
              static_cast<int>(options.optimizationOptions.postDragAlgorithm),
              static_cast<int>(postDropOptions.optimizationOptions.dragAlgorithm),
              affectedEdges.size(), movedNodes.size());
    for (EdgeId edgeId : affectedEdges) {
        auto it = edgeLayouts.find(edgeId);
        if (it != edgeLayouts.end()) {
            const auto& layout = it->second;
            LOG_DEBUG("[ROOT-CAUSE] BEFORE optimization: edge={} from={} to={} srcEdge={} tgtEdge={} srcPt=({},{}) tgtPt=({},{}) bendPts={}",
                      edgeId, layout.from, layout.to,
                      static_cast<int>(layout.sourceEdge), static_cast<int>(layout.targetEdge),
                      layout.sourcePoint.x, layout.sourcePoint.y,
                      layout.targetPoint.x, layout.targetPoint.y,
                      layout.bendPoints.size());
        }
    }

    // First pass: update directly affected edges (connected to moved nodes)
    routing.updateEdgeRoutingWithOptimization(
        edgeLayouts, nodeLayouts, affectedEdges, postDropOptions, movedNodes);
    
    // Second pass: find edges with segments passing through moved nodes' forbidden zones
    // These edges are not connected to moved nodes but their paths cross through them
    std::vector<EdgeId> edgesToRecalculate;
    for (const auto& [edgeId, edgeLayout] : edgeLayouts) {
        // Skip edges already processed (connected to moved nodes)
        if (std::find(affectedEdges.begin(), affectedEdges.end(), edgeId) != affectedEdges.end()) {
            continue;
        }
        
        // Check if any segment of this edge passes through any moved node's forbidden zone
        for (const auto& movedNodeId : movedNodes) {
            auto nodeIt = nodeLayouts.find(movedNodeId);
            if (nodeIt != nodeLayouts.end()) {
                if (edgePassesThroughNode(edgeLayout, nodeIt->second)) {
                    edgesToRecalculate.push_back(edgeId);
                    LOG_DEBUG("[LayoutUtils] Edge {} passes through moved node {} - will recalculate with A*",
                              edgeId, movedNodeId);
                    break;  // No need to check other moved nodes
                }
            }
        }
    }
    
    // Recalculate edges passing through moved nodes' forbidden zones
    // These edges are NOT connected to moved nodes, so don't apply endpoint constraints
    // (passing empty set allows free rerouting to find valid paths around moved nodes)
    if (!edgesToRecalculate.empty()) {
        LOG_DEBUG("[LayoutUtils] Re-routing {} edges using A*", edgesToRecalculate.size());
        std::unordered_set<NodeId> emptySet;  // No endpoint constraints for pass-through edges
        routing.updateEdgeRoutingWithOptimization(
            edgeLayouts, nodeLayouts, edgesToRecalculate, postDropOptions, emptySet);
    }
}

// Helper: Check if edge has diagonal segments
static bool hasDiagonalSegments(const EdgeLayout& edge) {
    auto points = edge.allPoints();
    if (points.size() < 2) return false;
    
    for (size_t i = 0; i + 1 < points.size(); ++i) {
        float dx = std::abs(points[i + 1].x - points[i].x);
        float dy = std::abs(points[i + 1].y - points[i].y);
        if (dx > 1.0f && dy > 1.0f) {
            return true;  // Diagonal detected
        }
    }
    return false;
}

// Helper: Check if a segment passes through node area (for indirect edge detection)
static bool segmentPassesThroughNodeArea(
    Point p1, Point p2,
    float nodeX, float nodeY, float nodeW, float nodeH) {
    
    // Expand node area slightly for margin
    float margin = NODE_AREA_MARGIN;
    float minX = nodeX - margin;
    float maxX = nodeX + nodeW + margin;
    float minY = nodeY - margin;
    float maxY = nodeY + nodeH + margin;
    
    // Check if segment endpoints are inside node area
    bool p1Inside = (p1.x >= minX && p1.x <= maxX && p1.y >= minY && p1.y <= maxY);
    bool p2Inside = (p2.x >= minX && p2.x <= maxX && p2.y >= minY && p2.y <= maxY);
    if (p1Inside || p2Inside) return true;
    
    // Check if segment crosses node boundaries
    // Horizontal segment
    if (std::abs(p1.y - p2.y) < 1.0f) {
        float segMinX = std::min(p1.x, p2.x);
        float segMaxX = std::max(p1.x, p2.x);
        if (p1.y >= minY && p1.y <= maxY && segMinX <= maxX && segMaxX >= minX) {
            return true;
        }
    }
    // Vertical segment
    if (std::abs(p1.x - p2.x) < 1.0f) {
        float segMinY = std::min(p1.y, p2.y);
        float segMaxY = std::max(p1.y, p2.y);
        if (p1.x >= minX && p1.x <= maxX && segMinY <= maxY && segMaxY >= minY) {
            return true;
        }
    }
    
    return false;
}

// Helper: Get all edges affected by node movement (direct + indirect)
static std::vector<EdgeId> getExpandedAffectedEdges(
    const std::vector<EdgeId>& directEdges,
    const std::unordered_set<NodeId>& movedNodes,
    const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
    const std::unordered_map<EdgeId, EdgeLayout>& edgeLayouts) {
    
    std::set<EdgeId> affected(directEdges.begin(), directEdges.end());
    
    // For each moved node, find edges that pass through its area
    for (NodeId nodeId : movedNodes) {
        auto nodeIt = nodeLayouts.find(nodeId);
        if (nodeIt == nodeLayouts.end()) continue;
        
        const auto& node = nodeIt->second;
        float nodeX = node.position.x;
        float nodeY = node.position.y;
        float nodeW = node.size.width;
        float nodeH = node.size.height;
        
        for (const auto& [edgeId, edge] : edgeLayouts) {
            if (affected.count(edgeId)) continue;  // Already in list
            
            // Check if any segment of this edge passes through node area
            auto points = edge.allPoints();
            for (size_t i = 0; i + 1 < points.size(); ++i) {
                if (segmentPassesThroughNodeArea(points[i], points[i + 1], 
                                          nodeX, nodeY, nodeW, nodeH)) {
                    affected.insert(edgeId);
                    break;
                }
            }
        }
    }
    
    return std::vector<EdgeId>(affected.begin(), affected.end());
}

bool LayoutUtils::updateEdgePositionsWithConstraints(
    std::unordered_map<EdgeId, EdgeLayout>& edgeLayouts,
    std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
    const std::vector<EdgeId>& affectedEdges,
    const Graph& graph,
    const LayoutOptions& options,
    const std::unordered_set<NodeId>& movedNodes) {

    constexpr int MAX_ADJUSTMENT_ITERATIONS = 5;
    float gridSize = options.gridConfig.cellSize > 0 ? options.gridConfig.cellSize : 10.0f;
    
    // Expand affected edges to include indirectly affected ones
    std::vector<EdgeId> expandedAffectedEdges = getExpandedAffectedEdges(
        affectedEdges, movedNodes, nodeLayouts, edgeLayouts);
    
    for (int iteration = 0; iteration < MAX_ADJUSTMENT_ITERATIONS; ++iteration) {
        // Step 1: Try to route edges with current node positions
        updateEdgePositions(edgeLayouts, nodeLayouts, expandedAffectedEdges, options, movedNodes);
        
        // Step 2: Check for diagonal segments (A* failures) in ALL edges
        std::vector<EdgeId> diagonalEdges;
        for (const auto& [edgeId, edge] : edgeLayouts) {
            if (hasDiagonalSegments(edge)) {
                diagonalEdges.push_back(edgeId);
            }
        }
        
        if (diagonalEdges.empty()) {
            // All edges have valid orthogonal paths
            return true;
        }
        
        // Step 3: Use PositionFinder to find valid positions for moved nodes
        PositionFinder finder({gridSize, CONSTRAINT_SEARCH_RADIUS, gridSize, CONSTRAINT_MAX_ITERATIONS});
        
        bool anyAdjusted = false;
        for (NodeId nodeId : movedNodes) {
            auto nodeIt = nodeLayouts.find(nodeId);
            if (nodeIt == nodeLayouts.end()) continue;
            
            // Try to find a valid position for this node
            auto result = finder.placeNode(
                nodeId,
                nodeIt->second.position,
                nodeIt->second.size,
                nodeLayouts,
                edgeLayouts,
                graph);
            
            if (result.success && result.positionAdjusted) {
                anyAdjusted = true;
            }
        }
        
        if (!anyAdjusted) {
            // Could not find better positions, return with current state
            return false;
        }
    }
    
    // Max iterations reached
    return false;
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

Point LayoutUtils::calculateSnapPointFromRatio(
    const NodeLayout& node,
    NodeEdge edge,
    float ratio,
    float gridSize)
{
    // Use SnapPointCalculator for grid-aligned calculation (A* standard)
    float effectiveGridSize = gridSize > 0.0f ? gridSize : constants::effectiveGridSize(0.0f);
    return SnapPointCalculator::calculateFromRatio(node, edge, ratio, effectiveGridSize);
}

Point LayoutUtils::calculateSnapPoint(
    const NodeLayout& node,
    NodeEdge edge,
    int snapIndex,
    int totalSnapPoints,
    float gridSize)
{
    // Use SnapPointCalculator for grid-aligned calculation (A* standard)
    float effectiveGridSize = gridSize > 0.0f ? gridSize : constants::effectiveGridSize(0.0f);
    return SnapPointCalculator::calculateFromIndex(node, edge, snapIndex, totalSnapPoints, effectiveGridSize);
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
    auto manager = ConstraintFactory::create(ConstraintConfig::createDefault());
    return canMoveNodeTo(nodeId, newPosition, nodeLayouts, edgeLayouts, *manager, gridSize);
}

LayoutUtils::DragValidation LayoutUtils::canMoveNodeTo(
    NodeId nodeId,
    Point newPosition,
    const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
    const std::unordered_map<EdgeId, EdgeLayout>& edgeLayouts,
    const IConstraintValidator& constraintValidator,
    float gridSize) {

    ConstraintContext ctx{nodeId, newPosition, nodeLayouts, edgeLayouts, nullptr, gridSize};
    auto validationResult = constraintValidator.validate(ctx);

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

// ========== Snap Point Manipulation API ==========

NodeEdge LayoutUtils::calculateSourceEdgeForPointNode(
    const NodeLayout& srcNode,
    const NodeLayout& tgtNode) {
    
    Point srcCenter = srcNode.center();
    Point tgtCenter = tgtNode.center();
    float deltaX = tgtCenter.x - srcCenter.x;
    float deltaY = tgtCenter.y - srcCenter.y;

    // Determine edge based on dominant axis
    if (std::abs(deltaX) > std::abs(deltaY)) {
        // X-axis dominant: use Right or Left
        return (deltaX > 0) ? NodeEdge::Right : NodeEdge::Left;
    } else {
        // Y-axis dominant: use Bottom or Top
        return (deltaY > 0) ? NodeEdge::Bottom : NodeEdge::Top;
    }
}

NodeEdge LayoutUtils::calculateTargetEdgeForPointNode(
    const NodeLayout& srcNode,
    const NodeLayout& tgtNode) {
    
    Point srcCenter = srcNode.center();
    Point tgtCenter = tgtNode.center();
    // Reverse direction for target: where does the edge come FROM?
    float deltaX = srcCenter.x - tgtCenter.x;
    float deltaY = srcCenter.y - tgtCenter.y;

    if (std::abs(deltaX) > std::abs(deltaY)) {
        return (deltaX > 0) ? NodeEdge::Right : NodeEdge::Left;
    } else {
        return (deltaY > 0) ? NodeEdge::Bottom : NodeEdge::Top;
    }
}

std::pair<NodeEdge, float> LayoutUtils::findClosestNodeEdge(
    const Point& point,
    const NodeLayout& node) {

    // Calculate distances to each edge
    float distTop = std::abs(point.y - node.position.y);
    float distBottom = std::abs(point.y - (node.position.y + node.size.height));
    float distLeft = std::abs(point.x - node.position.x);
    float distRight = std::abs(point.x - (node.position.x + node.size.width));

    // Find minimum distance
    float minDist = distTop;
    NodeEdge closestEdge = NodeEdge::Top;

    if (distBottom < minDist) {
        minDist = distBottom;
        closestEdge = NodeEdge::Bottom;
    }
    if (distLeft < minDist) {
        minDist = distLeft;
        closestEdge = NodeEdge::Left;
    }
    if (distRight < minDist) {
        closestEdge = NodeEdge::Right;
    }

    // Calculate relative position along the edge (0.0 to 1.0)
    float position = 0.5f;  // Default to center

    switch (closestEdge) {
        case NodeEdge::Top:
        case NodeEdge::Bottom: {
            // Horizontal edge - position based on x
            float edgeStart = node.position.x;
            float edgeLength = node.size.width;
            if (edgeLength > 0) {
                position = std::clamp((point.x - edgeStart) / edgeLength, 0.0f, 1.0f);
            }
            break;
        }
        case NodeEdge::Left:
        case NodeEdge::Right: {
            // Vertical edge - position based on y
            float edgeStart = node.position.y;
            float edgeLength = node.size.height;
            if (edgeLength > 0) {
                position = std::clamp((point.y - edgeStart) / edgeLength, 0.0f, 1.0f);
            }
            break;
        }
    }

    return {closestEdge, position};
}

LayoutUtils::SnapMoveResult LayoutUtils::moveSnapPoint(
    EdgeId edgeId,
    bool isSource,
    Point newPosition,
    const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
    std::unordered_map<EdgeId, EdgeLayout>& edgeLayouts,
    const Graph& graph,
    const LayoutOptions& options,
    const BendPointRegenerator& regenerator) {

    SnapMoveResult result;

    // Find the edge
    auto edgeIt = edgeLayouts.find(edgeId);
    if (edgeIt == edgeLayouts.end()) {
        result.success = false;
        result.reason = "Edge not found";
        return result;
    }

    EdgeLayout& edge = edgeIt->second;

    // Get the node this snap point is on
    NodeId nodeId = isSource ? edge.from : edge.to;
    auto nodeIt = nodeLayouts.find(nodeId);
    if (nodeIt == nodeLayouts.end()) {
        result.success = false;
        result.reason = "Node not found";
        return result;
    }

    const NodeLayout& node = nodeIt->second;

    // Save original position/edge BEFORE moving (for potential swap)
    Point originalPosition = isSource ? edge.sourcePoint : edge.targetPoint;
    NodeEdge originalEdge = isSource ? edge.sourceEdge : edge.targetEdge;
    // NOTE: snapIndex computed from position, not stored

    // Find which edge the new point maps to
    auto [newEdge, newPosition_] = findClosestNodeEdge(newPosition, node);

    // Calculate grid-snapped position using GridSnapCalculator
    // This ensures corners are excluded and positions are grid-aligned
    float gridSize = constants::effectiveGridSize(options.gridConfig.cellSize);

    // First get a rough position estimate, then convert to grid candidate index
    Point roughPosition = calculateSnapPointFromRatio(node, newEdge, newPosition_, options.gridConfig.cellSize);

    // Get the candidate index for this position (will clamp to valid range, excluding corners)
    int newSnapIndex = GridSnapCalculator::getCandidateIndexFromPosition(
        node, newEdge, roughPosition, gridSize);

    // Now get the actual grid-snapped position from the candidate index
    // This ensures the position is grid-aligned and corners are excluded
    result.actualPosition = GridSnapCalculator::getPositionFromCandidateIndex(
        node, newEdge, newSnapIndex, gridSize);
    result.newEdge = newEdge;
    result.newSnapIndex = newSnapIndex;

    // === Pre-validation: Check if valid path exists to this position ===
    // This prevents moving to blocked positions (same validation as SnapPointController)
    {
        // Handle self-loops: use SelfLoopRouter validation instead of A*
        // Self-loops can only use adjacent edges (not same or opposite edges)
        if (edge.from == edge.to) {
            NodeEdge otherNodeEdge = isSource ? edge.targetEdge : edge.sourceEdge;
            if (!SelfLoopRouter::isValidSelfLoopCombination(newEdge, otherNodeEdge)) {
                result.success = false;
                result.reason = "Self-loop requires adjacent edges (not same or opposite)";
                LOG_DEBUG("[moveSnapPoint] Self-loop validation failed: newEdge={} otherEdge={}",
                          static_cast<int>(newEdge), static_cast<int>(otherNodeEdge));
                return result;
            }
            LOG_DEBUG("[moveSnapPoint] Self-loop validation passed: newEdge={} otherEdge={}",
                      static_cast<int>(newEdge), static_cast<int>(otherNodeEdge));
        } else {
            // Regular edge: use A* pathfinding validation
            // Single Source of Truth: use edge's stored gridSize if available
            // This ensures consistent gridSize with the original layout creation
            float validationGridSize = edge.usedGridSize > 0.0f
                ? edge.usedGridSize
                : constants::effectiveGridSize(options.gridConfig.cellSize);

            // Build obstacle map
            // Include edge layouts in bounds calculation to prevent out-of-bounds segments
            ObstacleMap obstacles;
            obstacles.buildFromNodes(nodeLayouts, validationGridSize, 0, &edgeLayouts);
            obstacles.addEdgeSegments(edgeLayouts, edgeId);  // Other edges as obstacles
            
            // Get the other endpoint
            Point otherEndpoint = isSource ? edge.targetPoint : edge.sourcePoint;
            NodeEdge otherNodeEdge = isSource ? edge.targetEdge : edge.sourceEdge;
            NodeId otherNodeId = isSource ? edge.to : edge.from;
            
            // Calculate grid points
            GridPoint startGrid = obstacles.pixelToGrid(result.actualPosition);
            GridPoint goalGrid = obstacles.pixelToGrid(otherEndpoint);
            
            // Determine source/target for pathfinding
            NodeEdge srcEdge = isSource ? newEdge : otherNodeEdge;
            NodeEdge tgtEdge = isSource ? otherNodeEdge : newEdge;
            NodeId srcNode = isSource ? nodeId : otherNodeId;
            NodeId tgtNode = isSource ? otherNodeId : nodeId;
            GridPoint pathStart = isSource ? startGrid : goalGrid;
            GridPoint pathGoal = isSource ? goalGrid : startGrid;
            
            // Try A* pathfinding
            LOG_DEBUG("[CALLER:LayoutUtils.cpp] A* findPath called");
            AStarPathFinder pathFinder;
            auto pathResult = pathFinder.findPath(
                pathStart, pathGoal, obstacles,
                srcNode, tgtNode,
                srcEdge, tgtEdge,
                {}, {});
            
            if (!pathResult.found || pathResult.path.size() < 2) {
                result.success = false;
                result.reason = "No valid A* path to target position";
                return result;
            }
        }
    }

    result.success = true;

    // Check if another edge occupies the target position - if so, SWAP
    // Compare by computing snapIndex from position (position is the source of truth)
    EdgeId swapEdgeId = INVALID_EDGE;
    bool swapIsSource = false;

    for (auto& [eid, layout] : edgeLayouts) {
        if (eid == edgeId) continue;

        // Check source snap point (same node, same edge, same computed snapIndex)
        if (layout.from == nodeId && layout.sourceEdge == newEdge) {
            int otherSnapIndex = GridSnapCalculator::getCandidateIndexFromPosition(
                node, layout.sourceEdge, layout.sourcePoint, gridSize);
            if (otherSnapIndex == newSnapIndex) {
                swapEdgeId = eid;
                swapIsSource = true;
                break;
            }
        }

        // Check target snap point (same node, same edge, same computed snapIndex)
        if (layout.to == nodeId && layout.targetEdge == newEdge) {
            int otherSnapIndex = GridSnapCalculator::getCandidateIndexFromPosition(
                node, layout.targetEdge, layout.targetPoint, gridSize);
            if (otherSnapIndex == newSnapIndex) {
                swapEdgeId = eid;
                swapIsSource = false;
                break;
            }
        }
    }

    // Update the moving edge's snap point
    LOG_DEBUG("[moveSnapPoint] Edge {} BEFORE update: src=({},{}) tgt=({},{}) isSource={}",
              edgeId, edge.sourcePoint.x, edge.sourcePoint.y,
              edge.targetPoint.x, edge.targetPoint.y, isSource);
    LOG_DEBUG("[moveSnapPoint] actualPosition=({},{}) newEdge={} newSnapIndex={}",
              result.actualPosition.x, result.actualPosition.y,
              static_cast<int>(newEdge), newSnapIndex);

    if (isSource) {
        edge.sourceSnapIndex = newSnapIndex;
        edge.sourcePoint = result.actualPosition;
        edge.sourceEdge = newEdge;
    } else {
        edge.targetSnapIndex = newSnapIndex;
        edge.targetPoint = result.actualPosition;
        edge.targetEdge = newEdge;
    }

    LOG_DEBUG("[moveSnapPoint] Edge {} AFTER update: src=({},{}) tgt=({},{})",
              edgeId, edge.sourcePoint.x, edge.sourcePoint.y,
              edge.targetPoint.x, edge.targetPoint.y);

    // If swap partner found, move it to the original position
    if (swapEdgeId != INVALID_EDGE) {
        EdgeLayout& swapEdge = edgeLayouts[swapEdgeId];
        int originalSnapIndex = GridSnapCalculator::getCandidateIndexFromPosition(
            node, originalEdge, originalPosition, gridSize);
        if (swapIsSource) {
            swapEdge.sourceSnapIndex = originalSnapIndex;
            swapEdge.sourcePoint = originalPosition;
            swapEdge.sourceEdge = originalEdge;
        } else {
            swapEdge.targetSnapIndex = originalSnapIndex;
            swapEdge.targetPoint = originalPosition;
            swapEdge.targetEdge = originalEdge;
        }
        result.redistributedEdges.push_back(swapEdgeId);
    } else {
        // No swap needed - redistribute other snap points to avoid overlaps
        result.redistributedEdges = redistributeSnapPoints(
            nodeId, newEdge, edgeId, newPosition_,
            nodeLayouts, edgeLayouts, graph, options);
    }

    // Re-route the edge paths
    // IMPORTANT: We use regenerateBendPointsOnly (or custom callback) instead of
    // updateEdgeRoutingWithOptimization because moveSnapPoint explicitly sets the
    // user's desired position. The optimizer would overwrite the user's choice.
    std::vector<EdgeId> toReroute = {edgeId};
    toReroute.insert(toReroute.end(),
        result.redistributedEdges.begin(), result.redistributedEdges.end());

    LOG_DEBUG("[moveSnapPoint] BEFORE regenerateBendPointsOnly:");
    for (EdgeId eid : toReroute) {
        auto it = edgeLayouts.find(eid);
        if (it != edgeLayouts.end()) {
            LOG_DEBUG("  Edge {} src=({},{}) tgt=({},{}) bends={}",
                      eid, it->second.sourcePoint.x, it->second.sourcePoint.y,
                      it->second.targetPoint.x, it->second.targetPoint.y,
                      it->second.bendPoints.size());
        }
    }

    if (regenerator) {
        // Use custom callback (allows decoupling from EdgeRouting)
        regenerator(edgeLayouts, nodeLayouts, toReroute, options);
    } else {
        // Default: use EdgeRouting::regenerateBendPointsOnly
        EdgeRouting routing;
        routing.regenerateBendPointsOnly(edgeLayouts, nodeLayouts, toReroute, options);
    }

    LOG_DEBUG("[moveSnapPoint] AFTER regenerateBendPointsOnly:");
    for (EdgeId eid : toReroute) {
        auto it = edgeLayouts.find(eid);
        if (it != edgeLayouts.end()) {
            std::string bendStr;
            for (size_t i = 0; i < it->second.bendPoints.size(); ++i) {
                bendStr += " bend[" + std::to_string(i) + "]=(" 
                         + std::to_string(static_cast<int>(it->second.bendPoints[i].position.x)) + ","
                         + std::to_string(static_cast<int>(it->second.bendPoints[i].position.y)) + ")";
            }
            LOG_DEBUG("  Edge {} src=({},{}) tgt=({},{}){}",
                      eid, it->second.sourcePoint.x, it->second.sourcePoint.y,
                      it->second.targetPoint.x, it->second.targetPoint.y, bendStr);
        }
    }

    return result;
}

std::vector<EdgeId> LayoutUtils::redistributeSnapPoints(
    NodeId nodeId,
    NodeEdge edge,
    EdgeId fixedEdgeId,
    float fixedPosition,
    const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
    std::unordered_map<EdgeId, EdgeLayout>& edgeLayouts,
    [[maybe_unused]] const Graph& graph,
    const LayoutOptions& options) {

    std::vector<EdgeId> redistributed;

    auto nodeIt = nodeLayouts.find(nodeId);
    if (nodeIt == nodeLayouts.end()) {
        return redistributed;
    }

    const NodeLayout& node = nodeIt->second;

    // Collect all edges that have snap points on this edge of the node
    struct SnapInfo {
        EdgeId edgeId;
        bool isSource;
        float position;  // 0.0 to 1.0 along the edge
    };
    std::vector<SnapInfo> snapsOnEdge;

    for (auto& [eid, layout] : edgeLayouts) {
        // Check source snap
        if (layout.from == nodeId && layout.sourceEdge == edge) {
            float pos = 0.5f;
            // Calculate current position along edge
            if (edge == NodeEdge::Top || edge == NodeEdge::Bottom) {
                float edgeLength = node.size.width;
                if (edgeLength > 0) {
                    pos = (layout.sourcePoint.x - node.position.x) / edgeLength;
                }
            } else {
                float edgeLength = node.size.height;
                if (edgeLength > 0) {
                    pos = (layout.sourcePoint.y - node.position.y) / edgeLength;
                }
            }
            snapsOnEdge.push_back({eid, true, std::clamp(pos, 0.0f, 1.0f)});
        }

        // Check target snap
        if (layout.to == nodeId && layout.targetEdge == edge) {
            float pos = 0.5f;
            if (edge == NodeEdge::Top || edge == NodeEdge::Bottom) {
                float edgeLength = node.size.width;
                if (edgeLength > 0) {
                    pos = (layout.targetPoint.x - node.position.x) / edgeLength;
                }
            } else {
                float edgeLength = node.size.height;
                if (edgeLength > 0) {
                    pos = (layout.targetPoint.y - node.position.y) / edgeLength;
                }
            }
            snapsOnEdge.push_back({eid, false, std::clamp(pos, 0.0f, 1.0f)});
        }
    }

    if (snapsOnEdge.size() <= 1) {
        // Only one snap point on this edge, no redistribution needed
        return redistributed;
    }

    // Sort by position
    std::sort(snapsOnEdge.begin(), snapsOnEdge.end(),
        [](const SnapInfo& a, const SnapInfo& b) { return a.position < b.position; });

    // Find the fixed edge's position in the sorted list
    int fixedIndex = -1;
    for (size_t i = 0; i < snapsOnEdge.size(); ++i) {
        if (snapsOnEdge[i].edgeId == fixedEdgeId) {
            snapsOnEdge[i].position = fixedPosition;
            fixedIndex = static_cast<int>(i);
            break;
        }
    }

    if (fixedIndex < 0) {
        return redistributed;  // Fixed edge not found on this edge
    }

    // Re-sort after updating fixed position
    std::sort(snapsOnEdge.begin(), snapsOnEdge.end(),
        [](const SnapInfo& a, const SnapInfo& b) { return a.position < b.position; });

    // Minimum spacing between snap points (as fraction of edge length)
    constexpr float MIN_SPACING = 0.1f;
    constexpr float EDGE_MARGIN = 0.05f;  // Don't place snaps at very edges

    // Push neighbors away if too close
    // First pass: push right neighbors
    for (size_t i = 0; i + 1 < snapsOnEdge.size(); ++i) {
        if (snapsOnEdge[i].edgeId == fixedEdgeId) {
            // Don't move the fixed one, push the next one if too close
            float nextMinPos = snapsOnEdge[i].position + MIN_SPACING;
            if (snapsOnEdge[i + 1].position < nextMinPos) {
                snapsOnEdge[i + 1].position = std::min(nextMinPos, 1.0f - EDGE_MARGIN);
            }
        } else if (snapsOnEdge[i + 1].edgeId == fixedEdgeId) {
            // Next one is fixed, we might need to move this one left
            float maxPos = snapsOnEdge[i + 1].position - MIN_SPACING;
            if (snapsOnEdge[i].position > maxPos) {
                snapsOnEdge[i].position = std::max(maxPos, EDGE_MARGIN);
            }
        } else {
            // Neither is fixed, just ensure minimum spacing
            float nextMinPos = snapsOnEdge[i].position + MIN_SPACING;
            if (snapsOnEdge[i + 1].position < nextMinPos) {
                snapsOnEdge[i + 1].position = std::min(nextMinPos, 1.0f - EDGE_MARGIN);
            }
        }
    }

    // Second pass: push left neighbors (reverse)
    for (size_t i = snapsOnEdge.size() - 1; i > 0; --i) {
        float prevMaxPos = snapsOnEdge[i].position - MIN_SPACING;
        if (snapsOnEdge[i - 1].position > prevMaxPos && snapsOnEdge[i - 1].edgeId != fixedEdgeId) {
            snapsOnEdge[i - 1].position = std::max(prevMaxPos, EDGE_MARGIN);
        }
    }

    // Apply new positions
    float gridSize = constants::effectiveGridSize(options.gridConfig.cellSize);
    for (const auto& snap : snapsOnEdge) {
        if (snap.edgeId == fixedEdgeId) continue;  // Skip the fixed one

        auto edgeIt = edgeLayouts.find(snap.edgeId);
        if (edgeIt == edgeLayouts.end()) continue;

        Point newSnapPos = calculateSnapPointFromRatio(node, edge, snap.position, options.gridConfig.cellSize);
        int newSnapIndex = GridSnapCalculator::getCandidateIndexFromPosition(node, edge, newSnapPos, gridSize);

        int currentSnapIndex = snap.isSource ? edgeIt->second.sourceSnapIndex
                                             : edgeIt->second.targetSnapIndex;
        if (currentSnapIndex != newSnapIndex) {
            if (snap.isSource) {
                edgeIt->second.sourceSnapIndex = newSnapIndex;
                edgeIt->second.sourcePoint = newSnapPos;
            } else {
                edgeIt->second.targetSnapIndex = newSnapIndex;
                edgeIt->second.targetPoint = newSnapPos;
            }
            redistributed.push_back(snap.edgeId);
        }
    }

    return redistributed;
}

// ========== Direction Helpers ==========

MoveDirection LayoutUtils::getRequiredSourceDirection(NodeEdge sourceEdge) {
    switch (sourceEdge) {
        case NodeEdge::Top:    return MoveDirection::Up;
        case NodeEdge::Bottom: return MoveDirection::Down;
        case NodeEdge::Left:   return MoveDirection::Left;
        case NodeEdge::Right:  return MoveDirection::Right;
        default:               return MoveDirection::None;
    }
}

MoveDirection LayoutUtils::getRequiredTargetArrivalDirection(NodeEdge targetEdge) {
    // Arrival direction is opposite to the edge - we arrive "from" outside
    // e.g., entering Top edge means we come from above, so last move is Down
    switch (targetEdge) {
        case NodeEdge::Top:    return MoveDirection::Down;   // Enter from above
        case NodeEdge::Bottom: return MoveDirection::Up;     // Enter from below
        case NodeEdge::Left:   return MoveDirection::Right;  // Enter from left
        case NodeEdge::Right:  return MoveDirection::Left;   // Enter from right
        default:               return MoveDirection::None;
    }
}

MoveDirection LayoutUtils::getFirstSegmentDirection(const EdgeLayout& edge) {
    // First segment: source point to first bend (or target if no bends)
    Point p1 = edge.sourcePoint;
    Point p2 = edge.bendPoints.empty() ? edge.targetPoint : edge.bendPoints[0].position;

    float dx = p2.x - p1.x;
    float dy = p2.y - p1.y;

    // Determine primary direction
    if (std::abs(dx) > std::abs(dy)) {
        return dx > 0 ? MoveDirection::Right : MoveDirection::Left;
    } else if (std::abs(dy) > std::abs(dx)) {
        return dy > 0 ? MoveDirection::Down : MoveDirection::Up;
    }

    return MoveDirection::None;  // Zero-length or diagonal
}

MoveDirection LayoutUtils::getLastSegmentDirection(const EdgeLayout& edge) {
    // Last segment: last bend (or source if no bends) to target point
    Point p1 = edge.bendPoints.empty() ? edge.sourcePoint : edge.bendPoints.back().position;
    Point p2 = edge.targetPoint;

    float dx = p2.x - p1.x;
    float dy = p2.y - p1.y;

    // Determine primary direction
    if (std::abs(dx) > std::abs(dy)) {
        return dx > 0 ? MoveDirection::Right : MoveDirection::Left;
    } else if (std::abs(dy) > std::abs(dx)) {
        return dy > 0 ? MoveDirection::Down : MoveDirection::Up;
    }

    return MoveDirection::None;  // Zero-length or diagonal
}

}  // namespace arborvia
