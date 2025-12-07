#include "arborvia/layout/LayoutUtils.h"
#include "arborvia/core/GeometryUtils.h"
#include "arborvia/core/Graph.h"
#include "arborvia/layout/ConstraintManager.h"
#include "arborvia/layout/ConstraintConfig.h"
#include "arborvia/layout/PathRoutingCoordinator.h"
#include "sugiyama/EdgeRouting.h"
#include <cmath>
#include <iostream>
#include <algorithm>

namespace arborvia {

namespace {
    constexpr float EPSILON_LEN2 = 0.0001f;  // Minimum squared length for valid segment

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

    // Check if any segment of an edge passes through a node's bounding box
    bool edgePassesThroughNode(const EdgeLayout& edge, const NodeLayout& node) {
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
}

// ========== Primary API ==========

MoveResult LayoutUtils::moveNode(
    NodeId nodeId,
    Point newPosition,
    std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
    std::unordered_map<EdgeId, EdgeLayout>& edgeLayouts,
    const Graph& graph,
    const LayoutOptions& options) {

    // Calculate forbidden zones for this node
    auto zones = ValidRegionCalculator::calculate(
        nodeId, nodeLayouts, edgeLayouts, options.gridConfig.cellSize);

    return moveNode(nodeId, newPosition, nodeLayouts, edgeLayouts, graph, options, zones);
}

MoveResult LayoutUtils::moveNode(
    NodeId nodeId,
    Point newPosition,
    std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
    std::unordered_map<EdgeId, EdgeLayout>& edgeLayouts,
    const Graph& graph,
    const LayoutOptions& options,
    const std::vector<ForbiddenZone>& preCalculatedZones) {

    MoveResult result;

    // Check if node exists
    auto nodeIt = nodeLayouts.find(nodeId);
    if (nodeIt == nodeLayouts.end()) {
        result.success = false;
        result.reason = "Node not found";
        return result;
    }

    // Validate position against forbidden zones
    auto validation = canMoveNodeTo(nodeId, newPosition, nodeLayouts, preCalculatedZones);
    if (!validation.valid) {
        result.success = false;
        result.reason = "Position blocked by forbidden zone";
        result.actualPosition = nodeIt->second.position;  // Keep current position
        return result;
    }

    // Move is valid - update position
    nodeIt->second.position = newPosition;
    result.actualPosition = newPosition;

    // Get connected edges and update routing
    result.affectedEdges = graph.getConnectedEdges(nodeId);

    // Update edge routing
    EdgeRouting routing;
    routing.updateEdgeRoutingWithOptimization(
        edgeLayouts, nodeLayouts, result.affectedEdges, options, {nodeId});

    result.success = true;
    return result;
}

void LayoutUtils::updateEdgePositions(
    std::unordered_map<EdgeId, EdgeLayout>& edgeLayouts,
    const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
    const std::vector<EdgeId>& affectedEdges,
    const std::unordered_set<NodeId>& movedNodes,
    float gridSize) {

    EdgeRouting routing;
    routing.updateSnapPositions(
        edgeLayouts, nodeLayouts, affectedEdges, movedNodes, gridSize);
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
    // Use postDragAlgorithm (AStar) instead of dragAlgorithm (which may be HideUntilDrop)
    LayoutOptions postDropOptions = options;
    if (options.optimizationOptions.postDragAlgorithm == PostDragAlgorithm::AStar) {
        postDropOptions.optimizationOptions.dragAlgorithm = DragAlgorithm::AStar;
    } else {
        // Fallback to Geometric if no post-drag algorithm configured
        postDropOptions.optimizationOptions.dragAlgorithm = DragAlgorithm::Geometric;
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
                    std::cout << "[LayoutUtils] Edge " << edgeId 
                              << " passes through moved node " << movedNodeId 
                              << " - will recalculate with A*" << std::endl;
                    break;  // No need to check other moved nodes
                }
            }
        }
    }
    
    // Recalculate edges passing through moved nodes' forbidden zones
    if (!edgesToRecalculate.empty()) {
        std::cout << "[LayoutUtils] Re-routing " << edgesToRecalculate.size() 
                  << " edges using A*" << std::endl;
        routing.updateEdgeRoutingWithOptimization(
            edgeLayouts, nodeLayouts, edgesToRecalculate, postDropOptions, movedNodes);
    }
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

    // Use ValidRegionCalculator as single source of truth
    auto nodeIt = nodeLayouts.find(nodeId);
    if (nodeIt == nodeLayouts.end()) {
        return DragValidation{true, {}};  // Node not found, allow
    }

    auto zones = ValidRegionCalculator::calculate(nodeId, nodeLayouts, edgeLayouts, gridSize);
    bool valid = ValidRegionCalculator::isValid(newPosition, nodeIt->second.size, zones);

    DragValidation result;
    result.valid = valid;
    return result;
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

LayoutUtils::DragValidation LayoutUtils::canMoveNodeTo(
    NodeId nodeId,
    Point newPosition,
    const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
    const std::vector<ForbiddenZone>& zones) {

    auto nodeIt = nodeLayouts.find(nodeId);
    if (nodeIt == nodeLayouts.end()) {
        return DragValidation{true, {}};
    }

    bool valid = ValidRegionCalculator::isValid(newPosition, nodeIt->second.size, zones);
    return DragValidation{valid, {}};
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
    const LayoutOptions& options) {

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

    // Find which edge and position the new point maps to
    auto [newEdge, newPosition_] = findClosestNodeEdge(newPosition, node);

    // Calculate the actual snap point position on the node edge
    result.actualPosition = calculateSnapPointFromPosition(node, newEdge, newPosition_);
    result.newEdge = newEdge;
    result.success = true;

    // Update the edge's snap point
    if (isSource) {
        edge.sourcePoint = result.actualPosition;
        edge.sourceEdge = newEdge;
    } else {
        edge.targetPoint = result.actualPosition;
        edge.targetEdge = newEdge;
    }

    // Redistribute other snap points on this edge to avoid overlaps
    result.redistributedEdges = redistributeSnapPoints(
        nodeId, newEdge, edgeId, newPosition_,
        nodeLayouts, edgeLayouts, graph, options);

    // Re-route the edge path
    EdgeRouting routing;
    std::vector<EdgeId> toReroute = {edgeId};
    toReroute.insert(toReroute.end(),
        result.redistributedEdges.begin(), result.redistributedEdges.end());

    routing.updateEdgeRoutingWithOptimization(
        edgeLayouts, nodeLayouts, toReroute, options, {});

    return result;
}

std::vector<EdgeId> LayoutUtils::redistributeSnapPoints(
    NodeId nodeId,
    NodeEdge edge,
    EdgeId fixedEdgeId,
    float fixedPosition,
    const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
    std::unordered_map<EdgeId, EdgeLayout>& edgeLayouts,
    const Graph& graph,
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
    for (const auto& snap : snapsOnEdge) {
        if (snap.edgeId == fixedEdgeId) continue;  // Skip the fixed one

        auto edgeIt = edgeLayouts.find(snap.edgeId);
        if (edgeIt == edgeLayouts.end()) continue;

        Point newSnapPos = calculateSnapPointFromPosition(node, edge, snap.position);

        if (snap.isSource) {
            if (edgeIt->second.sourcePoint.x != newSnapPos.x ||
                edgeIt->second.sourcePoint.y != newSnapPos.y) {
                edgeIt->second.sourcePoint = newSnapPos;
                redistributed.push_back(snap.edgeId);
            }
        } else {
            if (edgeIt->second.targetPoint.x != newSnapPos.x ||
                edgeIt->second.targetPoint.y != newSnapPos.y) {
                edgeIt->second.targetPoint = newSnapPos;
                redistributed.push_back(snap.edgeId);
            }
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
