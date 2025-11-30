#include "EdgeRouting.h"
#include "arborvia/layout/LayoutTypes.h"
#include "arborvia/layout/LayoutUtils.h"

#include <cmath>
#include <set>

namespace arborvia {
namespace algorithms {

// =============================================================================
// Static Helper Function Implementations
// =============================================================================

Point EdgeRouting::calculateSnapPosition(const NodeLayout& node, NodeEdge edge, float position) {
    return LayoutUtils::calculateSnapPointFromPosition(node, edge, position);
}

float EdgeRouting::calculateRelativePosition(int snapIdx, int count, float rangeStart, float rangeEnd) {
    if (count <= 0) return (rangeStart + rangeEnd) / 2.0f;
    float range = rangeEnd - rangeStart;
    return rangeStart + static_cast<float>(snapIdx + 1) / static_cast<float>(count + 1) * range;
}

void EdgeRouting::recalculateBendPoints(EdgeLayout& layout) {
    layout.bendPoints.clear();
    
    // Simple orthogonal bend using Y midpoint
    if (std::abs(layout.sourcePoint.x - layout.targetPoint.x) > 1.0f ||
        std::abs(layout.sourcePoint.y - layout.targetPoint.y) > 1.0f) {
        float midY = (layout.sourcePoint.y + layout.targetPoint.y) / 2.0f;
        layout.bendPoints.push_back({{layout.sourcePoint.x, midY}});
        layout.bendPoints.push_back({{layout.targetPoint.x, midY}});
    }
}

std::pair<int, int> EdgeRouting::countConnectionsOnNodeEdge(
    const std::unordered_map<EdgeId, EdgeLayout>& edgeLayouts,
    NodeId nodeId,
    NodeEdge nodeEdge) {
    
    int inCount = 0;
    int outCount = 0;
    
    for (const auto& [edgeId, layout] : edgeLayouts) {
        // Outgoing from this node on this edge
        if (layout.from == nodeId && layout.sourceEdge == nodeEdge) {
            outCount++;
        }
        // Incoming to this node on this edge
        if (layout.to == nodeId && layout.targetEdge == nodeEdge) {
            inCount++;
        }
    }
    
    return {inCount, outCount};
}

// =============================================================================
// Edge Routing Core Methods
// =============================================================================

EdgeRouting::Result EdgeRouting::route(
    const Graph& graph,
    const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
    const std::unordered_set<EdgeId>& reversedEdges,
    const LayoutOptions& options) {
    
    Result result;
    
    for (EdgeId edgeId : graph.edges()) {
        const EdgeData& edge = graph.getEdge(edgeId);
        
        auto fromIt = nodeLayouts.find(edge.from);
        auto toIt = nodeLayouts.find(edge.to);
        
        if (fromIt == nodeLayouts.end() || toIt == nodeLayouts.end()) {
            continue;
        }
        
        bool isReversed = reversedEdges.count(edgeId) > 0;
        
        EdgeLayout layout;
        if (options.edgeRouting == arborvia::EdgeRouting::Orthogonal) {
            layout = routeOrthogonal(edge, fromIt->second, toIt->second, 
                                    isReversed, options);
        } else {
            layout = routePolyline(edge, fromIt->second, toIt->second, 
                                  isReversed, options);
        }
        
        result.edgeLayouts[edgeId] = layout;
    }
    
    return result;
}

EdgeLayout EdgeRouting::routeOrthogonal(
    const EdgeData& edge,
    const NodeLayout& fromLayout,
    const NodeLayout& toLayout,
    [[maybe_unused]] bool isReversed,
    const LayoutOptions& options) {
    
    EdgeLayout layout;
    layout.id = edge.id;
    layout.from = edge.from;
    layout.to = edge.to;
    
    // Compute source and target points
    Point fromCenter = fromLayout.center();
    Point toCenter = toLayout.center();
    
    // Determine connection points based on direction
    if (options.direction == Direction::TopToBottom || 
        options.direction == Direction::BottomToTop) {
        
        // Vertical layout: connect top/bottom of nodes
        if (fromCenter.y < toCenter.y) {
            // From is above To
            layout.sourcePoint = {fromCenter.x, fromLayout.position.y + fromLayout.size.height};
            layout.targetPoint = {toCenter.x, toLayout.position.y};
            layout.sourceEdge = NodeEdge::Bottom;
            layout.targetEdge = NodeEdge::Top;
        } else {
            // From is below To
            layout.sourcePoint = {fromCenter.x, fromLayout.position.y};
            layout.targetPoint = {toCenter.x, toLayout.position.y + toLayout.size.height};
            layout.sourceEdge = NodeEdge::Top;
            layout.targetEdge = NodeEdge::Bottom;
        }
        
        // Add bend points for orthogonal routing
        if (std::abs(layout.sourcePoint.x - layout.targetPoint.x) > 1.0f) {
            float midY = (layout.sourcePoint.y + layout.targetPoint.y) / 2.0f;
            layout.bendPoints.push_back({{layout.sourcePoint.x, midY}});
            layout.bendPoints.push_back({{layout.targetPoint.x, midY}});
        }
        
    } else {
        // Horizontal layout: connect left/right of nodes
        if (fromCenter.x < toCenter.x) {
            // From is left of To
            layout.sourcePoint = {fromLayout.position.x + fromLayout.size.width, fromCenter.y};
            layout.targetPoint = {toLayout.position.x, toCenter.y};
            layout.sourceEdge = NodeEdge::Right;
            layout.targetEdge = NodeEdge::Left;
        } else {
            // From is right of To
            layout.sourcePoint = {fromLayout.position.x, fromCenter.y};
            layout.targetPoint = {toLayout.position.x + toLayout.size.width, toCenter.y};
            layout.sourceEdge = NodeEdge::Left;
            layout.targetEdge = NodeEdge::Right;
        }
        
        // Add bend points for orthogonal routing
        if (std::abs(layout.sourcePoint.y - layout.targetPoint.y) > 1.0f) {
            float midX = (layout.sourcePoint.x + layout.targetPoint.x) / 2.0f;
            layout.bendPoints.push_back({{midX, layout.sourcePoint.y}});
            layout.bendPoints.push_back({{midX, layout.targetPoint.y}});
        }
    }
    
    return layout;
}

EdgeLayout EdgeRouting::routePolyline(
    const EdgeData& edge,
    const NodeLayout& fromLayout,
    const NodeLayout& toLayout,
    [[maybe_unused]] bool isReversed,
    [[maybe_unused]] const LayoutOptions& options) {
    
    EdgeLayout layout;
    layout.id = edge.id;
    layout.from = edge.from;
    layout.to = edge.to;
    
    Point fromCenter = fromLayout.center();
    Point toCenter = toLayout.center();
    
    // Compute connection points on node boundaries
    layout.sourcePoint = computeConnectionPoint(fromLayout, toCenter, true);
    layout.targetPoint = computeConnectionPoint(toLayout, fromCenter, false);
    
    // No bend points for straight polyline
    
    return layout;
}

Point EdgeRouting::computeConnectionPoint(
    const NodeLayout& node,
    const Point& targetCenter,
    [[maybe_unused]] bool isSource) {
    
    Point center = node.center();
    
    // Direction from center to target
    float dx = targetCenter.x - center.x;
    float dy = targetCenter.y - center.y;
    
    if (std::abs(dx) < 0.001f && std::abs(dy) < 0.001f) {
        // Target is at center, use bottom center
        return {center.x, node.position.y + node.size.height};
    }
    
    // Compute intersection with node boundary
    float halfWidth = node.size.width / 2.0f;
    float halfHeight = node.size.height / 2.0f;
    
    // Scale factor to reach boundary
    float scaleX = std::abs(dx) > 0.001f ? halfWidth / std::abs(dx) : 1000.0f;
    float scaleY = std::abs(dy) > 0.001f ? halfHeight / std::abs(dy) : 1000.0f;
    float scale = std::min(scaleX, scaleY);
    
    return {center.x + dx * scale, center.y + dy * scale};
}

Point EdgeRouting::computeSnapPoint(
    const NodeLayout& node,
    Direction direction,
    bool isSource) {
    
    Point center = node.center();
    
    switch (direction) {
        case Direction::TopToBottom:
            return isSource 
                ? Point{center.x, node.position.y + node.size.height}
                : Point{center.x, node.position.y};
        case Direction::BottomToTop:
            return isSource 
                ? Point{center.x, node.position.y}
                : Point{center.x, node.position.y + node.size.height};
        case Direction::LeftToRight:
            return isSource 
                ? Point{node.position.x + node.size.width, center.y}
                : Point{node.position.x, center.y};
        case Direction::RightToLeft:
            return isSource 
                ? Point{node.position.x, center.y}
                : Point{node.position.x + node.size.width, center.y};
    }
    
    return center;
}

// =============================================================================
// Snap Point Distribution Methods
// =============================================================================

void EdgeRouting::distributeAutoSnapPoints(
    Result& result,
    const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
    SnapDistribution distribution) {
    
    if (distribution == SnapDistribution::Unified) {
        // Unified mode: all connections on same edge distributed together
        // Key: (nodeId, nodeEdge) -> list of (edgeId, isSource)
        std::map<std::pair<NodeId, NodeEdge>, std::vector<std::pair<EdgeId, bool>>> allConnections;
        
        for (auto& [edgeId, layout] : result.edgeLayouts) {
            allConnections[{layout.from, layout.sourceEdge}].push_back({edgeId, true});
            allConnections[{layout.to, layout.targetEdge}].push_back({edgeId, false});
        }
        
        for (auto& [key, connections] : allConnections) {
            auto [nodeId, nodeEdge] = key;
            
            auto nodeIt = nodeLayouts.find(nodeId);
            if (nodeIt == nodeLayouts.end()) continue;
            
            const NodeLayout& node = nodeIt->second;
            int count = static_cast<int>(connections.size());
            
            for (int i = 0; i < count; ++i) {
                auto [edgeId, isSource] = connections[i];
                EdgeLayout& layout = result.edgeLayouts[edgeId];
                
                float position = calculateRelativePosition(i, count, 0.0f, 1.0f);
                Point snapPoint = calculateSnapPosition(node, nodeEdge, position);
                
                if (isSource) {
                    layout.sourcePoint = snapPoint;
                    layout.sourceSnapIndex = i;
                } else {
                    layout.targetPoint = snapPoint;
                    layout.targetSnapIndex = i;
                }
            }
        }
    } else {
        // Separated mode: incoming on first half, outgoing on second half
        // Key: (nodeId, nodeEdge) -> (incoming edges, outgoing edges)
        std::map<std::pair<NodeId, NodeEdge>, std::pair<std::vector<EdgeId>, std::vector<EdgeId>>> separatedConnections;
        
        for (auto& [edgeId, layout] : result.edgeLayouts) {
            // Outgoing from source node
            separatedConnections[{layout.from, layout.sourceEdge}].second.push_back(edgeId);
            // Incoming to target node
            separatedConnections[{layout.to, layout.targetEdge}].first.push_back(edgeId);
        }
        
        for (auto& [key, connections] : separatedConnections) {
            auto [nodeId, nodeEdge] = key;
            
            auto nodeIt = nodeLayouts.find(nodeId);
            if (nodeIt == nodeLayouts.end()) continue;
            
            const NodeLayout& node = nodeIt->second;
            auto& [incoming, outgoing] = connections;
            
            int inCount = static_cast<int>(incoming.size());
            int outCount = static_cast<int>(outgoing.size());
            
            // Determine ranges based on what exists
            float inStart = 0.0f, inEnd = 0.5f;
            float outStart = 0.5f, outEnd = 1.0f;
            
            if (inCount > 0 && outCount == 0) {
                inStart = 0.0f; inEnd = 1.0f;
            } else if (outCount > 0 && inCount == 0) {
                outStart = 0.0f; outEnd = 1.0f;
            }
            
            // Incoming edges
            for (int i = 0; i < inCount; ++i) {
                EdgeId edgeId = incoming[i];
                EdgeLayout& layout = result.edgeLayouts[edgeId];
                
                float position = calculateRelativePosition(i, inCount, inStart, inEnd);
                layout.targetPoint = calculateSnapPosition(node, nodeEdge, position);
                layout.targetSnapIndex = i;
            }
            
            // Outgoing edges
            for (int i = 0; i < outCount; ++i) {
                EdgeId edgeId = outgoing[i];
                EdgeLayout& layout = result.edgeLayouts[edgeId];
                
                float position = calculateRelativePosition(i, outCount, outStart, outEnd);
                layout.sourcePoint = calculateSnapPosition(node, nodeEdge, position);
                layout.sourceSnapIndex = i;
            }
        }
    }
    
    // Recalculate bend points after snap point redistribution
    for (auto& [edgeId, layout] : result.edgeLayouts) {
        recalculateBendPoints(layout);
    }
}

void EdgeRouting::updateEdgePositions(
    std::unordered_map<EdgeId, EdgeLayout>& edgeLayouts,
    const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
    const std::vector<EdgeId>& affectedEdges,
    SnapDistribution distribution,
    const std::unordered_set<NodeId>& movedNodes) {
    
    // Helper to check if a node should be updated
    auto shouldUpdateNode = [&movedNodes](NodeId nodeId) -> bool {
        return movedNodes.empty() || movedNodes.count(nodeId) > 0;
    };
    
    if (distribution == SnapDistribution::Unified) {
        // Unified mode: all connections on same edge distributed together
        std::map<std::pair<NodeId, NodeEdge>, std::vector<std::pair<EdgeId, bool>>> affectedConnections;
        
        for (EdgeId edgeId : affectedEdges) {
            auto it = edgeLayouts.find(edgeId);
            if (it == edgeLayouts.end()) continue;
            const EdgeLayout& layout = it->second;
            affectedConnections[{layout.from, layout.sourceEdge}].push_back({edgeId, true});
            affectedConnections[{layout.to, layout.targetEdge}].push_back({edgeId, false});
        }
        
        for (auto& [key, connections] : affectedConnections) {
            auto [nodeId, nodeEdge] = key;
            
            // Skip nodes that haven't moved
            if (!shouldUpdateNode(nodeId)) continue;
            
            auto nodeIt = nodeLayouts.find(nodeId);
            if (nodeIt == nodeLayouts.end()) continue;
            
            const NodeLayout& node = nodeIt->second;
            
            // Get TOTAL count from all edgeLayouts, not just affected edges
            auto [totalIn, totalOut] = countConnectionsOnNodeEdge(edgeLayouts, nodeId, nodeEdge);
            int totalCount = totalIn + totalOut;
            
            for (const auto& [edgeId, isSource] : connections) {
                EdgeLayout& layout = edgeLayouts[edgeId];
                
                // Use existing snap index
                int snapIdx = isSource ? layout.sourceSnapIndex : layout.targetSnapIndex;
                if (snapIdx < 0 || snapIdx >= totalCount) {
                    // Fallback: find index based on current position in connections
                    snapIdx = 0;
                    for (size_t i = 0; i < connections.size(); ++i) {
                        if (connections[i].first == edgeId) {
                            snapIdx = static_cast<int>(i);
                            break;
                        }
                    }
                }
                
                float position = calculateRelativePosition(snapIdx, totalCount, 0.0f, 1.0f);
                Point snapPoint = calculateSnapPosition(node, nodeEdge, position);
                
                if (isSource) {
                    layout.sourcePoint = snapPoint;
                } else {
                    layout.targetPoint = snapPoint;
                }
            }
        }
    } else {
        // Separated mode: incoming on first half, outgoing on second half
        std::map<std::pair<NodeId, NodeEdge>, std::pair<std::vector<EdgeId>, std::vector<EdgeId>>> affectedConnections;
        
        for (EdgeId edgeId : affectedEdges) {
            auto it = edgeLayouts.find(edgeId);
            if (it == edgeLayouts.end()) continue;
            const EdgeLayout& layout = it->second;
            // Outgoing from source node
            affectedConnections[{layout.from, layout.sourceEdge}].second.push_back(edgeId);
            // Incoming to target node
            affectedConnections[{layout.to, layout.targetEdge}].first.push_back(edgeId);
        }
        
        for (auto& [key, connections] : affectedConnections) {
            auto [nodeId, nodeEdge] = key;
            
            // Skip nodes that haven't moved
            if (!shouldUpdateNode(nodeId)) continue;
            
            auto nodeIt = nodeLayouts.find(nodeId);
            if (nodeIt == nodeLayouts.end()) continue;
            
            const NodeLayout& node = nodeIt->second;
            auto& [incoming, outgoing] = connections;
            
            // Get TOTAL counts from all edgeLayouts for this node edge
            auto [totalIn, totalOut] = countConnectionsOnNodeEdge(edgeLayouts, nodeId, nodeEdge);
            
            // Determine ranges based on TOTAL counts
            float inStart = 0.0f, inEnd = 0.5f;
            float outStart = 0.5f, outEnd = 1.0f;
            
            if (totalIn > 0 && totalOut == 0) {
                inStart = 0.0f; inEnd = 1.0f;
            } else if (totalOut > 0 && totalIn == 0) {
                outStart = 0.0f; outEnd = 1.0f;
            }
            
            // Update incoming edges (target point on this node)
            for (EdgeId edgeId : incoming) {
                EdgeLayout& layout = edgeLayouts[edgeId];
                
                // Use existing snap index, validate against TOTAL count
                int snapIdx = layout.targetSnapIndex;
                if (snapIdx < 0 || snapIdx >= totalIn) {
                    snapIdx = std::min(snapIdx, totalIn - 1);
                    if (snapIdx < 0) snapIdx = 0;
                }
                
                float position = calculateRelativePosition(snapIdx, totalIn, inStart, inEnd);
                layout.targetPoint = calculateSnapPosition(node, nodeEdge, position);
            }
            
            // Update outgoing edges (source point on this node)
            for (EdgeId edgeId : outgoing) {
                EdgeLayout& layout = edgeLayouts[edgeId];
                
                // Use existing snap index, validate against TOTAL count
                int snapIdx = layout.sourceSnapIndex;
                if (snapIdx < 0 || snapIdx >= totalOut) {
                    snapIdx = std::min(snapIdx, totalOut - 1);
                    if (snapIdx < 0) snapIdx = 0;
                }
                
                float position = calculateRelativePosition(snapIdx, totalOut, outStart, outEnd);
                layout.sourcePoint = calculateSnapPosition(node, nodeEdge, position);
            }
        }
    }
    
    // Recalculate bend points for affected edges
    for (EdgeId edgeId : affectedEdges) {
        auto it = edgeLayouts.find(edgeId);
        if (it == edgeLayouts.end()) continue;
        recalculateBendPoints(it->second);
    }
}

}  // namespace algorithms
}  // namespace arborvia
