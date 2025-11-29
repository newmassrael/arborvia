#include "EdgeRouting.h"
#include "arborvia/layout/ManualLayout.h"

#include <cmath>

namespace arborvia {
namespace algorithms {

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

void EdgeRouting::distributeAutoSnapPoints(
    Result& result,
    const std::unordered_map<NodeId, NodeLayout>& nodeLayouts) {
    
    // Group edges by (nodeId, nodeEdge) for sources and targets
    // Key: (nodeId, nodeEdge, isSource)
    std::map<std::tuple<NodeId, NodeEdge, bool>, std::vector<EdgeId>> edgeGroups;
    
    for (auto& [edgeId, layout] : result.edgeLayouts) {
        // Group by source node edge
        edgeGroups[{layout.from, layout.sourceEdge, true}].push_back(edgeId);
        // Group by target node edge
        edgeGroups[{layout.to, layout.targetEdge, false}].push_back(edgeId);
    }
    
    // For each group, distribute snap points evenly
    for (auto& [key, edges] : edgeGroups) {
        auto [nodeId, nodeEdge, isSource] = key;
        
        auto nodeIt = nodeLayouts.find(nodeId);
        if (nodeIt == nodeLayouts.end()) continue;
        
        const NodeLayout& node = nodeIt->second;
        int count = static_cast<int>(edges.size());
        
        // Assign snap indices to edges in this group
        for (int i = 0; i < count; ++i) {
            EdgeId edgeId = edges[i];
            EdgeLayout& layout = result.edgeLayouts[edgeId];
            
            // Calculate position along the edge (evenly distributed)
            float position = static_cast<float>(i + 1) / static_cast<float>(count + 1);
            
            Point snapPoint;
            switch (nodeEdge) {
                case NodeEdge::Top:
                    snapPoint = {
                        node.position.x + node.size.width * position,
                        node.position.y
                    };
                    break;
                case NodeEdge::Bottom:
                    snapPoint = {
                        node.position.x + node.size.width * position,
                        node.position.y + node.size.height
                    };
                    break;
                case NodeEdge::Left:
                    snapPoint = {
                        node.position.x,
                        node.position.y + node.size.height * position
                    };
                    break;
                case NodeEdge::Right:
                    snapPoint = {
                        node.position.x + node.size.width,
                        node.position.y + node.size.height * position
                    };
                    break;
            }
            
            if (isSource) {
                layout.sourcePoint = snapPoint;
                layout.sourceSnapIndex = i;
            } else {
                layout.targetPoint = snapPoint;
                layout.targetSnapIndex = i;
            }
        }
    }
    
    // Recalculate bend points after snap point redistribution
    for (auto& [edgeId, layout] : result.edgeLayouts) {
        layout.bendPoints.clear();
        
        // Simple orthogonal bend
        if (std::abs(layout.sourcePoint.x - layout.targetPoint.x) > 1.0f ||
            std::abs(layout.sourcePoint.y - layout.targetPoint.y) > 1.0f) {
            float midY = (layout.sourcePoint.y + layout.targetPoint.y) / 2.0f;
            layout.bendPoints.push_back({{layout.sourcePoint.x, midY}});
            layout.bendPoints.push_back({{layout.targetPoint.x, midY}});
        }
    }
}

}  // namespace algorithms
}  // namespace arborvia
