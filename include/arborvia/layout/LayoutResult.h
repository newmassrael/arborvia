#pragma once

#include "../core/Types.h"
#include "LayoutTypes.h"

#include <optional>
#include <sstream>
#include <unordered_map>
#include <vector>

namespace arborvia {

/// Positioned node in the layout result
struct NodeLayout {
    NodeId id = INVALID_NODE;
    Point position;           // Top-left corner position
    Size size;                // Node dimensions
    int layer = 0;            // Layer index in hierarchy
    int order = 0;            // Order within layer
    
    Point center() const {
        return {position.x + size.width / 2, position.y + size.height / 2};
    }
    
    Rect bounds() const {
        return {position.x, position.y, size.width, size.height};
    }
};

// Note: BendPoint is defined in LayoutTypes.h

/// Positioned edge in the layout result
struct EdgeLayout {
    EdgeId id = INVALID_EDGE;
    NodeId from = INVALID_NODE;
    NodeId to = INVALID_NODE;
    
    Point sourcePoint;        // Connection point on source node
    Point targetPoint;        // Connection point on target node
    std::vector<BendPoint> bendPoints;  // Intermediate points
    
    // Edge routing information (which node edge is used)
    NodeEdge sourceEdge = NodeEdge::Bottom;  // Which edge of source node
    NodeEdge targetEdge = NodeEdge::Top;     // Which edge of target node
    int sourceSnapIndex = 0;                  // Which snap point on source edge
    int targetSnapIndex = 0;                  // Which snap point on target edge
    
    // Channel routing information
    float channelY = -1.0f;                   // Channel Y position for orthogonal routing (-1 = not set)
    
    // Label positioning
    Point labelPosition;                      // Pre-computed label position (path midpoint)
    
    /// Get all points in order (source -> bends -> target)
    std::vector<Point> allPoints() const {
        std::vector<Point> result;
        result.push_back(sourcePoint);
        for (const auto& bp : bendPoints) {
            result.push_back(bp.position);
        }
        result.push_back(targetPoint);
        return result;
    }
};

/// Complete layout result for a graph
class LayoutResult {
public:
    LayoutResult() = default;
    
    // Node layout operations
    void setNodeLayout(NodeId id, const NodeLayout& layout);
    const NodeLayout* getNodeLayout(NodeId id) const;
    NodeLayout* getNodeLayout(NodeId id);
    bool hasNodeLayout(NodeId id) const;
    
    // Edge layout operations
    void setEdgeLayout(EdgeId id, const EdgeLayout& layout);
    const EdgeLayout* getEdgeLayout(EdgeId id) const;
    EdgeLayout* getEdgeLayout(EdgeId id);
    bool hasEdgeLayout(EdgeId id) const;
    
    // Iteration
    const std::unordered_map<NodeId, NodeLayout>& nodeLayouts() const { return nodeLayouts_; }
    const std::unordered_map<EdgeId, EdgeLayout>& edgeLayouts() const { return edgeLayouts_; }
    
    // Bounds
    Rect computeBounds() const;
    Rect computeBounds(float padding) const;
    
    // Layer information
    void setLayerCount(int count) { layerCount_ = count; }
    int layerCount() const { return layerCount_; }
    
    std::vector<NodeId> nodesInLayer(int layer) const;
    
    // Statistics
    size_t nodeCount() const { return nodeLayouts_.size(); }
    size_t edgeCount() const { return edgeLayouts_.size(); }
    
    // Translate all positions
    void translate(float dx, float dy);
    
    // Clear all
    void clear();
    
    // Serialization (JSON format)
    std::string toJson() const;
    static LayoutResult fromJson(const std::string& json);

private:
    std::unordered_map<NodeId, NodeLayout> nodeLayouts_;
    std::unordered_map<EdgeId, EdgeLayout> edgeLayouts_;
    int layerCount_ = 0;
};

}  // namespace arborvia
