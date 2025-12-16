#pragma once

#include "../../core/Types.h"
#include "LayoutTypes.h"

#include <optional>
#include <sstream>
#include <unordered_map>
#include <vector>

namespace arborvia {

// Forward declarations
class Graph;
struct ConstraintPlacementResult;

/// Positioned node in the layout result
struct NodeLayout {
    NodeId id = INVALID_NODE;
    Point position;           // Top-left corner for Regular, center for Point
    Size size;                // Node dimensions
    int layer = 0;            // Layer index in hierarchy
    int order = 0;            // Order within layer
    NodeType nodeType = NodeType::Regular;  // Geometry type
    
    /// Check if this is a point node (single snap point at center)
    bool isPointNode() const {
        return nodeType == NodeType::Point;
    }

    /// Check if this node should be treated as an obstacle for pathfinding
    ///
    /// Point nodes are NOT obstacles - edges converge on them freely.
    /// Regular nodes ARE obstacles - edges must route around them.
    bool isObstacle() const {
        return !isPointNode();
    }

    /// Check if this node should participate in overlap detection
    ///
    /// Point nodes do NOT participate - they have no area to overlap.
    /// Regular nodes DO participate - overlapping nodes is invalid.
    bool participatesInOverlapDetection() const {
        return !isPointNode();
    }

    Point center() const {
        if (nodeType == NodeType::Point) {
            return position;  // For point nodes, position IS the center
        }
        return {position.x + size.width / 2, position.y + size.height / 2};
    }
    
    /// Get bounds rectangle for collision/overlap detection
    ///
    /// For Regular nodes: position is top-left, returns {position, size}
    /// For Point nodes: position is center, returns area centered on position
    ///
    /// Note: Point nodes typically have size (0,0), which means bounds() returns
    /// a zero-area rectangle. This is intentional - Point nodes should not be
    /// treated as obstacles for A* pathfinding or overlap detection.
    Rect bounds() const {
        if (nodeType == NodeType::Point) {
            // Point node: position is center, so offset by half-size
            return {
                position.x - size.width / 2,
                position.y - size.height / 2,
                size.width,
                size.height
            };
        }
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
    
    // Snap point indices - stable identity for position calculation
    // Position = GridSnapCalculator::getPositionFromCandidateIndex(node, edge, snapIndex, gridSize)
    int sourceSnapIndex = -1;                 // -1 = not assigned, use position-based calculation
    int targetSnapIndex = -1;                 // -1 = not assigned, use position-based calculation
    
    // Channel routing information
    float channelY = -1.0f;                   // Channel Y position for orthogonal routing (-1 = not set)
    
    // Grid size used for A* pathfinding (Single Source of Truth)
    // This ensures consistent gridSize across layout creation and subsequent operations
    float usedGridSize = 0.0f;                // 0 = use default PATHFINDING_GRID_SIZE
    
    // Label positioning
    Point labelPosition;                      // Pre-computed label position (path midpoint)
    
    /// Get all points in order (source -> bends -> target)
    /// Note: Creates a new vector on each call. For performance-critical code,
    /// use forEachPoint() or forEachSegment() instead.
    std::vector<Point> allPoints() const {
        std::vector<Point> result;
        result.reserve(bendPoints.size() + 2);
        result.push_back(sourcePoint);
        for (const auto& bp : bendPoints) {
            result.push_back(bp.position);
        }
        result.push_back(targetPoint);
        return result;
    }

    /// Iterate over all points without allocation (source -> bends -> target)
    /// @param callback Function called for each point in order
    template<typename Func>
    void forEachPoint(Func&& callback) const {
        callback(sourcePoint);
        for (const auto& bp : bendPoints) {
            callback(bp.position);
        }
        callback(targetPoint);
    }

    /// Iterate over all segments without allocation
    /// @param callback Function called for each segment (p1, p2) in order
    template<typename Func>
    void forEachSegment(Func&& callback) const {
        Point prev = sourcePoint;
        for (const auto& bp : bendPoints) {
            callback(prev, bp.position);
            prev = bp.position;
        }
        callback(prev, targetPoint);
    }

    /// Get total number of points (source + bends + target)
    size_t pointCount() const {
        return bendPoints.size() + 2;
    }

    /// Get total number of segments
    size_t segmentCount() const {
        return bendPoints.size() + 1;
    }
};

/// Result of node placement validation
struct NodePlacementResult {
    bool success = false;           ///< True if placement is valid
    NodeId conflictingNode = INVALID_NODE;  ///< Node that would be overlapped (if any)
    std::string reason;             ///< Failure reason if !success
};

/// Complete layout result for a graph
class LayoutResult {
public:
    LayoutResult() = default;
    
    // ========== Validated Node Placement API (Single Source of Truth) ==========
    
    /// Check if a node can be placed at the given position without overlap
    /// @param layout The node layout to check
    /// @param margin Minimum gap between nodes (default 0)
    /// @return true if placement is valid (no overlap with existing nodes)
    bool canPlaceNode(const NodeLayout& layout, float margin = 0.0f) const;
    
    /// Try to place a node with validation - rejects overlapping placements
    /// This is the recommended API for adding nodes to ensure no overlaps
    /// @param layout The node layout to place
    /// @param margin Minimum gap between nodes (default 0)
    /// @return NodePlacementResult indicating success or failure with reason
    NodePlacementResult tryPlaceNode(const NodeLayout& layout, float margin = 0.0f);
    
    // ========== Constraint-Satisfying Placement API ==========
    
    /// Place a node at a position that satisfies all constraints (A* paths, no overlap)
    /// If the desired position violates constraints, automatically finds nearest valid position
    /// @param layout The node layout with desired position
    /// @param graph The graph for connectivity information
    /// @param gridSize Grid size for A* pathfinding (default 10.0)
    /// @return ConstraintPlacementResult with final position or failure reason
    struct ConstraintPlacementResult placeNodeWithConstraints(
        const NodeLayout& layout,
        const class Graph& graph,
        float gridSize = 10.0f);
    
    // ========== Legacy Node Operations (no validation) ==========
    // Note: setNodeLayout does NOT check for overlaps - prefer tryPlaceNode
    
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

    /// Get edges connected to a node (O(1) lookup using internal index)
    /// @param nodeId The node to query
    /// @return Vector of connected edge IDs (both incoming and outgoing)
    const std::vector<EdgeId>& getConnectedEdges(NodeId nodeId) const;

    /// Rebuild the node-to-edge index (called automatically on setEdgeLayout)
    /// Use this after bulk modifications via edgeLayouts() reference
    void rebuildEdgeIndex();

private:
    void updateEdgeIndex(EdgeId edgeId, const EdgeLayout& layout);
    void removeFromEdgeIndex(EdgeId edgeId);

    std::unordered_map<NodeId, NodeLayout> nodeLayouts_;
    std::unordered_map<EdgeId, EdgeLayout> edgeLayouts_;
    std::unordered_map<NodeId, std::vector<EdgeId>> nodeToEdges_;  ///< Index: node -> connected edges
    int layerCount_ = 0;
    
    static const std::vector<EdgeId> emptyEdgeList_;  ///< Empty list for nodes with no edges
};

}  // namespace arborvia
