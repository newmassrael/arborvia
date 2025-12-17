#pragma once

#include "Types.h"
#include "arborvia/layout/config/LayoutEnums.h"

#include <any>
#include <optional>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

namespace arborvia {

struct NodeData {
    NodeId id = INVALID_NODE;
    Size size = {100.0f, 50.0f};
    NodeType nodeType = NodeType::Regular;  ///< Explicit node type (Regular or Point)
    std::string label;
    std::any userData;

    NodeData() = default;
    explicit NodeData(Size s) : size(s), nodeType(inferNodeType(s)) {}
    explicit NodeData(NodeType type) : nodeType(type) {}
    NodeData(Size s, NodeType type) : size(s), nodeType(type) {}
    NodeData(Size s, std::string lbl) : size(s), nodeType(inferNodeType(s)), label(std::move(lbl)) {}
    NodeData(Size s, NodeType type, std::string lbl) : size(s), nodeType(type), label(std::move(lbl)) {}

private:
    /// Infer NodeType from size for backward compatibility
    /// Size {0,0} → Point, otherwise → Regular
    static NodeType inferNodeType(Size s) {
        return (s.width < 1.0f && s.height < 1.0f) ? NodeType::Point : NodeType::Regular;
    }
};

struct EdgeData {
    EdgeId id = INVALID_EDGE;
    NodeId from = INVALID_NODE;
    NodeId to = INVALID_NODE;
    std::string label;
    std::any userData;

    EdgeData() = default;
    EdgeData(NodeId f, NodeId t) : from(f), to(t) {}
    EdgeData(NodeId f, NodeId t, std::string lbl) : from(f), to(t), label(std::move(lbl)) {}
};

class Graph {
public:
    Graph() = default;
    virtual ~Graph() = default;

    // Node operations
    NodeId addNode();
    NodeId addNode(Size size);
    NodeId addNode(const std::string& label);
    NodeId addNode(Size size, const std::string& label);
    virtual NodeId addNode(const NodeData& data);

    void removeNode(NodeId id);
    bool hasNode(NodeId id) const;

    // Node access API:
    // - getNode(): Fast reference return. Use for guaranteed-valid IDs (e.g., from nodes() iteration).
    //   Throws std::out_of_range if ID is invalid.
    //   WARNING: Returned reference is invalidated by removeNode() or any graph modification!
    // - tryGetNode(): Safe optional return. Use for uncertain IDs (e.g., user input, after compact()).
    //   Returns std::nullopt if ID is invalid. Returns a copy, safe from graph modifications.
    const NodeData& getNode(NodeId id) const;
    NodeData& getNode(NodeId id);
    std::optional<NodeData> tryGetNode(NodeId id) const;

    void setNodeSize(NodeId id, Size size);
    void setNodeLabel(NodeId id, const std::string& label);

    // Edge operations
    EdgeId addEdge(NodeId from, NodeId to);
    EdgeId addEdge(NodeId from, NodeId to, const std::string& label);
    virtual EdgeId addEdge(const EdgeData& data);

    void removeEdge(EdgeId id);
    bool hasEdge(EdgeId id) const;

    // Edge access API:
    // - getEdge(): Fast reference return. Use for guaranteed-valid IDs (e.g., from edges() iteration).
    //   Throws std::out_of_range if ID is invalid.
    //   WARNING: Returned reference is invalidated by removeEdge() or any graph modification!
    // - tryGetEdge(): Safe optional return. Use for uncertain IDs (e.g., user input, after compact()).
    //   Returns std::nullopt if ID is invalid. Returns a copy, safe from graph modifications.
    const EdgeData& getEdge(EdgeId id) const;
    EdgeData& getEdge(EdgeId id);
    std::optional<EdgeData> tryGetEdge(EdgeId id) const;

    // Queries
    size_t nodeCount() const;
    size_t edgeCount() const;

    std::vector<NodeId> nodes() const;
    std::vector<EdgeId> edges() const;

    std::vector<NodeId> successors(NodeId id) const;
    std::vector<NodeId> predecessors(NodeId id) const;
    std::vector<EdgeId> outEdges(NodeId id) const;
    std::vector<EdgeId> inEdges(NodeId id) const;
    
    // Edge query methods for routing module integration
    std::vector<EdgeId> getOutEdges(NodeId id) const { return outEdges(id); }
    std::vector<EdgeId> getInEdges(NodeId id) const { return inEdges(id); }
    std::vector<EdgeId> getConnectedEdges(NodeId id) const;

    size_t inDegree(NodeId id) const;
    size_t outDegree(NodeId id) const;

    // Find edge between two nodes
    std::optional<EdgeId> findEdge(NodeId from, NodeId to) const;

    // Clear all
    virtual void clear();

    // Compact: Remove invalid nodes and edges from memory, remap all IDs to sequential 0,1,2...
    //
    // WARNING: This invalidates ALL existing NodeId and EdgeId references!
    // After calling compact(), any stored IDs become invalid and must not be used.
    //
    // Safe usage patterns:
    // 1. Use tryGetNode/tryGetEdge after compaction (returns nullopt for invalid IDs)
    // 2. Store stable identifiers in NodeData.userData before compaction
    // 3. Use NodeData.label to relocate nodes after compaction
    //
    // Example:
    //   NodeId saved = 5;
    //   graph.compact();
    //   graph.getNode(saved);  // CRASH: ID 5 may not exist!
    //
    //   // Safe approach:
    //   auto node = graph.tryGetNode(saved);
    //   if (!node) { saved = INVALID_NODE; }  // ID was invalidated
    void compact();

    // Dirty tracking for incremental updates
    bool isDirty() const { return dirty_; }
    void markDirty() { dirty_ = true; ++version_; }
    void markClean() { dirty_ = false; }
    uint64_t version() const { return version_; }

protected:
    void onGraphModified() { markDirty(); }
    std::vector<NodeData> nodes_;
    std::vector<EdgeData> edges_;
    std::vector<bool> nodeValid_;
    std::vector<bool> edgeValid_;

    // Adjacency lists
    std::unordered_map<NodeId, std::vector<EdgeId>> outEdges_;
    std::unordered_map<NodeId, std::vector<EdgeId>> inEdges_;

    NodeId nextNodeId_ = 0;
    EdgeId nextEdgeId_ = 0;
    size_t validNodeCount_ = 0;
    size_t validEdgeCount_ = 0;

    // Dirty tracking
    bool dirty_ = false;
    uint64_t version_ = 0;
};

}  // namespace arborvia
