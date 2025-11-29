#pragma once

#include "Types.h"

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
    std::string label;
    std::any userData;

    NodeData() = default;
    explicit NodeData(Size s) : size(s) {}
    NodeData(Size s, std::string lbl) : size(s), label(std::move(lbl)) {}
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

    const NodeData& getNode(NodeId id) const;
    NodeData& getNode(NodeId id);

    void setNodeSize(NodeId id, Size size);
    void setNodeLabel(NodeId id, const std::string& label);

    // Edge operations
    EdgeId addEdge(NodeId from, NodeId to);
    EdgeId addEdge(NodeId from, NodeId to, const std::string& label);
    virtual EdgeId addEdge(const EdgeData& data);

    void removeEdge(EdgeId id);
    bool hasEdge(EdgeId id) const;

    const EdgeData& getEdge(EdgeId id) const;
    EdgeData& getEdge(EdgeId id);

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
