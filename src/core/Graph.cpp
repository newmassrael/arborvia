#include "arborvia/core/Graph.h"

#include <algorithm>

namespace arborvia {

NodeId Graph::addNode() {
    return addNode(NodeData{});
}

NodeId Graph::addNode(Size size) {
    NodeData data;
    data.size = size;
    return addNode(data);
}

NodeId Graph::addNode(const std::string& label) {
    NodeData data;
    data.label = label;
    return addNode(data);
}

NodeId Graph::addNode(Size size, const std::string& label) {
    NodeData data;
    data.size = size;
    data.label = label;
    return addNode(data);
}

NodeId Graph::addNode(const NodeData& data) {
    NodeId id = nextNodeId_++;

    NodeData nodeData = data;
    nodeData.id = id;

    if (id < nodes_.size()) {
        nodes_[id] = std::move(nodeData);
        nodeValid_[id] = true;
    } else {
        nodes_.push_back(std::move(nodeData));
        nodeValid_.push_back(true);
    }

    outEdges_[id] = {};
    inEdges_[id] = {};
    ++validNodeCount_;

    onGraphModified();
    return id;
}

void Graph::removeNode(NodeId id) {
    if (!hasNode(id)) return;

    // Remove all edges connected to this node
    auto outEdgesCopy = outEdges_[id];
    for (EdgeId edgeId : outEdgesCopy) {
        removeEdge(edgeId);
    }

    auto inEdgesCopy = inEdges_[id];
    for (EdgeId edgeId : inEdgesCopy) {
        removeEdge(edgeId);
    }

    nodeValid_[id] = false;
    outEdges_.erase(id);
    inEdges_.erase(id);
    --validNodeCount_;

    onGraphModified();
}

bool Graph::hasNode(NodeId id) const {
    return id < nodeValid_.size() && nodeValid_[id];
}

const NodeData& Graph::getNode(NodeId id) const {
    if (!hasNode(id)) {
        throw std::out_of_range("Invalid node ID: " + std::to_string(id));
    }
    return nodes_[id];
}

NodeData& Graph::getNode(NodeId id) {
    if (!hasNode(id)) {
        throw std::out_of_range("Invalid node ID: " + std::to_string(id));
    }
    return nodes_[id];
}

std::optional<NodeData> Graph::tryGetNode(NodeId id) const {
    if (!hasNode(id)) {
        return std::nullopt;
    }
    return nodes_[id];
}



void Graph::setNodeSize(NodeId id, Size size) {
    getNode(id).size = size;
    onGraphModified();
}

void Graph::setNodeLabel(NodeId id, const std::string& label) {
    getNode(id).label = label;
    onGraphModified();
}

EdgeId Graph::addEdge(NodeId from, NodeId to) {
    EdgeData data;
    data.from = from;
    data.to = to;
    return addEdge(data);
}

EdgeId Graph::addEdge(NodeId from, NodeId to, const std::string& label) {
    EdgeData data;
    data.from = from;
    data.to = to;
    data.label = label;
    return addEdge(data);
}

EdgeId Graph::addEdge(const EdgeData& data) {
    if (!hasNode(data.from) || !hasNode(data.to)) {
        throw std::invalid_argument("Invalid node ID in edge");
    }

    EdgeId id = nextEdgeId_++;

    EdgeData edgeData = data;
    edgeData.id = id;

    if (id < edges_.size()) {
        edges_[id] = std::move(edgeData);
        edgeValid_[id] = true;
    } else {
        edges_.push_back(std::move(edgeData));
        edgeValid_.push_back(true);
    }

    outEdges_[data.from].push_back(id);
    inEdges_[data.to].push_back(id);
    ++validEdgeCount_;

    onGraphModified();
    return id;
}

void Graph::removeEdge(EdgeId id) {
    if (!hasEdge(id)) return;

    const EdgeData& edge = edges_[id];

    // Remove from adjacency lists
    auto& outList = outEdges_[edge.from];
    outList.erase(std::remove(outList.begin(), outList.end(), id), outList.end());

    auto& inList = inEdges_[edge.to];
    inList.erase(std::remove(inList.begin(), inList.end(), id), inList.end());

    edgeValid_[id] = false;
    --validEdgeCount_;

    onGraphModified();
}

bool Graph::hasEdge(EdgeId id) const {
    return id < edgeValid_.size() && edgeValid_[id];
}

const EdgeData& Graph::getEdge(EdgeId id) const {
    if (!hasEdge(id)) {
        throw std::out_of_range("Invalid edge ID: " + std::to_string(id));
    }
    return edges_[id];
}

EdgeData& Graph::getEdge(EdgeId id) {
    if (!hasEdge(id)) {
        throw std::out_of_range("Invalid edge ID: " + std::to_string(id));
    }
    return edges_[id];
}

std::optional<EdgeData> Graph::tryGetEdge(EdgeId id) const {
    if (!hasEdge(id)) {
        return std::nullopt;
    }
    return edges_[id];
}



size_t Graph::nodeCount() const {
    return validNodeCount_;
}

size_t Graph::edgeCount() const {
    return validEdgeCount_;
}

std::vector<NodeId> Graph::nodes() const {
    std::vector<NodeId> result;
    result.reserve(validNodeCount_);
    for (size_t i = 0; i < nodeValid_.size(); ++i) {
        if (nodeValid_[i]) {
            result.push_back(static_cast<NodeId>(i));
        }
    }
    return result;
}

std::vector<EdgeId> Graph::edges() const {
    std::vector<EdgeId> result;
    result.reserve(validEdgeCount_);
    for (size_t i = 0; i < edgeValid_.size(); ++i) {
        if (edgeValid_[i]) {
            result.push_back(static_cast<EdgeId>(i));
        }
    }
    return result;
}

std::vector<NodeId> Graph::successors(NodeId id) const {
    std::vector<NodeId> result;
    if (!hasNode(id)) return result;

    auto it = outEdges_.find(id);
    if (it != outEdges_.end()) {
        result.reserve(it->second.size());
        for (EdgeId edgeId : it->second) {
            if (hasEdge(edgeId)) {
                result.push_back(edges_[edgeId].to);
            }
        }
    }
    return result;
}

std::vector<NodeId> Graph::predecessors(NodeId id) const {
    std::vector<NodeId> result;
    if (!hasNode(id)) return result;

    auto it = inEdges_.find(id);
    if (it != inEdges_.end()) {
        result.reserve(it->second.size());
        for (EdgeId edgeId : it->second) {
            if (hasEdge(edgeId)) {
                result.push_back(edges_[edgeId].from);
            }
        }
    }
    return result;
}

std::vector<EdgeId> Graph::outEdges(NodeId id) const {
    std::vector<EdgeId> result;
    if (!hasNode(id)) return result;

    auto it = outEdges_.find(id);
    if (it != outEdges_.end()) {
        for (EdgeId edgeId : it->second) {
            if (hasEdge(edgeId)) {
                result.push_back(edgeId);
            }
        }
    }
    return result;
}

std::vector<EdgeId> Graph::inEdges(NodeId id) const {
    std::vector<EdgeId> result;
    if (!hasNode(id)) return result;

    auto it = inEdges_.find(id);
    if (it != inEdges_.end()) {
        for (EdgeId edgeId : it->second) {
            if (hasEdge(edgeId)) {
                result.push_back(edgeId);
            }
        }
    }
    return result;
}

std::vector<EdgeId> Graph::getConnectedEdges(NodeId id) const {
    std::vector<EdgeId> result;
    if (!hasNode(id)) return result;
    
    // Get outgoing edges
    auto outIt = outEdges_.find(id);
    if (outIt != outEdges_.end()) {
        for (EdgeId edgeId : outIt->second) {
            if (hasEdge(edgeId)) {
                result.push_back(edgeId);
            }
        }
    }
    
    // Get incoming edges (avoid duplicates for self-loops)
    auto inIt = inEdges_.find(id);
    if (inIt != inEdges_.end()) {
        for (EdgeId edgeId : inIt->second) {
            if (hasEdge(edgeId)) {
                // Check if this is a self-loop (already added via outEdges)
                const EdgeData& edge = edges_[edgeId];
                if (edge.from != id) {
                    result.push_back(edgeId);
                }
            }
        }
    }
    
    return result;
}

size_t Graph::inDegree(NodeId id) const {
    return inEdges(id).size();
}

size_t Graph::outDegree(NodeId id) const {
    return outEdges(id).size();
}

std::optional<EdgeId> Graph::findEdge(NodeId from, NodeId to) const {
    if (!hasNode(from) || !hasNode(to)) return std::nullopt;

    auto it = outEdges_.find(from);
    if (it != outEdges_.end()) {
        for (EdgeId edgeId : it->second) {
            if (hasEdge(edgeId) && edges_[edgeId].to == to) {
                return edgeId;
            }
        }
    }
    return std::nullopt;
}

void Graph::clear() {
    nodes_.clear();
    edges_.clear();
    nodeValid_.clear();
    edgeValid_.clear();
    outEdges_.clear();
    inEdges_.clear();
    nextNodeId_ = 0;
    nextEdgeId_ = 0;
    validNodeCount_ = 0;
    validEdgeCount_ = 0;

    onGraphModified();
}

void Graph::compact() {
    // Build node ID mapping: old ID -> new ID
    std::unordered_map<NodeId, NodeId> nodeMapping;
    std::vector<NodeData> newNodes;
    std::vector<bool> newNodeValid;
    NodeId newNodeId = 0;
    
    for (size_t i = 0; i < nodeValid_.size(); ++i) {
        if (nodeValid_[i]) {
            nodeMapping[static_cast<NodeId>(i)] = newNodeId;
            NodeData newNode = nodes_[i];
            newNode.id = newNodeId;
            newNodes.push_back(std::move(newNode));
            newNodeValid.push_back(true);
            ++newNodeId;
        }
    }
    
    // Build edge ID mapping: old ID -> new ID
    std::unordered_map<EdgeId, EdgeId> edgeMapping;
    std::vector<EdgeData> newEdges;
    std::vector<bool> newEdgeValid;
    EdgeId newEdgeId = 0;
    
    for (size_t i = 0; i < edgeValid_.size(); ++i) {
        if (edgeValid_[i]) {
            edgeMapping[static_cast<EdgeId>(i)] = newEdgeId;
            EdgeData newEdge = edges_[i];
            newEdge.id = newEdgeId;
            
            // Remap node IDs
            auto fromIt = nodeMapping.find(newEdge.from);
            auto toIt = nodeMapping.find(newEdge.to);
            if (fromIt != nodeMapping.end() && toIt != nodeMapping.end()) {
                newEdge.from = fromIt->second;
                newEdge.to = toIt->second;
                newEdges.push_back(std::move(newEdge));
                newEdgeValid.push_back(true);
                ++newEdgeId;
            }
        }
    }
    
    // Rebuild adjacency lists
    std::unordered_map<NodeId, std::vector<EdgeId>> newOutEdges;
    std::unordered_map<NodeId, std::vector<EdgeId>> newInEdges;
    
    for (const auto& edge : newEdges) {
        newOutEdges[edge.from].push_back(edge.id);
        newInEdges[edge.to].push_back(edge.id);
    }
    
    // Save counts before move
    size_t newNodeCount = newNodes.size();
    size_t newEdgeCount = newEdges.size();
    
    // Replace old data with compacted data
    nodes_ = std::move(newNodes);
    nodeValid_ = std::move(newNodeValid);
    edges_ = std::move(newEdges);
    edgeValid_ = std::move(newEdgeValid);
    outEdges_ = std::move(newOutEdges);
    inEdges_ = std::move(newInEdges);
    
    nextNodeId_ = newNodeId;
    nextEdgeId_ = newEdgeId;
    validNodeCount_ = newNodeCount;
    validEdgeCount_ = newEdgeCount;
    
    onGraphModified();
}

}  // namespace arborvia
