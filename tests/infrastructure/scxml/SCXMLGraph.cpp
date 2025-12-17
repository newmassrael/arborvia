#include "SCXMLGraph.h"
#include <iostream>
#include <stdexcept>

namespace arborvia::test::scxml {

NodeId SCXMLGraph::addState(const std::string& id, Size size) {
    NodeData data;
    data.size = size;
    data.label = id;
    
    NodeId nodeId = CompoundGraph::addNode(data);
    
    SCXMLNodeInfo info;
    info.type = SCXMLNodeType::State;  // Will be updated to Compound if children added
    info.scxmlId = id;
    nodeInfo_[nodeId] = info;
    scxmlIdToNode_[id] = nodeId;
    
    return nodeId;
}

NodeId SCXMLGraph::addParallelState(const std::string& id, Size size) {
    NodeData data;
    data.size = size;
    data.label = id;
    
    NodeId nodeId = addCompoundNode(data.size, CompoundType::Parallel);
    getNode(nodeId).label = id;
    
    SCXMLNodeInfo info;
    info.type = SCXMLNodeType::Parallel;
    info.scxmlId = id;
    nodeInfo_[nodeId] = info;
    scxmlIdToNode_[id] = nodeId;
    
    return nodeId;
}

NodeId SCXMLGraph::addFinalState(const std::string& id, Size size) {
    NodeData data;
    data.size = size;
    // Use id as label for regular nodes, empty for point nodes
    data.label = (size.width > 0 || size.height > 0) ? id : "";
    
    NodeId nodeId = CompoundGraph::addNode(data);
    
    SCXMLNodeInfo info;
    info.type = SCXMLNodeType::Final;
    info.scxmlId = id;
    nodeInfo_[nodeId] = info;
    scxmlIdToNode_[id] = nodeId;
    
    return nodeId;
}

NodeId SCXMLGraph::addInitialPseudo(const std::string& parentId) {
    std::string id = "_initial_" + std::to_string(initialPseudoCounter_++);
    if (!parentId.empty()) {
        id = parentId + id;
    }
    
    NodeData data;
    data.size = {0.0f, 0.0f};  // Point node - position is center
    data.label = "";  // No label for initial pseudo-state
    
    NodeId nodeId = CompoundGraph::addNode(data);
    
    SCXMLNodeInfo info;
    info.type = SCXMLNodeType::Initial;
    info.scxmlId = id;
    nodeInfo_[nodeId] = info;
    scxmlIdToNode_[id] = nodeId;
    
    return nodeId;
}

NodeId SCXMLGraph::addHistoryState(const std::string& id, bool deep) {
    NodeData data;
    data.size = {0.0f, 0.0f};  // Point node - position is center
    data.label = "";  // Label drawn in rendering
    
    NodeId nodeId = CompoundGraph::addNode(data);
    
    SCXMLNodeInfo info;
    info.type = deep ? SCXMLNodeType::HistoryDeep : SCXMLNodeType::History;
    info.scxmlId = id;
    nodeInfo_[nodeId] = info;
    scxmlIdToNode_[id] = nodeId;
    
    return nodeId;
}

EdgeId SCXMLGraph::addTransition(NodeId from, NodeId to,
                                  const std::string& event,
                                  const std::string& cond) {
    std::string label;
    if (!event.empty()) {
        label = event;
    }
    if (!cond.empty()) {
        label += (label.empty() ? "" : " ") + std::string("[") + cond + "]";
    }
    
    EdgeId edgeId = CompoundGraph::addEdge(from, to, label);
    
    SCXMLEdgeInfo info;
    info.type = SCXMLEdgeType::Transition;
    info.event = event;
    info.cond = cond;
    info.eventless = event.empty();
    edgeInfo_[edgeId] = info;
    
    return edgeId;
}

EdgeId SCXMLGraph::addTransition(const std::string& fromId, const std::string& toId,
                                  const std::string& event,
                                  const std::string& cond) {
    auto fromNode = findByScxmlId(fromId);
    auto toNode = findByScxmlId(toId);
    
    if (!fromNode) {
        throw std::runtime_error("Source node not found: " + fromId);
    }
    if (!toNode) {
        throw std::runtime_error("Target node not found: " + toId);
    }
    
    return addTransition(*fromNode, *toNode, event, cond);
}

EdgeId SCXMLGraph::addInitialEdge(NodeId initialPseudo, NodeId target) {
    EdgeId edgeId = CompoundGraph::addEdge(initialPseudo, target, "");
    
    SCXMLEdgeInfo info;
    info.type = SCXMLEdgeType::Initial;
    edgeInfo_[edgeId] = info;
    
    return edgeId;
}

std::optional<NodeId> SCXMLGraph::findByScxmlId(const std::string& scxmlId) const {
    auto it = scxmlIdToNode_.find(scxmlId);
    if (it != scxmlIdToNode_.end()) {
        return it->second;
    }
    return std::nullopt;
}

std::string SCXMLGraph::getScxmlId(NodeId id) const {
    auto it = nodeInfo_.find(id);
    if (it != nodeInfo_.end()) {
        return it->second.scxmlId;
    }
    return "";
}

SCXMLNodeType SCXMLGraph::getNodeType(NodeId id) const {
    auto it = nodeInfo_.find(id);
    if (it != nodeInfo_.end()) {
        return it->second.type;
    }
    return SCXMLNodeType::State;
}

SCXMLEdgeType SCXMLGraph::getEdgeType(EdgeId id) const {
    auto it = edgeInfo_.find(id);
    if (it != edgeInfo_.end()) {
        return it->second.type;
    }
    return SCXMLEdgeType::Transition;
}

const SCXMLNodeInfo* SCXMLGraph::getNodeInfo(NodeId id) const {
    auto it = nodeInfo_.find(id);
    return it != nodeInfo_.end() ? &it->second : nullptr;
}

SCXMLNodeInfo* SCXMLGraph::getNodeInfo(NodeId id) {
    auto it = nodeInfo_.find(id);
    return it != nodeInfo_.end() ? &it->second : nullptr;
}

const SCXMLEdgeInfo* SCXMLGraph::getEdgeInfo(EdgeId id) const {
    auto it = edgeInfo_.find(id);
    return it != edgeInfo_.end() ? &it->second : nullptr;
}

SCXMLEdgeInfo* SCXMLGraph::getEdgeInfo(EdgeId id) {
    auto it = edgeInfo_.find(id);
    return it != edgeInfo_.end() ? &it->second : nullptr;
}

bool SCXMLGraph::isInitialPseudo(NodeId id) const {
    return getNodeType(id) == SCXMLNodeType::Initial;
}

std::vector<NodeId> SCXMLGraph::getInitialPseudoStates() const {
    std::vector<NodeId> result;
    for (const auto& [nodeId, info] : nodeInfo_) {
        if (info.type == SCXMLNodeType::Initial) {
            result.push_back(nodeId);
        }
    }
    return result;
}

std::optional<NodeId> SCXMLGraph::getInitialState(NodeId compoundId) const {
    // First find the initial pseudo-state that's a child of this compound
    auto initialPseudo = getInitialPseudoFor(compoundId);
    if (!initialPseudo) {
        return std::nullopt;
    }
    
    // Find the outgoing edge from the initial pseudo-state
    auto successors = this->successors(*initialPseudo);
    if (successors.empty()) {
        return std::nullopt;
    }
    
    return successors[0];  // Return the first successor (initial target)
}

std::optional<NodeId> SCXMLGraph::getInitialPseudoFor(NodeId compoundId) const {
    // Look for an initial pseudo-state that's a child of this compound
    auto childIds = getChildren(compoundId);
    for (NodeId childId : childIds) {
        if (getNodeType(childId) == SCXMLNodeType::Initial) {
            return childId;
        }
    }
    return std::nullopt;
}

bool SCXMLGraph::isFinalState(NodeId id) const {
    return getNodeType(id) == SCXMLNodeType::Final;
}

bool SCXMLGraph::isHistoryState(NodeId id) const {
    auto type = getNodeType(id);
    return type == SCXMLNodeType::History || type == SCXMLNodeType::HistoryDeep;
}

void SCXMLGraph::addChild(const std::string& parentId, const std::string& childId) {
    auto parent = findByScxmlId(parentId);
    auto child = findByScxmlId(childId);
    
    if (!parent) {
        throw std::runtime_error("Parent node not found: " + parentId);
    }
    if (!child) {
        throw std::runtime_error("Child node not found: " + childId);
    }
    
    setParent(*child, *parent);
}

void SCXMLGraph::updateNodeTypes() {
    for (NodeId id : nodes()) {
        auto* info = getNodeInfo(id);
        if (!info) continue;
        
        // Skip special node types
        if (info->type == SCXMLNodeType::Final ||
            info->type == SCXMLNodeType::Initial ||
            info->type == SCXMLNodeType::History ||
            info->type == SCXMLNodeType::HistoryDeep ||
            info->type == SCXMLNodeType::Parallel) {
            continue;
        }
        
        // Update State to Compound if it has children
        if (hasChildren(id)) {
            info->type = SCXMLNodeType::State;  // Compound state
            setType(id, CompoundType::Compound);
        } else {
            info->type = SCXMLNodeType::State;  // Atomic state
            setType(id, CompoundType::Atomic);
        }
    }
}

void SCXMLGraph::dump() const {
    std::cout << "=== SCXMLGraph ===" << std::endl;
    std::cout << "Nodes: " << nodeCount() << ", Edges: " << edgeCount() << std::endl;
    
    std::cout << "\nNodes:" << std::endl;
    for (NodeId id : nodes()) {
        auto& node = getNode(id);
        auto* info = getNodeInfo(id);
        std::string scxmlId = getScxmlId(id);
        std::string displayLabel = !node.label.empty() ? node.label : scxmlId;
        std::cout << "  [" << id << "] " << displayLabel
                  << " (type=" << (info ? toString(info->type) : "?") << ")";
        
        auto parent = getParent(id);
        if (parent) {
            std::cout << " parent=" << getScxmlId(*parent);
        }
        
        auto children = getChildren(id);
        if (!children.empty()) {
            std::cout << " children=[";
            for (size_t i = 0; i < children.size(); ++i) {
                if (i > 0) std::cout << ", ";
                std::cout << getScxmlId(children[i]);
            }
            std::cout << "]";
        }
        std::cout << std::endl;
    }
    
    std::cout << "\nEdges:" << std::endl;
    for (EdgeId id : edges()) {
        auto& edge = getEdge(id);
        auto* info = getEdgeInfo(id);
        std::cout << "  [" << id << "] " << getScxmlId(edge.from) << " -> " << getScxmlId(edge.to);
        if (info) {
            std::cout << " (type=" << toString(info->type);
            if (!info->event.empty()) {
                std::cout << ", event=" << info->event;
            }
            if (!info->cond.empty()) {
                std::cout << ", cond=" << info->cond;
            }
            std::cout << ")";
        }
        std::cout << std::endl;
    }
    std::cout << "==================" << std::endl;
}

}  // namespace arborvia::test::scxml
