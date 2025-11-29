#include "arborvia/core/CompoundGraph.h"

#include <algorithm>
#include <queue>

namespace arborvia {

NodeId CompoundGraph::addNode(const NodeData& data) {
    NodeId id = Graph::addNode(data);
    ensureCompoundInfo(id);
    return id;
}

NodeId CompoundGraph::addCompoundNode(CompoundType type) {
    NodeId id = Graph::addNode();
    ensureCompoundInfo(id);
    compoundInfo_[id].type = type;
    return id;
}

NodeId CompoundGraph::addCompoundNode(Size size, CompoundType type) {
    NodeId id = Graph::addNode(size);
    ensureCompoundInfo(id);
    compoundInfo_[id].type = type;
    return id;
}

NodeId CompoundGraph::addCompoundNode(const std::string& label, CompoundType type) {
    NodeId id = Graph::addNode(label);
    ensureCompoundInfo(id);
    compoundInfo_[id].type = type;
    return id;
}

void CompoundGraph::setParent(NodeId child, NodeId parent) {
    if (!hasNode(child) || !hasNode(parent)) {
        throw std::invalid_argument("Invalid node ID");
    }

    if (child == parent) {
        throw std::invalid_argument("Node cannot be its own parent");
    }

    // Check for cycle: parent cannot be a descendant of child
    if (isAncestorOf(child, parent)) {
        throw std::invalid_argument("Setting parent would create a cycle");
    }

    ensureCompoundInfo(child);
    ensureCompoundInfo(parent);

    // Remove from current parent if any
    removeFromParent(child);

    // Set new parent
    compoundInfo_[child].parent = parent;
    compoundInfo_[parent].children.push_back(child);

    // Parent becomes compound if it was atomic
    if (compoundInfo_[parent].type == CompoundType::Atomic) {
        compoundInfo_[parent].type = CompoundType::Compound;
    }
}

void CompoundGraph::removeFromParent(NodeId child) {
    if (!hasNode(child)) return;

    auto it = compoundInfo_.find(child);
    if (it == compoundInfo_.end() || !it->second.parent.has_value()) {
        return;
    }

    NodeId parentId = it->second.parent.value();
    it->second.parent = std::nullopt;

    auto parentIt = compoundInfo_.find(parentId);
    if (parentIt != compoundInfo_.end()) {
        auto& children = parentIt->second.children;
        children.erase(std::remove(children.begin(), children.end(), child), children.end());

        // If parent has no more children, it becomes atomic
        if (children.empty() && parentIt->second.type != CompoundType::Parallel) {
            parentIt->second.type = CompoundType::Atomic;
        }
    }
}

std::optional<NodeId> CompoundGraph::getParent(NodeId id) const {
    auto it = compoundInfo_.find(id);
    if (it != compoundInfo_.end()) {
        return it->second.parent;
    }
    return std::nullopt;
}

std::vector<NodeId> CompoundGraph::getChildren(NodeId id) const {
    auto it = compoundInfo_.find(id);
    if (it != compoundInfo_.end()) {
        return it->second.children;
    }
    return {};
}

std::vector<NodeId> CompoundGraph::getDescendants(NodeId id) const {
    std::vector<NodeId> result;
    std::queue<NodeId> queue;

    for (NodeId child : getChildren(id)) {
        queue.push(child);
    }

    while (!queue.empty()) {
        NodeId current = queue.front();
        queue.pop();
        result.push_back(current);

        for (NodeId child : getChildren(current)) {
            queue.push(child);
        }
    }

    return result;
}

std::vector<NodeId> CompoundGraph::getAncestors(NodeId id) const {
    std::vector<NodeId> result;
    auto parent = getParent(id);
    while (parent.has_value()) {
        result.push_back(parent.value());
        parent = getParent(parent.value());
    }
    return result;
}

bool CompoundGraph::isCompound(NodeId id) const {
    auto it = compoundInfo_.find(id);
    return it != compoundInfo_.end() && it->second.type == CompoundType::Compound;
}

bool CompoundGraph::isParallel(NodeId id) const {
    auto it = compoundInfo_.find(id);
    return it != compoundInfo_.end() && it->second.type == CompoundType::Parallel;
}

bool CompoundGraph::isAtomic(NodeId id) const {
    auto it = compoundInfo_.find(id);
    return it == compoundInfo_.end() || it->second.type == CompoundType::Atomic;
}

bool CompoundGraph::isRoot(NodeId id) const {
    return hasNode(id) && !getParent(id).has_value();
}

bool CompoundGraph::hasChildren(NodeId id) const {
    auto it = compoundInfo_.find(id);
    return it != compoundInfo_.end() && !it->second.children.empty();
}

int CompoundGraph::getDepth(NodeId id) const {
    int depth = 0;
    auto parent = getParent(id);
    while (parent.has_value()) {
        ++depth;
        parent = getParent(parent.value());
    }
    return depth;
}

CompoundType CompoundGraph::getType(NodeId id) const {
    auto it = compoundInfo_.find(id);
    if (it != compoundInfo_.end()) {
        return it->second.type;
    }
    return CompoundType::Atomic;
}

void CompoundGraph::setType(NodeId id, CompoundType type) {
    if (!hasNode(id)) {
        throw std::invalid_argument("Invalid node ID");
    }
    ensureCompoundInfo(id);
    compoundInfo_[id].type = type;
}

std::vector<NodeId> CompoundGraph::rootNodes() const {
    std::vector<NodeId> result;
    for (NodeId id : nodes()) {
        if (isRoot(id)) {
            result.push_back(id);
        }
    }
    return result;
}

void CompoundGraph::setCollapsed(NodeId id, bool collapsed) {
    if (!hasNode(id)) return;
    ensureCompoundInfo(id);
    compoundInfo_[id].collapsed = collapsed;
}

bool CompoundGraph::isCollapsed(NodeId id) const {
    auto it = compoundInfo_.find(id);
    return it != compoundInfo_.end() && it->second.collapsed;
}

bool CompoundGraph::isVisible(NodeId id) const {
    if (!hasNode(id)) return false;

    // Check if any ancestor is collapsed
    auto parent = getParent(id);
    while (parent.has_value()) {
        if (isCollapsed(parent.value())) {
            return false;
        }
        parent = getParent(parent.value());
    }
    return true;
}

std::vector<NodeId> CompoundGraph::visibleNodes() const {
    std::vector<NodeId> result;
    for (NodeId id : nodes()) {
        if (isVisible(id)) {
            result.push_back(id);
        }
    }
    return result;
}

bool CompoundGraph::isInternalEdge(EdgeId id) const {
    if (!hasEdge(id)) return false;

    const EdgeData& edge = getEdge(id);
    auto fromParent = getParent(edge.from);
    auto toParent = getParent(edge.to);

    // Internal edge: both nodes have the same parent
    return fromParent == toParent;
}

bool CompoundGraph::isExternalEdge(EdgeId id) const {
    return hasEdge(id) && !isInternalEdge(id);
}

std::optional<NodeId> CompoundGraph::lowestCommonAncestor(NodeId a, NodeId b) const {
    if (!hasNode(a) || !hasNode(b)) return std::nullopt;

    // Get all ancestors of a
    std::unordered_set<NodeId> ancestorsA;
    ancestorsA.insert(a);
    auto parent = getParent(a);
    while (parent.has_value()) {
        ancestorsA.insert(parent.value());
        parent = getParent(parent.value());
    }

    // Find first ancestor of b that is also an ancestor of a
    if (ancestorsA.count(b)) {
        return b;
    }

    parent = getParent(b);
    while (parent.has_value()) {
        if (ancestorsA.count(parent.value())) {
            return parent.value();
        }
        parent = getParent(parent.value());
    }

    return std::nullopt;
}

void CompoundGraph::clear() {
    Graph::clear();
    compoundInfo_.clear();
}

void CompoundGraph::ensureCompoundInfo(NodeId id) {
    if (compoundInfo_.find(id) == compoundInfo_.end()) {
        compoundInfo_[id] = CompoundInfo{};
    }
}

bool CompoundGraph::isAncestorOf(NodeId ancestor, NodeId descendant) const {
    auto parent = getParent(descendant);
    while (parent.has_value()) {
        if (parent.value() == ancestor) {
            return true;
        }
        parent = getParent(parent.value());
    }
    return false;
}

}  // namespace arborvia
