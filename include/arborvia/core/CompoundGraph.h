#pragma once

#include "Graph.h"

#include <optional>
#include <unordered_map>
#include <unordered_set>

namespace arborvia {

enum class CompoundType {
    Atomic,      // No children (leaf node)
    Compound,    // Sequential children (vertical layout)
    Parallel     // Concurrent children (horizontal layout)
};

class CompoundGraph : public Graph {
public:
    CompoundGraph() = default;
    ~CompoundGraph() override = default;

    // Bring base class overloads into scope
    using Graph::addNode;

    // Override to track compound info
    NodeId addNode(const NodeData& data) override;

    // Compound node creation
    NodeId addCompoundNode(CompoundType type = CompoundType::Compound);
    NodeId addCompoundNode(Size size, CompoundType type);
    NodeId addCompoundNode(const std::string& label, CompoundType type);

    // Hierarchy management
    void setParent(NodeId child, NodeId parent);
    void removeFromParent(NodeId child);

    // Hierarchy queries
    std::optional<NodeId> getParent(NodeId id) const;
    std::vector<NodeId> getChildren(NodeId id) const;
    std::vector<NodeId> getDescendants(NodeId id) const;
    std::vector<NodeId> getAncestors(NodeId id) const;

    bool isCompound(NodeId id) const;
    bool isParallel(NodeId id) const;
    bool isAtomic(NodeId id) const;
    bool isRoot(NodeId id) const;
    bool hasChildren(NodeId id) const;

    int getDepth(NodeId id) const;
    CompoundType getType(NodeId id) const;
    void setType(NodeId id, CompoundType type);

    // Root nodes (nodes without parent)
    std::vector<NodeId> rootNodes() const;

    // Collapse/Expand functionality
    void setCollapsed(NodeId id, bool collapsed);
    bool isCollapsed(NodeId id) const;
    bool isVisible(NodeId id) const;  // Not hidden by collapsed ancestor

    // Get only visible nodes
    std::vector<NodeId> visibleNodes() const;

    // Check if an edge crosses compound boundaries
    bool isInternalEdge(EdgeId id) const;
    bool isExternalEdge(EdgeId id) const;

    // Get the lowest common ancestor of two nodes
    std::optional<NodeId> lowestCommonAncestor(NodeId a, NodeId b) const;

    // Clear all
    void clear() override;

private:
    struct CompoundInfo {
        CompoundType type = CompoundType::Atomic;
        std::optional<NodeId> parent;
        std::vector<NodeId> children;
        bool collapsed = false;
    };

    std::unordered_map<NodeId, CompoundInfo> compoundInfo_;

    void ensureCompoundInfo(NodeId id);
    bool isAncestorOf(NodeId ancestor, NodeId descendant) const;
};

}  // namespace arborvia
