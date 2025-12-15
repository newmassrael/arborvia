#pragma once

#include "SCXMLTypes.h"
#include <arborvia/core/CompoundGraph.h>

#include <optional>
#include <unordered_map>

namespace arborvia::test::scxml {

/// CompoundGraph extended with SCXML-specific metadata
class SCXMLGraph : public CompoundGraph {
public:
    SCXMLGraph() = default;
    ~SCXMLGraph() override = default;

    // =========================================================================
    // SCXML-specific node creation
    // =========================================================================
    
    /// Add a state node (atomic or compound based on children)
    NodeId addState(const std::string& id, Size size = {120.0f, 60.0f});
    
    /// Add a parallel state
    NodeId addParallelState(const std::string& id, Size size = {120.0f, 60.0f});
    
    /// Add a final state
    /// Add a final state (rendered as double circle, size ignored - point node)
    NodeId addFinalState(const std::string& id);
    
    /// Add an initial pseudo-state (visual marker for initial attribute)
    NodeId addInitialPseudo(const std::string& parentId);
    
    /// Add a history state
    /// Add a history state (rendered as circle with H/H*, size ignored - point node)
    NodeId addHistoryState(const std::string& id, bool deep = false);

    // =========================================================================
    // SCXML-specific edge creation
    // =========================================================================
    
    /// Add a transition edge
    EdgeId addTransition(NodeId from, NodeId to,
                        const std::string& event = "",
                        const std::string& cond = "");
    
    /// Add a transition edge (by SCXML id)
    EdgeId addTransition(const std::string& fromId, const std::string& toId,
                        const std::string& event = "",
                        const std::string& cond = "");
    
    /// Add an initial edge (from initial pseudo-state to target)
    EdgeId addInitialEdge(NodeId initialPseudo, NodeId target);

    // =========================================================================
    // SCXML ID lookup
    // =========================================================================
    
    /// Find node by SCXML id attribute
    std::optional<NodeId> findByScxmlId(const std::string& scxmlId) const;
    
    /// Get SCXML id for a node
    std::string getScxmlId(NodeId id) const;

    // =========================================================================
    // SCXML metadata access
    // =========================================================================
    
    /// Get node type
    SCXMLNodeType getNodeType(NodeId id) const;
    
    /// Get edge type
    SCXMLEdgeType getEdgeType(EdgeId id) const;
    
    /// Get full node info
    const SCXMLNodeInfo* getNodeInfo(NodeId id) const;
    SCXMLNodeInfo* getNodeInfo(NodeId id);
    
    /// Get full edge info
    const SCXMLEdgeInfo* getEdgeInfo(EdgeId id) const;
    SCXMLEdgeInfo* getEdgeInfo(EdgeId id);

    // =========================================================================
    // Initial state queries
    // =========================================================================
    
    /// Check if a node is an initial pseudo-state
    bool isInitialPseudo(NodeId id) const;
    
    /// Get all initial pseudo-state nodes
    std::vector<NodeId> getInitialPseudoStates() const;
    
    /// Get the initial state of a compound state (follows initial edge)
    /// Returns the target of the initial pseudo-state's outgoing edge
    std::optional<NodeId> getInitialState(NodeId compoundId) const;
    
    /// Get the initial pseudo-state for a compound state (if exists)
    std::optional<NodeId> getInitialPseudoFor(NodeId compoundId) const;
    
    /// Check if a node is a final state
    bool isFinalState(NodeId id) const;
    
    /// Check if a node is a history state (shallow or deep)
    bool isHistoryState(NodeId id) const;

    // =========================================================================
    // Hierarchy helpers
    // =========================================================================
    
    /// Set parent-child relationship (wrapper for CompoundGraph::setParent)
    void addChild(const std::string& parentId, const std::string& childId);
    
    /// Update node type based on children (Atomic if no children, Compound if has children)
    void updateNodeTypes();

    // =========================================================================
    // Debug/Info
    // =========================================================================
    
    /// Print graph structure for debugging
    void dump() const;

private:
    std::unordered_map<NodeId, SCXMLNodeInfo> nodeInfo_;
    std::unordered_map<EdgeId, SCXMLEdgeInfo> edgeInfo_;
    std::unordered_map<std::string, NodeId> scxmlIdToNode_;
    
    int initialPseudoCounter_ = 0;
};

}  // namespace arborvia::test::scxml
