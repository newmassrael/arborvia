#pragma once

#include <string>
#include <vector>

namespace arborvia::test::scxml {

/// SCXML node types (W3C SCXML specification)
enum class SCXMLNodeType {
    State,          ///< <state> - atomic or compound
    Parallel,       ///< <parallel> - concurrent regions
    Final,          ///< <final> - terminal state
    Initial,        ///< <initial> pseudo-state (visual marker)
    History,        ///< <history> - shallow history
    HistoryDeep     ///< <history type="deep"> - deep history
};

/// SCXML edge types
enum class SCXMLEdgeType {
    Transition,     ///< <transition> - state transition
    Initial         ///< Initial pseudo-state to first state
};

/// Additional info for SCXML nodes
struct SCXMLNodeInfo {
    SCXMLNodeType type = SCXMLNodeType::State;
    std::string scxmlId;            ///< Original SCXML id attribute
    std::string event;              ///< For transitions: triggering event
    std::string cond;               ///< Guard condition
    std::vector<std::string> onentry;   ///< onentry actions
    std::vector<std::string> onexit;    ///< onexit actions
};

/// Additional info for SCXML edges
struct SCXMLEdgeInfo {
    SCXMLEdgeType type = SCXMLEdgeType::Transition;
    std::string event;              ///< Triggering event (empty for eventless)
    std::string cond;               ///< Guard condition
    std::vector<std::string> actions;   ///< Transition actions
    bool eventless = false;         ///< True if <transition> has no event
};

/// Convert node type to string for debugging
inline const char* toString(SCXMLNodeType type) {
    switch (type) {
        case SCXMLNodeType::State: return "State";
        case SCXMLNodeType::Parallel: return "Parallel";
        case SCXMLNodeType::Final: return "Final";
        case SCXMLNodeType::Initial: return "Initial";
        case SCXMLNodeType::History: return "History";
        case SCXMLNodeType::HistoryDeep: return "HistoryDeep";
        default: return "Unknown";
    }
}

inline const char* toString(SCXMLEdgeType type) {
    switch (type) {
        case SCXMLEdgeType::Transition: return "Transition";
        case SCXMLEdgeType::Initial: return "Initial";
        default: return "Unknown";
    }
}

}  // namespace arborvia::test::scxml
