#include "SCXMLConverter.h"

#include <pugixml.hpp>

#include <iostream>
#include <sstream>
#include <vector>

namespace arborvia::test::scxml {

namespace {

/// Helper to parse 'initial' attribute which may contain multiple target IDs
std::vector<std::string> parseInitialTargets(const std::string& initial) {
    std::vector<std::string> targets;
    std::istringstream iss(initial);
    std::string token;
    while (iss >> token) {
        targets.push_back(token);
    }
    return targets;
}

/// Recursive function to parse state elements
void parseStateElement(pugi::xml_node node, SCXMLGraph& graph, 
                       const std::string& parentId, const ConvertOptions& opts) {
    std::string tagName = node.name();
    std::string id = node.attribute("id").as_string();
    
    if (id.empty() && tagName != "initial") {
        // Generate ID for unnamed states
        static int anonCounter = 0;
        id = "_anon_" + std::to_string(anonCounter++);
    }
    
    NodeId nodeId = INVALID_NODE;
    
    // Create node based on tag type
    if (tagName == "state") {
        nodeId = graph.addState(id, {opts.defaultStateWidth, opts.defaultStateHeight});
    } 
    else if (tagName == "parallel") {
        nodeId = graph.addParallelState(id, {opts.defaultStateWidth, opts.defaultStateHeight});
    }
    else if (tagName == "final") {
        // Use finalStateSize option: 0 = point node, >0 = regular node with that size
        if (opts.finalStateSize > 0) {
            nodeId = graph.addFinalState(id, {opts.finalStateSize, opts.finalStateSize});
        } else {
            nodeId = graph.addFinalState(id);  // Point node (default)
        }
    }
    else if (tagName == "history") {
        bool deep = std::string(node.attribute("type").as_string()) == "deep";
        nodeId = graph.addHistoryState(id, deep);
    }
    else if (tagName == "initial") {
        // <initial> element - process its <transition> child
        // This is handled separately via initial pseudo-state
        auto transition = node.child("transition");
        if (transition && !parentId.empty()) {
            std::string target = transition.attribute("target").as_string();
            if (!target.empty() && opts.createInitialPseudoStates) {
                NodeId initialPseudo = graph.addInitialPseudo(parentId);
                
                // Set parent for initial pseudo-state
                auto parentNode = graph.findByScxmlId(parentId);
                if (parentNode) {
                    graph.setParent(initialPseudo, *parentNode);
                }
                
                // Will create edge after all nodes are parsed
            }
        }
        return;  // Don't recurse into <initial>
    }
    else {
        // Unknown element, skip
        return;
    }
    
    // Set parent-child relationship
    if (!parentId.empty() && nodeId != INVALID_NODE) {
        auto parentNode = graph.findByScxmlId(parentId);
        if (parentNode) {
            graph.setParent(nodeId, *parentNode);
        }
    }
    
    // Handle 'initial' attribute - create initial pseudo-state
    std::string initialAttr = node.attribute("initial").as_string();
    if (!initialAttr.empty() && opts.createInitialPseudoStates) {
        NodeId initialPseudo = graph.addInitialPseudo(id);
        graph.setParent(initialPseudo, nodeId);
        // Edge will be created in second pass
    }
    
    // Parse child elements recursively
    for (auto child : node.children()) {
        std::string childTag = child.name();
        if (childTag == "state" || childTag == "parallel" || 
            childTag == "final" || childTag == "history" || childTag == "initial") {
            parseStateElement(child, graph, id, opts);
        }
    }
}

/// Second pass: create transitions and initial edges
void parseTransitions(pugi::xml_node node, SCXMLGraph& graph, 
                      const std::string& sourceId, const ConvertOptions& opts) {
    std::string tagName = node.name();
    std::string id = node.attribute("id").as_string();
    
    // Use source ID for this element
    std::string currentSourceId = !id.empty() ? id : sourceId;
    
    // Handle 'initial' attribute - create edge from pseudo-state to targets
    std::string initialAttr = node.attribute("initial").as_string();
    if (!initialAttr.empty() && opts.createInitialPseudoStates) {
        std::string pseudoId = id + "_initial_";
        // Find the pseudo-state we created
        for (NodeId nid : graph.nodes()) {
            std::string scxmlId = graph.getScxmlId(nid);
            if (scxmlId.find(id + "_initial_") == 0) {
                // Found the initial pseudo-state for this parent
                auto targets = parseInitialTargets(initialAttr);
                for (const auto& target : targets) {
                    auto targetNode = graph.findByScxmlId(target);
                    if (targetNode) {
                        graph.addInitialEdge(nid, *targetNode);
                    }
                }
                break;
            }
        }
    }
    
    // Handle <initial> element
    if (tagName == "initial") {
        auto transition = node.child("transition");
        if (transition) {
            std::string target = transition.attribute("target").as_string();
            if (!target.empty() && !sourceId.empty() && opts.createInitialPseudoStates) {
                // Find the initial pseudo-state for this parent
                for (NodeId nid : graph.nodes()) {
                    std::string scxmlId = graph.getScxmlId(nid);
                    if (scxmlId.find(sourceId + "_initial_") == 0) {
                        auto targets = parseInitialTargets(target);
                        for (const auto& t : targets) {
                            auto targetNode = graph.findByScxmlId(t);
                            if (targetNode) {
                                graph.addInitialEdge(nid, *targetNode);
                            }
                        }
                        break;
                    }
                }
            }
        }
        return;
    }
    
    // Parse <transition> elements at this level
    for (auto child : node.children("transition")) {
        std::string target = child.attribute("target").as_string();
        std::string event = child.attribute("event").as_string();
        std::string cond = child.attribute("cond").as_string();
        
        if (!target.empty() && !currentSourceId.empty()) {
            auto targets = parseInitialTargets(target);
            for (const auto& t : targets) {
                auto sourceNode = graph.findByScxmlId(currentSourceId);
                auto targetNode = graph.findByScxmlId(t);
                
                if (sourceNode && targetNode) {
                    graph.addTransition(*sourceNode, *targetNode, event, cond);
                }
            }
        }
    }
    
    // Recurse into child state elements
    for (auto child : node.children()) {
        std::string childTag = child.name();
        if (childTag == "state" || childTag == "parallel" || 
            childTag == "final" || childTag == "history" || childTag == "initial") {
            std::string childId = child.attribute("id").as_string();
            parseTransitions(child, graph, childId.empty() ? currentSourceId : childId, opts);
        }
    }
}

}  // anonymous namespace

void SCXMLConverter::setError(int line, int col, const std::string& msg) {
    lastError_ = ParseError{line, col, msg};
}

void SCXMLConverter::clearError() {
    lastError_ = std::nullopt;
}

std::unique_ptr<SCXMLGraph> SCXMLConverter::fromFile(const std::string& path,
                                                      const ConvertOptions& opts) {
    clearError();
    
    pugi::xml_document doc;
    pugi::xml_parse_result result = doc.load_file(path.c_str());
    
    if (!result) {
        setError(0, static_cast<int>(result.offset), 
                 std::string("XML parse error: ") + result.description());
        return nullptr;
    }
    
    auto graph = std::make_unique<SCXMLGraph>();
    
    // Find <scxml> root element
    auto scxml = doc.child("scxml");
    if (!scxml) {
        setError(0, 0, "No <scxml> root element found");
        return nullptr;
    }
    
    // First pass: create all nodes
    std::string rootInitial = scxml.attribute("initial").as_string();
    
    // Create root initial pseudo-state if 'initial' attribute exists
    if (!rootInitial.empty() && opts.createInitialPseudoStates) {
        graph->addInitialPseudo("");  // Root level pseudo-state
    }
    
    // Parse all state elements
    for (auto child : scxml.children()) {
        std::string childTag = child.name();
        if (childTag == "state" || childTag == "parallel" || 
            childTag == "final" || childTag == "history" || childTag == "initial") {
            parseStateElement(child, *graph, "", opts);
        }
    }
    
    // Update node types based on hierarchy
    graph->updateNodeTypes();
    
    // Second pass: create transitions and initial edges
    // Handle root 'initial' attribute
    if (!rootInitial.empty() && opts.createInitialPseudoStates) {
        for (NodeId nid : graph->nodes()) {
            std::string scxmlId = graph->getScxmlId(nid);
            if (scxmlId.find("_initial_") == 0) {
                auto targets = parseInitialTargets(rootInitial);
                for (const auto& t : targets) {
                    auto targetNode = graph->findByScxmlId(t);
                    if (targetNode) {
                        graph->addInitialEdge(nid, *targetNode);
                    }
                }
                break;
            }
        }
    }
    
    // Parse transitions in all elements
    for (auto child : scxml.children()) {
        std::string childTag = child.name();
        if (childTag == "state" || childTag == "parallel" || 
            childTag == "final" || childTag == "history" || childTag == "initial") {
            std::string childId = child.attribute("id").as_string();
            parseTransitions(child, *graph, childId, opts);
        }
    }
    
    return graph;
}

std::unique_ptr<SCXMLGraph> SCXMLConverter::fromString(const std::string& xml,
                                                        const ConvertOptions& opts) {
    clearError();
    
    pugi::xml_document doc;
    pugi::xml_parse_result result = doc.load_string(xml.c_str());
    
    if (!result) {
        setError(0, static_cast<int>(result.offset), 
                 std::string("XML parse error: ") + result.description());
        return nullptr;
    }
    
    auto graph = std::make_unique<SCXMLGraph>();
    
    // Find <scxml> root element
    auto scxml = doc.child("scxml");
    if (!scxml) {
        setError(0, 0, "No <scxml> root element found");
        return nullptr;
    }
    
    // First pass: create all nodes
    std::string rootInitial = scxml.attribute("initial").as_string();
    
    if (!rootInitial.empty() && opts.createInitialPseudoStates) {
        graph->addInitialPseudo("");
    }
    
    for (auto child : scxml.children()) {
        std::string childTag = child.name();
        if (childTag == "state" || childTag == "parallel" || 
            childTag == "final" || childTag == "history" || childTag == "initial") {
            parseStateElement(child, *graph, "", opts);
        }
    }
    
    graph->updateNodeTypes();
    
    // Second pass: create transitions
    if (!rootInitial.empty() && opts.createInitialPseudoStates) {
        for (NodeId nid : graph->nodes()) {
            std::string scxmlId = graph->getScxmlId(nid);
            if (scxmlId.find("_initial_") == 0) {
                auto targets = parseInitialTargets(rootInitial);
                for (const auto& t : targets) {
                    auto targetNode = graph->findByScxmlId(t);
                    if (targetNode) {
                        graph->addInitialEdge(nid, *targetNode);
                    }
                }
                break;
            }
        }
    }
    
    for (auto child : scxml.children()) {
        std::string childTag = child.name();
        if (childTag == "state" || childTag == "parallel" || 
            childTag == "final" || childTag == "history" || childTag == "initial") {
            std::string childId = child.attribute("id").as_string();
            parseTransitions(child, *graph, childId, opts);
        }
    }
    
    return graph;
}

}  // namespace arborvia::test::scxml
