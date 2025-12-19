#include "DemoCommandProcessor.h"
#include <arborvia/layout/api/LayoutController.h>
#include <arborvia/layout/util/LayoutSerializer.h>
#include <arborvia/layout/constraints/ConstraintGateway.h>
#include <arborvia/layout/constraints/ValidatedEdgeLayout.h>
#include <sstream>
#include <iostream>

namespace arborvia {

DemoCommandProcessor::DemoCommandProcessor(
    CommandServer& commandServer,
    std::shared_ptr<ManualLayoutManager> manualManager,
    std::shared_ptr<PathRoutingCoordinator> routingCoordinator)
    : commandServer_(commandServer)
    , manualManager_(std::move(manualManager))
    , routingCoordinator_(std::move(routingCoordinator)) {
}

bool DemoCommandProcessor::processCommand(const Command& cmd, DemoState& state) {
    if (cmd.name == "drag" && cmd.args.size() >= 3) {
        handleDrag(cmd, state);
        return true;
    }
    else if (cmd.name == "set_pos" && cmd.args.size() >= 3) {
        handleSetPos(cmd, state);
        return true;
    }
    else if (cmd.name == "pause") {
        handlePause(cmd);
        return true;
    }
    else if (cmd.name == "resume") {
        handleResume();
        return true;
    }
    else if (cmd.name == "get_state") {
        handleGetState(state);
        return true;
    }
    else if (cmd.name == "get_layout") {
        handleGetLayout(state);
        return true;
    }
    else if (cmd.name == "check_penetration") {
        handleCheckPenetration(state);
        return true;
    }
    else if (cmd.name == "get_log") {
        handleGetLog(cmd);
        return true;
    }
    else if (cmd.name == "clear_log") {
        handleClearLog();
        return true;
    }
    else if (cmd.name == "wait_idle") {
        handleWaitIdle(cmd);
        return true;
    }
    else if (cmd.name == "get_state_full") {
        handleGetStateFull(state);
        return true;
    }
    else if (cmd.name == "astar_viz") {
        handleAStarViz(cmd, state);
        return true;
    }
    else if (cmd.name == "astar_viz_off") {
        handleAStarVizOff(state);
        return true;
    }
    else if (cmd.name == "scxml_load" && !cmd.args.empty()) {
        handleSCXMLLoad(cmd);
        return true;
    }
    else if (cmd.name == "scxml_next") {
        handleSCXMLNext();
        return true;
    }
    else if (cmd.name == "scxml_prev") {
        handleSCXMLPrev();
        return true;
    }
    else if (cmd.name == "scxml_list") {
        handleSCXMLList();
        return true;
    }
    else if (cmd.name == "scxml_info") {
        handleSCXMLInfo();
        return true;
    }
    else if (cmd.name == "quit") {
        handleQuit();
        return true;
    }

    // Command not handled
    return false;
}

void DemoCommandProcessor::handleDrag(const Command& cmd, DemoState& state) {
    NodeId nodeId = std::stoi(cmd.args[0]);
    float dx = std::stof(cmd.args[1]);
    float dy = std::stof(cmd.args[2]);

    if (state.nodeLayouts->count(nodeId)) {
        Point proposedPos = {
            (*state.nodeLayouts)[nodeId].position.x + dx,
            (*state.nodeLayouts)[nodeId].position.y + dy
        };

        // Use LayoutController for move
        state.layoutController->initializeFrom(*state.nodeLayouts, *state.edgeLayouts);
        auto result = state.layoutController->moveNode(nodeId, proposedPos);

        if (result.success) {
            // Sync layouts from controller
            for (const auto& [id, layout] : state.layoutController->nodeLayouts()) {
                (*state.nodeLayouts)[id] = layout;
            }
            for (const auto& [id, layout] : state.layoutController->edgeLayouts()) {
                (*state.edgeLayouts)[id] = layout;
            }
            // Collect ALL edges for A* optimization
            std::vector<EdgeId> allEdges;
            allEdges.reserve(state.edgeLayouts->size());
            for (const auto& [edgeId, layout] : *state.edgeLayouts) {
                allEdges.push_back(edgeId);
            }
            // Trigger A* optimization
            routingCoordinator_->onDragStart(allEdges);
            routingCoordinator_->onDragEnd({nodeId});
            commandServer_.sendResponse("OK drag " + std::to_string(nodeId));
        } else {
            commandServer_.sendResponse("BLOCKED " + result.reason);
        }
    } else {
        commandServer_.sendResponse("ERROR node not found");
    }
}

void DemoCommandProcessor::handleSetPos(const Command& cmd, DemoState& state) {
    NodeId nodeId = std::stoi(cmd.args[0]);
    float x = std::stof(cmd.args[1]);
    float y = std::stof(cmd.args[2]);

    if (state.nodeLayouts->count(nodeId)) {
        Point proposedPos = {x, y};

        // Collect affected edges before move
        std::vector<EdgeId> affected;
        for (const auto& [edgeId, layout] : *state.edgeLayouts) {
            if (layout.from == nodeId || layout.to == nodeId) {
                affected.push_back(edgeId);
            }
        }

        // Simulate drag start
        routingCoordinator_->onDragStart(affected);

        // Use LayoutController for move
        state.layoutController->initializeFrom(*state.nodeLayouts, *state.edgeLayouts);
        auto result = state.layoutController->moveNode(nodeId, proposedPos);

        if (result.success) {
            // Sync layouts from controller
            for (const auto& [id, layout] : state.layoutController->nodeLayouts()) {
                (*state.nodeLayouts)[id] = layout;
            }
            for (const auto& [id, layout] : state.layoutController->edgeLayouts()) {
                (*state.edgeLayouts)[id] = layout;
            }
            state.interaction.affectedEdges = affected;

            // Trigger drag end - schedules A* optimization
            routingCoordinator_->onDragEnd({nodeId});
            commandServer_.sendResponse("OK set_pos " + std::to_string(nodeId) +
                " (" + std::to_string(x) + "," + std::to_string(y) + ")");
        } else {
            commandServer_.sendResponse("BLOCKED " + result.reason);
        }
    } else {
        commandServer_.sendResponse("ERROR node not found");
    }
}

void DemoCommandProcessor::handlePause(const Command& cmd) {
    paused_ = true;
    pauseMessage_ = cmd.args.empty() ? "Paused by external command" : cmd.args[0];
    commandServer_.sendResponse("OK paused");
}

void DemoCommandProcessor::handleResume() {
    paused_ = false;
    pauseMessage_.clear();
    commandServer_.sendResponse("OK resumed");
}

void DemoCommandProcessor::handleGetState(DemoState& state) {
    std::ostringstream oss;
    oss << "STATE nodes=" << state.nodeLayouts->size() << " edges=" << state.edgeLayouts->size();
    for (const auto& [id, layout] : *state.nodeLayouts) {
        oss << " n" << id << "=(" << layout.position.x << "," << layout.position.y << ")";
    }
    commandServer_.sendResponse(oss.str());
}

void DemoCommandProcessor::handleGetLayout(DemoState& state) {
    LayoutResult currentResult;
    for (const auto& [id, layout] : *state.nodeLayouts) {
        currentResult.setNodeLayout(id, layout);
    }
    for (const auto& [id, layout] : *state.edgeLayouts) {
        // Demo: trust layouts for JSON export (TEST/DEMO ONLY)
        currentResult.setEdgeLayout(id, InternalTestAccess::trustUnchecked(layout));
    }

    std::string layoutJson = LayoutSerializer::toJson(currentResult);
    commandServer_.sendResponse(layoutJson);
}

void DemoCommandProcessor::handleCheckPenetration(DemoState& state) {
    ConstraintGateway gateway;
    auto results = gateway.validateAll(*state.edgeLayouts, *state.nodeLayouts, state.renderOptions.gridSize);

    std::ostringstream oss;
    oss << "VIOLATIONS";
    int totalCount = 0;

    for (const auto& [edgeId, result] : results) {
        for (const auto& violation : result.violations) {
            totalCount++;
            oss << " e" << edgeId << ":";
            switch (violation.type) {
                case ConstraintViolationType::NodePenetration:
                    oss << "NodePen";
                    if (violation.nodeId) oss << "->n" << *violation.nodeId;
                    break;
                case ConstraintViolationType::DirectionalSourcePenetration:
                    oss << "SrcPen";
                    if (violation.nodeId) oss << "->n" << *violation.nodeId;
                    break;
                case ConstraintViolationType::DirectionalTargetPenetration:
                    oss << "TgtPen";
                    if (violation.nodeId) oss << "->n" << *violation.nodeId;
                    break;
                case ConstraintViolationType::Orthogonality:
                    oss << "Diagonal";
                    if (violation.segmentIndex >= 0) oss << "[" << violation.segmentIndex << "]";
                    break;
                case ConstraintViolationType::SegmentOverlap:
                    oss << "Overlap";
                    if (violation.otherEdgeId) oss << "->e" << *violation.otherEdgeId;
                    break;
            }
        }
    }
    oss << " count=" << totalCount;
    commandServer_.sendResponse(oss.str());
}

void DemoCommandProcessor::handleGetLog(const Command& cmd) {
    if (!getLogCallback_) {
        commandServer_.sendResponse("ERROR get_log callback not set");
        return;
    }

    std::string pattern = "";
    int tailLines = 200;

    if (!cmd.args.empty()) {
        pattern = cmd.args[0];
        if (pattern == "*") pattern = "";
    }
    if (cmd.args.size() >= 2) {
        tailLines = std::stoi(cmd.args[1]);
    }

    std::string logs = getLogCallback_(pattern, tailLines);
    commandServer_.sendResponse(logs);
}

void DemoCommandProcessor::handleClearLog() {
    if (clearLogCallback_) {
        clearLogCallback_();
    }
    commandServer_.sendResponse("OK logs cleared");
}

void DemoCommandProcessor::handleWaitIdle(const Command& cmd) {
    if (!waitIdleCallback_) {
        commandServer_.sendResponse("ERROR wait_idle callback not set");
        return;
    }

    int timeoutMs = 5000;
    if (!cmd.args.empty()) {
        timeoutMs = std::stoi(cmd.args[0]);
    }

    bool success = waitIdleCallback_(timeoutMs);
    if (success) {
        commandServer_.sendResponse("OK idle");
    } else {
        commandServer_.sendResponse("TIMEOUT");
    }
}

void DemoCommandProcessor::handleGetStateFull(DemoState& state) {
    std::ostringstream oss;
    oss << "STATE_FULL nodes=" << state.nodeLayouts->size() << " edges=" << state.edgeLayouts->size();

    // Node positions
    for (const auto& [id, layout] : *state.nodeLayouts) {
        oss << " n" << id << "=(" << layout.position.x << "," << layout.position.y << ")";
    }

    // Edge routing status
    oss << " routing=[";
    bool first = true;
    for (const auto& [id, layout] : *state.edgeLayouts) {
        if (!first) oss << ",";
        first = false;
        oss << "e" << id << ":bends=" << layout.bendPoints.size();
    }
    oss << "]";

    commandServer_.sendResponse(oss.str());
}

void DemoCommandProcessor::handleAStarViz(const Command& cmd, DemoState& state) {
    if (!cmd.args.empty()) {
        EdgeId edgeId = std::stoi(cmd.args[0]);
        state.astarDebug.debugEdgeId = edgeId;
    }
    state.renderOptions.showAStarDebug = true;
    commandServer_.sendResponse("OK astar_viz enabled");
}

void DemoCommandProcessor::handleAStarVizOff(DemoState& state) {
    state.renderOptions.showAStarDebug = false;
    state.astarDebug.debugEdgeId = INVALID_EDGE;
    commandServer_.sendResponse("OK astar_viz disabled");
}

void DemoCommandProcessor::handleSCXMLLoad(const Command& cmd) {
    if (!initSCXMLCallback_) {
        commandServer_.sendResponse("ERROR scxml callback not set");
        return;
    }

    int testIndex = std::stoi(cmd.args[0]);
    bool success = loadSCXMLTestCallback_ ? loadSCXMLTestCallback_(testIndex) : false;

    if (success) {
        commandServer_.sendResponse("OK scxml_load " + std::to_string(testIndex));
    } else {
        commandServer_.sendResponse("ERROR failed to load test " + std::to_string(testIndex));
    }
}

void DemoCommandProcessor::handleSCXMLNext() {
    if (nextTestCallback_) {
        nextTestCallback_();
        commandServer_.sendResponse("OK scxml_next");
    } else {
        commandServer_.sendResponse("ERROR scxml callback not set");
    }
}

void DemoCommandProcessor::handleSCXMLPrev() {
    if (prevTestCallback_) {
        prevTestCallback_();
        commandServer_.sendResponse("OK scxml_prev");
    } else {
        commandServer_.sendResponse("ERROR scxml callback not set");
    }
}

void DemoCommandProcessor::handleSCXMLList() {
    if (getSCXMLTestCountCallback_) {
        int count = getSCXMLTestCountCallback_();
        commandServer_.sendResponse("OK scxml_list count=" + std::to_string(count));
    } else {
        commandServer_.sendResponse("ERROR scxml callback not set");
    }
}

void DemoCommandProcessor::handleSCXMLInfo() {
    if (getCurrentTestInfoCallback_) {
        std::string info = getCurrentTestInfoCallback_();
        commandServer_.sendResponse(info);
    } else {
        commandServer_.sendResponse("ERROR scxml callback not set");
    }
}

void DemoCommandProcessor::handleQuit() {
    shouldQuit_ = true;
    commandServer_.sendResponse("OK quitting");
}

}  // namespace arborvia
