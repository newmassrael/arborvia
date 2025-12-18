#include "DemoUIPanel.h"
#include "DemoColors.h"
#include "SCXMLTestLoader.h"
#include <imgui.h>
#include <iostream>

namespace arborvia {

using test::scxml::SCXMLTestLoader;
using test::scxml::TestInfo;

DemoUIPanel::DemoUIPanel(std::shared_ptr<ManualLayoutManager> manualManager)
    : manualManager_(std::move(manualManager)) {
}

UIResult DemoUIPanel::render(DemoState& state) {
    UIResult result;

    ImGui::SetNextWindowPos(ImVec2(10, 10), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(300, 500), ImGuiCond_FirstUseEver);

    ImGui::Begin("ArborVia Demo");

    // Edge Routing Options
    bool routingChanged = false;
    renderEdgeRoutingOptions(state, routingChanged);
    if (routingChanged && reRouteEdgesCallback_) {
        reRouteEdgesCallback_();
    }

    ImGui::Separator();
    renderNodeInfo(state);
    renderSelectedNodeInfo(state);

    ImGui::Separator();
    renderDisplayOptions(state);
    renderViewControls(state);
    renderSCXMLSection(state);

    ImGui::Separator();
    renderActionButtons(state);

    ImGui::End();

    return result;
}

void DemoUIPanel::renderEdgeRoutingOptions(DemoState& state, bool& changed) {
    if (ImGui::TreeNode("Edge Routing Options")) {
        if (ImGui::SliderInt("Spacing (grids)", &state.layoutOptions->channelRouting.channelSpacingGrids, 1, 3)) {
            changed = true;
        }
        if (ImGui::IsItemHovered()) {
            ImGui::SetTooltip("Spacing between parallel edge channels (grid units)");
        }

        if (ImGui::SliderInt("Offset (grids)", &state.layoutOptions->channelRouting.channelOffsetGrids, 1, 3)) {
            changed = true;
        }
        if (ImGui::IsItemHovered()) {
            ImGui::SetTooltip("Minimum offset from layer boundary (grid units)");
        }

        ImGui::Checkbox("Center Single Edge", &state.layoutOptions->channelRouting.centerSingleEdge);

        // Self-loop direction
        ImGui::Text("Self-Loop Direction:");
        const char* loopDirs[] = {"Right", "Left", "Top", "Bottom", "Auto"};
        int currentDir = static_cast<int>(state.layoutOptions->channelRouting.selfLoop.preferredDirection);
        if (ImGui::Combo("##LoopDir", &currentDir, loopDirs, 5)) {
            state.layoutOptions->channelRouting.selfLoop.preferredDirection =
                static_cast<SelfLoopDirection>(currentDir);
            changed = true;
        }

        if (ImGui::SliderInt("Loop Offset (grids)", &state.layoutOptions->channelRouting.selfLoop.loopOffsetGrids, 1, 3)) {
            changed = true;
        }

        ImGui::TreePop();
    }
}

void DemoUIPanel::renderNodeInfo(DemoState& state) {
    ImGui::Text("Drag nodes to see edge re-routing");
    ImGui::Separator();
    ImGui::Text("Nodes: %zu", state.graph->nodeCount());
    ImGui::Text("Edges: %zu", state.graph->edgeCount());

    if (state.interaction.draggedNode != INVALID_NODE) {
        if (auto node = state.graph->tryGetNode(state.interaction.draggedNode)) {
            ImGui::TextColored(ImVec4(1, 0.5f, 0.5f, 1),
                "Dragging: %s", node->label.c_str());
        }
        ImGui::Text("Affected edges: %zu", state.interaction.affectedEdges.size());
    }
}

void DemoUIPanel::renderSelectedNodeInfo(DemoState& state) {
    if (state.interaction.selectedNode == INVALID_NODE ||
        state.interaction.draggedNode != INVALID_NODE) {
        return;
    }

    auto nodeIt = state.nodeLayouts->find(state.interaction.selectedNode);
    if (nodeIt == state.nodeLayouts->end()) {
        return;
    }

    const auto& layout = nodeIt->second;
    auto graphNode = state.graph->tryGetNode(state.interaction.selectedNode);

    ImGui::Separator();
    ImGui::TextColored(ImVec4(0.5f, 1.0f, 0.5f, 1),
        "Selected: %s", graphNode ? graphNode->label.c_str() : "Unknown");

    bool isPoint = layout.isPointNode();
    ImGui::Text("Type: %s", isPoint ? "Point (dot)" : "Normal (rect)");
    ImGui::Text("Position: (%.0f, %.0f)", layout.position.x, layout.position.y);
    if (!isPoint) {
        ImGui::Text("Size: %.0f x %.0f", layout.size.width, layout.size.height);
    }

    // Node type toggle button
    // Point → Regular: only Final can convert (Initial/History must stay Point)
    // Regular → Point: only Final can convert (State/Compound/Parallel must stay Regular)
    bool canConvertToRegular = !isPoint || (canConvertToRegularCallback_ && 
                                             canConvertToRegularCallback_(state.interaction.selectedNode));
    bool canConvertToPoint = isPoint || (canConvertToPointCallback_ && 
                                          canConvertToPointCallback_(state.interaction.selectedNode));
    
    if (isPoint) {
        // Point node - check if can convert to Regular
        if (canConvertToRegular) {
            if (ImGui::Button("To Normal Node") && setNodeTypeCallback_) {
                if (setNodeTypeCallback_(state.interaction.selectedNode, NodeType::Regular)) {
                    std::cout << "[Demo] Node " << state.interaction.selectedNode
                              << " converted to Normal" << std::endl;
                }
            }
        } else {
            // Initial/History - cannot convert
            ImGui::BeginDisabled();
            ImGui::Button("To Normal Node");
            ImGui::EndDisabled();
            if (ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled)) {
                ImGui::SetTooltip("Initial/History nodes must remain Point");
            }
        }
    } else {
        // Regular node - check if can convert to Point
        if (canConvertToPoint) {
            if (ImGui::Button("To Point Node") && setNodeTypeCallback_) {
                if (setNodeTypeCallback_(state.interaction.selectedNode, NodeType::Point)) {
                    std::cout << "[Demo] Node " << state.interaction.selectedNode
                              << " converted to Point" << std::endl;
                }
            }
        } else {
            // State/Compound/Parallel - cannot convert
            ImGui::BeginDisabled();
            ImGui::Button("To Point Node");
            ImGui::EndDisabled();
            if (ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled)) {
                ImGui::SetTooltip("Only Final nodes can be Point");
            }
        }
    }
}

void DemoUIPanel::renderDisplayOptions(DemoState& state) {
    ImGui::Checkbox("Show Snap Points", &state.renderOptions.showSnapPoints);
    if (state.renderOptions.showSnapPoints) {
        ImGui::SameLine();
        ImGui::Checkbox("Show Indices", &state.renderOptions.showSnapIndices);
        if (ImGui::IsItemHovered()) {
            ImGui::SetTooltip("T=Top, B=Bottom, L=Left, R=Right\nGreen=Outgoing, Red=Incoming");
        }
    }
    ImGui::Checkbox("Show Blocked Cells", &state.renderOptions.showBlockedCells);
    if (ImGui::IsItemHovered()) {
        ImGui::SetTooltip("Show no-drag zones while dragging (5 grid units margin)");
    }

    // A* debug visualization
    if (ImGui::Checkbox("Show A* Grid", &state.renderOptions.showAStarDebug)) {
        // Note: The actual obstacle map update is handled in InteractiveDemo
    }
    if (ImGui::IsItemHovered()) {
        ImGui::SetTooltip("Show A* pathfinding grid\nYellow=Node, Blue=H-Edge, Green=V-Edge");
    }
}

void DemoUIPanel::renderViewControls(DemoState& state) {
    ImGui::Separator();
    ImGui::Text("View: %.0f%% zoom", state.view.zoom * 100);
    ImGui::SameLine();
    if (ImGui::Button("Reset View")) {
        state.view.panOffset = {0, 0};
        state.view.zoom = 1.0f;
    }
    if (ImGui::IsItemHovered()) {
        ImGui::SetTooltip("Pan: Drag empty area or Middle mouse\nZoom: Mouse wheel");
    }
    ImGui::SliderFloat("Zoom", &state.view.zoom, 0.1f, 5.0f, "%.1f");
}

void DemoUIPanel::renderSCXMLSection(DemoState& state) {
    ImGui::Separator();

    if (scxmlLoader_ && scxmlLoader_->getTestCount() > 0) {
        if (ImGui::CollapsingHeader("SCXML Tests", ImGuiTreeNodeFlags_DefaultOpen)) {
            // Current test info
            if (state.scxml.modeActive && state.scxml.currentTestIndex >= 0) {
                const auto* testInfo = scxmlLoader_->getTest(
                    static_cast<size_t>(state.scxml.currentTestIndex));
                if (testInfo) {
                    ImGui::TextColored(ImVec4(0.4f, 0.8f, 1.0f, 1.0f), "Test %s", testInfo->id.c_str());
                    ImGui::TextWrapped("%s", testInfo->file.c_str());
                    if (!testInfo->description.empty()) {
                        ImGui::TextWrapped("%s", testInfo->description.c_str());
                    }
                }
                ImGui::Text("Index: %d / %zu", state.scxml.currentTestIndex + 1,
                    scxmlLoader_->getTestCount());
            } else {
                ImGui::TextDisabled("No test loaded");
                ImGui::Text("Tests available: %zu", scxmlLoader_->getTestCount());
            }

            // Navigation buttons
            ImGui::Spacing();
            if (ImGui::Button("Prev") && prevTestCallback_) {
                prevTestCallback_();
            }
            ImGui::SameLine();
            if (ImGui::Button("Next") && nextTestCallback_) {
                nextTestCallback_();
            }
            ImGui::SameLine();
            if (state.scxml.modeActive) {
                if (ImGui::Button("Exit SCXML") && exitSCXMLModeCallback_) {
                    exitSCXMLModeCallback_();
                }
            } else {
                if (ImGui::Button("Load First") && loadSCXMLTestCallback_) {
                    loadSCXMLTestCallback_(0);
                }
            }

            // Test selection combo
            ImGui::Spacing();
            const char* currentLabel = state.scxml.currentTestIndex >= 0 ?
                scxmlLoader_->getTest(static_cast<size_t>(state.scxml.currentTestIndex))->id.c_str() : "None";
            if (ImGui::BeginCombo("Select Test", currentLabel)) {
                for (size_t i = 0; i < scxmlLoader_->getTestCount(); ++i) {
                    const auto* info = scxmlLoader_->getTest(i);
                    bool isSelected = (static_cast<int>(i) == state.scxml.currentTestIndex);
                    std::string label = info->id + " - " + info->file;
                    if (ImGui::Selectable(label.c_str(), isSelected) && loadSCXMLTestCallback_) {
                        loadSCXMLTestCallback_(i);
                    }
                    if (isSelected) {
                        ImGui::SetItemDefaultFocus();
                    }
                }
                ImGui::EndCombo();
            }
        }
    } else {
        ImGui::TextDisabled("SCXML tests not loaded");
        if (ImGui::IsItemHovered()) {
            ImGui::SetTooltip("Place SCXML tests in resources/scxml/");
        }
    }
}

void DemoUIPanel::renderActionButtons(DemoState& state) {
    if (ImGui::Button("Reset Layout")) {
        manualManager_->clearManualState();
        state.interaction = InteractionState{};
        if (doLayoutCallback_) {
            doLayoutCallback_();
        }
    }

    ImGui::SameLine();
    if (ImGui::Button("Save") && saveLayoutCallback_) {
        saveLayoutCallback_("layout.json");
    }

    ImGui::SameLine();
    if (ImGui::Button("Load") && loadLayoutCallback_) {
        loadLayoutCallback_("layout.json");
    }
}

}  // namespace arborvia
