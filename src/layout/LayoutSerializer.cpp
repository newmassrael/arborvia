#include "../../include/arborvia/layout/util/LayoutSerializer.h"
#include "../../include/arborvia/layout/interactive/ManualLayoutManager.h"
#include "../../include/arborvia/layout/config/LayoutResult.h"
#include <nlohmann/json.hpp>
#include <fstream>
#include <stdexcept>

using json = nlohmann::json;

namespace arborvia {

std::string LayoutSerializer::nodeEdgeToString(NodeEdge edge) {
    switch (edge) {
        case NodeEdge::Top: return "top";
        case NodeEdge::Bottom: return "bottom";
        case NodeEdge::Left: return "left";
        case NodeEdge::Right: return "right";
    }
    return "bottom";
}

NodeEdge LayoutSerializer::stringToNodeEdge(const std::string& str) {
    if (str == "top") return NodeEdge::Top;
    if (str == "bottom") return NodeEdge::Bottom;
    if (str == "left") return NodeEdge::Left;
    if (str == "right") return NodeEdge::Right;
    return NodeEdge::Bottom;
}

std::string LayoutSerializer::toJson(const ManualLayoutManager& manager) {
    json j;
    j["version"] = 1;

    const auto& state = manager.getManualState();

    // Node positions
    json nodePositions = json::object();
    for (const auto& [id, pos] : state.nodePositions) {
        nodePositions[std::to_string(id)] = {{"x", pos.x}, {"y", pos.y}};
    }
    j["nodePositions"] = nodePositions;

    // Snap configs
    json snapConfigs = json::object();
    for (const auto& [id, config] : state.snapConfigs) {
        snapConfigs[std::to_string(id)] = {
            {"top", config.topCount},
            {"bottom", config.bottomCount},
            {"left", config.leftCount},
            {"right", config.rightCount}
        };
    }
    j["snapConfigs"] = snapConfigs;

    // Edge routings
    json edgeRoutings = json::object();
    for (const auto& [id, routing] : state.edgeRoutings) {
        json routingJson = {
            {"sourceEdge", nodeEdgeToString(routing.sourceEdge)},
            {"targetEdge", nodeEdgeToString(routing.targetEdge)},
            {"sourceSnapIndex", routing.sourceSnapIndex},
            {"targetSnapIndex", routing.targetSnapIndex}
        };

        if (!routing.manualBendPoints.empty()) {
            json bendPoints = json::array();
            for (const auto& bp : routing.manualBendPoints) {
                bendPoints.push_back({
                    {"x", bp.position.x},
                    {"y", bp.position.y},
                    {"isControl", bp.isControlPoint}
                });
            }
            routingJson["bendPoints"] = bendPoints;
        }

        edgeRoutings[std::to_string(id)] = routingJson;
    }
    j["edgeRoutings"] = edgeRoutings;

    return j.dump(2);
}

bool LayoutSerializer::fromJson(ManualLayoutManager& manager, const std::string& jsonStr) {
    try {
        json j = json::parse(jsonStr);

        // Clear existing state
        manager.clearManualState();

        // Parse nodePositions
        if (j.contains("nodePositions")) {
            for (auto& [key, value] : j["nodePositions"].items()) {
                NodeId id = static_cast<NodeId>(std::stoul(key));
                float x = value["x"].get<float>();
                float y = value["y"].get<float>();
                manager.setNodePosition(id, {x, y});
            }
        }

        // Parse snapConfigs
        if (j.contains("snapConfigs")) {
            for (auto& [key, value] : j["snapConfigs"].items()) {
                NodeId id = static_cast<NodeId>(std::stoul(key));
                SnapPointConfig config;
                config.topCount = value.value("top", 1);
                config.bottomCount = value.value("bottom", 1);
                config.leftCount = value.value("left", 1);
                config.rightCount = value.value("right", 1);
                manager.setSnapConfig(id, config);
            }
        }

        // Parse edgeRoutings
        if (j.contains("edgeRoutings")) {
            for (auto& [key, value] : j["edgeRoutings"].items()) {
                EdgeId id = static_cast<EdgeId>(std::stoul(key));
                EdgeRoutingConfig routing;

                routing.sourceEdge = stringToNodeEdge(value.value("sourceEdge", "bottom"));
                routing.targetEdge = stringToNodeEdge(value.value("targetEdge", "top"));
                routing.sourceSnapIndex = value.value("sourceSnapIndex", 0);
                routing.targetSnapIndex = value.value("targetSnapIndex", 0);

                // Parse bendPoints
                if (value.contains("bendPoints")) {
                    for (auto& bp : value["bendPoints"]) {
                        BendPoint bendPoint;
                        bendPoint.position.x = bp["x"].get<float>();
                        bendPoint.position.y = bp["y"].get<float>();
                        bendPoint.isControlPoint = bp.value("isControl", false);
                        routing.manualBendPoints.push_back(bendPoint);
                    }
                }

                manager.setEdgeRouting(id, routing);
            }
        }

        return true;
    } catch (const json::exception&) {
        return false;
    }
}

bool LayoutSerializer::saveToFile(const ManualLayoutManager& manager, const std::string& path) {
    std::ofstream file(path);
    if (!file.is_open()) return false;
    file << toJson(manager);
    return true;
}

bool LayoutSerializer::loadFromFile(ManualLayoutManager& manager, const std::string& path) {
    std::ifstream file(path);
    if (!file.is_open()) return false;
    std::stringstream buffer;
    buffer << file.rdbuf();
    return fromJson(manager, buffer.str());
}

std::string LayoutSerializer::toJson(const LayoutResult& result) {
    json j;
    j["layerCount"] = result.layerCount();

    // Node layouts
    json nodeLayouts = json::array();
    for (const auto& [id, layout] : result.nodeLayouts()) {
        json nodeJson;
        nodeJson["id"] = id;
        nodeJson["position"] = {{"x", layout.position.x}, {"y", layout.position.y}};
        nodeJson["size"] = {{"width", layout.size.width}, {"height", layout.size.height}};
        nodeJson["layer"] = layout.layer;
        nodeJson["order"] = layout.order;
        nodeLayouts.push_back(nodeJson);
    }
    j["nodeLayouts"] = nodeLayouts;

    // Edge layouts
    json edgeLayouts = json::array();
    for (const auto& [id, layout] : result.edgeLayouts()) {
        json edgeJson;
        edgeJson["id"] = id;
        edgeJson["from"] = layout.from;
        edgeJson["to"] = layout.to;

        // Source snap point info
        edgeJson["sourcePoint"] = {{"x", layout.sourcePoint.x}, {"y", layout.sourcePoint.y}};
        edgeJson["sourceEdge"] = nodeEdgeToString(layout.sourceEdge);
        // NOTE: sourceSnapIndex removed - compute from position using GridSnapCalculator

        // Target snap point info
        edgeJson["targetPoint"] = {{"x", layout.targetPoint.x}, {"y", layout.targetPoint.y}};
        edgeJson["targetEdge"] = nodeEdgeToString(layout.targetEdge);
        // NOTE: targetSnapIndex removed - compute from position using GridSnapCalculator

        // Label position
        edgeJson["labelPosition"] = {{"x", layout.labelPosition.x}, {"y", layout.labelPosition.y}};

        // Channel routing information
        edgeJson["channelY"] = layout.channelY;

        json bendPoints = json::array();
        for (const auto& bp : layout.bendPoints) {
            bendPoints.push_back({
                {"position", {{"x", bp.position.x}, {"y", bp.position.y}}},
                {"isControlPoint", bp.isControlPoint}
            });
        }
        edgeJson["bendPoints"] = bendPoints;
        edgeLayouts.push_back(edgeJson);
    }
    j["edgeLayouts"] = edgeLayouts;

    return j.dump(2);
}

LayoutResult LayoutSerializer::layoutResultFromJson(const std::string& jsonStr) {
    LayoutResult result;

    try {
        json j = json::parse(jsonStr);

        // Layer count
        if (j.contains("layerCount")) {
            result.setLayerCount(j["layerCount"].get<int>());
        }

        // Node layouts
        if (j.contains("nodeLayouts")) {
            for (const auto& nodeJson : j["nodeLayouts"]) {
                NodeLayout layout;
                layout.id = nodeJson["id"].get<NodeId>();
                layout.position.x = nodeJson["position"]["x"].get<float>();
                layout.position.y = nodeJson["position"]["y"].get<float>();
                layout.size.width = nodeJson["size"]["width"].get<float>();
                layout.size.height = nodeJson["size"]["height"].get<float>();
                layout.layer = nodeJson.value("layer", 0);
                layout.order = nodeJson.value("order", 0);
                result.setNodeLayout(layout.id, layout);
            }
        }

        // Edge layouts
        if (j.contains("edgeLayouts")) {
            for (const auto& edgeJson : j["edgeLayouts"]) {
                EdgeLayout layout;
                layout.id = edgeJson["id"].get<EdgeId>();
                layout.from = edgeJson["from"].get<NodeId>();
                layout.to = edgeJson["to"].get<NodeId>();
                layout.sourcePoint.x = edgeJson["sourcePoint"]["x"].get<float>();
                layout.sourcePoint.y = edgeJson["sourcePoint"]["y"].get<float>();
                layout.targetPoint.x = edgeJson["targetPoint"]["x"].get<float>();
                layout.targetPoint.y = edgeJson["targetPoint"]["y"].get<float>();

                // Label position (optional for backward compatibility)
                if (edgeJson.contains("labelPosition")) {
                    layout.labelPosition.x = edgeJson["labelPosition"]["x"].get<float>();
                    layout.labelPosition.y = edgeJson["labelPosition"]["y"].get<float>();
                }

                // Channel routing information (optional for backward compatibility)
                if (edgeJson.contains("channelY")) {
                    layout.channelY = edgeJson["channelY"].get<float>();
                }

                if (edgeJson.contains("bendPoints")) {
                    for (const auto& bpJson : edgeJson["bendPoints"]) {
                        BendPoint bp;
                        bp.position.x = bpJson["position"]["x"].get<float>();
                        bp.position.y = bpJson["position"]["y"].get<float>();
                        bp.isControlPoint = bpJson.value("isControlPoint", false);
                        layout.bendPoints.push_back(bp);
                    }
                }
                result.setEdgeLayout(layout.id, layout);
            }
        }
    } catch (const json::exception& e) {
        throw std::runtime_error(std::string("Failed to parse LayoutResult JSON: ") + e.what());
    }

    return result;
}

}  // namespace arborvia
