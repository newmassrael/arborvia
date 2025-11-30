#include "../../include/arborvia/layout/ManualLayoutManager.h"
#include <fstream>
#include <sstream>
#include <cmath>
#include <map>

namespace arborvia {

void ManualLayoutManager::setNodePosition(NodeId id, const Point& position) {
    manualState_.nodePositions[id] = position;
}

Point ManualLayoutManager::getNodePosition(NodeId id) const {
    auto it = manualState_.nodePositions.find(id);
    if (it != manualState_.nodePositions.end()) {
        return it->second;
    }
    return {0, 0};
}

bool ManualLayoutManager::hasNodePosition(NodeId id) const {
    return manualState_.hasNodePosition(id);
}

void ManualLayoutManager::setSnapPointCount(NodeId id, NodeEdge edge, int count) {
    manualState_.snapConfigs[id].setCount(edge, count);
}

int ManualLayoutManager::getSnapPointCount(NodeId id, NodeEdge edge) const {
    auto it = manualState_.snapConfigs.find(id);
    if (it != manualState_.snapConfigs.end()) {
        return it->second.getCount(edge);
    }
    return 1;  // Default
}

void ManualLayoutManager::setSnapConfig(NodeId id, const SnapPointConfig& config) {
    manualState_.snapConfigs[id] = config;
}

SnapPointConfig ManualLayoutManager::getSnapConfig(NodeId id) const {
    auto it = manualState_.snapConfigs.find(id);
    if (it != manualState_.snapConfigs.end()) {
        return it->second;
    }
    return defaultSnapConfig();
}

void ManualLayoutManager::setEdgeRouting(EdgeId id, const EdgeRoutingConfig& config) {
    manualState_.edgeRoutings[id] = config;
}

void ManualLayoutManager::setEdgeSourceEdge(EdgeId id, NodeEdge edge, int snapIndex) {
    auto& routing = manualState_.edgeRoutings[id];
    routing.sourceEdge = edge;
    routing.sourceSnapIndex = snapIndex;
}

void ManualLayoutManager::setEdgeTargetEdge(EdgeId id, NodeEdge edge, int snapIndex) {
    auto& routing = manualState_.edgeRoutings[id];
    routing.targetEdge = edge;
    routing.targetSnapIndex = snapIndex;
}

EdgeRoutingConfig ManualLayoutManager::getEdgeRouting(EdgeId id) const {
    auto it = manualState_.edgeRoutings.find(id);
    if (it != manualState_.edgeRoutings.end()) {
        return it->second;
    }
    return EdgeRoutingConfig{};
}

bool ManualLayoutManager::hasEdgeRouting(EdgeId id) const {
    return manualState_.hasEdgeRouting(id);
}

// Bend point management
void ManualLayoutManager::addBendPoint(EdgeId edgeId, size_t index, const Point& position) {
    auto& config = manualState_.edgeRoutings[edgeId];
    BendPoint bp{position, false};
    if (index >= config.manualBendPoints.size()) {
        config.manualBendPoints.push_back(bp);
    } else {
        config.manualBendPoints.insert(config.manualBendPoints.begin() + static_cast<std::ptrdiff_t>(index), bp);
    }
}

void ManualLayoutManager::appendBendPoint(EdgeId edgeId, const Point& position) {
    auto& config = manualState_.edgeRoutings[edgeId];
    config.manualBendPoints.push_back({position, false});
}

void ManualLayoutManager::removeBendPoint(EdgeId edgeId, size_t index) {
    auto it = manualState_.edgeRoutings.find(edgeId);
    if (it != manualState_.edgeRoutings.end() && 
        index < it->second.manualBendPoints.size()) {
        it->second.manualBendPoints.erase(
            it->second.manualBendPoints.begin() + static_cast<std::ptrdiff_t>(index));
    }
}

void ManualLayoutManager::moveBendPoint(EdgeId edgeId, size_t index, const Point& position) {
    auto it = manualState_.edgeRoutings.find(edgeId);
    if (it != manualState_.edgeRoutings.end() && 
        index < it->second.manualBendPoints.size()) {
        it->second.manualBendPoints[index].position = position;
    }
}

const std::vector<BendPoint>& ManualLayoutManager::getBendPoints(EdgeId edgeId) const {
    static const std::vector<BendPoint> empty;
    auto it = manualState_.edgeRoutings.find(edgeId);
    if (it != manualState_.edgeRoutings.end()) {
        return it->second.manualBendPoints;
    }
    return empty;
}

bool ManualLayoutManager::hasManualBendPoints(EdgeId edgeId) const {
    auto it = manualState_.edgeRoutings.find(edgeId);
    return it != manualState_.edgeRoutings.end() && 
           !it->second.manualBendPoints.empty();
}

void ManualLayoutManager::clearBendPoints(EdgeId edgeId) {
    auto it = manualState_.edgeRoutings.find(edgeId);
    if (it != manualState_.edgeRoutings.end()) {
        it->second.manualBendPoints.clear();
    }
}

void ManualLayoutManager::setBendPoints(EdgeId edgeId, const std::vector<BendPoint>& points) {
    manualState_.edgeRoutings[edgeId].manualBendPoints = points;
}

ManualLayoutManager::OrthogonalDragResult ManualLayoutManager::calculateOrthogonalDrag(
    const Point& prevPoint,
    const Point& currentPos,
    const Point& nextPoint,
    const Point& dragTarget,
    bool hasNextBend,
    bool isLastBend)
{
    // Tolerance for floating-point comparison (handles 45-degree edge case)
    constexpr float EPSILON = 0.001f;
    
    OrthogonalDragResult result;
    // result members are already default-initialized via struct definition

    // Determine incoming segment direction
    // Using EPSILON to handle edge case where dx == dy (45 degrees)
    // In that case, we default to treating it as horizontal
    float incomingDx = std::abs(currentPos.x - prevPoint.x);
    float incomingDy = std::abs(currentPos.y - prevPoint.y);
    bool incomingHorizontal = (incomingDx > incomingDy + EPSILON) || 
                               (std::abs(incomingDx - incomingDy) <= EPSILON);

    if (isLastBend) {
        // For the last bend point, must maintain orthogonality with BOTH prev and target
        float outgoingDx = std::abs(currentPos.x - nextPoint.x);
        float outgoingDy = std::abs(currentPos.y - nextPoint.y);
        bool outgoingHorizontal = (outgoingDx > outgoingDy + EPSILON) ||
                                   (std::abs(outgoingDx - outgoingDy) <= EPSILON);

        if (incomingHorizontal && !outgoingHorizontal) {
            // Incoming horizontal, outgoing vertical
            // Keep Y = prev.y (for horizontal) and X = target.x (for vertical)
            result.newCurrentPos = {nextPoint.x, prevPoint.y};
        } else if (!incomingHorizontal && outgoingHorizontal) {
            // Incoming vertical, outgoing horizontal
            // Keep X = prev.x (for vertical) and Y = target.y (for horizontal)
            result.newCurrentPos = {prevPoint.x, nextPoint.y};
        } else if (incomingHorizontal && outgoingHorizontal) {
            // Both horizontal - only change X freely
            result.newCurrentPos = {dragTarget.x, prevPoint.y};
        } else {
            // Both vertical - only change Y freely
            result.newCurrentPos = {prevPoint.x, dragTarget.y};
        }
    } else {
        // Not the last bend - constrain based on incoming direction only
        if (incomingHorizontal) {
            // Incoming is horizontal - keep Y same as prevPoint
            result.newCurrentPos = {dragTarget.x, prevPoint.y};
        } else {
            // Incoming is vertical - keep X same as prevPoint
            result.newCurrentPos = {prevPoint.x, dragTarget.y};
        }
    }

    // Adjust the next bend point to maintain orthogonality on outgoing segment
    if (hasNextBend) {
        if (incomingHorizontal) {
            // Outgoing should be vertical, so next.x = current.x
            result.adjustedNextPos = {result.newCurrentPos.x, nextPoint.y};
        } else {
            // Outgoing should be horizontal, so next.y = current.y
            result.adjustedNextPos = {nextPoint.x, result.newCurrentPos.y};
        }
        result.nextAdjusted = true;
    }

    return result;
}

void ManualLayoutManager::applyManualState(LayoutResult& result, [[maybe_unused]] const Graph& graph) const {
    if (mode_ != LayoutMode::Manual) {
        return;
    }

    // Apply node positions
    for (const auto& [nodeId, position] : manualState_.nodePositions) {
        NodeLayout* layout = result.getNodeLayout(nodeId);
        if (layout) {
            layout->position = position;
        }
    }

    // Apply edge routings with snap points
    for (const auto& [edgeId, routing] : manualState_.edgeRoutings) {
        EdgeLayout* layout = result.getEdgeLayout(edgeId);
        if (!layout) continue;

        const NodeLayout* fromNode = result.getNodeLayout(layout->from);
        const NodeLayout* toNode = result.getNodeLayout(layout->to);
        if (!fromNode || !toNode) continue;

        // Get snap configs
        SnapPointConfig fromConfig = getSnapConfig(layout->from);
        SnapPointConfig toConfig = getSnapConfig(layout->to);

        int sourceSnapCount = fromConfig.getCount(routing.sourceEdge);
        int targetSnapCount = toConfig.getCount(routing.targetEdge);

        // Calculate snap points
        layout->sourcePoint = calculateSnapPoint(
            *fromNode, routing.sourceEdge, routing.sourceSnapIndex, sourceSnapCount);
        layout->targetPoint = calculateSnapPoint(
            *toNode, routing.targetEdge, routing.targetSnapIndex, targetSnapCount);

        // Store routing info in edge layout
        layout->sourceEdge = routing.sourceEdge;
        layout->targetEdge = routing.targetEdge;
        layout->sourceSnapIndex = routing.sourceSnapIndex;
        layout->targetSnapIndex = routing.targetSnapIndex;

        // Apply bend points - manual takes priority over auto
        layout->bendPoints.clear();
        if (routing.hasManualBendPoints()) {
            // Use manual bend points
            layout->bendPoints = routing.manualBendPoints;
        } else {
            // Auto orthogonal bend points
            if (std::abs(layout->sourcePoint.x - layout->targetPoint.x) > 1.0f ||
                std::abs(layout->sourcePoint.y - layout->targetPoint.y) > 1.0f) {
                float midY = (layout->sourcePoint.y + layout->targetPoint.y) / 2.0f;
                layout->bendPoints.push_back({{layout->sourcePoint.x, midY}});
                layout->bendPoints.push_back({{layout->targetPoint.x, midY}});
            }
        }
    }
}

void ManualLayoutManager::captureFromResult(const LayoutResult& result) {
    // Capture node positions
    for (const auto& [id, layout] : result.nodeLayouts()) {
        manualState_.nodePositions[id] = layout.position;
    }

    // Count snap points needed per node edge
    // Key: (nodeId, edge) -> max snap index + 1
    std::map<std::pair<NodeId, NodeEdge>, int> snapCounts;

    // Capture edge routings and track snap point usage
    for (const auto& [id, layout] : result.edgeLayouts()) {
        EdgeRoutingConfig routing;
        routing.sourceEdge = layout.sourceEdge;
        routing.targetEdge = layout.targetEdge;
        routing.sourceSnapIndex = layout.sourceSnapIndex;
        routing.targetSnapIndex = layout.targetSnapIndex;
        manualState_.edgeRoutings[id] = routing;

        // Track max snap index for each node edge
        auto& srcCount = snapCounts[{layout.from, layout.sourceEdge}];
        srcCount = std::max(srcCount, layout.sourceSnapIndex + 1);

        auto& tgtCount = snapCounts[{layout.to, layout.targetEdge}];
        tgtCount = std::max(tgtCount, layout.targetSnapIndex + 1);
    }

    // Update snap configs based on captured edge routings
    for (const auto& [key, count] : snapCounts) {
        auto [nodeId, edge] = key;
        auto& config = manualState_.snapConfigs[nodeId];
        int currentCount = config.getCount(edge);
        if (count > currentCount) {
            config.setCount(edge, count);
        }
    }
}

Point ManualLayoutManager::calculateSnapPoint(
    const NodeLayout& node,
    NodeEdge edge,
    int snapIndex,
    int totalSnapPoints)
{
    if (totalSnapPoints <= 0) totalSnapPoints = 1;
    // Clamp snapIndex to valid range [0, totalSnapPoints - 1]
    if (snapIndex < 0) snapIndex = 0;
    if (snapIndex >= totalSnapPoints) snapIndex = totalSnapPoints - 1;
    float position = static_cast<float>(snapIndex + 1) / static_cast<float>(totalSnapPoints + 1);

    switch (edge) {
        case NodeEdge::Top:
            return {
                node.position.x + node.size.width * position,
                node.position.y
            };
        case NodeEdge::Bottom:
            return {
                node.position.x + node.size.width * position,
                node.position.y + node.size.height
            };
        case NodeEdge::Left:
            return {
                node.position.x,
                node.position.y + node.size.height * position
            };
        case NodeEdge::Right:
            return {
                node.position.x + node.size.width,
                node.position.y + node.size.height * position
            };
    }
    return node.center();
}

void ManualLayoutManager::clearManualState() {
    manualState_.clear();
}

SnapPointConfig ManualLayoutManager::defaultSnapConfig() {
    return SnapPointConfig{1, 1, 1, 1};
}

// JSON Serialization - simple implementation without external library
static std::string nodeEdgeToString(NodeEdge edge) {
    switch (edge) {
        case NodeEdge::Top: return "top";
        case NodeEdge::Bottom: return "bottom";
        case NodeEdge::Left: return "left";
        case NodeEdge::Right: return "right";
    }
    return "bottom";
}

static NodeEdge stringToNodeEdge(const std::string& str) {
    if (str == "top") return NodeEdge::Top;
    if (str == "bottom") return NodeEdge::Bottom;
    if (str == "left") return NodeEdge::Left;
    if (str == "right") return NodeEdge::Right;
    return NodeEdge::Bottom;
}

std::string ManualLayoutManager::toJson() const {
    std::ostringstream ss;
    ss << "{\n";
    ss << "  \"version\": 1,\n";
    ss << "  \"mode\": \"" << (mode_ == LayoutMode::Auto ? "auto" : "manual") << "\",\n";

    // Node positions
    ss << "  \"nodePositions\": {\n";
    bool first = true;
    for (const auto& [id, pos] : manualState_.nodePositions) {
        if (!first) ss << ",\n";
        ss << "    \"" << id << "\": {\"x\": " << pos.x << ", \"y\": " << pos.y << "}";
        first = false;
    }
    ss << "\n  },\n";

    // Snap configs
    ss << "  \"snapConfigs\": {\n";
    first = true;
    for (const auto& [id, config] : manualState_.snapConfigs) {
        if (!first) ss << ",\n";
        ss << "    \"" << id << "\": {\"top\": " << config.topCount
           << ", \"bottom\": " << config.bottomCount
           << ", \"left\": " << config.leftCount
           << ", \"right\": " << config.rightCount << "}";
        first = false;
    }
    ss << "\n  },\n";

    // Edge routings
    ss << "  \"edgeRoutings\": {\n";
    first = true;
    for (const auto& [id, routing] : manualState_.edgeRoutings) {
        if (!first) ss << ",\n";
        ss << "    \"" << id << "\": {\"sourceEdge\": \"" << nodeEdgeToString(routing.sourceEdge)
           << "\", \"targetEdge\": \"" << nodeEdgeToString(routing.targetEdge)
           << "\", \"sourceSnapIndex\": " << routing.sourceSnapIndex
           << ", \"targetSnapIndex\": " << routing.targetSnapIndex;
        
        // Include manual bend points if present
        if (!routing.manualBendPoints.empty()) {
            ss << ", \"bendPoints\": [";
            bool firstBp = true;
            for (const auto& bp : routing.manualBendPoints) {
                if (!firstBp) ss << ", ";
                ss << "{\"x\": " << bp.position.x << ", \"y\": " << bp.position.y
                   << ", \"isControl\": " << (bp.isControlPoint ? "true" : "false") << "}";
                firstBp = false;
            }
            ss << "]";
        }
        ss << "}";
        first = false;
    }
    ss << "\n  }\n";

    ss << "}\n";
    return ss.str();
}

bool ManualLayoutManager::fromJson(const std::string& json) {
    // Simple JSON parsing - production code would use a proper JSON library
    manualState_.clear();

    // Parse mode
    auto modePos = json.find("\"mode\":");
    if (modePos != std::string::npos) {
        auto autoPos = json.find("\"auto\"", modePos);
        auto manualPos = json.find("\"manual\"", modePos);
        if (autoPos != std::string::npos && (manualPos == std::string::npos || autoPos < manualPos)) {
            mode_ = LayoutMode::Auto;
        } else {
            mode_ = LayoutMode::Manual;
        }
    }

    // Helper to find matching brace
    auto findMatchingBrace = [](const std::string& s, size_t start) -> size_t {
        int depth = 1;
        for (size_t i = start + 1; i < s.size(); ++i) {
            if (s[i] == '{') depth++;
            else if (s[i] == '}') {
                depth--;
                if (depth == 0) return i;
            }
        }
        return std::string::npos;
    };

    // Helper to parse a number after a colon
    auto parseNumber = [](const std::string& s, size_t pos) -> float {
        while (pos < s.size() && (s[pos] == ':' || s[pos] == ' ')) pos++;
        return std::stof(s.substr(pos));
    };

    // Helper to parse an integer after a colon
    auto parseInt = [](const std::string& s, size_t pos) -> int {
        while (pos < s.size() && (s[pos] == ':' || s[pos] == ' ')) pos++;
        return std::stoi(s.substr(pos));
    };

    // Parse nodePositions
    auto posStart = json.find("\"nodePositions\"");
    if (posStart != std::string::npos) {
        auto blockStart = json.find("{", posStart + 15);
        if (blockStart != std::string::npos) {
            auto blockEnd = findMatchingBrace(json, blockStart);
            if (blockEnd != std::string::npos) {
                std::string block = json.substr(blockStart + 1, blockEnd - blockStart - 1);
                size_t pos = 0;
                while ((pos = block.find("\"", pos)) != std::string::npos) {
                    auto idEnd = block.find("\"", pos + 1);
                    if (idEnd == std::string::npos) break;
                    
                    std::string idStr = block.substr(pos + 1, idEnd - pos - 1);
                    if (idStr.empty() || !std::isdigit(idStr[0])) {
                        pos = idEnd + 1;
                        continue;
                    }
                    
                    auto objStart = block.find("{", idEnd);
                    if (objStart == std::string::npos) break;
                    
                    auto xPos = block.find("\"x\"", objStart);
                    auto yPos = block.find("\"y\"", objStart);
                    if (xPos == std::string::npos || yPos == std::string::npos) {
                        pos = idEnd + 1;
                        continue;
                    }

                    float x = parseNumber(block, xPos + 3);
                    float y = parseNumber(block, yPos + 3);
                    
                    NodeId id = static_cast<NodeId>(std::stoul(idStr));
                    manualState_.nodePositions[id] = {x, y};
                    
                    pos = block.find("}", objStart);
                    if (pos == std::string::npos) break;
                    pos++;
                }
            }
        }
    }

    // Parse snapConfigs
    auto snapStart = json.find("\"snapConfigs\"");
    if (snapStart != std::string::npos) {
        auto blockStart = json.find("{", snapStart + 13);
        if (blockStart != std::string::npos) {
            auto blockEnd = findMatchingBrace(json, blockStart);
            if (blockEnd != std::string::npos) {
                std::string block = json.substr(blockStart + 1, blockEnd - blockStart - 1);
                size_t pos = 0;
                while ((pos = block.find("\"", pos)) != std::string::npos) {
                    auto idEnd = block.find("\"", pos + 1);
                    if (idEnd == std::string::npos) break;
                    
                    std::string idStr = block.substr(pos + 1, idEnd - pos - 1);
                    if (idStr.empty() || !std::isdigit(idStr[0])) {
                        pos = idEnd + 1;
                        continue;
                    }
                    
                    auto objStart = block.find("{", idEnd);
                    if (objStart == std::string::npos) break;
                    
                    SnapPointConfig config;
                    auto topPos = block.find("\"top\"", objStart);
                    auto bottomPos = block.find("\"bottom\"", objStart);
                    auto leftPos = block.find("\"left\"", objStart);
                    auto rightPos = block.find("\"right\"", objStart);
                    
                    if (topPos != std::string::npos) config.topCount = parseInt(block, topPos + 5);
                    if (bottomPos != std::string::npos) config.bottomCount = parseInt(block, bottomPos + 8);
                    if (leftPos != std::string::npos) config.leftCount = parseInt(block, leftPos + 6);
                    if (rightPos != std::string::npos) config.rightCount = parseInt(block, rightPos + 7);
                    
                    NodeId id = static_cast<NodeId>(std::stoul(idStr));
                    manualState_.snapConfigs[id] = config;
                    
                    pos = block.find("}", objStart);
                    if (pos == std::string::npos) break;
                    pos++;
                }
            }
        }
    }

    // Helper to find matching bracket (for arrays)
    auto findMatchingBracket = [](const std::string& s, size_t start) -> size_t {
        int depth = 1;
        for (size_t i = start + 1; i < s.size(); ++i) {
            if (s[i] == '[') depth++;
            else if (s[i] == ']') {
                depth--;
                if (depth == 0) return i;
            }
        }
        return std::string::npos;
    };

    // Helper to parse a boolean after a colon
    auto parseBool = [](const std::string& s, size_t pos) -> bool {
        while (pos < s.size() && (s[pos] == ':' || s[pos] == ' ')) pos++;
        return s.substr(pos, 4) == "true";
    };

    // Parse edgeRoutings
    auto routeStart = json.find("\"edgeRoutings\"");
    if (routeStart != std::string::npos) {
        auto blockStart = json.find("{", routeStart + 14);
        if (blockStart != std::string::npos) {
            auto blockEnd = findMatchingBrace(json, blockStart);
            if (blockEnd != std::string::npos) {
                std::string block = json.substr(blockStart + 1, blockEnd - blockStart - 1);
                size_t pos = 0;
                while ((pos = block.find("\"", pos)) != std::string::npos) {
                    auto idEnd = block.find("\"", pos + 1);
                    if (idEnd == std::string::npos) break;
                    
                    std::string idStr = block.substr(pos + 1, idEnd - pos - 1);
                    if (idStr.empty() || !std::isdigit(idStr[0])) {
                        pos = idEnd + 1;
                        continue;
                    }
                    
                    auto objStart = block.find("{", idEnd);
                    if (objStart == std::string::npos) break;
                    // Use findMatchingBrace to handle nested structures (bendPoints array)
                    auto objEnd = findMatchingBrace(block, objStart);
                    if (objEnd == std::string::npos) break;
                    
                    std::string obj = block.substr(objStart, objEnd - objStart + 1);
                    
                    EdgeRoutingConfig routing;
                    
                    // Parse sourceEdge
                    auto srcEdgePos = obj.find("\"sourceEdge\"");
                    if (srcEdgePos != std::string::npos) {
                        auto valStart = obj.find("\"", srcEdgePos + 12);
                        auto valEnd = obj.find("\"", valStart + 1);
                        if (valStart != std::string::npos && valEnd != std::string::npos) {
                            routing.sourceEdge = stringToNodeEdge(obj.substr(valStart + 1, valEnd - valStart - 1));
                        }
                    }
                    
                    // Parse targetEdge
                    auto tgtEdgePos = obj.find("\"targetEdge\"");
                    if (tgtEdgePos != std::string::npos) {
                        auto valStart = obj.find("\"", tgtEdgePos + 12);
                        auto valEnd = obj.find("\"", valStart + 1);
                        if (valStart != std::string::npos && valEnd != std::string::npos) {
                            routing.targetEdge = stringToNodeEdge(obj.substr(valStart + 1, valEnd - valStart - 1));
                        }
                    }
                    
                    // Parse sourceSnapIndex
                    auto srcSnapPos = obj.find("\"sourceSnapIndex\"");
                    if (srcSnapPos != std::string::npos) {
                        routing.sourceSnapIndex = parseInt(obj, srcSnapPos + 17);
                    }
                    
                    // Parse targetSnapIndex
                    auto tgtSnapPos = obj.find("\"targetSnapIndex\"");
                    if (tgtSnapPos != std::string::npos) {
                        routing.targetSnapIndex = parseInt(obj, tgtSnapPos + 17);
                    }
                    
                    // Parse bendPoints array
                    auto bpArrayPos = obj.find("\"bendPoints\"");
                    if (bpArrayPos != std::string::npos) {
                        auto arrayStart = obj.find("[", bpArrayPos);
                        if (arrayStart != std::string::npos) {
                            auto arrayEnd = findMatchingBracket(obj, arrayStart);
                            if (arrayEnd != std::string::npos) {
                                std::string bpArray = obj.substr(arrayStart + 1, arrayEnd - arrayStart - 1);
                                size_t bpPos = 0;
                                while ((bpPos = bpArray.find("{", bpPos)) != std::string::npos) {
                                    auto bpEnd = bpArray.find("}", bpPos);
                                    if (bpEnd == std::string::npos) break;
                                    
                                    std::string bpObj = bpArray.substr(bpPos, bpEnd - bpPos + 1);
                                    
                                    BendPoint bp;
                                    auto xPos = bpObj.find("\"x\"");
                                    auto yPos = bpObj.find("\"y\"");
                                    auto ctrlPos = bpObj.find("\"isControl\"");
                                    
                                    if (xPos != std::string::npos) {
                                        bp.position.x = parseNumber(bpObj, xPos + 3);
                                    }
                                    if (yPos != std::string::npos) {
                                        bp.position.y = parseNumber(bpObj, yPos + 3);
                                    }
                                    if (ctrlPos != std::string::npos) {
                                        bp.isControlPoint = parseBool(bpObj, ctrlPos + 11);
                                    }
                                    
                                    routing.manualBendPoints.push_back(bp);
                                    bpPos = bpEnd + 1;
                                }
                            }
                        }
                    }
                    
                    EdgeId id = static_cast<EdgeId>(std::stoul(idStr));
                    manualState_.edgeRoutings[id] = routing;
                    
                    pos = objEnd + 1;
                }
            }
        }
    }

    return true;
}

bool ManualLayoutManager::saveToFile(const std::string& path) const {
    std::ofstream file(path);
    if (!file.is_open()) return false;
    file << toJson();
    return true;
}

bool ManualLayoutManager::loadFromFile(const std::string& path) {
    std::ifstream file(path);
    if (!file.is_open()) return false;
    std::stringstream buffer;
    buffer << file.rdbuf();
    return fromJson(buffer.str());
}

}  // namespace arborvia
