#include "arborvia/layout/LayoutResult.h"

#include <algorithm>
#include <iomanip>
#include <limits>
#include <sstream>
#include <stdexcept>

namespace arborvia {

namespace {
// Simple JSON helpers (zero external dependencies)
[[maybe_unused]] std::string escapeJsonString(const std::string& s) {
    std::ostringstream o;
    for (char c : s) {
        switch (c) {
            case '"': o << "\\\""; break;
            case '\\': o << "\\\\"; break;
            case '\n': o << "\\n"; break;
            case '\r': o << "\\r"; break;
            case '\t': o << "\\t"; break;
            default: o << c;
        }
    }
    return o.str();
}

void skipWhitespace(const std::string& json, size_t& pos) {
    while (pos < json.size() && std::isspace(json[pos])) ++pos;
}

bool expect(const std::string& json, size_t& pos, char c) {
    skipWhitespace(json, pos);
    if (pos < json.size() && json[pos] == c) {
        ++pos;
        return true;
    }
    return false;
}

float parseFloat(const std::string& json, size_t& pos) {
    skipWhitespace(json, pos);
    size_t start = pos;
    if (pos < json.size() && (json[pos] == '-' || json[pos] == '+')) ++pos;
    while (pos < json.size() && (std::isdigit(json[pos]) || json[pos] == '.' || json[pos] == 'e' || json[pos] == 'E' || json[pos] == '-' || json[pos] == '+')) ++pos;
    return std::stof(json.substr(start, pos - start));
}

int parseInt(const std::string& json, size_t& pos) {
    skipWhitespace(json, pos);
    size_t start = pos;
    if (pos < json.size() && json[pos] == '-') ++pos;
    while (pos < json.size() && std::isdigit(json[pos])) ++pos;
    return std::stoi(json.substr(start, pos - start));
}

bool parseBool(const std::string& json, size_t& pos) {
    skipWhitespace(json, pos);
    if (json.substr(pos, 4) == "true") { pos += 4; return true; }
    if (json.substr(pos, 5) == "false") { pos += 5; return false; }
    throw std::runtime_error("Invalid boolean");
}

std::string parseString(const std::string& json, size_t& pos) {
    skipWhitespace(json, pos);
    if (!expect(json, pos, '"')) throw std::runtime_error("Expected string");
    std::string result;
    while (pos < json.size() && json[pos] != '"') {
        if (json[pos] == '\\' && pos + 1 < json.size()) {
            ++pos;
            switch (json[pos]) {
                case 'n': result += '\n'; break;
                case 't': result += '\t'; break;
                case 'r': result += '\r'; break;
                default: result += json[pos];
            }
        } else {
            result += json[pos];
        }
        ++pos;
    }
    expect(json, pos, '"');
    return result;
}
} // namespace

void LayoutResult::setNodeLayout(NodeId id, const NodeLayout& layout) {
    nodeLayouts_[id] = layout;
}

const NodeLayout* LayoutResult::getNodeLayout(NodeId id) const {
    auto it = nodeLayouts_.find(id);
    return it != nodeLayouts_.end() ? &it->second : nullptr;
}

NodeLayout* LayoutResult::getNodeLayout(NodeId id) {
    auto it = nodeLayouts_.find(id);
    return it != nodeLayouts_.end() ? &it->second : nullptr;
}

bool LayoutResult::hasNodeLayout(NodeId id) const {
    return nodeLayouts_.find(id) != nodeLayouts_.end();
}

void LayoutResult::setEdgeLayout(EdgeId id, const EdgeLayout& layout) {
    edgeLayouts_[id] = layout;
}

const EdgeLayout* LayoutResult::getEdgeLayout(EdgeId id) const {
    auto it = edgeLayouts_.find(id);
    return it != edgeLayouts_.end() ? &it->second : nullptr;
}

EdgeLayout* LayoutResult::getEdgeLayout(EdgeId id) {
    auto it = edgeLayouts_.find(id);
    return it != edgeLayouts_.end() ? &it->second : nullptr;
}

bool LayoutResult::hasEdgeLayout(EdgeId id) const {
    return edgeLayouts_.find(id) != edgeLayouts_.end();
}

Rect LayoutResult::computeBounds() const {
    return computeBounds(0.0f);
}

Rect LayoutResult::computeBounds(float padding) const {
    if (nodeLayouts_.empty()) {
        return {0, 0, 0, 0};
    }
    
    float minX = std::numeric_limits<float>::max();
    float minY = std::numeric_limits<float>::max();
    float maxX = std::numeric_limits<float>::lowest();
    float maxY = std::numeric_limits<float>::lowest();
    
    for (const auto& [id, layout] : nodeLayouts_) {
        minX = std::min(minX, layout.position.x);
        minY = std::min(minY, layout.position.y);
        maxX = std::max(maxX, layout.position.x + layout.size.width);
        maxY = std::max(maxY, layout.position.y + layout.size.height);
    }
    
    // Also consider edge bend points
    for (const auto& [id, layout] : edgeLayouts_) {
        for (const auto& bp : layout.bendPoints) {
            minX = std::min(minX, bp.position.x);
            minY = std::min(minY, bp.position.y);
            maxX = std::max(maxX, bp.position.x);
            maxY = std::max(maxY, bp.position.y);
        }
    }
    
    return {
        minX - padding,
        minY - padding,
        maxX - minX + 2 * padding,
        maxY - minY + 2 * padding
    };
}

std::vector<NodeId> LayoutResult::nodesInLayer(int layer) const {
    std::vector<NodeId> result;
    for (const auto& [id, layout] : nodeLayouts_) {
        if (layout.layer == layer) {
            result.push_back(id);
        }
    }
    // Sort by order within layer
    std::sort(result.begin(), result.end(), [this](NodeId a, NodeId b) {
        return nodeLayouts_.at(a).order < nodeLayouts_.at(b).order;
    });
    return result;
}

void LayoutResult::translate(float dx, float dy) {
    for (auto& [id, layout] : nodeLayouts_) {
        layout.position.x += dx;
        layout.position.y += dy;
    }
    
    for (auto& [id, layout] : edgeLayouts_) {
        layout.sourcePoint.x += dx;
        layout.sourcePoint.y += dy;
        layout.targetPoint.x += dx;
        layout.targetPoint.y += dy;
        for (auto& bp : layout.bendPoints) {
            bp.position.x += dx;
            bp.position.y += dy;
        }
    }
}

void LayoutResult::clear() {
    nodeLayouts_.clear();
    edgeLayouts_.clear();
    layerCount_ = 0;
}

std::string LayoutResult::toJson() const {
    std::ostringstream ss;
    ss << std::fixed << std::setprecision(2);
    
    ss << "{";
    
    // Layer count
    ss << "\"layerCount\":" << layerCount_;
    
    // Node layouts
    ss << ",\"nodeLayouts\":[";
    bool firstNode = true;
    for (const auto& [id, layout] : nodeLayouts_) {
        if (!firstNode) ss << ",";
        firstNode = false;
        
        ss << "{";
        ss << "\"id\":" << id;
        ss << ",\"position\":{\"x\":" << layout.position.x << ",\"y\":" << layout.position.y << "}";
        ss << ",\"size\":{\"width\":" << layout.size.width << ",\"height\":" << layout.size.height << "}";
        ss << ",\"layer\":" << layout.layer;
        ss << ",\"order\":" << layout.order;
        ss << "}";
    }
    ss << "]";
    
    // Edge layouts
    ss << ",\"edgeLayouts\":[";
    bool firstEdge = true;
    for (const auto& [id, layout] : edgeLayouts_) {
        if (!firstEdge) ss << ",";
        firstEdge = false;
        
        ss << "{";
        ss << "\"id\":" << id;
        ss << ",\"from\":" << layout.from;
        ss << ",\"to\":" << layout.to;
        ss << ",\"sourcePoint\":{\"x\":" << layout.sourcePoint.x << ",\"y\":" << layout.sourcePoint.y << "}";
        ss << ",\"targetPoint\":{\"x\":" << layout.targetPoint.x << ",\"y\":" << layout.targetPoint.y << "}";
        
        ss << ",\"bendPoints\":[";
        bool firstBend = true;
        for (const auto& bp : layout.bendPoints) {
            if (!firstBend) ss << ",";
            firstBend = false;
            ss << "{\"position\":{\"x\":" << bp.position.x << ",\"y\":" << bp.position.y << "}";
            ss << ",\"isControlPoint\":" << (bp.isControlPoint ? "true" : "false") << "}";
        }
        ss << "]";
        
        ss << "}";
    }
    ss << "]";
    
    ss << "}";
    
    return ss.str();
}

LayoutResult LayoutResult::fromJson(const std::string& json) {
    LayoutResult result;
    size_t pos = 0;
    
    if (!expect(json, pos, '{')) {
        throw std::runtime_error("Expected '{' at start of JSON");
    }
    
    while (pos < json.size() && json[pos] != '}') {
        std::string key = parseString(json, pos);
        if (!expect(json, pos, ':')) {
            throw std::runtime_error("Expected ':' after key");
        }
        
        if (key == "layerCount") {
            result.layerCount_ = parseInt(json, pos);
        } else if (key == "nodeLayouts") {
            if (!expect(json, pos, '[')) {
                throw std::runtime_error("Expected '[' for nodeLayouts array");
            }
            
            while (pos < json.size() && json[pos] != ']') {
                skipWhitespace(json, pos);
                if (json[pos] == ']') break;
                
                if (!expect(json, pos, '{')) {
                    throw std::runtime_error("Expected '{' for node layout object");
                }
                
                NodeLayout layout;
                
                while (pos < json.size() && json[pos] != '}') {
                    std::string nodeKey = parseString(json, pos);
                    expect(json, pos, ':');
                    
                    if (nodeKey == "id") {
                        layout.id = static_cast<NodeId>(parseInt(json, pos));
                    } else if (nodeKey == "position") {
                        expect(json, pos, '{');
                        while (pos < json.size() && json[pos] != '}') {
                            std::string pKey = parseString(json, pos);
                            expect(json, pos, ':');
                            if (pKey == "x") layout.position.x = parseFloat(json, pos);
                            else if (pKey == "y") layout.position.y = parseFloat(json, pos);
                            skipWhitespace(json, pos);
                            if (json[pos] == ',') ++pos;
                        }
                        expect(json, pos, '}');
                    } else if (nodeKey == "size") {
                        expect(json, pos, '{');
                        while (pos < json.size() && json[pos] != '}') {
                            std::string sKey = parseString(json, pos);
                            expect(json, pos, ':');
                            if (sKey == "width") layout.size.width = parseFloat(json, pos);
                            else if (sKey == "height") layout.size.height = parseFloat(json, pos);
                            skipWhitespace(json, pos);
                            if (json[pos] == ',') ++pos;
                        }
                        expect(json, pos, '}');
                    } else if (nodeKey == "layer") {
                        layout.layer = parseInt(json, pos);
                    } else if (nodeKey == "order") {
                        layout.order = parseInt(json, pos);
                    }
                    
                    skipWhitespace(json, pos);
                    if (json[pos] == ',') ++pos;
                }
                
                expect(json, pos, '}');
                result.setNodeLayout(layout.id, layout);
                
                skipWhitespace(json, pos);
                if (json[pos] == ',') ++pos;
            }
            
            expect(json, pos, ']');
        } else if (key == "edgeLayouts") {
            if (!expect(json, pos, '[')) {
                throw std::runtime_error("Expected '[' for edgeLayouts array");
            }
            
            while (pos < json.size() && json[pos] != ']') {
                skipWhitespace(json, pos);
                if (json[pos] == ']') break;
                
                if (!expect(json, pos, '{')) {
                    throw std::runtime_error("Expected '{' for edge layout object");
                }
                
                EdgeLayout layout;
                
                while (pos < json.size() && json[pos] != '}') {
                    std::string edgeKey = parseString(json, pos);
                    expect(json, pos, ':');
                    
                    if (edgeKey == "id") {
                        layout.id = static_cast<EdgeId>(parseInt(json, pos));
                    } else if (edgeKey == "from") {
                        layout.from = static_cast<NodeId>(parseInt(json, pos));
                    } else if (edgeKey == "to") {
                        layout.to = static_cast<NodeId>(parseInt(json, pos));
                    } else if (edgeKey == "sourcePoint") {
                        expect(json, pos, '{');
                        while (pos < json.size() && json[pos] != '}') {
                            std::string pKey = parseString(json, pos);
                            expect(json, pos, ':');
                            if (pKey == "x") layout.sourcePoint.x = parseFloat(json, pos);
                            else if (pKey == "y") layout.sourcePoint.y = parseFloat(json, pos);
                            skipWhitespace(json, pos);
                            if (json[pos] == ',') ++pos;
                        }
                        expect(json, pos, '}');
                    } else if (edgeKey == "targetPoint") {
                        expect(json, pos, '{');
                        while (pos < json.size() && json[pos] != '}') {
                            std::string pKey = parseString(json, pos);
                            expect(json, pos, ':');
                            if (pKey == "x") layout.targetPoint.x = parseFloat(json, pos);
                            else if (pKey == "y") layout.targetPoint.y = parseFloat(json, pos);
                            skipWhitespace(json, pos);
                            if (json[pos] == ',') ++pos;
                        }
                        expect(json, pos, '}');
                    } else if (edgeKey == "bendPoints") {
                        expect(json, pos, '[');
                        while (pos < json.size() && json[pos] != ']') {
                            skipWhitespace(json, pos);
                            if (json[pos] == ']') break;
                            
                            expect(json, pos, '{');
                            BendPoint bp;
                            
                            while (pos < json.size() && json[pos] != '}') {
                                std::string bpKey = parseString(json, pos);
                                expect(json, pos, ':');
                                
                                if (bpKey == "position") {
                                    expect(json, pos, '{');
                                    while (pos < json.size() && json[pos] != '}') {
                                        std::string pKey = parseString(json, pos);
                                        expect(json, pos, ':');
                                        if (pKey == "x") bp.position.x = parseFloat(json, pos);
                                        else if (pKey == "y") bp.position.y = parseFloat(json, pos);
                                        skipWhitespace(json, pos);
                                        if (json[pos] == ',') ++pos;
                                    }
                                    expect(json, pos, '}');
                                } else if (bpKey == "isControlPoint") {
                                    bp.isControlPoint = parseBool(json, pos);
                                }
                                
                                skipWhitespace(json, pos);
                                if (json[pos] == ',') ++pos;
                            }
                            
                            expect(json, pos, '}');
                            layout.bendPoints.push_back(bp);
                            
                            skipWhitespace(json, pos);
                            if (json[pos] == ',') ++pos;
                        }
                        expect(json, pos, ']');
                    }
                    
                    skipWhitespace(json, pos);
                    if (json[pos] == ',') ++pos;
                }
                
                expect(json, pos, '}');
                result.setEdgeLayout(layout.id, layout);
                
                skipWhitespace(json, pos);
                if (json[pos] == ',') ++pos;
            }
            
            expect(json, pos, ']');
        }
        
        skipWhitespace(json, pos);
        if (json[pos] == ',') ++pos;
    }
    
    return result;
}

}  // namespace arborvia
