#include "../../include/arborvia/layout/ManualLayoutManager.h"
#include "arborvia/layout/ManualLayoutManager.h"
#include "arborvia/layout/LayoutSerializer.h"
#include "arborvia/layout/LayoutUtils.h"
#include <fstream>
#include <sstream>
#include <cmath>
#include <map>
#include <unordered_map>

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

void ManualLayoutManager::syncSnapConfigsFromEdgeLayouts(
    const std::unordered_map<EdgeId, EdgeLayout>& edgeLayouts)
{
    // Count snap points needed per node edge based on actual edge connections
    std::map<std::pair<NodeId, NodeEdge>, int> snapCounts;

    for (const auto& [edgeId, layout] : edgeLayouts) {
        // Track max snap index for each node edge
        auto& srcCount = snapCounts[{layout.from, layout.sourceEdge}];
        srcCount = std::max(srcCount, layout.sourceSnapIndex + 1);

        auto& tgtCount = snapCounts[{layout.to, layout.targetEdge}];
        tgtCount = std::max(tgtCount, layout.targetSnapIndex + 1);
    }

    // Update snap configs (only if greater than current to preserve manual settings)
    for (const auto& [key, count] : snapCounts) {
        auto [nodeId, edge] = key;
        SnapPointConfig config = getSnapConfig(nodeId);
        if (count > config.getCount(edge)) {
            config.setCount(edge, count);
            setSnapConfig(nodeId, config);
        }
    }
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
        layout->sourcePoint = LayoutUtils::calculateSnapPoint(
            *fromNode, routing.sourceEdge, routing.sourceSnapIndex, sourceSnapCount);
        layout->targetPoint = LayoutUtils::calculateSnapPoint(
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

void ManualLayoutManager::clearManualState() {
    manualState_.clear();
}

SnapPointConfig ManualLayoutManager::defaultSnapConfig() {
    return SnapPointConfig{1, 1, 1, 1};
}

std::string ManualLayoutManager::toJson() const {
    return LayoutSerializer::toJson(*this);
}

bool ManualLayoutManager::fromJson(const std::string& json) {
    return LayoutSerializer::fromJson(*this, json);
}

bool ManualLayoutManager::saveToFile(const std::string& path) const {
    return LayoutSerializer::saveToFile(*this, path);
}

bool ManualLayoutManager::loadFromFile(const std::string& path) {
    return LayoutSerializer::loadFromFile(*this, path);
}

}  // namespace arborvia
