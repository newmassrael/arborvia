#include "arborvia/layout/LayoutResult.h"
#include "arborvia/layout/LayoutSerializer.h"

#include <algorithm>
#include <limits>

namespace arborvia {

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
    // Remove from old index if edge already exists
    auto it = edgeLayouts_.find(id);
    if (it != edgeLayouts_.end()) {
        removeFromEdgeIndex(id);
    }
    
    // Store the layout
    edgeLayouts_[id] = layout;
    
    // Update the index
    updateEdgeIndex(id, layout);
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
    nodeToEdges_.clear();
    layerCount_ = 0;
}

std::string LayoutResult::toJson() const {
    return LayoutSerializer::toJson(*this);
}

LayoutResult LayoutResult::fromJson(const std::string& json) {
    return LayoutSerializer::layoutResultFromJson(json);
}

// Static empty list for nodes with no connected edges
const std::vector<EdgeId> LayoutResult::emptyEdgeList_;

const std::vector<EdgeId>& LayoutResult::getConnectedEdges(NodeId nodeId) const {
    auto it = nodeToEdges_.find(nodeId);
    if (it != nodeToEdges_.end()) {
        return it->second;
    }
    return emptyEdgeList_;
}

void LayoutResult::rebuildEdgeIndex() {
    nodeToEdges_.clear();
    for (const auto& [edgeId, layout] : edgeLayouts_) {
        updateEdgeIndex(edgeId, layout);
    }
}

void LayoutResult::updateEdgeIndex(EdgeId edgeId, const EdgeLayout& layout) {
    // Add to source node's list
    nodeToEdges_[layout.from].push_back(edgeId);
    
    // Add to target node's list (if different from source)
    if (layout.to != layout.from) {
        nodeToEdges_[layout.to].push_back(edgeId);
    }
}

void LayoutResult::removeFromEdgeIndex(EdgeId edgeId) {
    auto it = edgeLayouts_.find(edgeId);
    if (it == edgeLayouts_.end()) return;
    
    const EdgeLayout& layout = it->second;
    
    // Remove from source node's list
    auto& fromEdges = nodeToEdges_[layout.from];
    fromEdges.erase(std::remove(fromEdges.begin(), fromEdges.end(), edgeId), fromEdges.end());
    
    // Remove from target node's list
    if (layout.to != layout.from) {
        auto& toEdges = nodeToEdges_[layout.to];
        toEdges.erase(std::remove(toEdges.begin(), toEdges.end(), edgeId), toEdges.end());
    }
}

}  // namespace arborvia
