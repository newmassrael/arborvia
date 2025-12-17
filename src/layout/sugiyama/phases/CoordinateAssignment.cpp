#include "CoordinateAssignment.h"

#include <algorithm>
#include <arborvia/common/Logger.h>

namespace arborvia {


CoordinateAssignmentResult SimpleCoordinateAssignment::assign(
    const Graph& graph,
    const std::vector<std::vector<NodeId>>& layers,
    const LayoutOptions& options) const {

    // Build default node sizes
    std::unordered_map<NodeId, Size> nodeSizes;
    for (const auto& layer : layers) {
        for (NodeId node : layer) {
            if (graph.hasNode(node)) {
                nodeSizes[node] = graph.getNode(node).size;
            } else {
                nodeSizes[node] = options.defaultNodeSize();
            }
        }
    }

    return assignWithSizes(graph, layers, nodeSizes, options);
}

CoordinateAssignmentResult SimpleCoordinateAssignment::assignWithSizes(
    const Graph& graph,
    const std::vector<std::vector<NodeId>>& layers,
    const std::unordered_map<NodeId, Size>& nodeSizes,
    const LayoutOptions& options) const {

    CoordinateAssignmentResult result;

    if (layers.empty()) {
        return result;
    }

    simpleAssignment(graph, layers, nodeSizes, options, result);

    return result;
}

void SimpleCoordinateAssignment::simpleAssignment(
    [[maybe_unused]] const Graph& graph,
    const std::vector<std::vector<NodeId>>& layers,
    const std::unordered_map<NodeId, Size>& nodeSizes,
    const LayoutOptions& options,
    CoordinateAssignmentResult& result) const {

    // Compute positions based on direction
    bool horizontal = (options.direction == Direction::LeftToRight ||
                      options.direction == Direction::RightToLeft);

    // Ensure minimum layer spacing for valid edge routing
    // Edge routing needs at least one grid cell between layers for bend points
    const float gridSize = options.gridConfig.cellSize;
    const float minLayerSpacing = gridSize * 2.0f;  // Minimum gap for edge routing channel
    const float effectiveVerticalSpacing = std::max(options.nodeSpacingVertical(), minLayerSpacing);
    const float effectiveHorizontalSpacing = std::max(options.nodeSpacingHorizontal(), gridSize);

    LOG_DEBUG("[CoordinateAssignment] gridSize={} minLayerSpacing={} effectiveVerticalSpacing={} effectiveHorizontalSpacing={}",
              gridSize, minLayerSpacing, effectiveVerticalSpacing, effectiveHorizontalSpacing);
    LOG_DEBUG("[CoordinateAssignment] nodeSpacingVertical={} nodeSpacingHorizontal={} horizontal={}",
              options.nodeSpacingVertical(), options.nodeSpacingHorizontal(), horizontal);

    float currentLayerPos = 0.0f;

    for (size_t layerIdx = 0; layerIdx < layers.size(); ++layerIdx) {
        const auto& layer = layers[layerIdx];

        // Calculate layer height (max node height in layer)
        float layerHeight = 0.0f;
        float totalWidth = 0.0f;

        for (NodeId node : layer) {
            auto it = nodeSizes.find(node);
            Size size = (it != nodeSizes.end()) ? it->second : options.defaultNodeSize();

            if (horizontal) {
                layerHeight = std::max(layerHeight, size.width);
                totalWidth += size.height;
            } else {
                layerHeight = std::max(layerHeight, size.height);
                totalWidth += size.width;
            }
        }

        // Add spacing between nodes
        if (!layer.empty()) {
            totalWidth += effectiveHorizontalSpacing * (layer.size() - 1);
        }

        // Assign positions within layer
        float currentNodePos = 0.0f;

        // Center alignment: offset by half of remaining space
        // For simplicity, start at 0

        for (NodeId node : layer) {
            auto it = nodeSizes.find(node);
            Size size = (it != nodeSizes.end()) ? it->second : options.defaultNodeSize();

            Point pos;
            if (horizontal) {
                pos.x = currentLayerPos;
                pos.y = currentNodePos;
                currentNodePos += size.height + effectiveHorizontalSpacing;
            } else {
                pos.x = currentNodePos;
                pos.y = currentLayerPos;
                currentNodePos += size.width + effectiveHorizontalSpacing;
            }

            // Handle different directions
            switch (options.direction) {
                case Direction::BottomToTop:
                    pos.y = -pos.y - (horizontal ? size.width : size.height);
                    break;
                case Direction::RightToLeft:
                    pos.x = -pos.x - (horizontal ? size.height : size.width);
                    break;
                default:
                    break;
            }

            result.positions[node] = pos;
            LOG_DEBUG("[CoordinateAssignment] node {} pos=({},{}) size=({},{})",
                      node, pos.x, pos.y, size.width, size.height);
        }

        LOG_DEBUG("[CoordinateAssignment] layer {} done, layerHeight={} nextLayerPos={}",
                  layerIdx, layerHeight, currentLayerPos + layerHeight + effectiveVerticalSpacing);
        // Move to next layer with minimum spacing for edge routing
        currentLayerPos += layerHeight + effectiveVerticalSpacing;
    }

    // Log final node distances for constraint validation analysis
    LOG_DEBUG("[CoordinateAssignment] === FINAL NODE DISTANCES ===");
    for (const auto& [nodeA, posA] : result.positions) {
        auto itA = nodeSizes.find(nodeA);
        Size sizeA = (itA != nodeSizes.end()) ? itA->second : options.defaultNodeSize();
        
        for (const auto& [nodeB, posB] : result.positions) {
            if (nodeA >= nodeB) continue;  // Skip self and duplicates
            
            auto itB = nodeSizes.find(nodeB);
            Size sizeB = (itB != nodeSizes.end()) ? itB->second : options.defaultNodeSize();
            
            // Calculate actual gap between nodes
            float gapX = 0.0f, gapY = 0.0f;
            
            // X gap
            float rightA = posA.x + sizeA.width;
            float rightB = posB.x + sizeB.width;
            if (posA.x > rightB) {
                gapX = posA.x - rightB;
            } else if (posB.x > rightA) {
                gapX = posB.x - rightA;
            }
            
            // Y gap
            float bottomA = posA.y + sizeA.height;
            float bottomB = posB.y + sizeB.height;
            if (posA.y > bottomB) {
                gapY = posA.y - bottomB;
            } else if (posB.y > bottomA) {
                gapY = posB.y - bottomA;
            }
            
            LOG_DEBUG("[CoordinateAssignment] node {} <-> node {}: gapX={} gapY={} (gridUnits: {}, {})",
                      nodeA, nodeB, gapX, gapY, gapX / gridSize, gapY / gridSize);
        }
    }
}


}  // namespace arborvia
