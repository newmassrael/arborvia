#include "CoordinateAssignment.h"

#include <algorithm>

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
                nodeSizes[node] = options.defaultNodeSize;
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
    const float effectiveVerticalSpacing = std::max(options.nodeSpacingVertical, minLayerSpacing);
    const float effectiveHorizontalSpacing = std::max(options.nodeSpacingHorizontal, gridSize);

    float currentLayerPos = 0.0f;

    for (size_t layerIdx = 0; layerIdx < layers.size(); ++layerIdx) {
        const auto& layer = layers[layerIdx];

        // Calculate layer height (max node height in layer)
        float layerHeight = 0.0f;
        float totalWidth = 0.0f;

        for (NodeId node : layer) {
            auto it = nodeSizes.find(node);
            Size size = (it != nodeSizes.end()) ? it->second : options.defaultNodeSize;

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
            Size size = (it != nodeSizes.end()) ? it->second : options.defaultNodeSize;

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
        }

        // Move to next layer with minimum spacing for edge routing
        currentLayerPos += layerHeight + effectiveVerticalSpacing;
    }
}


}  // namespace arborvia
