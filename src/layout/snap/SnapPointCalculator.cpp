#include "SnapPointCalculator.h"

#include <algorithm>
#include <cmath>

namespace arborvia {

float SnapPointCalculator::getEdgeLength(const NodeLayout& node, NodeEdge edge) {
    if (edge == NodeEdge::Top || edge == NodeEdge::Bottom) {
        return node.size.width;
    }
    return node.size.height;
}

int SnapPointCalculator::calculateTotalSnapPoints(float edgeLength, float gridSize) {
    if (gridSize <= 0.0f) {
        return 1;  // Fallback: at least 1 snap point
    }
    // A* standard formula: exclude corners, at least 1 snap point
    return std::max(1, static_cast<int>(edgeLength / gridSize) - 1);
}

int SnapPointCalculator::calculateTotalSnapPoints(const NodeLayout& node, NodeEdge edge, float gridSize) {
    return calculateTotalSnapPoints(getEdgeLength(node, edge), gridSize);
}

Point SnapPointCalculator::calculateFromRatio(
    const NodeLayout& node,
    NodeEdge edge,
    float ratio,
    float gridSize)
{
    // Calculate edge length using helper
    float edgeLength = getEdgeLength(node, edge);

    // Calculate position from ratio (A* standard implementation)
    Point position;
    if (edge == NodeEdge::Top) {
        position = {node.position.x + edgeLength * ratio, node.position.y};
    } else if (edge == NodeEdge::Bottom) {
        position = {node.position.x + edgeLength * ratio, node.position.y + node.size.height};
    } else if (edge == NodeEdge::Left) {
        position = {node.position.x, node.position.y + edgeLength * ratio};
    } else { // Right
        position = {node.position.x + node.size.width, node.position.y + edgeLength * ratio};
    }

    // Quantize to grid (CRITICAL - A* standard)
    // Division by zero protection: if gridSize <= 0, return position as-is
    if (gridSize > 0.0f) {
        position.x = std::round(position.x / gridSize) * gridSize;
        position.y = std::round(position.y / gridSize) * gridSize;
    }

    return position;
}

Point SnapPointCalculator::calculateFromIndex(
    const NodeLayout& node,
    NodeEdge edge,
    int snapIndex,
    int totalSnapPoints,
    float gridSize)
{
    // Clamp input values
    if (totalSnapPoints <= 0) totalSnapPoints = 1;
    if (snapIndex < 0) snapIndex = 0;
    if (snapIndex >= totalSnapPoints) snapIndex = totalSnapPoints - 1;

    // Convert snap index to ratio (A* standard formula)
    // Formula: (snapIndex + 1) / (totalSnapPoints + 1)
    // This distributes snap points evenly INSIDE the edge, excluding corners
    // Example: 2 snap points -> ratios 1/3, 2/3 (positions at 33% and 66%)
    float ratio = static_cast<float>(snapIndex + 1) / static_cast<float>(totalSnapPoints + 1);

    return calculateFromRatio(node, edge, ratio, gridSize);
}

}  // namespace arborvia
