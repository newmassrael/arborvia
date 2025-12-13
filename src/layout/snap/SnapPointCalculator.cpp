#include "SnapPointCalculator.h"
#include "GridSnapCalculator.h"

#include <algorithm>
#include <cmath>
#include "arborvia/common/Logger.h"

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
    // Delegate to GridSnapCalculator for consistent computation
    // This ensures:
    // - Coordinate along edge is grid-quantized
    // - Coordinate perpendicular to edge is EXACT node edge position
    Point position = GridSnapCalculator::computeSnapPointFromRatio(node, edge, ratio, gridSize);

    // Debug logging for ratio-based snap point calculation
    const char* edgeName = (edge == NodeEdge::Top) ? "top" : 
                           (edge == NodeEdge::Bottom) ? "bottom" :
                           (edge == NodeEdge::Left) ? "left" : "right";
    LOG_DEBUG("[SnapCalc-Ratio] node={} edge={} ratio={:.3f} -> ({:.0f},{:.0f}) gridSize={}",
              node.id, edgeName, ratio, position.x, position.y, gridSize);

    return position;
}

Point SnapPointCalculator::calculateFromIndex(
    const NodeLayout& node,
    NodeEdge edge,
    int snapIndex,
    int totalSnapPoints,
    float gridSize)
{
    int originalSnapIndex = snapIndex;
    int originalTotalSnapPoints = totalSnapPoints;
    
    // Clamp input values
    if (totalSnapPoints <= 0) totalSnapPoints = 1;
    if (snapIndex < 0) snapIndex = 0;
    if (snapIndex >= totalSnapPoints) snapIndex = totalSnapPoints - 1;

    // Convert snap index to ratio (A* standard formula)
    // Formula: (snapIndex + 1) / (totalSnapPoints + 1)
    // This distributes snap points evenly INSIDE the edge, excluding corners
    // Example: 2 snap points -> ratios 1/3, 2/3 (positions at 33% and 66%)
    float ratio = static_cast<float>(snapIndex + 1) / static_cast<float>(totalSnapPoints + 1);

    Point result = calculateFromRatio(node, edge, ratio, gridSize);
    
    // Debug logging for snap point calculation
    const char* edgeName = (edge == NodeEdge::Top) ? "top" : 
                           (edge == NodeEdge::Bottom) ? "bottom" :
                           (edge == NodeEdge::Left) ? "left" : "right";
    LOG_DEBUG("[SnapCalc] node={} edge={} snapIndex={}/{} (orig={}/{}) ratio={:.3f} -> ({:.0f},{:.0f}) gridSize={}",
              node.id, edgeName, snapIndex, totalSnapPoints, 
              originalSnapIndex, originalTotalSnapPoints,
              ratio, result.x, result.y, gridSize);
    
    return result;
}

}  // namespace arborvia
