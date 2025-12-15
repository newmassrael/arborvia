#include "GridSnapCalculator.h"
#include "arborvia/core/GeometryUtils.h"

#include <cmath>
#include <algorithm>

namespace arborvia {

// =============================================================================
// Low-level Grid Operations
// =============================================================================

std::pair<int, int> GridSnapCalculator::getEdgeGridRange(
    const NodeLayout& node,
    NodeEdge edge,
    float gridSize)
{
    // For Top/Bottom edges: range is along X axis
    // For Left/Right edges: range is along Y axis
    // IMPORTANT: Corners must be EXCLUDED from snap points
    // Corners are points where two edges meet, causing direction ambiguity
    const float epsilon = 0.001f;

    switch (edge) {
        case NodeEdge::Top:
        case NodeEdge::Bottom: {
            float leftCorner = node.position.x;
            float rightCorner = node.position.x + node.size.width;

            int gridLeft = static_cast<int>(std::ceil(leftCorner / gridSize));
            int gridRight = static_cast<int>(std::floor(rightCorner / gridSize));

            // Exclude corners: if grid position is exactly at corner, move inward
            if (std::abs(gridLeft * gridSize - leftCorner) < epsilon) {
                gridLeft += 1;
            }
            if (std::abs(gridRight * gridSize - rightCorner) < epsilon) {
                gridRight -= 1;
            }
            return {gridLeft, gridRight};
        }
        case NodeEdge::Left:
        case NodeEdge::Right: {
            float topCorner = node.position.y;
            float bottomCorner = node.position.y + node.size.height;

            int gridTop = static_cast<int>(std::ceil(topCorner / gridSize));
            int gridBottom = static_cast<int>(std::floor(bottomCorner / gridSize));

            // Exclude corners: if grid position is exactly at corner, move inward
            if (std::abs(gridTop * gridSize - topCorner) < epsilon) {
                gridTop += 1;
            }
            if (std::abs(gridBottom * gridSize - bottomCorner) < epsilon) {
                gridBottom -= 1;
            }
            return {gridTop, gridBottom};
        }
    }
    return {0, 0};
}

int GridSnapCalculator::getCandidateCount(
    const NodeLayout& node,
    NodeEdge edge,
    float gridSize)
{
    auto [gridStart, gridEnd] = getEdgeGridRange(node, edge, gridSize);
    return std::max(0, gridEnd - gridStart + 1);
}

// =============================================================================
// Mid-level Candidate Management
// =============================================================================

std::vector<int> GridSnapCalculator::selectCandidateIndices(
    int candidateCount,
    int connectionCount)
{
    std::vector<int> indices;
    if (candidateCount <= 0 || connectionCount <= 0) return indices;

    indices.reserve(connectionCount);

    if (connectionCount >= candidateCount) {
        // More connections than candidates: distribute evenly with sharing
        // Each connection gets the candidate at position (i * candidateCount) / connectionCount
        for (int i = 0; i < connectionCount; ++i) {
            int idx = (i * candidateCount) / connectionCount;
            idx = std::min(idx, candidateCount - 1);  // Clamp to valid range
            indices.push_back(idx);
        }
    } else {
        // Fewer connections than candidates: consecutive allocation, center-aligned
        // This ensures visual position matches index number (no gaps like [2,3,5,6])
        int start = (candidateCount - connectionCount) / 2;
        for (int i = 0; i < connectionCount; ++i) {
            indices.push_back(start + i);
        }
    }
    return indices;
}

Point GridSnapCalculator::getPositionFromCandidateIndex(
    const NodeLayout& node,
    NodeEdge edge,
    int candidateIndex,
    float gridSize)
{
    // Point node handling (Single Source of Truth)
    // Point nodes have no valid snap candidates, always return center
    int candidateCount = getCandidateCount(node, edge, gridSize);
    if (candidateCount <= 0) {
        return node.center();
    }

    auto [gridStart, gridEnd] = getEdgeGridRange(node, edge, gridSize);
    int gridPos = gridStart + candidateIndex;
    float pixelCoord = gridPos * gridSize;

    // Quantize BOTH coordinates to grid for orthogonality with A* paths
    float perpendicular;
    switch (edge) {
        case NodeEdge::Top:
            perpendicular = std::round(node.position.y / gridSize) * gridSize;
            return {pixelCoord, perpendicular};
        case NodeEdge::Bottom:
            perpendicular = std::round((node.position.y + node.size.height) / gridSize) * gridSize;
            return {pixelCoord, perpendicular};
        case NodeEdge::Left:
            perpendicular = std::round(node.position.x / gridSize) * gridSize;
            return {perpendicular, pixelCoord};
        case NodeEdge::Right:
            perpendicular = std::round((node.position.x + node.size.width) / gridSize) * gridSize;
            return {perpendicular, pixelCoord};
    }
    return node.center();
}

Point GridSnapCalculator::getPositionFromStoredIndex(
    const NodeLayout& node,
    NodeEdge edge,
    int storedSnapIndex,
    float gridSize)
{
    int candidateCount = getCandidateCount(node, edge, gridSize);

    if (candidateCount <= 0) {
        return node.center();
    }

    // Clamp to valid range
    int candidateIndex = std::max(0, std::min(storedSnapIndex, candidateCount - 1));
    return getPositionFromCandidateIndex(node, edge, candidateIndex, gridSize);
}

int GridSnapCalculator::getCandidateIndexFromPosition(
    const NodeLayout& node,
    NodeEdge edge,
    const Point& position,
    float gridSize)
{
    auto [gridStart, gridEnd] = getEdgeGridRange(node, edge, gridSize);
    int candidateCount = gridEnd - gridStart + 1;

    if (candidateCount <= 0) {
        return 0;
    }

    // Extract the relevant coordinate based on edge type
    // Top/Bottom edges use X coordinate, Left/Right use Y coordinate
    float pixelCoord;
    switch (edge) {
        case NodeEdge::Top:
        case NodeEdge::Bottom:
            pixelCoord = position.x;
            break;
        case NodeEdge::Left:
        case NodeEdge::Right:
            pixelCoord = position.y;
            break;
        default:
            return 0;
    }

    // Convert pixel coordinate to grid position (with rounding)
    int gridPos = static_cast<int>(std::round(pixelCoord / gridSize));

    // Calculate candidate index from grid position
    int candidateIndex = gridPos - gridStart;

    // Clamp to valid range [0, candidateCount-1]
    return std::max(0, std::min(candidateIndex, candidateCount - 1));
}

// =============================================================================
// High-level Snap Calculation
// =============================================================================

Point GridSnapCalculator::calculateSnapPosition(
    const NodeLayout& node,
    NodeEdge edge,
    int connectionIndex,
    int totalConnections,
    float gridSize,
    int* outCandidateIndex)
{
    // Use fixed grid-based candidate system
    int candidateCount = getCandidateCount(node, edge, gridSize);
    
    if (candidateCount <= 0) {
        // Fallback to center if no candidates available
        if (outCandidateIndex) *outCandidateIndex = 0;
        return node.center();
    }
    
    // Select candidate index based on connection index
    std::vector<int> selectedIndices = selectCandidateIndices(candidateCount, totalConnections);
    
    int candidateIndex = 0;
    if (connectionIndex >= 0 && connectionIndex < static_cast<int>(selectedIndices.size())) {
        candidateIndex = selectedIndices[connectionIndex];
    } else {
        // Fallback: clamp to valid candidate range
        // This handles edge cases where connectionIndex is out of expected range
        candidateIndex = std::max(0, std::min(connectionIndex, candidateCount - 1));
    }
    
    if (outCandidateIndex) *outCandidateIndex = candidateIndex;
    return getPositionFromCandidateIndex(node, edge, candidateIndex, gridSize);
}

Point GridSnapCalculator::calculatePositionInContext(
    const std::vector<NodeLayout>& nodeLayouts,
    const EdgeLayout& targetEdge,
    const std::vector<EdgeLayout>& allEdgeLayouts,
    bool isSource,
    float gridSize,
    int* outCandidateIndex)
{
    // Get the node and edge we're calculating for
    NodeId nodeId = isSource ? targetEdge.from : targetEdge.to;
    NodeEdge nodeEdge = isSource ? targetEdge.sourceEdge : targetEdge.targetEdge;
    
    // Find the node layout
    const NodeLayout* nodeLayout = nullptr;
    for (const auto& nl : nodeLayouts) {
        if (nl.id == nodeId) {
            nodeLayout = &nl;
            break;
        }
    }
    
    if (!nodeLayout) {
        // Node not found - fallback
        if (outCandidateIndex) *outCandidateIndex = 0;
        return isSource ? targetEdge.sourcePoint : targetEdge.targetPoint;
    }
    
    // Count connections on this node edge and find this edge's index
    int totalConnections = 0;
    int connectionIndex = 0;
    bool foundTargetEdge = false;
    
    for (const auto& el : allEdgeLayouts) {
        // Check source side connections
        if (el.from == nodeId && el.sourceEdge == nodeEdge) {
            if (el.id == targetEdge.id && isSource) {
                connectionIndex = totalConnections;
                foundTargetEdge = true;
            }
            ++totalConnections;
        }
        // Check target side connections
        if (el.to == nodeId && el.targetEdge == nodeEdge) {
            if (el.id == targetEdge.id && !isSource) {
                connectionIndex = totalConnections;
                foundTargetEdge = true;
            }
            ++totalConnections;
        }
    }
    
    if (!foundTargetEdge || totalConnections == 0) {
        // Edge not found or no connections - use single connection fallback
        totalConnections = 1;
        connectionIndex = 0;
    }
    
    return calculateSnapPosition(*nodeLayout, nodeEdge, connectionIndex, totalConnections, gridSize, outCandidateIndex);
}

Point GridSnapCalculator::computeSnapPointFromRatio(
    const NodeLayout& node,
    NodeEdge edge,
    float ratio,
    float gridSize)
{
    // Calculate edge properties
    float edgeLength, edgeStart, perpendicular;
    
    switch (edge) {
        case NodeEdge::Top:
            edgeLength = node.size.width;
            edgeStart = node.position.x;
            perpendicular = node.position.y;
            break;
        case NodeEdge::Bottom:
            edgeLength = node.size.width;
            edgeStart = node.position.x;
            perpendicular = node.position.y + node.size.height;
            break;
        case NodeEdge::Left:
            edgeLength = node.size.height;
            edgeStart = node.position.y;
            perpendicular = node.position.x;
            break;
        case NodeEdge::Right:
            edgeLength = node.size.height;
            edgeStart = node.position.y;
            perpendicular = node.position.x + node.size.width;
            break;
        default:
            return node.center();
    }
    
    // Calculate position along edge
    float rawAlongEdge = edgeStart + edgeLength * ratio;
    
    // Quantize BOTH coordinates to grid for orthogonality
    if (gridSize > 0.0f) {
        float quantizedAlongEdge = std::round(rawAlongEdge / gridSize) * gridSize;
        float quantizedPerpendicular = std::round(perpendicular / gridSize) * gridSize;
        
        switch (edge) {
            case NodeEdge::Top:
            case NodeEdge::Bottom:
                return {quantizedAlongEdge, quantizedPerpendicular};
            case NodeEdge::Left:
            case NodeEdge::Right:
                return {quantizedPerpendicular, quantizedAlongEdge};
            default:
                return node.center();
        }
    }
    
    // No grid - return raw position
    switch (edge) {
        case NodeEdge::Top:
        case NodeEdge::Bottom:
            return {rawAlongEdge, perpendicular};
        case NodeEdge::Left:
        case NodeEdge::Right:
            return {perpendicular, rawAlongEdge};
        default:
            return node.center();
    }
}

// =============================================================================
// Utility Functions
// =============================================================================

void GridSnapCalculator::distributePositionsQuantized(
    int start,
    int end,
    int count,
    std::vector<int>& outPositions)
{
    if (count <= 0) return;
    int length = end - start;
    int divisor = count + 1;

    for (int i = 0; i < count; ++i) {
        // Integer division with rounding: (a * b + divisor/2) / divisor
        int gridPos = start + (length * (i + 1) * 2 + divisor) / (2 * divisor);
        outPositions.push_back(gridPos);
    }
}

}  // namespace arborvia
