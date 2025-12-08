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
    switch (edge) {
        case NodeEdge::Top:
        case NodeEdge::Bottom: {
            int gridLeft = static_cast<int>(std::ceil(node.position.x / gridSize));
            int gridRight = static_cast<int>(std::floor((node.position.x + node.size.width) / gridSize));
            return {gridLeft, gridRight};
        }
        case NodeEdge::Left:
        case NodeEdge::Right: {
            int gridTop = static_cast<int>(std::ceil(node.position.y / gridSize));
            int gridBottom = static_cast<int>(std::floor((node.position.y + node.size.height) / gridSize));
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
        // Fewer connections than candidates: distribute evenly among candidates
        // Use formula that spreads connections evenly across the available candidates
        int divisor = connectionCount + 1;
        for (int i = 0; i < connectionCount; ++i) {
            int idx = ((candidateCount - 1) * (i + 1) * 2 + divisor) / (2 * divisor);
            indices.push_back(idx);
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
    auto [gridStart, gridEnd] = getEdgeGridRange(node, edge, gridSize);
    int gridPos = gridStart + candidateIndex;
    float pixelCoord = gridPos * gridSize;

    switch (edge) {
        case NodeEdge::Top: {
            float edgeY = std::floor(node.position.y / gridSize) * gridSize;
            return {pixelCoord, edgeY};
        }
        case NodeEdge::Bottom: {
            float edgeY = std::ceil((node.position.y + node.size.height) / gridSize) * gridSize;
            return {pixelCoord, edgeY};
        }
        case NodeEdge::Left: {
            float edgeX = std::floor(node.position.x / gridSize) * gridSize;
            return {edgeX, pixelCoord};
        }
        case NodeEdge::Right: {
            float edgeX = std::ceil((node.position.x + node.size.width) / gridSize) * gridSize;
            return {edgeX, pixelCoord};
        }
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

float GridSnapCalculator::getEffectiveGridSize(float gridSize)
{
    return gridSize > 0.0f ? gridSize : constants::PATHFINDING_GRID_SIZE;
}

}  // namespace arborvia
