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

    // NOTE: Snap points must be ON the actual node edge, not grid-snapped edge.
    // Only the coordinate ALONG the edge (X for Top/Bottom, Y for Left/Right) is grid-aligned.
    // The coordinate PERPENDICULAR to the edge is the exact node edge position.
    switch (edge) {
        case NodeEdge::Top: {
            // X is grid-aligned, Y is exact node top edge
            return {pixelCoord, node.position.y};
        }
        case NodeEdge::Bottom: {
            // X is grid-aligned, Y is exact node bottom edge
            return {pixelCoord, node.position.y + node.size.height};
        }
        case NodeEdge::Left: {
            // Y is grid-aligned, X is exact node left edge
            return {node.position.x, pixelCoord};
        }
        case NodeEdge::Right: {
            // Y is grid-aligned, X is exact node right edge
            return {node.position.x + node.size.width, pixelCoord};
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
