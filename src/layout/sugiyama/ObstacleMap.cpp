#include "ObstacleMap.h"
#include <algorithm>
#include <cmath>

namespace arborvia {
namespace algorithms {

void ObstacleMap::buildFromNodes(
    const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
    float gridSize,
    int margin) {
    
    if (nodeLayouts.empty() || gridSize <= 0) {
        grid_.clear();
        width_ = 0;
        height_ = 0;
        return;
    }

    gridSize_ = gridSize;

    // Find bounding box of all nodes in pixel coordinates
    float minX = std::numeric_limits<float>::max();
    float minY = std::numeric_limits<float>::max();
    float maxX = std::numeric_limits<float>::lowest();
    float maxY = std::numeric_limits<float>::lowest();

    for (const auto& [nodeId, node] : nodeLayouts) {
        minX = std::min(minX, node.position.x);
        minY = std::min(minY, node.position.y);
        maxX = std::max(maxX, node.position.x + node.size.width);
        maxY = std::max(maxY, node.position.y + node.size.height);
    }

    // Convert to grid coordinates with padding for safe zones
    const int padding = margin + 2;  // Extra padding for safe zone routing
    
    int gridMinX = static_cast<int>(std::floor(minX / gridSize)) - padding;
    int gridMinY = static_cast<int>(std::floor(minY / gridSize)) - padding;
    int gridMaxX = static_cast<int>(std::ceil(maxX / gridSize)) + padding;
    int gridMaxY = static_cast<int>(std::ceil(maxY / gridSize)) + padding;

    offsetX_ = gridMinX;
    offsetY_ = gridMinY;
    width_ = gridMaxX - gridMinX + 1;
    height_ = gridMaxY - gridMinY + 1;

    // Initialize grid
    grid_.clear();
    grid_.resize(static_cast<size_t>(width_) * height_);

    // Mark cells blocked by each node (with margin)
    for (const auto& [nodeId, node] : nodeLayouts) {
        // Calculate node bounds in grid coordinates
        int nodeGridMinX = static_cast<int>(std::floor(node.position.x / gridSize)) - margin;
        int nodeGridMinY = static_cast<int>(std::floor(node.position.y / gridSize)) - margin;
        int nodeGridMaxX = static_cast<int>(std::ceil((node.position.x + node.size.width) / gridSize)) + margin;
        int nodeGridMaxY = static_cast<int>(std::ceil((node.position.y + node.size.height) / gridSize)) + margin;

        // Mark all cells in this range as blocked
        for (int gy = nodeGridMinY; gy <= nodeGridMaxY; ++gy) {
            for (int gx = nodeGridMinX; gx <= nodeGridMaxX; ++gx) {
                int idx = toIndex(gx, gy);
                if (idx >= 0) {
                    grid_[idx].blocked = true;
                    grid_[idx].blockingNodes.push_back(nodeId);
                }
            }
        }
    }

    // Compute safe zones (grid coordinates guaranteed to be outside all nodes)
    safeZones_.yAbove = static_cast<int>(std::floor(minY / gridSize)) - margin - 1;
    safeZones_.yBelow = static_cast<int>(std::ceil(maxY / gridSize)) + margin + 1;
    safeZones_.xLeft = static_cast<int>(std::floor(minX / gridSize)) - margin - 1;
    safeZones_.xRight = static_cast<int>(std::ceil(maxX / gridSize)) + margin + 1;
}

bool ObstacleMap::isBlocked(int gridX, int gridY) const {
    int idx = toIndex(gridX, gridY);
    if (idx < 0) {
        return false;  // Out of bounds = not blocked (infinite free space)
    }
    return grid_[idx].blocked;
}

bool ObstacleMap::isBlocked(int gridX, int gridY,
                            const std::unordered_set<NodeId>& exclude) const {
    int idx = toIndex(gridX, gridY);
    if (idx < 0) {
        return false;  // Out of bounds = not blocked
    }
    
    const GridCell& cell = grid_[idx];
    if (!cell.blocked) {
        return false;
    }
    
    // Blocked - check if ANY blocking node is NOT excluded
    // If at least one blocking node is not in the exclude set, cell is blocked
    for (NodeId blockingNode : cell.blockingNodes) {
        if (exclude.find(blockingNode) == exclude.end()) {
            return true;  // Found a non-excluded blocking node
        }
    }
    
    // All blocking nodes are excluded
    return false;
}

bool ObstacleMap::segmentBlocked(int x1, int y1, int x2, int y2,
                                  const std::unordered_set<NodeId>& exclude) const {
    // Determine if horizontal or vertical
    if (y1 == y2) {
        // Horizontal segment
        int minX = std::min(x1, x2);
        int maxX = std::max(x1, x2);
        for (int gx = minX; gx <= maxX; ++gx) {
            if (isBlocked(gx, y1, exclude)) {
                return true;
            }
        }
    } else if (x1 == x2) {
        // Vertical segment
        int minY = std::min(y1, y2);
        int maxY = std::max(y1, y2);
        for (int gy = minY; gy <= maxY; ++gy) {
            if (isBlocked(x1, gy, exclude)) {
                return true;
            }
        }
    } else {
        // Diagonal segment - check all cells along the line using Bresenham-like approach
        int dx = std::abs(x2 - x1);
        int dy = std::abs(y2 - y1);
        int sx = (x1 < x2) ? 1 : -1;
        int sy = (y1 < y2) ? 1 : -1;
        int err = dx - dy;
        
        int cx = x1, cy = y1;
        while (true) {
            if (isBlocked(cx, cy, exclude)) {
                return true;
            }
            if (cx == x2 && cy == y2) {
                break;
            }
            int e2 = 2 * err;
            if (e2 > -dy) {
                err -= dy;
                cx += sx;
            }
            if (e2 < dx) {
                err += dx;
                cy += sy;
            }
        }
    }
    
    return false;
}

NodeId ObstacleMap::getBlockingNode(int gridX, int gridY) const {
    int idx = toIndex(gridX, gridY);
    if (idx < 0) {
        return INVALID_NODE;
    }
    const GridCell& cell = grid_[idx];
    if (cell.blockingNodes.empty()) {
        return INVALID_NODE;
    }
    return cell.blockingNodes[0];  // Return first blocking node
}

GridPoint ObstacleMap::pixelToGrid(const Point& p) const {
    return {
        static_cast<int>(std::round(p.x / gridSize_)),
        static_cast<int>(std::round(p.y / gridSize_))
    };
}

Point ObstacleMap::gridToPixel(int gridX, int gridY) const {
    return {
        gridX * gridSize_,
        gridY * gridSize_
    };
}

bool ObstacleMap::inBounds(int gridX, int gridY) const {
    return toIndex(gridX, gridY) >= 0;
}

int ObstacleMap::toIndex(int gridX, int gridY) const {
    int localX = gridX - offsetX_;
    int localY = gridY - offsetY_;
    
    if (localX < 0 || localX >= width_ || localY < 0 || localY >= height_) {
        return -1;
    }
    
    return localY * width_ + localX;
}

}  // namespace algorithms
}  // namespace arborvia
