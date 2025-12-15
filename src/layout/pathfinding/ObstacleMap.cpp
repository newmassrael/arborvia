#include "ObstacleMap.h"
#include "arborvia/common/Logger.h"
#include <algorithm>
#include <cmath>


namespace arborvia {


void ObstacleMap::buildFromNodes(
    const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
    float gridSize,
    int margin,
    const std::unordered_map<EdgeId, EdgeLayout>* edgeLayouts) {

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

    // Expand bounds to include edge segment extents if provided
    // This ensures edge segments outside node bounds can be registered as obstacles
    if (edgeLayouts) {
        for (const auto& [edgeId, edge] : *edgeLayouts) {
            edge.forEachSegment([&](const Point& p1, const Point& p2) {
                minX = std::min({minX, p1.x, p2.x});
                minY = std::min({minY, p1.y, p2.y});
                maxX = std::max({maxX, p1.x, p2.x});
                maxY = std::max({maxY, p1.y, p2.y});
            });
        }
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

    // Pre-calculate proximity map for O(1) lookup
    buildProximityMap();
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
    
    // If this cell is blocked by edge segments (no blocking nodes), always blocked
    // Edge segment blocking cannot be excluded
    if (cell.blockingNodes.empty()) {
        return true;  // Blocked by edge segment, cannot be excluded
    }
    
    // Blocked by nodes - check if ANY blocking node is NOT excluded
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

void ObstacleMap::addEdgeSegments(
    const std::unordered_map<EdgeId, EdgeLayout>& edgeLayouts,
    EdgeId excludeEdgeId) {

    // Initialize edge segment tracking if needed
    if (edgeSegmentCells_.size() != grid_.size()) {
        edgeSegmentCells_.resize(grid_.size(), false);
    }


    for (const auto& [edgeId, layout] : edgeLayouts) {
        if (edgeId == excludeEdgeId) {
            continue;  // Skip the edge being routed
        }
        // Note: Self-loops are NOT skipped - they should block other edges too


        // Build segments from edge layout
        std::vector<std::pair<Point, Point>> segments;
        layout.forEachSegment([&](const Point& p1, const Point& p2) {
            segments.emplace_back(p1, p2);
        });

        // Register ALL segments with direction-aware blocking, but skip endpoint cells
        // to prevent FIRST MOVE BLOCKED issues at snap points
        // - First segment: skip start cell (source snap point)
        // - Last segment: skip end cell (target snap point)
        // - Middle segments: full blocking
        for (size_t i = 0; i < segments.size(); ++i) {
            bool skipStartCell = (i == 0);  // First segment: skip source point
            bool skipEndCell = (i + 1 == segments.size());  // Last segment: skip target point
            
            markSegmentBlockedWithSkip(segments[i].first, segments[i].second, skipStartCell, skipEndCell);
        }
    }
}

void ObstacleMap::addEdgeSegmentsWithPointNodeAwareness(
    const std::unordered_map<EdgeId, EdgeLayout>& edgeLayouts,
    const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
    EdgeId excludeEdgeId,
    NodeId targetNodeId) {

    // Initialize edge segment tracking if needed
    if (edgeSegmentCells_.size() != grid_.size()) {
        edgeSegmentCells_.resize(grid_.size(), false);
    }

    // Check if target node is a Point node
    bool targetIsPointNode = false;
    auto targetIt = nodeLayouts.find(targetNodeId);
    if (targetIt != nodeLayouts.end()) {
        targetIsPointNode = targetIt->second.isPointNode();
    }

    for (const auto& [edgeId, layout] : edgeLayouts) {
        if (edgeId == excludeEdgeId) {
            continue;  // Skip the edge being routed
        }

        // Check if this edge shares the same Point target
        bool sharesPointTarget = targetIsPointNode && (layout.to == targetNodeId);

        // Build segments from edge layout
        std::vector<std::pair<Point, Point>> segments;
        layout.forEachSegment([&](const Point& p1, const Point& p2) {
            segments.emplace_back(p1, p2);
        });

        // Register segments with direction-aware blocking
        // For edges sharing a Point target: skip the ENTIRE last segment
        // For normal edges: skip only endpoint cells
        for (size_t i = 0; i < segments.size(); ++i) {
            bool isLastSegment = (i + 1 == segments.size());
            
            // Skip entire last segment for edges sharing a Point target
            if (sharesPointTarget && isLastSegment) {
                LOG_DEBUG("[addEdgeSegmentsWithPointNodeAwareness] Edge {} shares Point target {}, skipping last segment",
                          edgeId, targetNodeId);
                continue;
            }
            
            bool skipStartCell = (i == 0);  // First segment: skip source point
            bool skipEndCell = isLastSegment;  // Last segment: skip target point
            
            markSegmentBlockedWithSkip(segments[i].first, segments[i].second, skipStartCell, skipEndCell);
        }
    }
}

void ObstacleMap::addSingleEdgeSegments(const EdgeLayout& layout) {
    // Initialize edge segment tracking if needed
    if (edgeSegmentCells_.size() != grid_.size()) {
        edgeSegmentCells_.resize(grid_.size(), false);
    }


    // Build segments from edge layout
    std::vector<std::pair<Point, Point>> segments;
    layout.forEachSegment([&](const Point& p1, const Point& p2) {
        segments.emplace_back(p1, p2);
    });

    // Register ALL segments with direction-aware blocking, but skip endpoint cells
    // to prevent FIRST MOVE BLOCKED issues at snap points
    // - First segment: skip start cell (source snap point)
    // - Last segment: skip end cell (target snap point)
    // - Middle segments: full blocking
    for (size_t i = 0; i < segments.size(); ++i) {
        bool skipStartCell = (i == 0);  // First segment: skip source point
        bool skipEndCell = (i + 1 == segments.size());  // Last segment: skip target point
        
        markSegmentBlockedWithSkip(segments[i].first, segments[i].second, skipStartCell, skipEndCell);
    }
}

void ObstacleMap::clearEdgeSegments() {
    // Only clear cells that were blocked by edge segments
    for (size_t i = 0; i < grid_.size() && i < edgeSegmentCells_.size(); ++i) {
        if (edgeSegmentCells_[i]) {
            // Only unblock if no nodes are blocking this cell
            if (grid_[i].blockingNodes.empty()) {
                grid_[i].blocked = false;
            }
            // Clear direction-aware edge segment flags
            grid_[i].horizontalSegment = false;
            grid_[i].verticalSegment = false;
            edgeSegmentCells_[i] = false;
        }
    }
}

void ObstacleMap::markSegmentBlockedWithSkip(const Point& p1, const Point& p2, bool skipStartCell, bool skipEndCell) {
    // Convert to grid coordinates
    GridPoint g1 = pixelToGrid(p1);
    GridPoint g2 = pixelToGrid(p2);


    // Determine if horizontal or vertical
    if (g1.y == g2.y) {
        // Horizontal segment
        int startX = std::min(g1.x, g2.x);
        int endX = std::max(g1.x, g2.x);
        
        // Adjust for skip flags (g1 is logical start, g2 is logical end)
        int actualStartX = (skipStartCell && g1.x == startX) ? startX + 1 : 
                           (skipEndCell && g2.x == startX) ? startX + 1 : startX;
        int actualEndX = (skipEndCell && g2.x == endX) ? endX - 1 :
                         (skipStartCell && g1.x == endX) ? endX - 1 : endX;
        
        for (int gx = actualStartX; gx <= actualEndX; ++gx) {
            int idx = toIndex(gx, g1.y);
            if (idx >= 0) {
                grid_[idx].horizontalSegment = true;
                edgeSegmentCells_[idx] = true;
            }
        }
    } else if (g1.x == g2.x) {
        // Vertical segment
        int startY = std::min(g1.y, g2.y);
        int endY = std::max(g1.y, g2.y);
        
        // Adjust for skip flags (g1 is logical start, g2 is logical end)
        int actualStartY = (skipStartCell && g1.y == startY) ? startY + 1 :
                           (skipEndCell && g2.y == startY) ? startY + 1 : startY;
        int actualEndY = (skipEndCell && g2.y == endY) ? endY - 1 :
                         (skipStartCell && g1.y == endY) ? endY - 1 : endY;
        
        for (int gy = actualStartY; gy <= actualEndY; ++gy) {
            int idx = toIndex(g1.x, gy);
            if (idx >= 0) {
                grid_[idx].verticalSegment = true;
                edgeSegmentCells_[idx] = true;
            }
        }
    }
    // Diagonal segments are ignored for skip logic (shouldn't happen in orthogonal routing)
}

void ObstacleMap::markSegmentBlocked(const Point& p1, const Point& p2, bool isEdgeSegment) {
    // Convert to grid coordinates
    GridPoint g1 = pixelToGrid(p1);
    GridPoint g2 = pixelToGrid(p2);

    // Determine if horizontal or vertical
    if (g1.y == g2.y) {
        // Horizontal segment - set horizontalSegment flag for direction-aware blocking
        int minX = std::min(g1.x, g2.x);
        int maxX = std::max(g1.x, g2.x);
        for (int gx = minX; gx <= maxX; ++gx) {
            int idx = toIndex(gx, g1.y);
            if (idx >= 0) {
                if (isEdgeSegment) {
                    // Edge segments use direction-aware blocking
                    grid_[idx].horizontalSegment = true;
                    edgeSegmentCells_[idx] = true;
                } else {
                    // Non-edge segments still use full blocking
                    grid_[idx].blocked = true;
                }
            }
        }
    } else if (g1.x == g2.x) {
        // Vertical segment - set verticalSegment flag for direction-aware blocking
        int minY = std::min(g1.y, g2.y);
        int maxY = std::max(g1.y, g2.y);
        for (int gy = minY; gy <= maxY; ++gy) {
            int idx = toIndex(g1.x, gy);
            if (idx >= 0) {
                if (isEdgeSegment) {
                    // Edge segments use direction-aware blocking
                    grid_[idx].verticalSegment = true;
                    edgeSegmentCells_[idx] = true;
                } else {
                    // Non-edge segments still use full blocking
                    grid_[idx].blocked = true;
                }
            }
        }
    } else {
        // Diagonal segment - use Bresenham-like line drawing
        // For diagonal segments, set both direction flags (shouldn't happen in orthogonal routing)
        int dx = std::abs(g2.x - g1.x);
        int dy = std::abs(g2.y - g1.y);
        int sx = (g1.x < g2.x) ? 1 : -1;
        int sy = (g1.y < g2.y) ? 1 : -1;
        int err = dx - dy;

        int cx = g1.x, cy = g1.y;
        while (true) {
            int idx = toIndex(cx, cy);
            if (idx >= 0) {
                if (isEdgeSegment) {
                    // Diagonal segments block both directions
                    grid_[idx].horizontalSegment = true;
                    grid_[idx].verticalSegment = true;
                    edgeSegmentCells_[idx] = true;
                } else {
                    grid_[idx].blocked = true;
                }
            }
            if (cx == g2.x && cy == g2.y) {
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
}

void ObstacleMap::markSegmentBlockedSkipEndpoints(const Point& p1, const Point& p2) {
    // Convert to grid coordinates
    GridPoint g1 = pixelToGrid(p1);
    GridPoint g2 = pixelToGrid(p2);

    // Skip very short segments (1 or 2 cells)
    int segmentLength = 0;
    if (g1.y == g2.y) {
        segmentLength = std::abs(g2.x - g1.x) + 1;
    } else if (g1.x == g2.x) {
        segmentLength = std::abs(g2.y - g1.y) + 1;
    }
    
    if (segmentLength <= 2) {
        // Too short to skip endpoints - don't block at all
        return;
    }

    // Determine if horizontal or vertical
    if (g1.y == g2.y) {
        // Horizontal segment - skip first and last cell
        int minX = std::min(g1.x, g2.x);
        int maxX = std::max(g1.x, g2.x);
        for (int gx = minX + 1; gx < maxX; ++gx) {  // Skip endpoints
            int idx = toIndex(gx, g1.y);
            if (idx >= 0) {
                grid_[idx].horizontalSegment = true;
                edgeSegmentCells_[idx] = true;
            }
        }
    } else if (g1.x == g2.x) {
        // Vertical segment - skip first and last cell
        int minY = std::min(g1.y, g2.y);
        int maxY = std::max(g1.y, g2.y);
        for (int gy = minY + 1; gy < maxY; ++gy) {  // Skip endpoints
            int idx = toIndex(g1.x, gy);
            if (idx >= 0) {
                grid_[idx].verticalSegment = true;
                edgeSegmentCells_[idx] = true;
            }
        }
    }
    // Diagonal segments are not handled - they shouldn't exist in orthogonal routing
}

bool ObstacleMap::isBlockedForDirection(int gridX, int gridY,
                                         MoveDirection moveDir,
                                         const std::unordered_set<NodeId>& exclude) const {
    int idx = toIndex(gridX, gridY);
    if (idx < 0) {
        return false;  // Out of bounds = not blocked
    }

    const GridCell& cell = grid_[idx];

    // First check node blocking (always blocks all directions)
    if (cell.blocked) {
        // If blocked by nodes, check exclusion list
        if (cell.blockingNodes.empty()) {
            return true;  // Blocked but no specific nodes (shouldn't happen)
        }
        // Check if any blocking node is not excluded
        for (NodeId blockingNode : cell.blockingNodes) {
            if (exclude.find(blockingNode) == exclude.end()) {
                return true;  // Found non-excluded blocking node
            }
        }
    }

    // Now check direction-aware edge segment blocking
    // Edge segments only block movement parallel to them
    switch (moveDir) {
        case MoveDirection::Left:
        case MoveDirection::Right:
            // Horizontal movement blocked by horizontal segments
            return cell.horizontalSegment;

        case MoveDirection::Up:
        case MoveDirection::Down:
            // Vertical movement blocked by vertical segments
            return cell.verticalSegment;

        case MoveDirection::None:
        default:
            // No specific direction - not blocked by direction flags
            return false;
    }
}

ObstacleMap::CellVisInfo ObstacleMap::getCellVisInfo(int gridX, int gridY) const {
    CellVisInfo info;

    int idx = toIndex(gridX, gridY);
    if (idx < 0) {
        return info;  // Out of bounds - return default (all false)
    }

    const GridCell& cell = grid_[idx];
    info.isNodeBlocked = !cell.blockingNodes.empty();
    info.hasHorizontalSegment = cell.horizontalSegment;
    info.hasVerticalSegment = cell.verticalSegment;
    info.blockingNodes = cell.blockingNodes;

    return info;
}

// === Cost-based Pathfinding Support ===

int ObstacleMap::getCost(int gridX, int gridY) const {
    int idx = toIndex(gridX, gridY);
    if (idx < 0) {
        return COST_FREE;  // Out of bounds = free (infinite space)
    }

    const GridCell& cell = grid_[idx];

    // Node blocking = completely blocked
    if (cell.blocked && !cell.blockingNodes.empty()) {
        return COST_BLOCKED;
    }

    // Base cost
    int cost = COST_FREE;

    // Add edge path overlay cost
    auto overlayIt = edgeCostOverlay_.find(idx);
    if (overlayIt != edgeCostOverlay_.end()) {
        cost += overlayIt->second;
    }

    return cost;
}

int ObstacleMap::getCostForDirection(int gridX, int gridY, MoveDirection moveDir) const {
    int idx = toIndex(gridX, gridY);
    if (idx < 0) {
        return COST_FREE;  // Out of bounds = free
    }

    const GridCell& cell = grid_[idx];

    // Node blocking = completely blocked (all directions)
    if (cell.blocked && !cell.blockingNodes.empty()) {
        return COST_BLOCKED;
    }

    // Base cost
    int cost = COST_FREE;

    // Add edge path overlay cost
    auto overlayIt = edgeCostOverlay_.find(idx);
    if (overlayIt != edgeCostOverlay_.end()) {
        cost += overlayIt->second;
    }

    // Direction-aware edge segment cost
    // Parallel movement through existing segment = higher cost (discouraged but not blocked)
    switch (moveDir) {
        case MoveDirection::Left:
        case MoveDirection::Right:
            if (cell.horizontalSegment) {
                cost += COST_EDGE_PATH;  // Moving parallel to existing horizontal segment
            }
            break;

        case MoveDirection::Up:
        case MoveDirection::Down:
            if (cell.verticalSegment) {
                cost += COST_EDGE_PATH;  // Moving parallel to existing vertical segment
            }
            break;

        case MoveDirection::None:
        default:
            break;
    }

    return cost;
}

int ObstacleMap::getCostForDirectionWithExcludes(
    int gridX, int gridY,
    MoveDirection moveDir,
    const std::unordered_set<NodeId>& exclude) const {

    // First check if blocked (returns COST_BLOCKED if so)
    if (isBlockedForDirection(gridX, gridY, moveDir, exclude)) {
        return COST_BLOCKED;
    }

    // Get base cost from direction-aware calculation
    int cost = getCostForDirection(gridX, gridY, moveDir);
    if (cost >= COST_BLOCKED) {
        return cost;  // Already blocked
    }

    // O(1) proximity lookup from pre-calculated map
    int idx = toIndex(gridX, gridY);
    if (idx >= 0 && static_cast<size_t>(idx) < proximityMap_.size()) {
        const auto& entries = proximityMap_[idx];
        
        // Find minimum distance among nodes NOT in exclude set
        for (const auto& entry : entries) {
            if (entry.nodeId == INVALID_NODE) {
                break;  // No more entries
            }
            
            // Skip nodes in exclude set (source/target of this edge)
            if (exclude.find(entry.nodeId) != exclude.end()) {
                continue;
            }
            
            // Found a non-excluded node - apply proximity penalty
            if (entry.distance <= PROXIMITY_RADIUS && entry.distance > 0) {
                float ratio = 1.0f - (static_cast<float>(entry.distance - 1) / PROXIMITY_RADIUS);
                int proximityCost = static_cast<int>(ratio * COST_PROXIMITY_MAX);
                cost += proximityCost;
            }
            break;  // Only consider closest non-excluded node
        }
    }

    return cost;
}

void ObstacleMap::buildProximityMap() {
    size_t totalCells = static_cast<size_t>(width_) * height_;
    proximityMap_.clear();
    proximityMap_.resize(totalCells);
    
    // Initialize all entries
    for (auto& entries : proximityMap_) {
        for (auto& entry : entries) {
            entry.nodeId = INVALID_NODE;
            entry.distance = PROXIMITY_RADIUS + 1;
        }
    }
    
    // Better approach: For each node, expand outward marking proximity
    // We need to track node bounds - let's store them temporarily
    struct NodeBounds {
        NodeId nodeId;
        int minX, minY, maxX, maxY;
    };
    std::vector<NodeBounds> nodeBounds;
    
    // Extract node bounds from grid (cells that are blocked by each node)
    std::unordered_map<NodeId, NodeBounds> boundsMap;
    for (int y = 0; y < height_; ++y) {
        for (int x = 0; x < width_; ++x) {
            int idx = y * width_ + x;
            const auto& cell = grid_[idx];
            for (NodeId blockingNode : cell.blockingNodes) {
                int gridX = x + offsetX_;
                int gridY = y + offsetY_;
                
                auto it = boundsMap.find(blockingNode);
                if (it == boundsMap.end()) {
                    boundsMap[blockingNode] = {blockingNode, gridX, gridY, gridX, gridY};
                } else {
                    it->second.minX = std::min(it->second.minX, gridX);
                    it->second.minY = std::min(it->second.minY, gridY);
                    it->second.maxX = std::max(it->second.maxX, gridX);
                    it->second.maxY = std::max(it->second.maxY, gridY);
                }
            }
        }
    }
    
    // Convert to vector for easier iteration
    for (const auto& [nodeId, bounds] : boundsMap) {
        nodeBounds.push_back(bounds);
    }
    
    // For each node, mark proximity in surrounding cells
    for (const auto& bounds : nodeBounds) {
        // Expand by PROXIMITY_RADIUS in each direction
        int expandedMinX = bounds.minX - PROXIMITY_RADIUS;
        int expandedMaxX = bounds.maxX + PROXIMITY_RADIUS;
        int expandedMinY = bounds.minY - PROXIMITY_RADIUS;
        int expandedMaxY = bounds.maxY + PROXIMITY_RADIUS;
        
        for (int gridY = expandedMinY; gridY <= expandedMaxY; ++gridY) {
            for (int gridX = expandedMinX; gridX <= expandedMaxX; ++gridX) {
                int idx = toIndex(gridX, gridY);
                if (idx < 0) continue;
                
                // Calculate distance to node boundary
                int dx = 0;
                if (gridX < bounds.minX) dx = bounds.minX - gridX;
                else if (gridX > bounds.maxX) dx = gridX - bounds.maxX;
                
                int dy = 0;
                if (gridY < bounds.minY) dy = bounds.minY - gridY;
                else if (gridY > bounds.maxY) dy = gridY - bounds.maxY;
                
                int distance = std::max(dx, dy);
                
                // Skip if outside proximity radius or inside node
                if (distance > PROXIMITY_RADIUS || distance == 0) continue;
                
                // Insert into proximity map (keep sorted by distance)
                auto& entries = proximityMap_[idx];
                for (int i = 0; i < MAX_PROXIMITY_ENTRIES; ++i) {
                    if (entries[i].nodeId == INVALID_NODE) {
                        // Empty slot
                        entries[i].nodeId = bounds.nodeId;
                        entries[i].distance = distance;
                        break;
                    } else if (distance < entries[i].distance) {
                        // Insert here, shift others down
                        for (int j = MAX_PROXIMITY_ENTRIES - 1; j > i; --j) {
                            entries[j] = entries[j-1];
                        }
                        entries[i].nodeId = bounds.nodeId;
                        entries[i].distance = distance;
                        break;
                    }
                }
            }
        }
    }
}

// === Per-Edge Path Tracking ===

void ObstacleMap::markEdgePath(EdgeId edgeId, const std::vector<GridPoint>& path) {
    // Clear existing path for this edge if any
    clearEdgePath(edgeId);

    if (path.empty()) {
        return;
    }

    // Store the path
    edgePaths_[edgeId] = path;

    // Update cost overlay and cell-to-edge mapping
    for (const auto& gp : path) {
        int idx = toIndex(gp.x, gp.y);
        if (idx >= 0) {
            // Add edge cost to overlay
            edgeCostOverlay_[idx] += COST_EDGE_PATH;

            // Track which edges use this cell
            cellToEdges_[idx].push_back(edgeId);
        }
    }

}

void ObstacleMap::clearEdgePath(EdgeId edgeId) {
    auto it = edgePaths_.find(edgeId);
    if (it == edgePaths_.end()) {
        return;  // No path registered for this edge
    }

    // Remove cost overlay for this edge's path
    for (const auto& gp : it->second) {
        int idx = toIndex(gp.x, gp.y);
        if (idx >= 0) {
            // Subtract edge cost from overlay
            auto overlayIt = edgeCostOverlay_.find(idx);
            if (overlayIt != edgeCostOverlay_.end()) {
                overlayIt->second -= COST_EDGE_PATH;
                if (overlayIt->second <= 0) {
                    edgeCostOverlay_.erase(overlayIt);
                }
            }

            // Remove edge from cell-to-edge mapping
            auto cellIt = cellToEdges_.find(idx);
            if (cellIt != cellToEdges_.end()) {
                auto& edges = cellIt->second;
                edges.erase(std::remove(edges.begin(), edges.end(), edgeId), edges.end());
                if (edges.empty()) {
                    cellToEdges_.erase(cellIt);
                }
            }
        }
    }

    // Remove the path
    edgePaths_.erase(it);

}

void ObstacleMap::clearAllEdgePaths() {
    edgePaths_.clear();
    edgeCostOverlay_.clear();
    cellToEdges_.clear();

}


}  // namespace arborvia
