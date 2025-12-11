#include "ChannelRouter.h"
#include "SelfLoopRouter.h"
#include "arborvia/common/Logger.h"
#include "../../pathfinding/ObstacleMap.h"
#include "EdgeValidator.h"
#include "arborvia/layout/api/IPathFinder.h"
#include "arborvia/layout/util/LayoutUtils.h"
#include "arborvia/core/GeometryUtils.h"

// Include full definitions of structs from EdgeRouting.h
#include "EdgeRouting.h"

#include <algorithm>
#include <cmath>

namespace arborvia {

ChannelRouter::ChannelRouter(IPathFinder* pathFinder)
    : pathFinder_(pathFinder) {}

// =============================================================================
// Channel Region Computation
// =============================================================================

std::vector<ChannelRegion> ChannelRouter::computeChannelRegions(
    const Graph& graph,
    const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
    const std::unordered_set<EdgeId>& reversedEdges,
    Direction direction) {

    bool isHorizontal = (direction == Direction::LeftToRight || direction == Direction::RightToLeft);

    // Find min/max position for each layer (Y for vertical, X for horizontal)
    std::map<int, std::pair<float, float>> layerBounds;  // layer -> (min, max)

    for (const auto& [nodeId, layout] : nodeLayouts) {
        int layer = layout.layer;
        float nodeStart, nodeEnd;

        if (isHorizontal) {
            nodeStart = layout.position.x;
            nodeEnd = layout.position.x + layout.size.width;
        } else {
            nodeStart = layout.position.y;
            nodeEnd = layout.position.y + layout.size.height;
        }

        auto it = layerBounds.find(layer);
        if (it == layerBounds.end()) {
            layerBounds[layer] = {nodeStart, nodeEnd};
        } else {
            it->second.first = std::min(it->second.first, nodeStart);
            it->second.second = std::max(it->second.second, nodeEnd);
        }
    }

    if (layerBounds.empty()) {
        return {};
    }

    // Create channel regions between adjacent layers
    std::vector<ChannelRegion> regions;
    std::vector<int> sortedLayers;
    for (const auto& [layer, bounds] : layerBounds) {
        sortedLayers.push_back(layer);
    }
    std::sort(sortedLayers.begin(), sortedLayers.end());

    for (size_t i = 0; i + 1 < sortedLayers.size(); ++i) {
        int fromLayer = sortedLayers[i];
        int toLayer = sortedLayers[i + 1];

        ChannelRegion region;
        region.fromLayer = fromLayer;
        region.toLayer = toLayer;
        region.regionStart = layerBounds[fromLayer].second;  // Bottom of upper layer
        region.regionEnd = layerBounds[toLayer].first;       // Top of lower layer

        regions.push_back(region);
    }

    // Assign edges to regions based on their layer span
    for (EdgeId edgeId : graph.edges()) {
        const EdgeData edge = graph.getEdge(edgeId);

        auto fromIt = nodeLayouts.find(edge.from);
        auto toIt = nodeLayouts.find(edge.to);
        if (fromIt == nodeLayouts.end() || toIt == nodeLayouts.end()) {
            continue;
        }

        int fromLayer = fromIt->second.layer;
        int toLayer = toIt->second.layer;

        // Skip self-loops (handled separately)
        if (edge.from == edge.to) {
            continue;
        }

        // Handle reversed edges (swap layers for routing purposes)
        if (reversedEdges.count(edgeId) > 0) {
            std::swap(fromLayer, toLayer);
        }

        // Find regions this edge passes through
        int minLayer = std::min(fromLayer, toLayer);
        int maxLayer = std::max(fromLayer, toLayer);

        for (auto& region : regions) {
            if (region.fromLayer >= minLayer && region.toLayer <= maxLayer) {
                region.edges.push_back(edgeId);
            }
        }
    }

    return regions;
}

// =============================================================================
// Channel Allocation
// =============================================================================

std::unordered_map<EdgeId, ChannelAssignment> ChannelRouter::allocateChannels(
    const Graph& graph,
    const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
    const std::unordered_set<EdgeId>& reversedEdges,
    const LayoutOptions& options) {

    std::unordered_map<EdgeId, ChannelAssignment> assignments;

    // Handle self-loops first
    std::map<NodeId, int> selfLoopCounts;
    for (EdgeId edgeId : graph.edges()) {
        const EdgeData edge = graph.getEdge(edgeId);
        if (edge.from == edge.to) {
            auto nodeIt = nodeLayouts.find(edge.from);
            if (nodeIt != nodeLayouts.end()) {
                ChannelAssignment assignment;
                assignment.channel = selfLoopCounts[edge.from]++;
                assignment.sourceLayer = nodeIt->second.layer;
                assignment.targetLayer = nodeIt->second.layer;
                assignment.isSelfLoop = true;
                assignments[edgeId] = assignment;
            }
        }
    }

    // Compute channel regions
    auto regions = computeChannelRegions(graph, nodeLayouts, reversedEdges, options.direction);

    // Identify bidirectional edge pairs (edges going in opposite directions between same nodes)
    // Track which edges should get offset channels
    // O(N) algorithm using unordered_map for O(1) lookup
    std::unordered_set<EdgeId> offsetEdges;
    std::unordered_set<EdgeId> processedBidirectional;

    // Build edge lookup map using helper function
    auto edgeMap = buildEdgeMap<PairHash>(graph);

    // Find bidirectional pairs by checking for reverse edges (O(1) lookup)
    for (EdgeId edgeId : graph.edges()) {
        const EdgeData edge = graph.getEdge(edgeId);
        if (edge.from == edge.to) continue; // Skip self-loops
        if (processedBidirectional.count(edgeId) > 0) continue; // Already processed

        // Look for reverse edge in map - O(1) average case
        auto reverseIt = edgeMap.find({edge.to, edge.from});
        if (reverseIt != edgeMap.end()) {
            EdgeId otherId = reverseIt->second;
            // Found bidirectional pair - mark the second one for offset
            offsetEdges.insert(otherId);
            processedBidirectional.insert(edgeId);
            processedBidirectional.insert(otherId);
        }
    }

    // Sort edges within each region by source X coordinate to minimize crossings
    for (auto& region : regions) {
        std::sort(region.edges.begin(), region.edges.end(),
            [&](EdgeId a, EdgeId b) {
                const EdgeData edgeA = graph.getEdge(a);
                const EdgeData edgeB = graph.getEdge(b);

                auto fromAIt = nodeLayouts.find(edgeA.from);
                auto fromBIt = nodeLayouts.find(edgeB.from);

                if (fromAIt == nodeLayouts.end() || fromBIt == nodeLayouts.end()) {
                    return a < b;
                }

                return fromAIt->second.position.x < fromBIt->second.position.x;
            });

        // Assign channels sequentially
        int maxChannels = options.channelRouting.maxChannelsPerRegion;
        for (size_t i = 0; i < region.edges.size(); ++i) {
            EdgeId edgeId = region.edges[i];

            // Skip if already has assignment (e.g., from another region)
            if (assignments.find(edgeId) != assignments.end()) {
                continue;
            }

            const EdgeData edge = graph.getEdge(edgeId);
            auto fromIt = nodeLayouts.find(edge.from);
            auto toIt = nodeLayouts.find(edge.to);

            ChannelAssignment assignment;
            assignment.channel = static_cast<int>(i) % maxChannels;

            // If this edge is marked for offset (part of bidirectional pair), add offset
            if (offsetEdges.count(edgeId) > 0) {
                assignment.channel += 1;
                if (assignment.channel >= maxChannels) {
                    assignment.channel = maxChannels - 1;
                }
            }

            assignment.sourceLayer = fromIt->second.layer;
            assignment.targetLayer = toIt->second.layer;
            assignment.isSelfLoop = false;

            // Compute Y position using region info (quantized when gridSize > 0)
            assignment.yPosition = computeChannelY(region, assignment.channel,
                                                   options.channelRouting,
                                                   options.gridConfig.cellSize);

            assignments[edgeId] = assignment;
        }
    }

    return assignments;
}

// =============================================================================
// Channel Position Calculation
// =============================================================================

float ChannelRouter::computeChannelY(
    const ChannelRegion& region,
    int channelIndex,
    const ChannelRoutingOptions& opts,
    float gridSize) {

    // Edge count in this region
    int count = static_cast<int>(region.edges.size());

    // === Quantized-First Calculation (when gridSize > 0) ===
    if (gridSize > 0.0f) {
        // Convert region bounds to grid units
        int gridStart = static_cast<int>(std::round(region.regionStart / gridSize));
        int gridEnd = static_cast<int>(std::round(region.regionEnd / gridSize));
        int gridLength = gridEnd - gridStart;

        // Single edge centered
        if (count == 1 && opts.centerSingleEdge) {
            int gridCenter = gridStart + gridLength / 2;
            return gridCenter * gridSize;
        }

        // Offset in grid units (convert from pixel offset)
        int gridOffset = static_cast<int>(std::round(opts.channelOffset / gridSize));
        int usableLength = gridLength - 2 * gridOffset;
        if (usableLength < 0) {
            usableLength = gridLength;
            gridOffset = 0;
        }

        // Distribute evenly within usable region using integer arithmetic
        // Same formula as distributeSnapPointsQuantized for symmetry preservation
        int divisor = (count > 1) ? (count - 1) : 1;
        int gridPos;
        if (count <= 1) {
            gridPos = gridStart + gridLength / 2;  // Center single channel
        } else {
            // Evenly spaced: gridStart + offset + (usableLength * channelIndex) / (count-1)
            // Use rounding: (usableLength * channelIndex * 2 + divisor) / (2 * divisor)
            gridPos = gridStart + gridOffset +
                      (usableLength * channelIndex * 2 + divisor) / (2 * divisor);
        }

        return gridPos * gridSize;
    }

    // === Legacy Float Calculation (when gridSize == 0) ===
    float regionHeight = region.regionEnd - region.regionStart;

    // Single edge centered
    if (count == 1 && opts.centerSingleEdge) {
        return region.regionStart + regionHeight / 2.0f;
    }

    // Compute usable height (excluding offset margins)
    float usableHeight = regionHeight - 2 * opts.channelOffset;
    if (usableHeight < 0) {
        usableHeight = regionHeight;  // Fall back if region too small
    }

    // Compute spacing between channels
    float spacing = opts.channelSpacing;
    if (count > 1) {
        float maxSpacing = usableHeight / static_cast<float>(count - 1);
        spacing = std::min(spacing, maxSpacing);
    }

    // Compute channel Y position
    float startY = region.regionStart + opts.channelOffset;
    if (usableHeight < regionHeight) {
        startY = region.regionStart + (regionHeight - (count - 1) * spacing) / 2.0f;
    }

    return startY + static_cast<float>(channelIndex) * spacing;
}

// =============================================================================
// Edge Routing
// =============================================================================

EdgeLayout ChannelRouter::routeChannelOrthogonal(
    const EdgeData& edge,
    const NodeLayout& fromLayout,
    const NodeLayout& toLayout,
    bool isReversed,
    const ChannelAssignment& channel,
    const LayoutOptions& options,
    const std::unordered_map<NodeId, NodeLayout>* allNodeLayouts) {

    EdgeLayout layout;
    layout.id = edge.id;
    layout.from = edge.from;
    layout.to = edge.to;

    Point fromCenter = fromLayout.center();
    Point toCenter = toLayout.center();

    bool isVertical = (options.direction == Direction::TopToBottom ||
                       options.direction == Direction::BottomToTop);

    if (isVertical) {
        // Vertical layout: source bottom, target top
        if (fromCenter.y < toCenter.y) {
            layout.sourcePoint = {fromCenter.x, fromLayout.position.y + fromLayout.size.height};
            layout.targetPoint = {toCenter.x, toLayout.position.y};
            layout.sourceEdge = NodeEdge::Bottom;
            layout.targetEdge = NodeEdge::Top;
        } else {
            layout.sourcePoint = {fromCenter.x, fromLayout.position.y};
            layout.targetPoint = {toCenter.x, toLayout.position.y + toLayout.size.height};
            layout.sourceEdge = NodeEdge::Top;
            layout.targetEdge = NodeEdge::Bottom;
        }

        // Store channel Y for recalculation (already grid-aligned from computeChannelY)
        layout.channelY = channel.yPosition;
    } else {
        // Horizontal layout: source right, target left
        if (fromCenter.x < toCenter.x) {
            layout.sourcePoint = {fromLayout.position.x + fromLayout.size.width, fromCenter.y};
            layout.targetPoint = {toLayout.position.x, toCenter.y};
            layout.sourceEdge = NodeEdge::Right;
            layout.targetEdge = NodeEdge::Left;
        } else {
            layout.sourcePoint = {fromLayout.position.x, fromCenter.y};
            layout.targetPoint = {toLayout.position.x + toLayout.size.width, toCenter.y};
            layout.sourceEdge = NodeEdge::Left;
            layout.targetEdge = NodeEdge::Right;
        }

        // Store channel X (stored in channelY field, already grid-aligned from computeChannelY)
        layout.channelY = channel.yPosition;
    }

    // Calculate bend points
    float gridSize = options.gridConfig.cellSize;
    if (allNodeLayouts) {
        calculateBendPoints(layout, *allNodeLayouts, gridSize);
    } else {
        std::unordered_map<NodeId, NodeLayout> emptyMap;
        calculateBendPoints(layout, emptyMap, gridSize);
    }

    // Mark as reversed if needed (for arrow rendering)
    (void)isReversed;  // Could be used for visual indication

    // Final validation: Verify no segments intersect node interiors
    if (allNodeLayouts && !allNodeLayouts->empty()) {
        std::vector<Point> allPoints;
        allPoints.push_back(layout.sourcePoint);
        for (const auto& bend : layout.bendPoints) {
            allPoints.push_back(bend.position);
        }
        allPoints.push_back(layout.targetPoint);

        // Check ALL segments for intersection (skip only source/target nodes appropriately)
        bool hasIntersection = false;
        NodeId intersectingNode = 0;
        for (size_t i = 0; i + 1 < allPoints.size(); ++i) {
            const Point& p1 = allPoints[i];
            const Point& p2 = allPoints[i + 1];

            for (const auto& [nodeId, nodeLayout] : *allNodeLayouts) {
                // Skip source node for first segment, target node for last segment
                if (i == 0 && nodeId == layout.from) continue;
                if (i == allPoints.size() - 2 && nodeId == layout.to) continue;

                if (EdgeValidator::segmentIntersectsNode(p1, p2, nodeLayout)) {
                    hasIntersection = true;
                    intersectingNode = nodeId;
                    break;
                }
            }
            if (hasIntersection) break;
        }

        // If intersection found, create bypass path directly
        if (hasIntersection) {
            auto nodeIt = allNodeLayouts->find(intersectingNode);
            if (nodeIt != allNodeLayouts->end()) {
                createBypassPath(layout, nodeIt->second, *allNodeLayouts, gridSize);
            }
        }
    }

    // Final clearance check
    ensureSourceClearance(layout, gridSize);

    // Calculate label position
    layout.labelPosition = LayoutUtils::calculateEdgeLabelPosition(layout);

    return layout;
}

EdgeLayout ChannelRouter::routeSelfLoop(
    const EdgeData& edge,
    const NodeLayout& nodeLayout,
    int loopIndex,
    const LayoutOptions& options) {
    // Delegate to SelfLoopRouter
    return SelfLoopRouter::route(edge.id, edge.from, nodeLayout, loopIndex, options);
}

// =============================================================================
// Private Helper Methods
// =============================================================================

void ChannelRouter::calculateBendPoints(
    EdgeLayout& layout,
    const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
    float gridSize) {

    float gridSizeToUse = constants::effectiveGridSize(gridSize);

    // Try A* pathfinding if pathfinder is available
    if (pathFinder_ && !nodeLayouts.empty()) {
        ObstacleMap obstacles;
        obstacles.buildFromNodes(nodeLayouts, gridSizeToUse, 0);

        GridPoint startGrid = obstacles.pixelToGrid(layout.sourcePoint);
        GridPoint goalGrid = obstacles.pixelToGrid(layout.targetPoint);

        LOG_DEBUG("[CALLER:ChannelRouter.cpp] A* findPath called");
        PathResult pathResult = pathFinder_->findPath(
            startGrid, goalGrid, obstacles,
            layout.from, layout.to,
            layout.sourceEdge, layout.targetEdge,
            {}, {});

        if (pathResult.found && pathResult.path.size() >= 2) {
            layout.bendPoints.clear();
            for (size_t i = 1; i + 1 < pathResult.path.size(); ++i) {
                Point pixelPoint = obstacles.gridToPixel(pathResult.path[i].x, pathResult.path[i].y);
                layout.bendPoints.push_back({pixelPoint});
            }
            return;
        }
    }

    // Fallback: Create simple orthogonal path using channel
    layout.bendPoints.clear();

    // Snap source and target to grid
    Point src = layout.sourcePoint;
    Point tgt = layout.targetPoint;
    if (gridSize > 0.0f) {
        src.x = std::round(src.x / gridSize) * gridSize;
        src.y = std::round(src.y / gridSize) * gridSize;
        tgt.x = std::round(tgt.x / gridSize) * gridSize;
        tgt.y = std::round(tgt.y / gridSize) * gridSize;
    }

    // Use channelY for routing (vertical layout)
    float channelY = layout.channelY;
    if (channelY == 0.0f) {
        channelY = (src.y + tgt.y) / 2.0f;
        if (gridSize > 0.0f) {
            channelY = std::round(channelY / gridSize) * gridSize;
        }
    }

    // Create bend points: source -> (src.x, channelY) -> (tgt.x, channelY) -> target
    if (std::abs(src.y - channelY) > 1.0f || std::abs(src.x - tgt.x) > 1.0f) {
        layout.bendPoints.push_back({{src.x, channelY}});
    }
    if (std::abs(src.x - tgt.x) > 1.0f) {
        layout.bendPoints.push_back({{tgt.x, channelY}});
    }
}

void ChannelRouter::createBypassPath(
    EdgeLayout& layout,
    const NodeLayout& blockingNode,
    const std::unordered_map<NodeId, NodeLayout>& allNodeLayouts,
    float gridSize) {

    float margin = constants::effectiveGridSize(gridSize);

    // Find leftmost and rightmost edges of all intermediate nodes
    float leftmostEdge = blockingNode.position.x;
    float rightmostEdge = blockingNode.position.x + blockingNode.size.width;

    for (const auto& [nid, nlayout] : allNodeLayouts) {
        if (nid == layout.from || nid == layout.to) continue;
        leftmostEdge = std::min(leftmostEdge, nlayout.position.x);
        rightmostEdge = std::max(rightmostEdge, nlayout.position.x + nlayout.size.width);
    }

    // Choose side based on source and target positions
    float sourceX = layout.sourcePoint.x;
    float targetX = layout.targetPoint.x;
    float avgX = (sourceX + targetX) / 2.0f;
    float centerNodes = (leftmostEdge + rightmostEdge) / 2.0f;

    float bypassX;
    if (avgX < centerNodes) {
        // Prefer left bypass
        bypassX = leftmostEdge - margin;
    } else {
        // Prefer right bypass
        bypassX = rightmostEdge + margin;
    }

    // Snap to grid if enabled
    if (gridSize > 0.0f) {
        bypassX = std::round(bypassX / gridSize) * gridSize;
    }

    // Create bypass path directly: source -> (bypassX, src.y) -> (bypassX, tgt.y) -> target
    layout.bendPoints.clear();

    float exitY, entryY;
    if (layout.sourceEdge == NodeEdge::Bottom) {
        exitY = layout.sourcePoint.y + margin;
    } else if (layout.sourceEdge == NodeEdge::Top) {
        exitY = layout.sourcePoint.y - margin;
    } else {
        exitY = layout.sourcePoint.y;
    }

    if (layout.targetEdge == NodeEdge::Top) {
        entryY = layout.targetPoint.y - margin;
    } else if (layout.targetEdge == NodeEdge::Bottom) {
        entryY = layout.targetPoint.y + margin;
    } else {
        entryY = layout.targetPoint.y;
    }

    // Snap Y coordinates to grid
    if (gridSize > 0.0f) {
        exitY = std::round(exitY / gridSize) * gridSize;
        entryY = std::round(entryY / gridSize) * gridSize;
    }

    // Create bend points for bypass route
    layout.bendPoints.push_back({{layout.sourcePoint.x, exitY}});
    layout.bendPoints.push_back({{bypassX, exitY}});
    layout.bendPoints.push_back({{bypassX, entryY}});
    layout.bendPoints.push_back({{layout.targetPoint.x, entryY}});
}

void ChannelRouter::ensureSourceClearance(EdgeLayout& layout, float gridSize) {
    float minClearance = constants::effectiveGridSize(gridSize);
    if (!layout.bendPoints.empty()) {
        Point& firstBend = layout.bendPoints[0].position;

        switch (layout.sourceEdge) {
            case NodeEdge::Top:
                if (firstBend.y > layout.sourcePoint.y - minClearance) {
                    float newY = layout.sourcePoint.y - minClearance;
                    firstBend.y = newY;
                    if (layout.bendPoints.size() >= 2) {
                        layout.bendPoints[1].position.y = newY;
                    }
                }
                break;
            case NodeEdge::Bottom:
                if (firstBend.y < layout.sourcePoint.y + minClearance) {
                    float newY = layout.sourcePoint.y + minClearance;
                    firstBend.y = newY;
                    if (layout.bendPoints.size() >= 2) {
                        layout.bendPoints[1].position.y = newY;
                    }
                }
                break;
            case NodeEdge::Left:
                if (firstBend.x > layout.sourcePoint.x - minClearance) {
                    float newX = layout.sourcePoint.x - minClearance;
                    firstBend.x = newX;
                    if (layout.bendPoints.size() >= 2) {
                        layout.bendPoints[1].position.x = newX;
                    }
                }
                break;
            case NodeEdge::Right:
                if (firstBend.x < layout.sourcePoint.x + minClearance) {
                    float newX = layout.sourcePoint.x + minClearance;
                    firstBend.x = newX;
                    if (layout.bendPoints.size() >= 2) {
                        layout.bendPoints[1].position.x = newX;
                    }
                }
                break;
        }
    }
}

}  // namespace arborvia
