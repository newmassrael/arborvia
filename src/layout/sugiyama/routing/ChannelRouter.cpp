#include "ChannelRouter.h"
#include "SelfLoopRouter.h"
#include "arborvia/common/Logger.h"
#include "../../pathfinding/ObstacleMap.h"
#include "EdgeValidator.h"
#include "arborvia/layout/api/IPathFinder.h"
#include "arborvia/layout/util/LayoutUtils.h"
#include "arborvia/core/GeometryUtils.h"
#include "../../snap/SnapPointCalculator.h"
#include "../../snap/GridSnapCalculator.h"

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

        // Offset in grid units (already stored as grid units)
        int gridOffset = opts.channelOffsetGrids;
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
    float usableHeight = regionHeight - 2 * opts.channelOffset(gridSize);
    if (usableHeight < 0) {
        usableHeight = regionHeight;  // Fall back if region too small
    }

    // Compute spacing between channels
    float spacing = opts.channelSpacing(gridSize);
    if (count > 1) {
        float maxSpacing = usableHeight / static_cast<float>(count - 1);
        spacing = std::min(spacing, maxSpacing);
    }

    // Compute channel Y position
    float startY = region.regionStart + opts.channelOffset(gridSize);
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
    const std::unordered_map<NodeId, NodeLayout>* allNodeLayouts,
    const std::unordered_map<EdgeId, EdgeLayout>* alreadyRoutedEdges) {

    EdgeLayout layout;
    layout.id = edge.id;
    layout.from = edge.from;
    layout.to = edge.to;

    Point fromCenter = fromLayout.center();
    Point toCenter = toLayout.center();

    bool isVertical = (options.direction == Direction::TopToBottom ||
                       options.direction == Direction::BottomToTop);

    float gridSize = options.gridConfig.cellSize;

    // Point nodes always connect at center (no distribution needed)
    // Normal nodes use center as temporary - SnapDistributor will redistribute later
    bool fromIsPoint = fromLayout.isPointNode();
    bool toIsPoint = toLayout.isPointNode();

    // Determine sourceEdge/targetEdge
    // Point nodes: use dominant axis direction (consistent with LayoutUtils)
    // Normal nodes: use layout direction (isVertical)
    if (fromIsPoint) {
        layout.sourceEdge = LayoutUtils::calculateSourceEdgeForPointNode(fromLayout, toLayout);
    } else if (isVertical) {
        layout.sourceEdge = (fromCenter.y < toCenter.y) ? NodeEdge::Bottom : NodeEdge::Top;
    } else {
        layout.sourceEdge = (fromCenter.x < toCenter.x) ? NodeEdge::Right : NodeEdge::Left;
    }

    if (toIsPoint) {
        layout.targetEdge = LayoutUtils::calculateTargetEdgeForPointNode(fromLayout, toLayout);
    } else if (isVertical) {
        layout.targetEdge = (fromCenter.y < toCenter.y) ? NodeEdge::Top : NodeEdge::Bottom;
    } else {
        layout.targetEdge = (fromCenter.x < toCenter.x) ? NodeEdge::Left : NodeEdge::Right;
    }

    // Store channel position for recalculation
    layout.channelY = channel.yPosition;

    // Set snap points:
    // - Point nodes: snapIndex=0 at center (SSOT)
    // - Normal nodes: temporary position, SnapDistributor will assign proper snapIndex
    if (fromIsPoint) {
        layout.sourceSnapIndex = constants::SNAP_INDEX_POINT_NODE_CENTER;
        layout.sourcePoint = fromCenter;
    } else {
        // Calculate initial snap position - SnapDistributor may reassign later
        Point tempPos = SnapPointCalculator::calculateFromRatio(
            fromLayout, layout.sourceEdge, 0.5f, gridSize);
        int snapIdx = GridSnapCalculator::getCandidateIndexFromPosition(
            fromLayout, layout.sourceEdge, tempPos, gridSize);
        Point snapPos = GridSnapCalculator::getPositionFromCandidateIndex(
            fromLayout, layout.sourceEdge, snapIdx, gridSize);
        layout.sourceSnapIndex = snapIdx;
        layout.sourcePoint = snapPos;
    }

    if (toIsPoint) {
        layout.targetSnapIndex = constants::SNAP_INDEX_POINT_NODE_CENTER;
        layout.targetPoint = toCenter;
    } else {
        // Calculate initial snap position - SnapDistributor may reassign later
        Point tempPos = SnapPointCalculator::calculateFromRatio(
            toLayout, layout.targetEdge, 0.5f, gridSize);
        int snapIdx = GridSnapCalculator::getCandidateIndexFromPosition(
            toLayout, layout.targetEdge, tempPos, gridSize);
        Point snapPos = GridSnapCalculator::getPositionFromCandidateIndex(
            toLayout, layout.targetEdge, snapIdx, gridSize);
        layout.targetSnapIndex = snapIdx;
        layout.targetPoint = snapPos;
    }

    // ROOT CAUSE ANALYSIS: Check if snap points are on grid vertices
    // ARCHITECTURE PROBLEM: ChannelRouter uses center.x directly without grid quantization,
    // while UnifiedRetryChain::calculateSnapPointForRatio does quantize to grid.
    // This inconsistency causes snap points to be off-grid in ChannelRouter path.
    if (gridSize > 0) {
        bool srcOnGridX = (std::fmod(layout.sourcePoint.x, gridSize) == 0.0f);
        bool srcOnGridY = (std::fmod(layout.sourcePoint.y, gridSize) == 0.0f);
        bool tgtOnGridX = (std::fmod(layout.targetPoint.x, gridSize) == 0.0f);
        bool tgtOnGridY = (std::fmod(layout.targetPoint.y, gridSize) == 0.0f);

        if (!srcOnGridX || !srcOnGridY) {
            LOG_DEBUG("[ChannelRouter] SNAP POINT OFF-GRID: edge {} src=({},{}) gridSize={} remainder=({},{})",
                      edge.id, layout.sourcePoint.x, layout.sourcePoint.y, gridSize,
                      std::fmod(layout.sourcePoint.x, gridSize), std::fmod(layout.sourcePoint.y, gridSize));
            LOG_DEBUG("[ChannelRouter]   fromCenter=({},{}) fromLayout.pos=({},{}) fromLayout.size=({},{})",
                      fromCenter.x, fromCenter.y, fromLayout.position.x, fromLayout.position.y,
                      fromLayout.size.width, fromLayout.size.height);
        }
        if (!tgtOnGridX || !tgtOnGridY) {
            LOG_DEBUG("[ChannelRouter] SNAP POINT OFF-GRID: edge {} tgt=({},{}) gridSize={} remainder=({},{})",
                      edge.id, layout.targetPoint.x, layout.targetPoint.y, gridSize,
                      std::fmod(layout.targetPoint.x, gridSize), std::fmod(layout.targetPoint.y, gridSize));
            LOG_DEBUG("[ChannelRouter]   toCenter=({},{}) toLayout.pos=({},{}) toLayout.size=({},{})",
                      toCenter.x, toCenter.y, toLayout.position.x, toLayout.position.y,
                      toLayout.size.width, toLayout.size.height);
        }
    }

    // Calculate bend points
    // gridSize already declared above
    if (allNodeLayouts) {
        calculateBendPoints(layout, *allNodeLayouts, gridSize, alreadyRoutedEdges);
    } else {
        std::unordered_map<NodeId, NodeLayout> emptyMap;
        calculateBendPoints(layout, emptyMap, gridSize, alreadyRoutedEdges);
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
    std::string beforeBends;
    for (const auto& bp : layout.bendPoints) {
        beforeBends += " (" + std::to_string(static_cast<int>(bp.position.x)) + "," 
                    + std::to_string(static_cast<int>(bp.position.y)) + ")";
    }
    ensureSourceClearance(layout, gridSize);
    std::string afterBends;
    for (const auto& bp : layout.bendPoints) {
        afterBends += " (" + std::to_string(static_cast<int>(bp.position.x)) + "," 
                   + std::to_string(static_cast<int>(bp.position.y)) + ")";
    }
    if (beforeBends != afterBends) {
        LOG_DEBUG("[routeChannelOrthogonal] Edge {} ensureSourceClearance MODIFIED bends! before:{} after:{}",
                  layout.id, beforeBends, afterBends);
    }

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
    float gridSize,
    const std::unordered_map<EdgeId, EdgeLayout>* alreadyRoutedEdges) {

    float gridSizeToUse = constants::effectiveGridSize(gridSize);

    // Try A* pathfinding if pathfinder is available
    if (pathFinder_ && !nodeLayouts.empty()) {
        ObstacleMap obstacles;
        obstacles.buildFromNodes(nodeLayouts, gridSizeToUse, 0);

        // Add already-routed edges as obstacles to prevent overlaps
        if (alreadyRoutedEdges && !alreadyRoutedEdges->empty()) {
            obstacles.addEdgeSegments(*alreadyRoutedEdges, layout.id);
        }

        GridPoint startGrid = obstacles.pixelToGrid(layout.sourcePoint);
        GridPoint goalGrid = obstacles.pixelToGrid(layout.targetPoint);

        LOG_DEBUG("[CALLER:ChannelRouter.cpp] A* findPath called");
        PathResult pathResult = pathFinder_->findPath(
            startGrid, goalGrid, obstacles,
            layout.from, layout.to,
            layout.sourceEdge, layout.targetEdge,
            {}, {});

        if (!pathResult.found) {
            LOG_WARN("[calculateBendPoints] Edge {} A* FAILED: start=({},{}) goal=({},{}) - no path found",
                     layout.id, startGrid.x, startGrid.y, goalGrid.x, goalGrid.y);
        } else if (pathResult.path.size() < 2) {
            LOG_WARN("[calculateBendPoints] Edge {} A* FAILED: start=({},{}) goal=({},{}) - path too short (size={})",
                     layout.id, startGrid.x, startGrid.y, goalGrid.x, goalGrid.y, pathResult.path.size());
        }

        if (pathResult.found && pathResult.path.size() >= 2) {
            layout.bendPoints.clear();
            for (size_t i = 1; i + 1 < pathResult.path.size(); ++i) {
                Point pixelPoint = obstacles.gridToPixel(pathResult.path[i].x, pathResult.path[i].y);
                layout.bendPoints.push_back({pixelPoint});
            }
            std::string bendsStr;
            for (const auto& bp : layout.bendPoints) {
                bendsStr += " (" + std::to_string(static_cast<int>(bp.position.x)) + "," 
                         + std::to_string(static_cast<int>(bp.position.y)) + ")";
            }
            LOG_DEBUG("[calculateBendPoints] Edge {} A* SUCCESS: bends={}{}", layout.id, layout.bendPoints.size(), bendsStr);
            return;
        }
    }

    // Fallback: Create simple orthogonal path using channel
    // NOTE: If A* failed, CooperativeRerouter should handle overlap resolution later
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
    LOG_DEBUG("[calculateBendPoints] Edge {} FALLBACK: src=({},{}) tgt=({},{}) channelY={} bends={}",
              layout.id, src.x, src.y, tgt.x, tgt.y, channelY, layout.bendPoints.size());
}

void ChannelRouter::createBypassPath(
    EdgeLayout& layout,
    const NodeLayout& blockingNode,
    const std::unordered_map<NodeId, NodeLayout>& allNodeLayouts,
    float gridSize) {

    float effectiveGrid = constants::effectiveGridSize(gridSize);
    int marginCells = grid::pixelToCells(effectiveGrid, effectiveGrid);  // At least 1 cell

    // Find leftmost and rightmost edges in grid coordinates
    auto blockingGrid = GridPoint::fromPixel(blockingNode.position, effectiveGrid);
    int blockingWidthCells = grid::pixelToCells(blockingNode.size.width, effectiveGrid);
    int leftmostGrid = blockingGrid.x;
    int rightmostGrid = blockingGrid.x + blockingWidthCells;

    for (const auto& [nid, nlayout] : allNodeLayouts) {
        if (nid == layout.from || nid == layout.to) continue;
        auto nodeGrid = GridPoint::fromPixel(nlayout.position, effectiveGrid);
        int nodeWidthCells = grid::pixelToCells(nlayout.size.width, effectiveGrid);
        leftmostGrid = std::min(leftmostGrid, nodeGrid.x);
        rightmostGrid = std::max(rightmostGrid, nodeGrid.x + nodeWidthCells);
    }

    // Choose side based on source and target positions (grid-based)
    auto srcGrid = GridPoint::fromPixel(layout.sourcePoint, effectiveGrid);
    auto tgtGrid = GridPoint::fromPixel(layout.targetPoint, effectiveGrid);
    int avgXGrid = (srcGrid.x + tgtGrid.x) / 2;
    int centerNodesGrid = (leftmostGrid + rightmostGrid) / 2;

    int bypassXGrid;
    if (avgXGrid < centerNodesGrid) {
        // Prefer left bypass
        bypassXGrid = leftmostGrid - marginCells;
    } else {
        // Prefer right bypass
        bypassXGrid = rightmostGrid + marginCells;
    }

    // Convert to pixel once
    float bypassX = bypassXGrid * effectiveGrid;
    LOG_DEBUG("[createBypassPath] Edge {} gridSize={} marginCells={} leftmostGrid={} bypassXGrid={} bypassX={}",
              layout.id, gridSize, marginCells, leftmostGrid, bypassXGrid, bypassX);

    // Create bypass path directly: source -> (bypassX, src.y) -> (bypassX, tgt.y) -> target
    layout.bendPoints.clear();

    // Calculate exit/entry Y in grid coordinates
    int exitYGrid, entryYGrid;
    if (layout.sourceEdge == NodeEdge::Bottom) {
        exitYGrid = srcGrid.y + marginCells;
    } else if (layout.sourceEdge == NodeEdge::Top) {
        exitYGrid = srcGrid.y - marginCells;
    } else {
        exitYGrid = srcGrid.y;
    }

    if (layout.targetEdge == NodeEdge::Top) {
        entryYGrid = tgtGrid.y - marginCells;
    } else if (layout.targetEdge == NodeEdge::Bottom) {
        entryYGrid = tgtGrid.y + marginCells;
    } else {
        entryYGrid = tgtGrid.y;
    }

    // Convert to pixel once
    float exitY = exitYGrid * effectiveGrid;
    float entryY = entryYGrid * effectiveGrid;

    // Create bend points for bypass route
    layout.bendPoints.push_back({{layout.sourcePoint.x, exitY}});
    layout.bendPoints.push_back({{bypassX, exitY}});
    layout.bendPoints.push_back({{bypassX, entryY}});
    layout.bendPoints.push_back({{layout.targetPoint.x, entryY}});
}

void ChannelRouter::ensureSourceClearance(EdgeLayout& layout, float gridSize) {
    float effectiveGrid = constants::effectiveGridSize(gridSize);
    int clearanceCells = grid::pixelToCells(effectiveGrid, effectiveGrid);  // At least 1 cell

    if (!layout.bendPoints.empty()) {
        Point& firstBend = layout.bendPoints[0].position;
        auto srcGrid = GridPoint::fromPixel(layout.sourcePoint, effectiveGrid);
        auto bendGrid = GridPoint::fromPixel(firstBend, effectiveGrid);

        switch (layout.sourceEdge) {
            case NodeEdge::Top:
                if (bendGrid.y > srcGrid.y - clearanceCells) {
                    int newYGrid = srcGrid.y - clearanceCells;
                    float newY = newYGrid * effectiveGrid;
                    firstBend.y = newY;
                    if (layout.bendPoints.size() >= 2) {
                        layout.bendPoints[1].position.y = newY;
                    }
                }
                break;
            case NodeEdge::Bottom:
                if (bendGrid.y < srcGrid.y + clearanceCells) {
                    int newYGrid = srcGrid.y + clearanceCells;
                    float newY = newYGrid * effectiveGrid;
                    firstBend.y = newY;
                    if (layout.bendPoints.size() >= 2) {
                        layout.bendPoints[1].position.y = newY;
                    }
                }
                break;
            case NodeEdge::Left:
                if (bendGrid.x > srcGrid.x - clearanceCells) {
                    int newXGrid = srcGrid.x - clearanceCells;
                    float newX = newXGrid * effectiveGrid;
                    LOG_DEBUG("[ensureSourceClearance] Edge {} LEFT: srcGrid.x={} clearanceCells={} newXGrid={} newX={}",
                              layout.id, srcGrid.x, clearanceCells, newXGrid, newX);
                    firstBend.x = newX;
                    if (layout.bendPoints.size() >= 2) {
                        layout.bendPoints[1].position.x = newX;
                    }
                }
                break;
            case NodeEdge::Right:
                if (bendGrid.x < srcGrid.x + clearanceCells) {
                    int newXGrid = srcGrid.x + clearanceCells;
                    float newX = newXGrid * effectiveGrid;
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
