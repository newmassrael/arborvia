#include "EdgeRouting.h"
#include "SnapIndexManager.h"
#include "arborvia/layout/LayoutTypes.h"
#include "arborvia/layout/LayoutUtils.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <set>
#include <unordered_map>

namespace arborvia {
namespace algorithms {

// =============================================================================
// Hash Function for std::pair (for unordered_map)
// =============================================================================

namespace {
    struct PairHash {
        template <typename T1, typename T2>
        std::size_t operator()(const std::pair<T1, T2>& p) const {
            auto h1 = std::hash<T1>{}(p.first);
            auto h2 = std::hash<T2>{}(p.second);
            return h1 ^ (h2 << 1);
        }
    };
}

// =============================================================================
// Static Helper Function Implementations
// =============================================================================

Point EdgeRouting::calculateSnapPosition(const NodeLayout& node, NodeEdge edge, float position) {
    return LayoutUtils::calculateSnapPointFromPosition(node, edge, position);
}

float EdgeRouting::calculateRelativePosition(int snapIdx, int count, float rangeStart, float rangeEnd) {
    // Delegate to SnapIndexManager for centralized logic
    SnapRange range{rangeStart, rangeEnd};
    return SnapIndexManager::calculatePosition(snapIdx, count, range);
}

int EdgeRouting::unifiedToLocalIndex(int unifiedIdx, int offset, int count) {
    // Delegate to SnapIndexManager for centralized logic
    return SnapIndexManager::unifiedToLocal(unifiedIdx, offset, count);
}

void EdgeRouting::recalculateBendPoints(EdgeLayout& layout) {
    layout.bendPoints.clear();

    // Determine layout direction from source/target edges
    bool isHorizontal = (layout.sourceEdge == NodeEdge::Right ||
                         layout.sourceEdge == NodeEdge::Left);

    // Check if this edge has channel-based routing (-1 means not set)
    if (layout.channelY >= 0.0f) {
        if (std::abs(layout.sourcePoint.x - layout.targetPoint.x) > 1.0f ||
            std::abs(layout.sourcePoint.y - layout.targetPoint.y) > 1.0f) {

            if (isHorizontal) {
                // Horizontal layout: channelY stores X coordinate
                float channelX = layout.channelY;
                layout.bendPoints.push_back({{channelX, layout.sourcePoint.y}});
                layout.bendPoints.push_back({{channelX, layout.targetPoint.y}});
            } else {
                // Vertical layout: channelY stores Y coordinate
                layout.bendPoints.push_back({{layout.sourcePoint.x, layout.channelY}});
                layout.bendPoints.push_back({{layout.targetPoint.x, layout.channelY}});
            }
        }
    } else {
        // Simple orthogonal bend (fallback for non-channel edges)
        if (std::abs(layout.sourcePoint.x - layout.targetPoint.x) > 1.0f ||
            std::abs(layout.sourcePoint.y - layout.targetPoint.y) > 1.0f) {

            if (isHorizontal) {
                // Horizontal: use X midpoint
                float midX = (layout.sourcePoint.x + layout.targetPoint.x) / 2.0f;
                layout.bendPoints.push_back({{midX, layout.sourcePoint.y}});
                layout.bendPoints.push_back({{midX, layout.targetPoint.y}});
            } else {
                // Vertical: use Y midpoint
                float midY = (layout.sourcePoint.y + layout.targetPoint.y) / 2.0f;
                layout.bendPoints.push_back({{layout.sourcePoint.x, midY}});
                layout.bendPoints.push_back({{layout.targetPoint.x, midY}});
            }
        }
    }
}

std::pair<int, int> EdgeRouting::countConnectionsOnNodeEdge(
    const std::unordered_map<EdgeId, EdgeLayout>& edgeLayouts,
    NodeId nodeId,
    NodeEdge nodeEdge) {
    
    // Delegate to SnapIndexManager for centralized logic
    auto connections = SnapIndexManager::getConnections(edgeLayouts, nodeId, nodeEdge);
    return {connections.incomingCount(), connections.outgoingCount()};
}

// =============================================================================
// Edge Routing Core Methods
// =============================================================================

EdgeRouting::Result EdgeRouting::route(
    const Graph& graph,
    const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
    const std::unordered_set<EdgeId>& reversedEdges,
    const LayoutOptions& options) {

    Result result;

    // Allocate channels for all edges
    auto channelAssignments = allocateChannels(graph, nodeLayouts, reversedEdges, options);
    result.channelAssignments = channelAssignments;

    // Track self-loop counts per node
    std::map<NodeId, int> selfLoopIndices;

    for (EdgeId edgeId : graph.edges()) {
        const EdgeData edge = graph.getEdge(edgeId);

        auto fromIt = nodeLayouts.find(edge.from);
        auto toIt = nodeLayouts.find(edge.to);

        if (fromIt == nodeLayouts.end() || toIt == nodeLayouts.end()) {
            continue;
        }

        bool isReversed = reversedEdges.count(edgeId) > 0;

        EdgeLayout layout;

        // Self-loop handling
        if (edge.from == edge.to) {
            int loopIndex = selfLoopIndices[edge.from]++;
            layout = routeSelfLoop(edge, fromIt->second, loopIndex, options);
        } else {
            // Regular edge with channel assignment
            auto channelIt = channelAssignments.find(edgeId);
            if (channelIt != channelAssignments.end()) {
                layout = routeChannelOrthogonal(edge, fromIt->second, toIt->second,
                                               isReversed, channelIt->second, options);
            } else {
                // Fallback: create simple orthogonal routing without channel
                layout = routeChannelOrthogonal(edge, fromIt->second, toIt->second,
                                               isReversed, ChannelAssignment{}, options);
            }
        }

        result.edgeLayouts[edgeId] = layout;
    }

    return result;
}

Point EdgeRouting::computeSnapPoint(
    const NodeLayout& node,
    Direction direction,
    bool isSource) {
    
    Point center = node.center();
    
    switch (direction) {
        case Direction::TopToBottom:
            return isSource 
                ? Point{center.x, node.position.y + node.size.height}
                : Point{center.x, node.position.y};
        case Direction::BottomToTop:
            return isSource 
                ? Point{center.x, node.position.y}
                : Point{center.x, node.position.y + node.size.height};
        case Direction::LeftToRight:
            return isSource 
                ? Point{node.position.x + node.size.width, center.y}
                : Point{node.position.x, center.y};
        case Direction::RightToLeft:
            return isSource 
                ? Point{node.position.x, center.y}
                : Point{node.position.x + node.size.width, center.y};
    }
    
    return center;
}

// =============================================================================
// Snap Point Distribution Methods
// =============================================================================

void EdgeRouting::distributeAutoSnapPoints(
    Result& result,
    const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
    SnapDistribution distribution) {
    
    if (distribution == SnapDistribution::Unified) {
        // Unified mode: all connections on same edge distributed together
        // Key: (nodeId, nodeEdge) -> list of (edgeId, isSource)
        std::map<std::pair<NodeId, NodeEdge>, std::vector<std::pair<EdgeId, bool>>> allConnections;
        
        for (auto& [edgeId, layout] : result.edgeLayouts) {
            allConnections[{layout.from, layout.sourceEdge}].push_back({edgeId, true});
            allConnections[{layout.to, layout.targetEdge}].push_back({edgeId, false});
        }
        
        for (auto& [key, connections] : allConnections) {
            auto [nodeId, nodeEdge] = key;
            
            auto nodeIt = nodeLayouts.find(nodeId);
            if (nodeIt == nodeLayouts.end()) continue;
            
            const NodeLayout& node = nodeIt->second;
            int count = static_cast<int>(connections.size());
            
            for (int i = 0; i < count; ++i) {
                auto [edgeId, isSource] = connections[i];
                EdgeLayout& layout = result.edgeLayouts[edgeId];
                
                float position = calculateRelativePosition(i, count, 0.0f, 1.0f);
                Point snapPoint = calculateSnapPosition(node, nodeEdge, position);
                
                if (isSource) {
                    layout.sourcePoint = snapPoint;
                    layout.sourceSnapIndex = i;
                } else {
                    layout.targetPoint = snapPoint;
                    layout.targetSnapIndex = i;
                }
            }
        }
    } else {
        // Separated mode: incoming on first half, outgoing on second half
        // Key: (nodeId, nodeEdge) -> (incoming edges, outgoing edges)
        std::map<std::pair<NodeId, NodeEdge>, std::pair<std::vector<EdgeId>, std::vector<EdgeId>>> separatedConnections;
        
        for (auto& [edgeId, layout] : result.edgeLayouts) {
            // Outgoing from source node
            separatedConnections[{layout.from, layout.sourceEdge}].second.push_back(edgeId);
            // Incoming to target node
            separatedConnections[{layout.to, layout.targetEdge}].first.push_back(edgeId);
        }
        
        for (auto& [key, connections] : separatedConnections) {
            auto [nodeId, nodeEdge] = key;
            
            auto nodeIt = nodeLayouts.find(nodeId);
            if (nodeIt == nodeLayouts.end()) continue;
            
            const NodeLayout& node = nodeIt->second;
            auto& [incoming, outgoing] = connections;
            
            int inCount = static_cast<int>(incoming.size());
            int outCount = static_cast<int>(outgoing.size());
            
            // Determine ranges based on what exists
            float inStart = 0.0f, inEnd = 0.5f;
            float outStart = 0.5f, outEnd = 1.0f;
            
            if (inCount > 0 && outCount == 0) {
                inStart = 0.0f; inEnd = 1.0f;
            } else if (outCount > 0 && inCount == 0) {
                outStart = 0.0f; outEnd = 1.0f;
            }
            
            // Unified snap index counter for this node edge
            int unifiedIndex = 0;
            
            // Incoming edges
            for (int i = 0; i < inCount; ++i) {
                EdgeId edgeId = incoming[i];
                EdgeLayout& layout = result.edgeLayouts[edgeId];
                
                float position = calculateRelativePosition(i, inCount, inStart, inEnd);
                layout.targetPoint = calculateSnapPosition(node, nodeEdge, position);
                layout.targetSnapIndex = unifiedIndex++;
            }
            
            // Outgoing edges (continue from where incoming left off)
            for (int i = 0; i < outCount; ++i) {
                EdgeId edgeId = outgoing[i];
                EdgeLayout& layout = result.edgeLayouts[edgeId];
                
                float position = calculateRelativePosition(i, outCount, outStart, outEnd);
                layout.sourcePoint = calculateSnapPosition(node, nodeEdge, position);
                layout.sourceSnapIndex = unifiedIndex++;
            }
        }
    }
    
    // Recalculate bend points and label positions after snap point redistribution
    for (auto& [edgeId, layout] : result.edgeLayouts) {
        recalculateBendPoints(layout);
        layout.labelPosition = LayoutUtils::calculateEdgeLabelPosition(layout);
    }
}

void EdgeRouting::updateEdgePositions(
    std::unordered_map<EdgeId, EdgeLayout>& edgeLayouts,
    const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
    const std::vector<EdgeId>& affectedEdges,
    SnapDistribution distribution,
    const std::unordered_set<NodeId>& movedNodes) {
    
    // Helper to check if a node should be updated
    auto shouldUpdateNode = [&movedNodes](NodeId nodeId) -> bool {
        return movedNodes.empty() || movedNodes.count(nodeId) > 0;
    };
    
    if (distribution == SnapDistribution::Unified) {
        // Unified mode: all connections on same edge distributed together
        std::map<std::pair<NodeId, NodeEdge>, std::vector<std::pair<EdgeId, bool>>> affectedConnections;
        
        for (EdgeId edgeId : affectedEdges) {
            auto it = edgeLayouts.find(edgeId);
            if (it == edgeLayouts.end()) continue;
            const EdgeLayout& layout = it->second;
            affectedConnections[{layout.from, layout.sourceEdge}].push_back({edgeId, true});
            affectedConnections[{layout.to, layout.targetEdge}].push_back({edgeId, false});
        }
        
        for (auto& [key, connections] : affectedConnections) {
            auto [nodeId, nodeEdge] = key;
            
            // Skip nodes that haven't moved
            if (!shouldUpdateNode(nodeId)) continue;
            
            auto nodeIt = nodeLayouts.find(nodeId);
            if (nodeIt == nodeLayouts.end()) continue;
            
            const NodeLayout& node = nodeIt->second;
            
            // Get TOTAL count from all edgeLayouts, not just affected edges
            auto [totalIn, totalOut] = countConnectionsOnNodeEdge(edgeLayouts, nodeId, nodeEdge);
            int totalCount = totalIn + totalOut;
            
            for (const auto& [edgeId, isSource] : connections) {
                EdgeLayout& layout = edgeLayouts[edgeId];
                
                // Use existing snap index
                int snapIdx = isSource ? layout.sourceSnapIndex : layout.targetSnapIndex;
                if (snapIdx < 0 || snapIdx >= totalCount) {
                    // Fallback: find index based on current position in connections
                    snapIdx = 0;
                    for (size_t i = 0; i < connections.size(); ++i) {
                        if (connections[i].first == edgeId) {
                            snapIdx = static_cast<int>(i);
                            break;
                        }
                    }
                }
                
                float position = calculateRelativePosition(snapIdx, totalCount, 0.0f, 1.0f);
                Point snapPoint = calculateSnapPosition(node, nodeEdge, position);
                
                if (isSource) {
                    layout.sourcePoint = snapPoint;
                } else {
                    layout.targetPoint = snapPoint;
                }
            }
        }
    } else {
        // Separated mode: incoming on first half, outgoing on second half
        std::map<std::pair<NodeId, NodeEdge>, std::pair<std::vector<EdgeId>, std::vector<EdgeId>>> affectedConnections;
        
        for (EdgeId edgeId : affectedEdges) {
            auto it = edgeLayouts.find(edgeId);
            if (it == edgeLayouts.end()) continue;
            const EdgeLayout& layout = it->second;
            // Outgoing from source node
            affectedConnections[{layout.from, layout.sourceEdge}].second.push_back(edgeId);
            // Incoming to target node
            affectedConnections[{layout.to, layout.targetEdge}].first.push_back(edgeId);
        }
        
        for (auto& [key, connections] : affectedConnections) {
            auto [nodeId, nodeEdge] = key;
            
            // Skip nodes that haven't moved
            if (!shouldUpdateNode(nodeId)) continue;
            
            auto nodeIt = nodeLayouts.find(nodeId);
            if (nodeIt == nodeLayouts.end()) continue;
            
            const NodeLayout& node = nodeIt->second;
            auto& [incoming, outgoing] = connections;
            
            // Get TOTAL counts from all edgeLayouts for this node edge
            auto [totalIn, totalOut] = countConnectionsOnNodeEdge(edgeLayouts, nodeId, nodeEdge);
            
            // Determine ranges based on TOTAL counts
            float inStart = 0.0f, inEnd = 0.5f;
            float outStart = 0.5f, outEnd = 1.0f;
            
            if (totalIn > 0 && totalOut == 0) {
                inStart = 0.0f; inEnd = 1.0f;
            } else if (totalOut > 0 && totalIn == 0) {
                outStart = 0.0f; outEnd = 1.0f;
            }
            
            // Update incoming edges (target point on this node)
            // Note: In Separated mode, snap indices are unified:
            //   incoming get [0, totalIn), outgoing get [totalIn, totalIn+totalOut)
            for (EdgeId edgeId : incoming) {
                EdgeLayout& layout = edgeLayouts[edgeId];
                int localIdx = unifiedToLocalIndex(layout.targetSnapIndex, 0, totalIn);
                float position = calculateRelativePosition(localIdx, totalIn, inStart, inEnd);
                layout.targetPoint = calculateSnapPosition(node, nodeEdge, position);
            }

            // Update outgoing edges (source point on this node)
            for (EdgeId edgeId : outgoing) {
                EdgeLayout& layout = edgeLayouts[edgeId];
                int localIdx = unifiedToLocalIndex(layout.sourceSnapIndex, totalIn, totalOut);
                float position = calculateRelativePosition(localIdx, totalOut, outStart, outEnd);
                layout.sourcePoint = calculateSnapPosition(node, nodeEdge, position);
            }
        }
    }
    
    // Check which edges are bidirectional (have a reverse edge)
    // O(N) algorithm using unordered_map for O(1) lookup
    std::unordered_set<EdgeId> bidirectionalEdges;
    
    // Build edge lookup map using helper function
    auto edgeMap = buildEdgeMapFromLayouts<PairHash>(edgeLayouts);
    
    // Check affected edges for bidirectionality (O(1) lookup per edge)
    for (EdgeId edgeId : affectedEdges) {
        auto it = edgeLayouts.find(edgeId);
        if (it == edgeLayouts.end()) continue;
        
        NodeId from = it->second.from;
        NodeId to = it->second.to;
        
        // Look for reverse edge in map - O(1) average case
        auto reverseIt = edgeMap.find({to, from});
        if (reverseIt != edgeMap.end()) {
            bidirectionalEdges.insert(edgeId);
        }
    }
    
    // Recalculate bend points and label positions for affected edges
    for (EdgeId edgeId : affectedEdges) {
        auto it = edgeLayouts.find(edgeId);
        if (it == edgeLayouts.end()) continue;
        
        // For bidirectional edges with channel routing, preserve the channel position
        // channelY stores absolute coordinates (Y for vertical, X for horizontal layout)
        // and is already set correctly by allocateChannels(), so we keep it unchanged.
        // recalculateBendPoints() will use this absolute position to maintain separation.
        //
        // For non-bidirectional edges, clear channel routing to use dynamic midpoint.
        if (bidirectionalEdges.count(edgeId) == 0 || it->second.channelY < 0.0f) {
            it->second.channelY = -1.0f;
        }
        // else: keep channelY unchanged (already in absolute coordinates)
        
        recalculateBendPoints(it->second);
        it->second.labelPosition = LayoutUtils::calculateEdgeLabelPosition(it->second);
    }
}

// =============================================================================
// Channel-Based Routing Methods
// =============================================================================

std::vector<ChannelRegion> EdgeRouting::computeChannelRegions(
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

std::unordered_map<EdgeId, ChannelAssignment> EdgeRouting::allocateChannels(
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
    auto edgeMap = buildEdgeMapFromGraph<PairHash>(graph);
    
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

            // Compute Y position using region info
            assignment.yPosition = computeChannelY(region, assignment.channel,
                                                   options.channelRouting);

            assignments[edgeId] = assignment;
        }
    }

    return assignments;
}

float EdgeRouting::computeChannelY(
    const ChannelRegion& region,
    int channelIndex,
    const ChannelRoutingOptions& opts) {

    float regionHeight = region.regionEnd - region.regionStart;

    // Edge count in this region
    int count = static_cast<int>(region.edges.size());

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

EdgeLayout EdgeRouting::routeChannelOrthogonal(
    const EdgeData& edge,
    const NodeLayout& fromLayout,
    const NodeLayout& toLayout,
    bool isReversed,
    const ChannelAssignment& channel,
    const LayoutOptions& options) {

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

        // Use channel Y for horizontal segment
        float channelY = channel.yPosition;
        layout.channelY = channelY;  // Store for later recalculation

        // Create orthogonal bend points using channel
        if (std::abs(layout.sourcePoint.x - layout.targetPoint.x) > 1.0f) {
            layout.bendPoints.push_back({{layout.sourcePoint.x, channelY}});
            layout.bendPoints.push_back({{layout.targetPoint.x, channelY}});
        } else {
            // Nodes are vertically aligned - simple straight connection
            // No bend points needed
        }
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

        // For horizontal layout, channel Y becomes channel X
        float channelX = channel.yPosition;  // Reused as X coordinate
        layout.channelY = channelX;  // Store for later recalculation (represents X in horizontal)

        if (std::abs(layout.sourcePoint.y - layout.targetPoint.y) > 1.0f) {
            layout.bendPoints.push_back({{channelX, layout.sourcePoint.y}});
            layout.bendPoints.push_back({{channelX, layout.targetPoint.y}});
        }
    }

    // Mark as reversed if needed (for arrow rendering)
    (void)isReversed;  // Could be used for visual indication

    // Calculate label position
    layout.labelPosition = LayoutUtils::calculateEdgeLabelPosition(layout);

    return layout;
}

EdgeLayout EdgeRouting::routeSelfLoop(
    const EdgeData& edge,
    const NodeLayout& nodeLayout,
    int loopIndex,
    const LayoutOptions& options) {

    EdgeLayout layout;
    layout.id = edge.id;
    layout.from = edge.from;
    layout.to = edge.to;

    const auto& config = options.channelRouting.selfLoop;
    float offset = config.loopOffset + static_cast<float>(loopIndex) * config.stackSpacing;

    // Determine direction based on config or auto-detect
    SelfLoopDirection dir = config.preferredDirection;

    // For auto, default to Right (most common for state diagrams)
    if (dir == SelfLoopDirection::Auto) {
        dir = SelfLoopDirection::Right;
    }

    Point center = nodeLayout.center();
    float spacing = 10.0f;  // Source/target point spacing

    switch (dir) {
        case SelfLoopDirection::Right:
            layout.sourcePoint = {nodeLayout.position.x + nodeLayout.size.width,
                                  center.y + spacing};
            layout.targetPoint = {nodeLayout.position.x + nodeLayout.size.width,
                                  center.y - spacing};
            layout.sourceEdge = NodeEdge::Right;
            layout.targetEdge = NodeEdge::Right;
            layout.bendPoints.push_back({{nodeLayout.position.x + nodeLayout.size.width + offset,
                                          layout.sourcePoint.y}});
            layout.bendPoints.push_back({{nodeLayout.position.x + nodeLayout.size.width + offset,
                                          layout.targetPoint.y}});
            break;

        case SelfLoopDirection::Left:
            layout.sourcePoint = {nodeLayout.position.x, center.y - spacing};
            layout.targetPoint = {nodeLayout.position.x, center.y + spacing};
            layout.sourceEdge = NodeEdge::Left;
            layout.targetEdge = NodeEdge::Left;
            layout.bendPoints.push_back({{nodeLayout.position.x - offset, layout.sourcePoint.y}});
            layout.bendPoints.push_back({{nodeLayout.position.x - offset, layout.targetPoint.y}});
            break;

        case SelfLoopDirection::Top:
            layout.sourcePoint = {center.x - spacing, nodeLayout.position.y};
            layout.targetPoint = {center.x + spacing, nodeLayout.position.y};
            layout.sourceEdge = NodeEdge::Top;
            layout.targetEdge = NodeEdge::Top;
            layout.bendPoints.push_back({{layout.sourcePoint.x, nodeLayout.position.y - offset}});
            layout.bendPoints.push_back({{layout.targetPoint.x, nodeLayout.position.y - offset}});
            break;

        case SelfLoopDirection::Bottom:
            layout.sourcePoint = {center.x + spacing,
                                  nodeLayout.position.y + nodeLayout.size.height};
            layout.targetPoint = {center.x - spacing,
                                  nodeLayout.position.y + nodeLayout.size.height};
            layout.sourceEdge = NodeEdge::Bottom;
            layout.targetEdge = NodeEdge::Bottom;
            layout.bendPoints.push_back({{layout.sourcePoint.x,
                                          nodeLayout.position.y + nodeLayout.size.height + offset}});
            layout.bendPoints.push_back({{layout.targetPoint.x,
                                          nodeLayout.position.y + nodeLayout.size.height + offset}});
            break;

        case SelfLoopDirection::Auto:
            // Already handled above, but keep for completeness
            break;
    }

    // Calculate label position
    layout.labelPosition = LayoutUtils::calculateEdgeLabelPosition(layout);

    return layout;
}

SelfLoopDirection EdgeRouting::analyzeSelfLoopDirection(
    const NodeLayout& node,
    const std::unordered_map<EdgeId, EdgeLayout>& existingLayouts,
    const std::unordered_map<NodeId, NodeLayout>& allNodes) {

    // Cost analysis for each direction
    std::map<SelfLoopDirection, float> costs;
    costs[SelfLoopDirection::Right] = 0.0f;
    costs[SelfLoopDirection::Left] = 0.0f;
    costs[SelfLoopDirection::Top] = 0.0f;
    costs[SelfLoopDirection::Bottom] = 0.0f;

    // Check for edges using each direction
    for (const auto& [edgeId, layout] : existingLayouts) {
        if (layout.from == node.id || layout.to == node.id) {
            // Penalize directions with existing connections
            if (layout.sourceEdge == NodeEdge::Right || layout.targetEdge == NodeEdge::Right) {
                costs[SelfLoopDirection::Right] += 10.0f;
            }
            if (layout.sourceEdge == NodeEdge::Left || layout.targetEdge == NodeEdge::Left) {
                costs[SelfLoopDirection::Left] += 10.0f;
            }
            if (layout.sourceEdge == NodeEdge::Top || layout.targetEdge == NodeEdge::Top) {
                costs[SelfLoopDirection::Top] += 10.0f;
            }
            if (layout.sourceEdge == NodeEdge::Bottom || layout.targetEdge == NodeEdge::Bottom) {
                costs[SelfLoopDirection::Bottom] += 10.0f;
            }
        }
    }

    // Check for nearby nodes in each direction
    Point center = node.center();
    for (const auto& [otherId, otherNode] : allNodes) {
        if (otherId == node.id) continue;

        Point otherCenter = otherNode.center();
        float dx = otherCenter.x - center.x;
        float dy = otherCenter.y - center.y;

        // Proximity penalty based on direction
        float proximity = 100.0f / (std::abs(dx) + std::abs(dy) + 1.0f);

        if (dx > 0 && std::abs(dx) > std::abs(dy)) {
            costs[SelfLoopDirection::Right] += proximity;
        }
        if (dx < 0 && std::abs(dx) > std::abs(dy)) {
            costs[SelfLoopDirection::Left] += proximity;
        }
        if (dy < 0 && std::abs(dy) > std::abs(dx)) {
            costs[SelfLoopDirection::Top] += proximity;
        }
        if (dy > 0 && std::abs(dy) > std::abs(dx)) {
            costs[SelfLoopDirection::Bottom] += proximity;
        }
    }

    // Find minimum cost direction
    SelfLoopDirection bestDir = SelfLoopDirection::Right;
    float minCost = costs[SelfLoopDirection::Right];

    for (const auto& [dir, cost] : costs) {
        if (cost < minCost) {
            minCost = cost;
            bestDir = dir;
        }
    }

    return bestDir;
}

}  // namespace algorithms
}  // namespace arborvia
