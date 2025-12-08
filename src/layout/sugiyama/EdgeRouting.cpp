#include "EdgeRouting.h"
#include "PathCalculator.h"
#include "GridSnapCalculator.h"
#include "GridCoordinates.h"
#include "ChannelRouter.h"
#include "SnapIndexManager.h"
#include "ObstacleMap.h"
#include "AStarPathFinder.h"
#include "AStarEdgeOptimizer.h"
#include "GeometricEdgeOptimizer.h"
#include "EdgeValidator.h"
#include "SelfLoopRouter.h"
#include "PathCleanup.h"
#include "arborvia/core/GeometryUtils.h"
#include "arborvia/layout/IEdgeOptimizer.h"
#include "arborvia/layout/LayoutTypes.h"
#include "arborvia/layout/LayoutUtils.h"
#include "arborvia/layout/PathRoutingCoordinator.h"
#include "arborvia/layout/OptimizerRegistry.h"
#include "arborvia/layout/OptimizerConfig.h"
#include "arborvia/layout/EdgePenaltySystem.h"
#include "arborvia/layout/EdgeNudger.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <set>
#include <unordered_map>

// Debug flag for edge routing
#ifndef EDGE_ROUTING_DEBUG
#define EDGE_ROUTING_DEBUG 0
#endif

namespace arborvia {


// =============================================================================
// Routing Constants
// =============================================================================

namespace {
    /// Tolerance for floating point comparisons in path calculations
    constexpr float EPSILON = 0.1f;

    // =========================================================================
    // Grid-Relative Offset Functions
    // =========================================================================
    // These functions return offsets that are guaranteed to be grid-aligned
    // when gridSize > 0, preventing post-hoc snap from creating spikes/duplicates.

    // =========================================================================
    // Grid Coordinate Types
    // =========================================================================
    // Note: GridNodeBounds and AxisConfig have been moved to GridCoordinates.h

    // =========================================================================
    // Segment-Node Intersection (Namespace Version)
    // =========================================================================

    /// Check if an orthogonal segment passes through a node's INTERIOR (not just touching boundary)
    /// Returns true only if the segment enters the interior region, not just touches edges.
    /// Uses a small margin (1px) to avoid false positives from boundary-touching segments.
    /// @param p1 First point of the segment
    /// @param p2 Second point of the segment
    /// @param node The node to check against
    /// @return true if segment passes through node interior
    inline bool segmentPenetratesNodeInterior(const Point& p1, const Point& p2, const NodeLayout& node) {
        constexpr float MARGIN = 1.0f;  // Margin to distinguish interior from boundary
        
        float left = node.position.x;
        float right = node.position.x + node.size.width;
        float top = node.position.y;
        float bottom = node.position.y + node.size.height;

        // Check vertical segment (same X coordinate)
        if (std::abs(p1.x - p2.x) < 0.1f) {
            float x = p1.x;
            float minY = std::min(p1.y, p2.y);
            float maxY = std::max(p1.y, p2.y);
            
            // Segment X must be strictly inside node horizontal bounds (not on boundary)
            if (x > left + MARGIN && x < right - MARGIN) {
                // Segment Y range must overlap with node Y range (with margin)
                if (minY < bottom - MARGIN && maxY > top + MARGIN) {
                    return true;
                }
            }
        }
        // Check horizontal segment (same Y coordinate)
        else if (std::abs(p1.y - p2.y) < 0.1f) {
            float y = p1.y;
            float minX = std::min(p1.x, p2.x);
            float maxX = std::max(p1.x, p2.x);
            
            // Segment Y must be strictly inside node vertical bounds (not on boundary)
            if (y > top + MARGIN && y < bottom - MARGIN) {
                // Segment X range must overlap with node X range (with margin)
                if (minX < right - MARGIN && maxX > left + MARGIN) {
                    return true;
                }
            }
        }
        return false;
    }

    /// Check if shifting a segment would cause it to penetrate any node's interior
    /// @param p1 First point of segment (will be shifted)
    /// @param p2 Second point of segment (will be shifted)
    /// @param isVertical Whether this is a vertical segment (shift in X direction)
    /// @param shift The shift amount to apply
    /// @param nodeLayouts All node layouts to check against
    /// @param excludeFrom Node ID to exclude (source node)
    /// @param excludeTo Node ID to exclude (target node)
    /// @return true if shifted segment would penetrate any node
    inline bool shiftWouldPenetrateNode(
        const Point& p1, const Point& p2,
        bool isVertical, float shift,
        const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
        NodeId excludeFrom, NodeId excludeTo) {
        
        Point shiftedP1 = p1;
        Point shiftedP2 = p2;
        
        if (isVertical) {
            shiftedP1.x += shift;
            shiftedP2.x += shift;
        } else {
            shiftedP1.y += shift;
            shiftedP2.y += shift;
        }
        
        for (const auto& [nodeId, nodeLayout] : nodeLayouts) {
            // Skip source and target nodes
            if (nodeId == excludeFrom || nodeId == excludeTo) continue;
            
            if (segmentPenetratesNodeInterior(shiftedP1, shiftedP2, nodeLayout)) {
                return true;
            }
        }
        return false;
    }

    // =========================================================================
    // Axis Abstraction for Horizontal/Vertical Routing
    // =========================================================================

    // AxisConfig moved to GridCoordinates.h







    struct PairHash {
        template <typename T1, typename T2>
        std::size_t operator()(const std::pair<T1, T2>& p) const {
            auto h1 = std::hash<T1>{}(p.first);
            auto h2 = std::hash<T2>{}(p.second);
            return h1 ^ (h2 << 1);
        }
    };

    // =========================================================================
    // BendPoints Validity Check
    // =========================================================================

    /// Check if an EdgeLayout has fresh (non-stale) bendPoints.
    /// Stale bendPoints occur when sourcePoint was updated (e.g., due to node drag)
    /// but bendPoints still reflect the old position, creating a diagonal path.
    /// @param layout The edge layout to check
    /// @param gridSize Grid size for diagonal detection threshold
    /// @return true if bendPoints are fresh and consistent with sourcePoint
    inline bool hasFreshBendPoints(const EdgeLayout& layout, float gridSize) {
        if (layout.bendPoints.empty()) {
            return true;  // No bendPoints = direct connection, considered fresh
        }

        // Check source side: sourcePoint → firstBend
        const Point& src = layout.sourcePoint;
        const Point& firstBend = layout.bendPoints[0].position;
        float dx_src = std::abs(src.x - firstBend.x);
        float dy_src = std::abs(src.y - firstBend.y);
        bool sourceDiagonal = (dx_src > gridSize && dy_src > gridSize);

        // Check target side: lastBend → targetPoint
        const Point& lastBend = layout.bendPoints.back().position;
        const Point& tgt = layout.targetPoint;
        float dx_tgt = std::abs(lastBend.x - tgt.x);
        float dy_tgt = std::abs(lastBend.y - tgt.y);
        bool targetDiagonal = (dx_tgt > gridSize && dy_tgt > gridSize);

        // If either side has diagonal, bendPoints are stale
        return !(sourceDiagonal || targetDiagonal);
    }

    // =========================================================================
    // Edge Count on NodeEdge
    // =========================================================================

    /// Count how many edges are connected to a specific NodeEdge.
    /// Used for proper snap index allocation when switching NodeEdges.
    /// @param edgeLayouts All edge layouts
    /// @param nodeId The node to check
    /// @param edge Which edge of the node
    /// @param excludeId Edge ID to exclude from counting (the edge being routed)
    /// @return Number of edges connected to this NodeEdge
    inline int countEdgesOnNodeEdge(
        const std::unordered_map<EdgeId, EdgeLayout>& edgeLayouts,
        NodeId nodeId,
        NodeEdge edge,
        EdgeId excludeId) {

        int count = 0;
        for (const auto& [id, layout] : edgeLayouts) {
            if (id == excludeId) continue;

            // Check source side
            if (layout.from == nodeId && layout.sourceEdge == edge) {
                count++;
            }
            // Check target side
            if (layout.to == nodeId && layout.targetEdge == edge) {
                count++;
            }
        }
        return count;
    }

}

// =============================================================================
// Constructor
// =============================================================================

EdgeRouting::EdgeRouting(std::shared_ptr<IPathFinder> pathFinder)
    : pathFinder_(pathFinder ? std::move(pathFinder)
                             : std::make_shared<AStarPathFinder>())
    , channelRouter_(std::make_unique<ChannelRouter>(pathFinder_.get())) {
}

EdgeRouting::EdgeRouting(PathRoutingCoordinator* coordinator)
    : pathFinder_(std::make_shared<AStarPathFinder>())
    , channelRouter_(std::make_unique<ChannelRouter>(pathFinder_.get()))
    , coordinator_(coordinator) {
}

EdgeRouting::~EdgeRouting() = default;

const IPathFinder& EdgeRouting::pathFinder() const {
    return activePathFinder();
}

void EdgeRouting::setRoutingCoordinator(PathRoutingCoordinator* coordinator) {
    coordinator_ = coordinator;
}

PathRoutingCoordinator* EdgeRouting::routingCoordinator() const {
    return coordinator_;
}

void EdgeRouting::setEdgeOptimizer(std::shared_ptr<IEdgeOptimizer> optimizer) {
    edgeOptimizer_ = std::move(optimizer);
}

IEdgeOptimizer* EdgeRouting::edgeOptimizer() const {
    return edgeOptimizer_.get();
}

IPathFinder& EdgeRouting::activePathFinder() const {
    if (coordinator_) {
        return coordinator_->currentPathFinder();
    }
    return *pathFinder_;
}

// =============================================================================
// Static Helper Function Implementations
// =============================================================================

Point EdgeRouting::calculateSnapPosition(const NodeLayout& node, NodeEdge edge, float position) {
    return LayoutUtils::calculateSnapPointFromPosition(node, edge, position);
}

int EdgeRouting::unifiedToLocalIndex(int unifiedIdx, int offset, int count) {
    // Delegate to SnapIndexManager for centralized logic
    return SnapIndexManager::unifiedToLocal(unifiedIdx, offset, count);
}

// =============================================================================
// Main recalculateBendPoints Method (delegates to PathCalculator)
// =============================================================================

void EdgeRouting::recalculateBendPoints(
    EdgeLayout& layout,
    const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
    float gridSize,
    const std::unordered_map<EdgeId, EdgeLayout>* otherEdges) {
    
    // Delegate to PathCalculator which encapsulates bend point calculation logic
    PathCalculator calculator(activePathFinder());
    calculator.recalculateBendPoints(layout, nodeLayouts, gridSize, otherEdges);
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

// =============================================================================
// Segment-Node Intersection Detection and Avoidance
// =============================================================================

bool EdgeRouting::segmentIntersectsNode(
    const Point& p1,
    const Point& p2,
    const NodeLayout& node,
    float margin) {
    // Delegate to EdgeValidator
    return EdgeValidator::segmentIntersectsNode(p1, p2, node, margin);
}

EdgeRouting::Result EdgeRouting::route(
    const Graph& graph,
    const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
    const std::unordered_set<EdgeId>& reversedEdges,
    const LayoutOptions& options,
    bool skipOptimization) {

    Result result;

    // Allocate channels for all edges (delegated to ChannelRouter)
    auto channelAssignments = channelRouter_->allocateChannels(graph, nodeLayouts, reversedEdges, options);
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

        // Self-loop handling (delegated to ChannelRouter)
        if (edge.from == edge.to) {
            int loopIndex = selfLoopIndices[edge.from]++;
            layout = channelRouter_->routeSelfLoop(edge, fromIt->second, loopIndex, options);
        } else {
            // Regular edge with channel assignment (delegated to ChannelRouter)
            auto channelIt = channelAssignments.find(edgeId);
            if (channelIt != channelAssignments.end()) {
                layout = channelRouter_->routeChannelOrthogonal(edge, fromIt->second, toIt->second,
                                               isReversed, channelIt->second, options, &nodeLayouts);
            } else {
                // Fallback: create simple orthogonal routing without channel
                layout = channelRouter_->routeChannelOrthogonal(edge, fromIt->second, toIt->second,
                                               isReversed, ChannelAssignment{}, options, &nodeLayouts);
            }
        }

        result.edgeLayouts[edgeId] = layout;
    }

    // Skip optimization if requested (optimization will be called later after snap distribution)
    if (skipOptimization) {
        return result;
    }

    // Apply edge optimization if optimizer is injected or postDragAlgorithm is enabled
    IEdgeOptimizer* optimizer = edgeOptimizer_.get();
    std::unique_ptr<IEdgeOptimizer> fallbackOptimizer;

    // Use injected optimizer, or create based on postDragAlgorithm
    if (!optimizer && options.optimizationOptions.postDragAlgorithm != PostDragAlgorithm::None) {
        const float gridSize = options.gridConfig.cellSize;

        switch (options.optimizationOptions.postDragAlgorithm) {
            case PostDragAlgorithm::AStar: {
                // Create optimizer with penalty system for unified constraint handling
                OptimizerConfig config = OptimizerConfig::balanced();
                config.gridSize = gridSize;
                config.pathFinder = pathFinder_;  // Share pathfinder with optimizer
                config.penaltySystem = EdgePenaltySystem::createDefault();
                fallbackOptimizer = OptimizerRegistry::instance().create("AStar", config);
                break;
            }
            case PostDragAlgorithm::None:
                break;
        }

        // Note: Penalty system is already configured during creation via OptimizerRegistry
        // The EdgePenaltySystem replaces both EdgeConstraintManager and EdgeScorer
        optimizer = fallbackOptimizer.get();
    }

    if (optimizer && !result.edgeLayouts.empty()) {
        // Collect all edge IDs (skip self-loops which have fixed routing)
        std::vector<EdgeId> edgeIds;
        edgeIds.reserve(result.edgeLayouts.size());
        for (const auto& [edgeId, layout] : result.edgeLayouts) {
            if (layout.from != layout.to) {
                edgeIds.push_back(edgeId);
            }
        }

        // Optimize edge layouts (optimizer now calculates actual paths internally)
        auto optimizedLayouts = optimizer->optimize(edgeIds, result.edgeLayouts, nodeLayouts);

        // Merge optimized layouts (bend points already calculated by optimizer)
        // IMPORTANT: Preserve snap indices assigned by distributeAutoSnapPoints
        // BUT: Only if NodeEdge hasn't changed (indices are specific to NodeEdge)
        for (auto& [edgeId, layout] : optimizedLayouts) {
            auto& existing = result.edgeLayouts[edgeId];
            
            // Save original values
            NodeEdge origSourceEdge = existing.sourceEdge;
            NodeEdge origTargetEdge = existing.targetEdge;
            int preservedSourceSnapIndex = existing.sourceSnapIndex;
            int preservedTargetSnapIndex = existing.targetSnapIndex;
            
            existing = std::move(layout);
            
            // Only restore snap indices if NodeEdge didn't change
            // If NodeEdge changed, the old index is invalid for the new edge
            if (existing.sourceEdge == origSourceEdge) {
                existing.sourceSnapIndex = preservedSourceSnapIndex;
            }
            if (existing.targetEdge == origTargetEdge) {
                existing.targetSnapIndex = preservedTargetSnapIndex;
            }
        }
        
        // Fix any -1 indices that weren't restored (NodeEdge changed during retry)
        // Group edges by (node, nodeEdge) to properly calculate snap indices
        float effectiveGridSize = GridSnapCalculator::getEffectiveGridSize(options.gridConfig.cellSize);
        std::map<std::pair<NodeId, NodeEdge>, std::vector<std::pair<EdgeId, bool>>> connectionsByNodeEdge;
        for (auto& [edgeId, layout] : result.edgeLayouts) {
            connectionsByNodeEdge[{layout.from, layout.sourceEdge}].push_back({edgeId, true});
            connectionsByNodeEdge[{layout.to, layout.targetEdge}].push_back({edgeId, false});
        }
        
        for (auto& [key, connections] : connectionsByNodeEdge) {
            auto [nodeId, nodeEdge] = key;
            auto nodeIt = nodeLayouts.find(nodeId);
            if (nodeIt == nodeLayouts.end()) continue;
            const NodeLayout& node = nodeIt->second;
            
            int totalConnections = static_cast<int>(connections.size());
            for (int i = 0; i < totalConnections; ++i) {
                auto [edgeId, isSource] = connections[i];
                EdgeLayout& layout = result.edgeLayouts[edgeId];
                
                int existingIndex = isSource ? layout.sourceSnapIndex : layout.targetSnapIndex;
                if (existingIndex < 0) {  // -1 means needs fixing
                    int candidateIndex = 0;
                    Point snapPoint = GridSnapCalculator::calculateSnapPosition(node, nodeEdge, i, totalConnections, effectiveGridSize, &candidateIndex);
                    if (isSource) {
                        layout.sourceSnapIndex = candidateIndex;
                        layout.sourcePoint = snapPoint;
                    } else {
                        layout.targetSnapIndex = candidateIndex;
                        layout.targetPoint = snapPoint;
                    }
                }
            }
        }
    }

    return result;
}

// =============================================================================
// Snap Point Distribution Methods
// =============================================================================

void EdgeRouting::distributeAutoSnapPoints(
    Result& result,
    const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
    float gridSize,
    bool sortSnapPoints) {

    float effectiveGridSize = GridSnapCalculator::getEffectiveGridSize(gridSize);

    // Unified mode: all connections on same edge distributed together
    // Key: (nodeId, nodeEdge) -> list of (edgeId, isSource)
    std::map<std::pair<NodeId, NodeEdge>, std::vector<std::pair<EdgeId, bool>>> allConnections;

    for (auto& [edgeId, layout] : result.edgeLayouts) {
        allConnections[{layout.from, layout.sourceEdge}].push_back({edgeId, true});
        allConnections[{layout.to, layout.targetEdge}].push_back({edgeId, false});
    }

    for (auto& [key, connections] : allConnections) {
        auto [nodeId, nodeEdge] = key;

        // Sort snap points by other node position to minimize edge crossings
        if (sortSnapPoints && connections.size() > 1) {
            auto sortedPairs = SnapIndexManager::sortSnapPointsByOtherNode(
                nodeId, nodeEdge, result.edgeLayouts, nodeLayouts);

            // Replace original connections if sorting succeeded
            // sortedPairs contains (EdgeId, isSource) pairs in correct order
            if (sortedPairs.size() == connections.size()) {
                connections = std::move(sortedPairs);
            }
        }

        // Move self-loop endpoints to corner positions
        SelfLoopRouter::applySelfLoopCornerPositioning(connections, result.edgeLayouts, nodeEdge);

        auto nodeIt = nodeLayouts.find(nodeId);
        if (nodeIt == nodeLayouts.end()) continue;

        const NodeLayout& node = nodeIt->second;
        int connectionCount = static_cast<int>(connections.size());

        // Use grid-based calculation for all snap point positions
        for (int i = 0; i < connectionCount; ++i) {
            auto [edgeId, isSource] = connections[i];
            EdgeLayout& layout = result.edgeLayouts[edgeId];

            // Calculate snap position and store the candidate index (fixed grid position)
            // Store the candidate index (not connection index) for later retrieval
            int candidateIndex = 0;
            Point snapPoint = GridSnapCalculator::calculateSnapPosition(node, nodeEdge, i, connectionCount, effectiveGridSize, &candidateIndex);

            if (isSource) {
                layout.sourcePoint = snapPoint;
                layout.sourceSnapIndex = candidateIndex;  // Store candidate index
            } else {
                layout.targetPoint = snapPoint;
                layout.targetSnapIndex = candidateIndex;  // Store candidate index
            }
        }
    }

    // Recalculate bend points (snap points are already on grid from quantized calculation)
    for (auto& [edgeId, layout] : result.edgeLayouts) {
        recalculateBendPoints(layout, nodeLayouts, effectiveGridSize);

        // Grid mode: bend points are already orthogonal from quantized routing.
        // Remove spikes/duplicates for path cleanup.
        std::vector<Point> fullPath;
        fullPath.push_back(layout.sourcePoint);
        for (const auto& bp : layout.bendPoints) {
            fullPath.push_back(bp.position);
        }
        fullPath.push_back(layout.targetPoint);

        PathCleanup::removeSpikesAndDuplicates(fullPath);

        layout.bendPoints.clear();
        for (size_t i = 1; i + 1 < fullPath.size(); ++i) {
            layout.bendPoints.push_back({fullPath[i]});
        }

        // Ensure first bend has proper clearance from source
        if (!layout.bendPoints.empty()) {
            Point& firstBend = layout.bendPoints[0].position;
            float minClearance = std::max(effectiveGridSize, constants::PATHFINDING_GRID_SIZE);

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

        layout.labelPosition = LayoutUtils::calculateEdgeLabelPosition(layout);
    }
}

void EdgeRouting::optimizeRouting(
    Result& result,
    const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
    const LayoutOptions& options) {

    // Create optimizer based on postDragAlgorithm
    IEdgeOptimizer* optimizer = edgeOptimizer_.get();
    std::unique_ptr<IEdgeOptimizer> fallbackOptimizer;

    if (!optimizer && options.optimizationOptions.postDragAlgorithm != PostDragAlgorithm::None) {
        const float gridSize = options.gridConfig.cellSize;

        switch (options.optimizationOptions.postDragAlgorithm) {
            case PostDragAlgorithm::AStar: {
                OptimizerConfig config = OptimizerConfig::balanced();
                config.gridSize = gridSize;
                config.pathFinder = pathFinder_;
                config.penaltySystem = EdgePenaltySystem::createDefault();
                fallbackOptimizer = OptimizerRegistry::instance().create("AStar", config);
                break;
            }
            case PostDragAlgorithm::None:
                break;
        }
        optimizer = fallbackOptimizer.get();
    }

    if (!optimizer || result.edgeLayouts.empty()) {
        return;
    }

    // Collect all edge IDs for optimization
    std::vector<EdgeId> edgeIds;
    edgeIds.reserve(result.edgeLayouts.size());
    for (const auto& [edgeId, layout] : result.edgeLayouts) {
        edgeIds.push_back(edgeId);
    }

    // Run optimizer on edges with their final snap positions
    auto optimizedLayouts = optimizer->optimize(edgeIds, result.edgeLayouts, nodeLayouts);

    // Merge optimized layouts
    // IMPORTANT: Preserve snap indices (optimizer sets them to -1, we keep grid-based values)
    // BUT: Only if NodeEdge hasn't changed (indices are specific to NodeEdge)
    for (auto& [edgeId, layout] : optimizedLayouts) {
        auto& existing = result.edgeLayouts[edgeId];
        
        // Save original values
        NodeEdge origSourceEdge = existing.sourceEdge;
        NodeEdge origTargetEdge = existing.targetEdge;
        int preservedSourceSnapIndex = existing.sourceSnapIndex;
        int preservedTargetSnapIndex = existing.targetSnapIndex;
        
        existing = std::move(layout);
        
        // Only restore snap indices if NodeEdge didn't change
        if (existing.sourceEdge == origSourceEdge) {
            existing.sourceSnapIndex = preservedSourceSnapIndex;
        }
        if (existing.targetEdge == origTargetEdge) {
            existing.targetSnapIndex = preservedTargetSnapIndex;
        }
    }

    // Fix any -1 indices that weren't restored (NodeEdge changed during optimization)
    // Group edges by (node, nodeEdge) to properly calculate snap indices
    float effectiveGridSize = GridSnapCalculator::getEffectiveGridSize(options.gridConfig.cellSize);
    std::map<std::pair<NodeId, NodeEdge>, std::vector<std::pair<EdgeId, bool>>> connectionsByNodeEdge;
    for (auto& [edgeId, layout] : result.edgeLayouts) {
        connectionsByNodeEdge[{layout.from, layout.sourceEdge}].push_back({edgeId, true});
        connectionsByNodeEdge[{layout.to, layout.targetEdge}].push_back({edgeId, false});
    }

    for (auto& [key, connections] : connectionsByNodeEdge) {
        auto [nodeId, nodeEdge] = key;
        auto nodeIt = nodeLayouts.find(nodeId);
        if (nodeIt == nodeLayouts.end()) continue;
        const NodeLayout& node = nodeIt->second;

        int totalConnections = static_cast<int>(connections.size());
        for (int i = 0; i < totalConnections; ++i) {
            auto [edgeId, isSource] = connections[i];
            EdgeLayout& layout = result.edgeLayouts[edgeId];

            int existingIndex = isSource ? layout.sourceSnapIndex : layout.targetSnapIndex;
            if (existingIndex < 0) {  // -1 means needs fixing
                int candidateIndex = 0;
                Point snapPoint = GridSnapCalculator::calculateSnapPosition(node, nodeEdge, i, totalConnections, effectiveGridSize, &candidateIndex);
                if (isSource) {
                    layout.sourceSnapIndex = candidateIndex;
                    layout.sourcePoint = snapPoint;
                } else {
                    layout.targetSnapIndex = candidateIndex;
                    layout.targetPoint = snapPoint;
                }
            }
        }
    }

    // Update label positions after optimization
    for (auto& [edgeId, layout] : result.edgeLayouts) {
        layout.labelPosition = LayoutUtils::calculateEdgeLabelPosition(layout);
    }
}

// =============================================================================
// updateSnapPositions Helper Methods
// =============================================================================

EdgeRouting::AffectedConnectionsMap EdgeRouting::collectAffectedConnections(
    const std::unordered_map<EdgeId, EdgeLayout>& edgeLayouts,
    const std::vector<EdgeId>& affectedEdges) {

    AffectedConnectionsMap affectedConnections;

    for (EdgeId edgeId : affectedEdges) {
        auto it = edgeLayouts.find(edgeId);
        if (it == edgeLayouts.end()) continue;
        const EdgeLayout& layout = it->second;
        affectedConnections[{layout.from, layout.sourceEdge}].push_back({edgeId, true});
        affectedConnections[{layout.to, layout.targetEdge}].push_back({edgeId, false});
    }

    return affectedConnections;
}

void EdgeRouting::calculateSnapPositionsForNodeEdge(
    const std::pair<NodeId, NodeEdge>& key,
    const std::vector<std::pair<EdgeId, bool>>& connections,
    std::unordered_map<EdgeId, EdgeLayout>& edgeLayouts,
    const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
    const std::unordered_set<NodeId>& movedNodes,
    float effectiveGridSize,
    SnapUpdateResult& result) {

    auto [nodeId, nodeEdge] = key;

    // Helper to check if a node should be updated
    auto shouldUpdateNode = [&movedNodes](NodeId nid) -> bool {
        return movedNodes.empty() || movedNodes.count(nid) > 0;
    };

    // Get candidate count for this node-edge to validate indices
    auto nodeIt = nodeLayouts.find(nodeId);
    if (nodeIt == nodeLayouts.end()) return;
    const NodeLayout& node = nodeIt->second;
    int candidateCount = GridSnapCalculator::getCandidateCount(node, nodeEdge, effectiveGridSize);

    bool needsRedistribution = false;
    std::set<int> usedIndices;
    for (const auto& [edgeId, isSource] : connections) {
        auto it = edgeLayouts.find(edgeId);
        if (it != edgeLayouts.end()) {
            int snapIdx = isSource ? it->second.sourceSnapIndex : it->second.targetSnapIndex;
            // Trigger redistribution for invalid indices
            if (snapIdx < 0 || snapIdx >= candidateCount) {
                needsRedistribution = true;
                break;
            }
            // Also check for duplicates when node has moved
            if (shouldUpdateNode(nodeId)) {
                if (usedIndices.count(snapIdx) > 0) {
                    needsRedistribution = true;
                    break;
                }
                usedIndices.insert(snapIdx);
            }
        }
    }

    // Skip nodes that haven't moved AND don't need redistribution
    if (!shouldUpdateNode(nodeId) && !needsRedistribution) return;

    // Get ALL connections on this node-edge for correct totalCount calculation
    auto allConnections = SnapIndexManager::getConnections(edgeLayouts, nodeId, nodeEdge);
    int totalCount = static_cast<int>(allConnections.incoming.size() + allConnections.outgoing.size());

    // Determine which edges to process
    std::vector<std::pair<EdgeId, bool>> edgesToProcess;
    if (needsRedistribution) {
        for (EdgeId edgeId : allConnections.incoming) {
            edgesToProcess.push_back(std::make_pair(edgeId, false));
        }
        for (EdgeId edgeId : allConnections.outgoing) {
            edgesToProcess.push_back(std::make_pair(edgeId, true));
        }
    } else {
        edgesToProcess = connections;
    }

    // Move self-loop endpoints to corner positions
    SelfLoopRouter::applySelfLoopCornerPositioning(edgesToProcess, edgeLayouts, nodeEdge);

    // Identify which edges need new candidate indices
    std::set<EdgeId> edgesNeedingNewIndex;
    if (needsRedistribution) {
        std::map<int, EdgeId> indexToFirstEdge;
        for (const auto& [edgeId, isSource] : edgesToProcess) {
            const EdgeLayout& el = edgeLayouts[edgeId];
            int idx = isSource ? el.sourceSnapIndex : el.targetSnapIndex;
            if (idx < 0 || idx >= candidateCount) {
                edgesNeedingNewIndex.insert(edgeId);
            } else {
                auto it = indexToFirstEdge.find(idx);
                if (it != indexToFirstEdge.end()) {
                    edgesNeedingNewIndex.insert(it->second);
                    edgesNeedingNewIndex.insert(edgeId);
                } else {
                    indexToFirstEdge[idx] = edgeId;
                }
            }
        }
    }

    // Track used candidate indices to avoid conflicts
    std::set<int> usedCandidateIndices;
    for (const auto& [edgeId, isSource] : edgesToProcess) {
        if (edgesNeedingNewIndex.count(edgeId) > 0) continue;
        const EdgeLayout& el = edgeLayouts[edgeId];
        int idx = isSource ? el.sourceSnapIndex : el.targetSnapIndex;
        if (idx >= 0 && idx < candidateCount) {
            usedCandidateIndices.insert(idx);
        }
    }

    for (size_t connIdx = 0; connIdx < edgesToProcess.size(); ++connIdx) {
        auto [edgeId, isSource] = edgesToProcess[connIdx];
        EdgeLayout& layout = edgeLayouts[edgeId];

        bool nodeHasMoved = shouldUpdateNode(nodeId);
        int candidateIndex;
        Point snapPoint;
        bool thisEdgeNeedsNewIndex = edgesNeedingNewIndex.count(edgeId) > 0;

        if (thisEdgeNeedsNewIndex) {
            int newCandidateIndex = -1;
            std::vector<int> preferredIndices = GridSnapCalculator::selectCandidateIndices(candidateCount, totalCount);

            for (int preferred : preferredIndices) {
                if (usedCandidateIndices.count(preferred) == 0) {
                    newCandidateIndex = preferred;
                    break;
                }
            }

            if (newCandidateIndex < 0) {
                for (int i = 0; i < candidateCount; ++i) {
                    if (usedCandidateIndices.count(i) == 0) {
                        newCandidateIndex = i;
                        break;
                    }
                }
            }

            if (newCandidateIndex < 0 && !preferredIndices.empty()) {
                newCandidateIndex = preferredIndices[static_cast<int>(connIdx) % preferredIndices.size()];
            }

            candidateIndex = std::max(0, newCandidateIndex);
            usedCandidateIndices.insert(candidateIndex);
            snapPoint = GridSnapCalculator::getPositionFromStoredIndex(node, nodeEdge, candidateIndex, effectiveGridSize);
        } else {
            candidateIndex = isSource ? layout.sourceSnapIndex : layout.targetSnapIndex;
            if (candidateIndex < 0 || candidateIndex >= candidateCount) {
                snapPoint = GridSnapCalculator::calculateSnapPosition(node, nodeEdge, static_cast<int>(connIdx), totalCount, effectiveGridSize, &candidateIndex);
            } else {
                snapPoint = GridSnapCalculator::getPositionFromStoredIndex(node, nodeEdge, candidateIndex, effectiveGridSize);
            }
        }

        if (!nodeHasMoved) {
            if (thisEdgeNeedsNewIndex) {
                if (isSource) {
                    layout.sourcePoint = snapPoint;
                    layout.sourceSnapIndex = candidateIndex;
                } else {
                    layout.targetPoint = snapPoint;
                    layout.targetSnapIndex = candidateIndex;
                }
            }
        } else {
            if (isSource) {
                layout.sourcePoint = snapPoint;
                layout.sourceSnapIndex = candidateIndex;
            } else {
                layout.targetPoint = snapPoint;
                layout.targetSnapIndex = candidateIndex;
            }
        }

        if (needsRedistribution) {
            result.redistributedEdges.insert(edgeId);
        }
    }
}

bool EdgeRouting::detectAndFixDiagonals(
    EdgeId edgeId,
    std::unordered_map<EdgeId, EdgeLayout>& edgeLayouts,
    const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
    const std::unordered_set<NodeId>& movedNodes,
    float effectiveGridSize,
    std::unordered_map<EdgeId, EdgeLayout>& otherEdges) {

    auto shouldUpdateNode = [&movedNodes](NodeId nid) -> bool {
        return movedNodes.empty() || movedNodes.count(nid) > 0;
    };

    auto it = edgeLayouts.find(edgeId);
    if (it == edgeLayouts.end()) return true;

    EdgeLayout& layout = it->second;
    bool needsRetry = false;

    if (layout.bendPoints.empty()) {
        float dx = std::abs(layout.sourcePoint.x - layout.targetPoint.x);
        float dy = std::abs(layout.sourcePoint.y - layout.targetPoint.y);
        if (dx > 1.0f && dy > 1.0f) {
            needsRetry = true;
        }
    } else {
        float dx_src = std::abs(layout.sourcePoint.x - layout.bendPoints[0].position.x);
        float dy_src = std::abs(layout.sourcePoint.y - layout.bendPoints[0].position.y);
        const auto& lastBend = layout.bendPoints.back();
        float dx_tgt = std::abs(lastBend.position.x - layout.targetPoint.x);
        float dy_tgt = std::abs(lastBend.position.y - layout.targetPoint.y);

        bool sourceDiagonal = (dx_src > 1.0f && dy_src > 1.0f);
        bool targetDiagonal = (dx_tgt > 1.0f && dy_tgt > 1.0f);

        if (sourceDiagonal || targetDiagonal) {
            needsRetry = true;
        }
    }

    if (!needsRetry) return true;

    // Check if both nodes are stationary
    bool srcNodeMoved = shouldUpdateNode(layout.from);
    bool tgtNodeMoved = shouldUpdateNode(layout.to);

    if (!srcNodeMoved && !tgtNodeMoved) {
        return true;  // Skip retry for stationary nodes
    }

    // Save original state
    Point originalSourcePoint = layout.sourcePoint;
    Point originalTargetPoint = layout.targetPoint;
    int originalSourceSnapIndex = layout.sourceSnapIndex;
    int originalTargetSnapIndex = layout.targetSnapIndex;
    std::vector<BendPoint> originalBendPoints = layout.bendPoints;
    NodeEdge origSrcEdge = layout.sourceEdge;
    NodeEdge origTgtEdge = layout.targetEdge;

    bool swapSucceeded = false;

    // Try source-side swaps
    std::vector<EdgeId> srcSwapCandidates;
    for (const auto& [otherEdgeId, otherLayout] : edgeLayouts) {
        if (otherEdgeId == edgeId) continue;
        if (otherLayout.from == layout.from && otherLayout.sourceEdge == layout.sourceEdge) {
            srcSwapCandidates.push_back(otherEdgeId);
        }
    }

    for (EdgeId swapWithId : srcSwapCandidates) {
        if (swapSucceeded) break;
        auto swapIt = edgeLayouts.find(swapWithId);
        if (swapIt == edgeLayouts.end()) continue;

        Point swapOriginalSourcePoint = swapIt->second.sourcePoint;
        int swapOriginalSnapIndex = swapIt->second.sourceSnapIndex;
        std::vector<BendPoint> swapOriginalBendPoints = swapIt->second.bendPoints;

        std::swap(layout.sourcePoint, swapIt->second.sourcePoint);
        std::swap(layout.sourceSnapIndex, swapIt->second.sourceSnapIndex);

        recalculateBendPoints(layout, nodeLayouts, effectiveGridSize, nullptr);
        recalculateBendPoints(swapIt->second, nodeLayouts, effectiveGridSize, nullptr);

        bool stillDiagonal = false;
        if (layout.bendPoints.empty()) {
            float dx = std::abs(layout.sourcePoint.x - layout.targetPoint.x);
            float dy = std::abs(layout.sourcePoint.y - layout.targetPoint.y);
            stillDiagonal = (dx > 1.0f && dy > 1.0f);
        }

        bool swapBecameDiagonal = false;
        if (swapIt->second.bendPoints.empty()) {
            float dx = std::abs(swapIt->second.sourcePoint.x - swapIt->second.targetPoint.x);
            float dy = std::abs(swapIt->second.sourcePoint.y - swapIt->second.targetPoint.y);
            swapBecameDiagonal = (dx > 1.0f && dy > 1.0f);
        }

        if (!stillDiagonal && !swapBecameDiagonal) {
            swapSucceeded = true;
        } else {
            layout.sourcePoint = originalSourcePoint;
            layout.sourceSnapIndex = originalSourceSnapIndex;
            layout.bendPoints = originalBendPoints;
            swapIt->second.sourcePoint = swapOriginalSourcePoint;
            swapIt->second.sourceSnapIndex = swapOriginalSnapIndex;
            swapIt->second.bendPoints = swapOriginalBendPoints;
        }
    }

    // Try target-side swaps if source swaps failed
    if (!swapSucceeded) {
        std::vector<EdgeId> tgtSwapCandidates;
        for (const auto& [otherEdgeId, otherLayout] : edgeLayouts) {
            if (otherEdgeId == edgeId) continue;
            if (otherLayout.to == layout.to && otherLayout.targetEdge == layout.targetEdge) {
                tgtSwapCandidates.push_back(otherEdgeId);
            }
        }

        for (EdgeId swapWithId : tgtSwapCandidates) {
            if (swapSucceeded) break;
            auto swapIt = edgeLayouts.find(swapWithId);
            if (swapIt == edgeLayouts.end()) continue;

            Point swapOriginalTargetPoint = swapIt->second.targetPoint;
            int swapOriginalSnapIndex = swapIt->second.targetSnapIndex;
            std::vector<BendPoint> swapOriginalBendPoints = swapIt->second.bendPoints;

            std::swap(layout.targetPoint, swapIt->second.targetPoint);
            std::swap(layout.targetSnapIndex, swapIt->second.targetSnapIndex);

            recalculateBendPoints(layout, nodeLayouts, effectiveGridSize, nullptr);
            recalculateBendPoints(swapIt->second, nodeLayouts, effectiveGridSize, nullptr);

            bool stillDiagonal = false;
            if (layout.bendPoints.empty()) {
                float dx = std::abs(layout.sourcePoint.x - layout.targetPoint.x);
                float dy = std::abs(layout.sourcePoint.y - layout.targetPoint.y);
                stillDiagonal = (dx > 1.0f && dy > 1.0f);
            }

            bool swapBecameDiagonal = false;
            if (swapIt->second.bendPoints.empty()) {
                float dx = std::abs(swapIt->second.sourcePoint.x - swapIt->second.targetPoint.x);
                float dy = std::abs(swapIt->second.sourcePoint.y - swapIt->second.targetPoint.y);
                swapBecameDiagonal = (dx > 1.0f && dy > 1.0f);
            }

            if (!stillDiagonal && !swapBecameDiagonal) {
                swapSucceeded = true;
            } else {
                layout.targetPoint = originalTargetPoint;
                layout.targetSnapIndex = originalTargetSnapIndex;
                layout.bendPoints = originalBendPoints;
                swapIt->second.targetPoint = swapOriginalTargetPoint;
                swapIt->second.targetSnapIndex = swapOriginalSnapIndex;
                swapIt->second.bendPoints = swapOriginalBendPoints;
            }
        }
    }

    // Exhaustive NodeEdge search if swaps failed
    if (!swapSucceeded) {
        auto srcNodeIt = nodeLayouts.find(layout.from);
        auto tgtNodeIt = nodeLayouts.find(layout.to);

        if (srcNodeIt != nodeLayouts.end() && tgtNodeIt != nodeLayouts.end()) {
            const auto& srcNode = srcNodeIt->second;
            const auto& tgtNode = tgtNodeIt->second;

            constexpr std::array<NodeEdge, 4> allEdges = {
                NodeEdge::Top, NodeEdge::Bottom, NodeEdge::Left, NodeEdge::Right
            };

            bool exhaustiveSuccess = false;

            for (NodeEdge srcEdge : allEdges) {
                if (exhaustiveSuccess) break;
                for (NodeEdge tgtEdge : allEdges) {
                    if (exhaustiveSuccess) break;

                    int srcCandidateCount = GridSnapCalculator::getCandidateCount(srcNode, srcEdge, effectiveGridSize);
                    int tgtCandidateCount = GridSnapCalculator::getCandidateCount(tgtNode, tgtEdge, effectiveGridSize);

                    for (int srcConnIdx = 0; srcConnIdx < std::max(1, srcCandidateCount) && !exhaustiveSuccess; ++srcConnIdx) {
                        for (int tgtConnIdx = 0; tgtConnIdx < std::max(1, tgtCandidateCount) && !exhaustiveSuccess; ++tgtConnIdx) {
                            int srcCandidateIdx = 0, tgtCandidateIdx = 0;
                            Point newSrc = GridSnapCalculator::calculateSnapPosition(srcNode, srcEdge, srcConnIdx, std::max(1, srcCandidateCount), effectiveGridSize, &srcCandidateIdx);
                            Point newTgt = GridSnapCalculator::calculateSnapPosition(tgtNode, tgtEdge, tgtConnIdx, std::max(1, tgtCandidateCount), effectiveGridSize, &tgtCandidateIdx);

                            if (srcEdge == origSrcEdge && tgtEdge == origTgtEdge &&
                                srcCandidateIdx == originalSourceSnapIndex && tgtCandidateIdx == originalTargetSnapIndex) {
                                continue;
                            }

                            ObstacleMap obstacles;
                            obstacles.buildFromNodes(nodeLayouts, effectiveGridSize, 0);

                            GridPoint startGrid = obstacles.pixelToGrid(newSrc);
                            GridPoint goalGrid = obstacles.pixelToGrid(newTgt);

                            PathResult pathResult = activePathFinder().findPath(
                                startGrid, goalGrid, obstacles,
                                layout.from, layout.to,
                                srcEdge, tgtEdge,
                                {}, {});

                            if (pathResult.found && pathResult.path.size() >= 2) {
                                layout.sourceEdge = srcEdge;
                                layout.targetEdge = tgtEdge;
                                layout.sourcePoint = newSrc;
                                layout.targetPoint = newTgt;
                                layout.sourceSnapIndex = srcCandidateIdx;
                                layout.targetSnapIndex = tgtCandidateIdx;
                                layout.bendPoints.clear();

                                for (size_t i = 1; i + 1 < pathResult.path.size(); ++i) {
                                    const auto& gp = pathResult.path[i];
                                    Point pixelPoint = obstacles.gridToPixel(gp.x, gp.y);
                                    layout.bendPoints.push_back(BendPoint{pixelPoint, false});
                                }
                                exhaustiveSuccess = true;
                            }
                        }
                    }
                }
            }

            if (!exhaustiveSuccess) {
                layout.sourcePoint = originalSourcePoint;
                layout.targetPoint = originalTargetPoint;
                layout.sourceSnapIndex = originalSourceSnapIndex;
                layout.targetSnapIndex = originalTargetSnapIndex;
                layout.sourceEdge = origSrcEdge;
                layout.targetEdge = origTgtEdge;
                layout.bendPoints = originalBendPoints;
                return false;
            }
        }
    }

    return true;
}

bool EdgeRouting::validateAndFixDirectionConstraints(
    EdgeLayout& layout,
    float effectiveGridSize) {

    if (layout.bendPoints.empty()) return false;

    bool sourceVertical = (layout.sourceEdge == NodeEdge::Top || layout.sourceEdge == NodeEdge::Bottom);
    bool targetVertical = (layout.targetEdge == NodeEdge::Top || layout.targetEdge == NodeEdge::Bottom);

    const Point& firstBend = layout.bendPoints.front().position;
    const Point& lastBend = layout.bendPoints.back().position;

    constexpr float DIRECTION_TOLERANCE = 0.5f;
    bool firstSegmentVertical = std::abs(layout.sourcePoint.x - firstBend.x) < DIRECTION_TOLERANCE;
    bool lastSegmentVertical = std::abs(lastBend.x - layout.targetPoint.x) < DIRECTION_TOLERANCE;

    bool orthogonalViolation = (sourceVertical != firstSegmentVertical || targetVertical != lastSegmentVertical);

    // Check source direction
    bool sourceDirectionViolation = false;
    if (!orthogonalViolation && sourceVertical) {
        float dyFirst = firstBend.y - layout.sourcePoint.y;
        bool shouldGoUp = (layout.sourceEdge == NodeEdge::Top);
        bool shouldGoDown = (layout.sourceEdge == NodeEdge::Bottom);
        if ((shouldGoUp && dyFirst > DIRECTION_TOLERANCE) || (shouldGoDown && dyFirst < -DIRECTION_TOLERANCE)) {
            sourceDirectionViolation = true;
        }
    } else if (!orthogonalViolation && !sourceVertical) {
        float dxFirst = firstBend.x - layout.sourcePoint.x;
        bool shouldGoLeft = (layout.sourceEdge == NodeEdge::Left);
        bool shouldGoRight = (layout.sourceEdge == NodeEdge::Right);
        if ((shouldGoLeft && dxFirst > DIRECTION_TOLERANCE) || (shouldGoRight && dxFirst < -DIRECTION_TOLERANCE)) {
            sourceDirectionViolation = true;
        }
    }

    // Check target direction
    bool targetDirectionViolation = false;
    if (!orthogonalViolation && targetVertical) {
        float dyLast = layout.targetPoint.y - lastBend.y;
        bool shouldEnterFromAbove = (layout.targetEdge == NodeEdge::Top);
        bool shouldEnterFromBelow = (layout.targetEdge == NodeEdge::Bottom);
        if ((shouldEnterFromAbove && dyLast < -DIRECTION_TOLERANCE) || (shouldEnterFromBelow && dyLast > DIRECTION_TOLERANCE)) {
            targetDirectionViolation = true;
        }
    } else if (!orthogonalViolation && !targetVertical) {
        float dxLast = layout.targetPoint.x - lastBend.x;
        bool shouldEnterFromLeft = (layout.targetEdge == NodeEdge::Left);
        bool shouldEnterFromRight = (layout.targetEdge == NodeEdge::Right);
        if ((shouldEnterFromLeft && dxLast < -DIRECTION_TOLERANCE) || (shouldEnterFromRight && dxLast > DIRECTION_TOLERANCE)) {
            targetDirectionViolation = true;
        }
    }

    // Fix source direction violation
    if (sourceDirectionViolation && !layout.bendPoints.empty()) {
        bool srcVertical = (layout.sourceEdge == NodeEdge::Top || layout.sourceEdge == NodeEdge::Bottom);
        const Point& fBend = layout.bendPoints.front().position;

        if (srcVertical) {
            bool shouldGoUp = (layout.sourceEdge == NodeEdge::Top);
            float clearanceY = shouldGoUp
                ? std::min(layout.sourcePoint.y - effectiveGridSize * 2, fBend.y - effectiveGridSize)
                : std::max(layout.sourcePoint.y + effectiveGridSize * 2, fBend.y + effectiveGridSize);
            if (effectiveGridSize > 0) {
                clearanceY = std::round(clearanceY / effectiveGridSize) * effectiveGridSize;
            }

            bool sameVerticalLine = std::abs(layout.sourcePoint.x - fBend.x) < 0.5f;
            Point clearancePoint = {layout.sourcePoint.x, clearanceY};

            if (sameVerticalLine) {
                bool sourceToClearanceGoesUp = clearanceY < layout.sourcePoint.y;
                bool clearanceToFirstBendGoesUp = fBend.y < clearanceY;
                bool wouldCreateSpike = (sourceToClearanceGoesUp != clearanceToFirstBendGoesUp);

                if (wouldCreateSpike) {
                    float offsetX = effectiveGridSize > 0 ? effectiveGridSize * 2 : 40.0f;
                    if (layout.targetPoint.x > layout.sourcePoint.x) offsetX = -offsetX;
                    float detourX = layout.sourcePoint.x + offsetX;
                    if (effectiveGridSize > 0) detourX = std::round(detourX / effectiveGridSize) * effectiveGridSize;

                    Point detourPoint1 = {detourX, clearanceY};
                    Point detourPoint2 = {detourX, fBend.y};

                    layout.bendPoints.insert(layout.bendPoints.begin(), {detourPoint2});
                    layout.bendPoints.insert(layout.bendPoints.begin(), {detourPoint1});
                    layout.bendPoints.insert(layout.bendPoints.begin(), {clearancePoint});
                } else {
                    layout.bendPoints.insert(layout.bendPoints.begin(), {clearancePoint});
                }
            } else {
                Point connectionPoint = {fBend.x, clearanceY};
                if (effectiveGridSize > 0) connectionPoint.x = std::round(connectionPoint.x / effectiveGridSize) * effectiveGridSize;
                layout.bendPoints.insert(layout.bendPoints.begin(), {connectionPoint});
                layout.bendPoints.insert(layout.bendPoints.begin(), {clearancePoint});
            }
        } else {
            bool shouldGoLeft = (layout.sourceEdge == NodeEdge::Left);
            float clearanceX = shouldGoLeft
                ? std::min(layout.sourcePoint.x - effectiveGridSize * 2, fBend.x - effectiveGridSize)
                : std::max(layout.sourcePoint.x + effectiveGridSize * 2, fBend.x + effectiveGridSize);
            if (effectiveGridSize > 0) {
                clearanceX = std::round(clearanceX / effectiveGridSize) * effectiveGridSize;
            }

            bool sameHorizontalLine = std::abs(layout.sourcePoint.y - fBend.y) < 0.5f;
            Point clearancePoint = {clearanceX, layout.sourcePoint.y};

            if (sameHorizontalLine) {
                bool sourceToClearanceGoesLeft = clearanceX < layout.sourcePoint.x;
                bool clearanceToFirstBendGoesLeft = fBend.x < clearanceX;
                bool wouldCreateSpike = (sourceToClearanceGoesLeft != clearanceToFirstBendGoesLeft);

                if (wouldCreateSpike) {
                    float offsetY = effectiveGridSize > 0 ? effectiveGridSize * 2 : 40.0f;
                    if (layout.targetPoint.y > layout.sourcePoint.y) offsetY = -offsetY;
                    float detourY = layout.sourcePoint.y + offsetY;
                    if (effectiveGridSize > 0) detourY = std::round(detourY / effectiveGridSize) * effectiveGridSize;

                    Point detourPoint1 = {clearanceX, detourY};
                    Point detourPoint2 = {fBend.x, detourY};

                    layout.bendPoints.insert(layout.bendPoints.begin(), {detourPoint2});
                    layout.bendPoints.insert(layout.bendPoints.begin(), {detourPoint1});
                    layout.bendPoints.insert(layout.bendPoints.begin(), {clearancePoint});
                } else {
                    layout.bendPoints.insert(layout.bendPoints.begin(), {clearancePoint});
                }
            } else {
                Point connectionPoint = {clearanceX, fBend.y};
                if (effectiveGridSize > 0) connectionPoint.y = std::round(connectionPoint.y / effectiveGridSize) * effectiveGridSize;
                layout.bendPoints.insert(layout.bendPoints.begin(), {connectionPoint});
                layout.bendPoints.insert(layout.bendPoints.begin(), {clearancePoint});
            }
        }
    }

    // Fix target direction violation
    if (targetDirectionViolation && !layout.bendPoints.empty()) {
        bool tgtVertical = (layout.targetEdge == NodeEdge::Top || layout.targetEdge == NodeEdge::Bottom);
        const Point& lBend = layout.bendPoints.back().position;

        if (tgtVertical) {
            bool shouldEnterFromAbove = (layout.targetEdge == NodeEdge::Top);
            float clearanceY = shouldEnterFromAbove
                ? std::min(layout.targetPoint.y - effectiveGridSize * 2, lBend.y - effectiveGridSize)
                : std::max(layout.targetPoint.y + effectiveGridSize * 2, lBend.y + effectiveGridSize);
            if (effectiveGridSize > 0) {
                clearanceY = std::round(clearanceY / effectiveGridSize) * effectiveGridSize;
            }

            bool sameVerticalLine = std::abs(lBend.x - layout.targetPoint.x) < 0.5f;

            if (sameVerticalLine) {
                bool lastBendToClearanceGoesUp = clearanceY < lBend.y;
                bool clearanceToTargetGoesUp = layout.targetPoint.y < clearanceY;
                bool wouldCreateSpike = (lastBendToClearanceGoesUp != clearanceToTargetGoesUp);

                if (wouldCreateSpike) {
                    float offsetX = effectiveGridSize > 0 ? effectiveGridSize * 2 : 40.0f;
                    if (layout.sourcePoint.x > layout.targetPoint.x) offsetX = -offsetX;
                    float detourX = lBend.x + offsetX;
                    if (effectiveGridSize > 0) detourX = std::round(detourX / effectiveGridSize) * effectiveGridSize;

                    Point detourPoint1 = {detourX, lBend.y};
                    Point detourPoint2 = {detourX, clearanceY};
                    Point clearancePoint = {layout.targetPoint.x, clearanceY};

                    layout.bendPoints.push_back({detourPoint1});
                    layout.bendPoints.push_back({detourPoint2});
                    layout.bendPoints.push_back({clearancePoint});
                } else {
                    Point clearancePoint = {layout.targetPoint.x, clearanceY};
                    layout.bendPoints.push_back({clearancePoint});
                }
            } else {
                Point connectionPoint = {lBend.x, clearanceY};
                if (effectiveGridSize > 0) connectionPoint.x = std::round(connectionPoint.x / effectiveGridSize) * effectiveGridSize;
                Point clearancePoint = {layout.targetPoint.x, clearanceY};
                layout.bendPoints.push_back({connectionPoint});
                layout.bendPoints.push_back({clearancePoint});
            }
        } else {
            bool shouldEnterFromLeft = (layout.targetEdge == NodeEdge::Left);
            float clearanceX = shouldEnterFromLeft
                ? std::min(layout.targetPoint.x - effectiveGridSize * 2, lBend.x - effectiveGridSize)
                : std::max(layout.targetPoint.x + effectiveGridSize * 2, lBend.x + effectiveGridSize);
            if (effectiveGridSize > 0) {
                clearanceX = std::round(clearanceX / effectiveGridSize) * effectiveGridSize;
            }

            bool sameHorizontalLine = std::abs(lBend.y - layout.targetPoint.y) < 0.5f;

            if (sameHorizontalLine) {
                bool lastBendToClearanceGoesLeft = clearanceX < lBend.x;
                bool clearanceToTargetGoesLeft = layout.targetPoint.x < clearanceX;
                bool wouldCreateSpike = (lastBendToClearanceGoesLeft != clearanceToTargetGoesLeft);

                if (wouldCreateSpike) {
                    float offsetY = effectiveGridSize > 0 ? effectiveGridSize * 2 : 40.0f;
                    if (layout.sourcePoint.y > layout.targetPoint.y) offsetY = -offsetY;
                    float detourY = lBend.y + offsetY;
                    if (effectiveGridSize > 0) detourY = std::round(detourY / effectiveGridSize) * effectiveGridSize;

                    Point detourPoint1 = {lBend.x, detourY};
                    Point detourPoint2 = {clearanceX, detourY};
                    Point clearancePoint = {clearanceX, layout.targetPoint.y};

                    layout.bendPoints.push_back({detourPoint1});
                    layout.bendPoints.push_back({detourPoint2});
                    layout.bendPoints.push_back({clearancePoint});
                } else {
                    Point clearancePoint = {clearanceX, layout.targetPoint.y};
                    layout.bendPoints.push_back({clearancePoint});
                }
            } else {
                Point connectionPoint = {clearanceX, lBend.y};
                if (effectiveGridSize > 0) connectionPoint.y = std::round(connectionPoint.y / effectiveGridSize) * effectiveGridSize;
                Point clearancePoint = {clearanceX, layout.targetPoint.y};
                layout.bendPoints.push_back({connectionPoint});
                layout.bendPoints.push_back({clearancePoint});
            }
        }
    }

    // Clean up after direction fix
    if (sourceDirectionViolation || targetDirectionViolation) {
        std::vector<Point> allPoints;
        allPoints.push_back(layout.sourcePoint);
        for (const auto& bp : layout.bendPoints) {
            allPoints.push_back(bp.position);
        }
        allPoints.push_back(layout.targetPoint);

        PathCleanup::removeSpikesAndDuplicates(allPoints);

        layout.bendPoints.clear();
        for (size_t j = 1; j + 1 < allPoints.size(); ++j) {
            layout.bendPoints.push_back({allPoints[j]});
        }
        return true;  // Direction was fixed
    }
    return false;  // No fix needed
}

// =============================================================================
// updateSnapPositions Main Method
// =============================================================================

EdgeRouting::SnapUpdateResult EdgeRouting::updateSnapPositions(
    std::unordered_map<EdgeId, EdgeLayout>& edgeLayouts,
    const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
    const std::vector<EdgeId>& affectedEdges,
    const std::unordered_set<NodeId>& movedNodes,
    float gridSize,
    bool skipBendPointRecalc) {

    static int callCounter = 0;
    int thisCallId = ++callCounter;

    SnapUpdateResult result;
    float effectiveGridSize = GridSnapCalculator::getEffectiveGridSize(gridSize);

#if EDGE_ROUTING_DEBUG
    std::cout << "\n[updateSnapPositions CALL #" << thisCallId << "] "
              << "affectedEdges=" << affectedEdges.size()
              << " skipBendPointRecalc=" << (skipBendPointRecalc ? "true" : "false")
              << " edges: ";
    for (EdgeId eid : affectedEdges) std::cout << eid << " ";
    std::cout << std::endl;
#endif

    // Helper to check if a node should be updated
    auto shouldUpdateNode = [&movedNodes](NodeId nodeId) -> bool {
        return movedNodes.empty() || movedNodes.count(nodeId) > 0;
    };

    // === Phase 1: Collect affected connections by node-edge ===
    auto affectedConnections = collectAffectedConnections(edgeLayouts, affectedEdges);

    // === Phase 2: Calculate snap positions and track redistributed edges ===
    for (auto& [key, connections] : affectedConnections) {
        calculateSnapPositionsForNodeEdge(key, connections, edgeLayouts, nodeLayouts, movedNodes, effectiveGridSize, result);
    }

    // === Phase 3: Merge affected edges and redistributed edges for processing ===
    // When snap redistribution occurs, edges not in affectedEdges may have
    // their snap positions changed. These edges also need recalculateBendPoints.
    result.processedEdges.insert(affectedEdges.begin(), affectedEdges.end());
    result.processedEdges.insert(result.redistributedEdges.begin(), result.redistributedEdges.end());

#if EDGE_ROUTING_DEBUG
    std::cout << "[CALL #" << thisCallId << " Phase2 DONE] redistributedEdges: ";
    for (EdgeId eid : result.redistributedEdges) std::cout << eid << " ";
    std::cout << std::endl;

    if (!result.redistributedEdges.empty()) {
        std::cout << "[EdgeRouting] updateSnapPositions: redistributedEdges=" << result.redistributedEdges.size()
                  << ", processedEdges=" << result.processedEdges.size() << std::endl;
        if (result.hasIndirectUpdates(affectedEdges)) {
            std::cout << "  WARNING: Edges outside affectedEdges were updated due to redistribution" << std::endl;
        }
    }
#endif

    // Check which edges are bidirectional (have a reverse edge)
    // O(N) algorithm using unordered_map for O(1) lookup
    std::unordered_set<EdgeId> bidirectionalEdges;

    // Build edge lookup map using helper function
    auto edgeMap = buildEdgeMapFromLayouts<PairHash>(edgeLayouts);

    for (EdgeId edgeId : result.processedEdges) {
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

    // === Phase 4: Recalculate bend points for all processed edges ===
    // Skip this phase when bendPoints are already set by optimizer (e.g., during drag)
    if (skipBendPointRecalc) {
#if EDGE_ROUTING_DEBUG
        std::cout << "[CALL #" << thisCallId << " Phase4] SKIPPED for optimizer-processed edges" << std::endl;
#endif
        // Collect all edges that need bendPoints regeneration:
        // 1. affectedEdges - optimizer-processed edges with potentially stale bendPoints
        // 2. redistributedEdges NOT in affectedEdges - snap positions changed by redistribution
        std::unordered_set<EdgeId> affectedSet(affectedEdges.begin(), affectedEdges.end());
        std::vector<EdgeId> edgesToRegenerate;
        
        // Add all affected edges
        for (EdgeId edgeId : affectedEdges) {
            edgesToRegenerate.push_back(edgeId);
        }
        
        // Add redistributed edges that are NOT in affectedEdges
        for (EdgeId edgeId : result.redistributedEdges) {
            if (affectedSet.find(edgeId) == affectedSet.end()) {
#if EDGE_ROUTING_DEBUG
                std::cout << "[CALL #" << thisCallId << " Phase4] Edge " << edgeId
                          << " redistributed but not affected - adding to regeneration list" << std::endl;
#endif
                edgesToRegenerate.push_back(edgeId);
            }
        }

        // Regenerate bendPoints using optimizer's path generation algorithm
        // This delegates to the appropriate algorithm (Geometric or A*) based on
        // what optimizer is set, maintaining algorithm separation during drag.
        // 
        // Note: Full optimize() would be ideal for cooperative rerouting, but it
        // would need to be called with special handling to preserve the redistributed
        // snap positions. For now, use regenerateBendPoints which keeps snap positions
        // and only regenerates paths.
        if (edgeOptimizer_ && !edgesToRegenerate.empty()) {
#if EDGE_ROUTING_DEBUG
            std::cout << "[CALL #" << thisCallId << " Phase4] Regenerating bendPoints for "
                      << edgesToRegenerate.size() << " edges via " << edgeOptimizer_->algorithmName()
                      << " optimizer: ";
            for (EdgeId eid : edgesToRegenerate) std::cout << eid << " ";
            std::cout << std::endl;
#endif
            edgeOptimizer_->regenerateBendPoints(edgesToRegenerate, edgeLayouts, nodeLayouts);
        }

        // Update label positions for all processed edges
        for (EdgeId edgeId : result.processedEdges) {
            auto it = edgeLayouts.find(edgeId);
            if (it != edgeLayouts.end()) {
                it->second.labelPosition = LayoutUtils::calculateEdgeLabelPosition(it->second);
            }
        }
        return result;
    }

    // Build "other edges" map for overlap detection
    std::unordered_map<EdgeId, EdgeLayout> otherEdges;
    for (const auto& [edgeId, layout] : edgeLayouts) {
        if (std::find(result.processedEdges.begin(), result.processedEdges.end(), edgeId)
            == result.processedEdges.end()) {
            otherEdges[edgeId] = layout;
        }
    }

    for (EdgeId edgeId : result.processedEdges) {
        auto it = edgeLayouts.find(edgeId);
        if (it == edgeLayouts.end()) continue;

        // 1. A* pathfinding with overlap avoidance
        recalculateBendPoints(it->second, nodeLayouts, effectiveGridSize, &otherEdges);

        // 2. Check for diagonal and attempt fix
        bool needsRetry = false;
        if (it->second.bendPoints.empty()) {
            float dx = std::abs(it->second.sourcePoint.x - it->second.targetPoint.x);
            float dy = std::abs(it->second.sourcePoint.y - it->second.targetPoint.y);
            needsRetry = (dx > 1.0f && dy > 1.0f);
        } else {
            float dx_src = std::abs(it->second.sourcePoint.x - it->second.bendPoints[0].position.x);
            float dy_src = std::abs(it->second.sourcePoint.y - it->second.bendPoints[0].position.y);
            const auto& lastBend = it->second.bendPoints.back();
            float dx_tgt = std::abs(lastBend.position.x - it->second.targetPoint.x);
            float dy_tgt = std::abs(lastBend.position.y - it->second.targetPoint.y);
            needsRetry = (dx_src > 1.0f && dy_src > 1.0f) || (dx_tgt > 1.0f && dy_tgt > 1.0f);
        }

        if (needsRetry) {
            bool srcNodeMoved = movedNodes.empty() || movedNodes.count(it->second.from) > 0;
            bool tgtNodeMoved = movedNodes.empty() || movedNodes.count(it->second.to) > 0;
            if (!srcNodeMoved && !tgtNodeMoved) {
                continue;  // Skip stationary edge
            }
            detectAndFixDiagonals(edgeId, edgeLayouts, nodeLayouts, movedNodes, effectiveGridSize, otherEdges);
        }

        // 3. PathCleanup - remove spikes and duplicates
        std::vector<Point> fullPath;
        fullPath.push_back(it->second.sourcePoint);
        for (const auto& bp : it->second.bendPoints) {
            fullPath.push_back(bp.position);
        }
        fullPath.push_back(it->second.targetPoint);
        PathCleanup::removeSpikesAndDuplicates(fullPath);
        it->second.bendPoints.clear();
        for (size_t i = 1; i + 1 < fullPath.size(); ++i) {
            it->second.bendPoints.push_back({fullPath[i]});
        }

        // 4. Direction validation - fix constraint violations
        bool directionWasFixed = validateAndFixDirectionConstraints(it->second, effectiveGridSize);
        if (directionWasFixed) {
            recalculateBendPoints(it->second, nodeLayouts, effectiveGridSize, &otherEdges);
        }

        // 5. Update otherEdges for subsequent edge overlap detection
        otherEdges[edgeId] = it->second;

        // 6. Update label position
        it->second.labelPosition = LayoutUtils::calculateEdgeLabelPosition(it->second);
    }

    // Option B: If retry required full re-routing, re-route all affected edges now
    // This ensures proper obstacle avoidance after snap point changes
    if (result.needsFullReroute && !result.edgesNeedingReroute.empty()) {
        std::cout << "[EdgeRouting] updateSnapPositions: Full re-route needed for "
                  << result.edgesNeedingReroute.size() << " edges after retry" << std::endl;

        // Build list of all edges to re-route (affected + those needing re-route)
        std::vector<EdgeId> edgesToReroute;
        for (EdgeId edgeId : affectedEdges) {
            if (edgeLayouts.find(edgeId) != edgeLayouts.end()) {
                edgesToReroute.push_back(edgeId);
            }
        }

        // Sort by priority (layer, etc.) to route in correct order
        std::sort(edgesToReroute.begin(), edgesToReroute.end(), [&](EdgeId a, EdgeId b) {
            auto itA = edgeLayouts.find(a);
            auto itB = edgeLayouts.find(b);
            if (itA == edgeLayouts.end() || itB == edgeLayouts.end()) return a < b;
            // Sort by source layer, then target layer
            auto srcA = nodeLayouts.find(itA->second.from);
            auto srcB = nodeLayouts.find(itB->second.from);
            if (srcA != nodeLayouts.end() && srcB != nodeLayouts.end()) {
                if (srcA->second.layer != srcB->second.layer) {
                    return srcA->second.layer < srcB->second.layer;
                }
            }
            return a < b;
        });

        // Re-route each edge with proper obstacle map
        std::unordered_map<EdgeId, EdgeLayout> otherEdges;
        for (EdgeId edgeId : edgesToReroute) {
            auto it = edgeLayouts.find(edgeId);
            if (it == edgeLayouts.end()) continue;

            // Build obstacle map with already-routed edges
            ObstacleMap obstacles;
            obstacles.buildFromNodes(nodeLayouts, effectiveGridSize);

            // Add segments from edges already routed in this pass
            if (!otherEdges.empty()) {
                obstacles.addEdgeSegments(otherEdges, edgeId);
            }

            // Also add segments from edges not being re-routed
            std::unordered_map<EdgeId, EdgeLayout> staticEdges;
            for (const auto& [otherId, otherLayout] : edgeLayouts) {
                if (otherId == edgeId) continue;
                if (std::find(edgesToReroute.begin(), edgesToReroute.end(), otherId) != edgesToReroute.end()) continue;
                if (hasFreshBendPoints(otherLayout, effectiveGridSize)) {
                    staticEdges[otherId] = otherLayout;
                }
            }
            obstacles.addEdgeSegments(staticEdges, edgeId);

            GridPoint startGrid = obstacles.pixelToGrid(it->second.sourcePoint);
            GridPoint goalGrid = obstacles.pixelToGrid(it->second.targetPoint);

            PathResult pathResult = activePathFinder().findPath(
                startGrid, goalGrid, obstacles,
                it->second.from, it->second.to,
                it->second.sourceEdge, it->second.targetEdge,
                {}, {});

            if (pathResult.found && pathResult.path.size() >= 2) {
                it->second.bendPoints.clear();
                for (size_t i = 1; i + 1 < pathResult.path.size(); ++i) {
                    Point pixelPoint = obstacles.gridToPixel(pathResult.path[i].x, pathResult.path[i].y);
                    it->second.bendPoints.push_back({pixelPoint});
                }
                std::cout << "[EdgeRouting] Full re-route Edge " << edgeId 
                          << " SUCCESS: bends=" << it->second.bendPoints.size() << std::endl;
            } else {
                std::cout << "[EdgeRouting] Full re-route Edge " << edgeId << " FAILED" << std::endl;
            }

            // Add this edge to otherEdges for subsequent routing
            otherEdges[edgeId] = it->second;

            // Update label position
            it->second.labelPosition = LayoutUtils::calculateEdgeLabelPosition(it->second);
        }
    }

    // Post-processing removed - A* should find correct paths with obstacles
    return result;
}

void EdgeRouting::updateEdgeRoutingWithOptimization(
    std::unordered_map<EdgeId, EdgeLayout>& edgeLayouts,
    const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
    const std::vector<EdgeId>& affectedEdges,
    const LayoutOptions& options,
    const std::unordered_set<NodeId>& movedNodes) {

#if EDGE_ROUTING_DEBUG
    std::cout << "\n[EdgeRouting] updateEdgeRoutingWithOptimization called" << std::endl;
    std::cout << "  affectedEdges: " << affectedEdges.size() << std::endl;
    std::cout << "  movedNodes: " << movedNodes.size() << std::endl;
    for (EdgeId edgeId : affectedEdges) {
        auto it = edgeLayouts.find(edgeId);
        if (it != edgeLayouts.end()) {
            const auto& e = it->second;
            std::cout << "  Edge " << edgeId << " BEFORE: "
                      << "src=(" << e.sourcePoint.x << "," << e.sourcePoint.y << ") "
                      << "tgt=(" << e.targetPoint.x << "," << e.targetPoint.y << ") "
                      << "bends=" << e.bendPoints.size() << std::endl;
        }
    }
#endif

    float gridSize = options.gridConfig.cellSize;
    bool usedDragOptimizer = false;  // Track if we used drag optimizer (skip A* if true)

    // Apply drag algorithm if enabled
    if (options.optimizationOptions.dragAlgorithm != DragAlgorithm::None && !affectedEdges.empty()) {
        std::unique_ptr<IEdgeOptimizer> optimizer;

        switch (options.optimizationOptions.dragAlgorithm) {
            case DragAlgorithm::Geometric: {
                // Create optimizer with penalty system and direction preservation
                OptimizerConfig config = OptimizerConfig::aggressive();  // Fast for drag
                config.preserveDirections = true;  // Keep existing edge directions during drag
                config.penaltySystem = EdgePenaltySystem::createDefault();
                optimizer = OptimizerRegistry::instance().create("Geometric", config);
                break;
            }
            case DragAlgorithm::AStar: {
                // A* pathfinding after drop - optimal paths with full optimization
                OptimizerConfig config = OptimizerConfig::aggressive();
                config.preserveDirections = false;  // Try all 16 edge combinations for best path
                config.gridSize = options.gridConfig.cellSize;
                config.pathFinder = pathFinder_;  // Share pathfinder
                config.penaltySystem = EdgePenaltySystem::createDefault();
                optimizer = OptimizerRegistry::instance().create("AStar", config);
                break;
            }
            case DragAlgorithm::None:
                // No optimization
                break;
            case DragAlgorithm::HideUntilDrop:
                // Hide edges during drag - skip all processing
                // Edges will be recalculated with A* on drop via PostDragAlgorithm
                return;
        }

        // Note: Penalty system and preserveDirections already configured via OptimizerRegistry

        if (optimizer) {
            // Store optimizer in edgeOptimizer_ so updateSnapPositions can use it
            // for regenerating bendPoints of redistributed edges
            edgeOptimizer_ = std::move(optimizer);

            // Collect all edges (including self-loops - optimizer handles them with penalty system)
            std::vector<EdgeId> edgesToOptimize;
            edgesToOptimize.reserve(affectedEdges.size());
            for (EdgeId edgeId : affectedEdges) {
                auto it = edgeLayouts.find(edgeId);
                if (it != edgeLayouts.end()) {
                    edgesToOptimize.push_back(edgeId);
                }
            }

            // Step 1: Optimizer calculates best paths while preserving edge directions
            auto optimizedLayouts = edgeOptimizer_->optimize(edgesToOptimize, edgeLayouts, nodeLayouts);

            // Merge optimized layouts from optimizer - including bendPoints for drag
            // During drag, we use Geometric optimizer's bendPoints directly (fast)
            // This avoids expensive A* recalculation in updateSnapPositions
            for (auto& [edgeId, layout] : optimizedLayouts) {
                auto it = edgeLayouts.find(edgeId);
                if (it != edgeLayouts.end()) {
                    it->second.sourceEdge = layout.sourceEdge;
                    it->second.targetEdge = layout.targetEdge;
                    it->second.sourcePoint = layout.sourcePoint;
                    it->second.targetPoint = layout.targetPoint;
                    it->second.bendPoints = layout.bendPoints;
                    // Reset snap indices for redistribution
                    it->second.sourceSnapIndex = constants::SNAP_INDEX_UNASSIGNED;
                    it->second.targetSnapIndex = constants::SNAP_INDEX_UNASSIGNED;
                }
            }

            // Step 2: Snap distribution for new edge combinations
            // Skip bendPoint recalculation - already set by optimizer
            updateSnapPositions(edgeLayouts, nodeLayouts, edgesToOptimize, movedNodes, gridSize, true);

            // Note: Self-loops are now included in edgesToOptimize and go through optimizer
            // with penalty-based direction selection (SegmentOverlapPenalty etc.)

            usedDragOptimizer = true;  // Mark that drag optimizer was used
        }
    } else {
        // No optimizer: just update snap positions
        updateSnapPositions(edgeLayouts, nodeLayouts, affectedEdges, movedNodes, gridSize);
    }

    // === Skip A* post-processing when drag optimizer was used ===
    // During drag, we prioritize speed over perfect paths.
    // A* will run after drop (via debounce callback) to fix any issues.
    if (usedDragOptimizer) {
#if EDGE_ROUTING_DEBUG
        std::cout << "[EdgeRouting] Skipping A* post-processing (drag optimizer mode)" << std::endl;
#endif
        return;
    }

    // === Check if any non-affected edges now penetrate moved nodes ===
    // When a node is moved, edges that don't connect to it might now pass through it.
    // Detect and re-route such edges to avoid penetration.
    if (!movedNodes.empty()) {
        std::vector<EdgeId> penetratingEdges;
        
        for (const auto& [edgeId, edgeLayout] : edgeLayouts) {
            // Skip edges already updated
            if (std::find(affectedEdges.begin(), affectedEdges.end(), edgeId) != affectedEdges.end()) {
                continue;
            }
            // Skip self-loops
            if (edgeLayout.from == edgeLayout.to) {
                continue;
            }
            
            // Check if this edge penetrates any moved node
            bool penetrates = false;
            for (NodeId movedNodeId : movedNodes) {
                if (penetrates) break;
                
                // Skip if this edge connects to the moved node
                if (edgeLayout.from == movedNodeId || edgeLayout.to == movedNodeId) continue;
                
                auto nodeIt = nodeLayouts.find(movedNodeId);
                if (nodeIt == nodeLayouts.end()) continue;
                const NodeLayout& movedNode = nodeIt->second;
                
                // Check each segment
                std::vector<Point> path;
                path.push_back(edgeLayout.sourcePoint);
                for (const auto& bp : edgeLayout.bendPoints) {
                    path.push_back(bp.position);
                }
                path.push_back(edgeLayout.targetPoint);
                
                for (size_t i = 0; i + 1 < path.size(); ++i) {
                    if (segmentPenetratesNodeInterior(path[i], path[i+1], movedNode)) {
                        penetrates = true;
                        break;
                    }
                }
            }
            
            if (penetrates) {
                penetratingEdges.push_back(edgeId);
            }
        }
        
        // Re-route edges that penetrate moved nodes
        if (!penetratingEdges.empty()) {
#if EDGE_ROUTING_DEBUG
            std::cout << "[EdgeRouting] updateEdgeRoutingWithOptimization: "
                      << penetratingEdges.size() << " non-affected edges penetrate moved nodes, re-routing" << std::endl;
#endif
            updateSnapPositions(edgeLayouts, nodeLayouts, penetratingEdges, movedNodes, gridSize);
        }
    }
    
    // === Final validation: Check ALL edges against ALL nodes ===
    // After all routing is done, do a final sweep to ensure no edge penetrates any node.
    // This catches cases where re-routing created new penetrations with non-moved nodes.
    {
        std::vector<EdgeId> finalPenetratingEdges;
        
        for (auto& [edgeId, edgeLayout] : edgeLayouts) {
            // Skip self-loops
            if (edgeLayout.from == edgeLayout.to) continue;
            
            // Build path
            std::vector<Point> path;
            path.push_back(edgeLayout.sourcePoint);
            for (const auto& bp : edgeLayout.bendPoints) {
                path.push_back(bp.position);
            }
            path.push_back(edgeLayout.targetPoint);
            
            // Check against all nodes
            bool penetrates = false;
            for (const auto& [nodeId, nodeLayout] : nodeLayouts) {
                if (penetrates) break;
                // Skip source and target nodes
                if (nodeId == edgeLayout.from || nodeId == edgeLayout.to) continue;
                
                for (size_t i = 0; i + 1 < path.size(); ++i) {
                    if (segmentPenetratesNodeInterior(path[i], path[i+1], nodeLayout)) {
                        penetrates = true;
                        break;
                    }
                }
            }
            
            if (penetrates) {
                finalPenetratingEdges.push_back(edgeId);
            }
        }
        
        // For any remaining penetrating edges, recalculate with fresh A* pathfinding
        if (!finalPenetratingEdges.empty()) {
#if EDGE_ROUTING_DEBUG
            std::cout << "[EdgeRouting] updateEdgeRoutingWithOptimization: "
                      << "Final validation found " << finalPenetratingEdges.size() 
                      << " edges still penetrating nodes, forcing A* recalculation" << std::endl;
#endif
            for (EdgeId edgeId : finalPenetratingEdges) {
                auto it = edgeLayouts.find(edgeId);
                if (it != edgeLayouts.end()) {
                    // Force recalculation with A* pathfinder (no overlap avoidance)
                    recalculateBendPoints(it->second, nodeLayouts, gridSize, nullptr);
                }
            }
        }
    }

    // === Post-Nudging: Visual separation of overlapping segments ===
    // EdgeNudger applies small offsets to visually separate segments that share coordinates.
    // This is the final fallback after Rip-up and Reroute has eliminated most overlaps.
    if (options.optimizationOptions.enablePostNudging) {
        EdgeNudger::Config nudgeConfig;
        nudgeConfig.nudgeOffset = options.optimizationOptions.nudgeOffset;
        nudgeConfig.minSegmentLength = options.optimizationOptions.minNudgeSegmentLength;
        nudgeConfig.enabled = true;

        EdgeNudger nudger(nudgeConfig);
        auto nudgeResult = nudger.applyNudging(edgeLayouts);

#if EDGE_ROUTING_DEBUG
        if (nudgeResult.totalOverlaps > 0) {
            std::cout << "[EdgeNudger] Applied nudging to " << nudgeResult.totalOverlaps
                      << " overlap groups, " << nudgeResult.nudgedSegments.size()
                      << " segments adjusted" << std::endl;
        }
#endif

        // Apply nudged positions to edge layouts
        // Note: EdgeNudger returns NudgedSegment with edgeId and segmentIndex
        // We need to update the corresponding bendPoint positions
        for (const auto& nudged : nudgeResult.nudgedSegments) {
            auto it = edgeLayouts.find(nudged.edgeId);
            if (it == edgeLayouts.end()) continue;

            EdgeLayout& layout = it->second;
            
            // segmentIndex: 0 = sourcePoint→first bend (or target if no bends)
            //               1..n-1 = bend-to-bend segments
            //               n = last bend→targetPoint
            // We need to adjust the START point of the segment (which is bendPoints[segmentIndex-1])
            // and the END point of the segment (which is bendPoints[segmentIndex] or targetPoint)
            
            if (nudged.segmentIndex == 0) {
                // First segment: adjust sourcePoint
                layout.sourcePoint = nudged.p1;
                if (layout.bendPoints.empty()) {
                    layout.targetPoint = nudged.p2;
                } else {
                    layout.bendPoints[0].position = nudged.p2;
                }
            } else if (nudged.segmentIndex < layout.bendPoints.size()) {
                // Middle segment: adjust bend points
                layout.bendPoints[nudged.segmentIndex - 1].position = nudged.p1;
                layout.bendPoints[nudged.segmentIndex].position = nudged.p2;
            } else if (nudged.segmentIndex == layout.bendPoints.size()) {
                // Last segment: adjust last bendPoint and targetPoint
                if (!layout.bendPoints.empty()) {
                    layout.bendPoints.back().position = nudged.p1;
                }
                layout.targetPoint = nudged.p2;
            }
        }
    }

#if EDGE_ROUTING_DEBUG
    std::cout << "[EdgeRouting] updateEdgeRoutingWithOptimization DONE" << std::endl;
    for (EdgeId edgeId : affectedEdges) {
        auto it = edgeLayouts.find(edgeId);
        if (it != edgeLayouts.end()) {
            const auto& e = it->second;
            float dx = std::abs(e.targetPoint.x - e.sourcePoint.x);
            float dy = std::abs(e.targetPoint.y - e.sourcePoint.y);
            bool needsBend = (dx > 0.1f && dy > 0.1f);
            bool hasBend = !e.bendPoints.empty();
            std::cout << "  Edge " << edgeId << " AFTER: "
                      << "src=(" << e.sourcePoint.x << "," << e.sourcePoint.y << ") "
                      << "tgt=(" << e.targetPoint.x << "," << e.targetPoint.y << ") "
                      << "bends=" << e.bendPoints.size();
            if (needsBend && !hasBend) {
                std::cout << " [WARNING: DIAGONAL! dx=" << dx << " dy=" << dy << "]";
            }
            std::cout << std::endl;
            for (size_t i = 0; i < e.bendPoints.size(); ++i) {
                std::cout << "    bend[" << i << "]: (" << e.bendPoints[i].position.x
                          << "," << e.bendPoints[i].position.y << ")" << std::endl;
            }
        }
    }
#endif
}

// =============================================================================
// Edge Layout Validation (delegated to EdgeValidator)
// =============================================================================

std::string EdgeRouting::ValidationResult::getErrorDescription() const {
    // Create equivalent EdgeValidator result for delegation
    EdgeValidator::ValidationResult evResult;
    evResult.valid = valid;
    evResult.orthogonal = orthogonal;
    evResult.noNodeIntersection = noNodeIntersection;
    evResult.sourceDirectionOk = sourceDirectionOk;
    evResult.targetDirectionOk = targetDirectionOk;
    return evResult.getErrorDescription();
}

EdgeRouting::ValidationResult EdgeRouting::validateEdgeLayout(
    const EdgeLayout& layout,
    const std::unordered_map<NodeId, NodeLayout>& nodeLayouts) {
    // Delegate to EdgeValidator and convert result
    auto evResult = EdgeValidator::validate(layout, nodeLayouts);

    ValidationResult result;
    result.valid = evResult.valid;
    result.orthogonal = evResult.orthogonal;
    result.noNodeIntersection = evResult.noNodeIntersection;
    result.sourceDirectionOk = evResult.sourceDirectionOk;
    result.targetDirectionOk = evResult.targetDirectionOk;
    return result;
}

// Channel routing methods have been moved to ChannelRouter class

}  // namespace arborvia
