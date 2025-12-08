#include "EdgeRouting.h"
#include "PathCalculator.h"
#include "EdgePathFixer.h"
#include "DragOptimizationHandler.h"
#include "SnapPositionUpdater.h"
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
// EdgePathFixer Delegation Methods
// =============================================================================

bool EdgeRouting::detectAndFixDiagonals(
    EdgeId edgeId,
    std::unordered_map<EdgeId, EdgeLayout>& edgeLayouts,
    const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
    const std::unordered_set<NodeId>& movedNodes,
    float effectiveGridSize,
    std::unordered_map<EdgeId, EdgeLayout>& otherEdges) {

    // Delegate to EdgePathFixer which encapsulates diagonal detection and fixing logic
    EdgePathFixer fixer(activePathFinder());
    return fixer.detectAndFixDiagonals(edgeId, edgeLayouts, nodeLayouts, movedNodes, effectiveGridSize, otherEdges);
}

bool EdgeRouting::validateAndFixDirectionConstraints(
    EdgeLayout& layout,
    float effectiveGridSize) {

    // Delegate to EdgePathFixer which encapsulates direction constraint validation and fixing
    EdgePathFixer fixer(activePathFinder());
    return fixer.validateAndFixDirectionConstraints(layout, effectiveGridSize);
}

// =============================================================================
// updateSnapPositions Main Method (delegates to SnapPositionUpdater)
// =============================================================================

EdgeRouting::SnapUpdateResult EdgeRouting::updateSnapPositions(
    std::unordered_map<EdgeId, EdgeLayout>& edgeLayouts,
    const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
    const std::vector<EdgeId>& affectedEdges,
    const std::unordered_set<NodeId>& movedNodes,
    float gridSize,
    bool skipBendPointRecalc) {

    // Create callbacks for SnapPositionUpdater
    auto recalcFunc = [this](EdgeLayout& layout,
                             const std::unordered_map<NodeId, NodeLayout>& nodes,
                             float gs,
                             const std::unordered_map<EdgeId, EdgeLayout>* otherEdges) {
        recalculateBendPoints(layout, nodes, gs, otherEdges);
    };

    auto detectFixFunc = [this](EdgeId edgeId,
                                std::unordered_map<EdgeId, EdgeLayout>& layouts,
                                const std::unordered_map<NodeId, NodeLayout>& nodes,
                                const std::unordered_set<NodeId>& moved,
                                float gs,
                                std::unordered_map<EdgeId, EdgeLayout>& others) {
        return detectAndFixDiagonals(edgeId, layouts, nodes, moved, gs, others);
    };

    auto validateFixFunc = [this](EdgeLayout& layout, float gs) {
        return validateAndFixDirectionConstraints(layout, gs);
    };

    // Delegate to SnapPositionUpdater
    SnapPositionUpdater updater(pathFinder_, recalcFunc, detectFixFunc, validateFixFunc);
    arborvia::SnapUpdateResult result = updater.updateSnapPositions(
        edgeLayouts, nodeLayouts, affectedEdges, movedNodes, gridSize,
        skipBendPointRecalc, edgeOptimizer_.get());

    // Convert SnapPositionUpdater::SnapUpdateResult to EdgeRouting::SnapUpdateResult
    // (std::set → std::unordered_set, std::set → std::vector)
    SnapUpdateResult edgeRoutingResult;
    edgeRoutingResult.processedEdges.insert(result.processedEdges.begin(), result.processedEdges.end());
    edgeRoutingResult.redistributedEdges.insert(result.redistributedEdges.begin(), result.redistributedEdges.end());
    edgeRoutingResult.needsFullReroute = result.needsFullReroute;
    edgeRoutingResult.edgesNeedingReroute.assign(result.edgesNeedingReroute.begin(), result.edgesNeedingReroute.end());
    return edgeRoutingResult;
}

void EdgeRouting::updateEdgeRoutingWithOptimization(
    std::unordered_map<EdgeId, EdgeLayout>& edgeLayouts,
    const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
    const std::vector<EdgeId>& affectedEdges,
    const LayoutOptions& options,
    const std::unordered_set<NodeId>& movedNodes) {

    // Create callbacks for DragOptimizationHandler
    auto snapUpdateFunc = [this](
        std::unordered_map<EdgeId, EdgeLayout>& layouts,
        const std::unordered_map<NodeId, NodeLayout>& nodes,
        const std::vector<EdgeId>& edges,
        const std::unordered_set<NodeId>& moved,
        float gridSize,
        bool skipBendPointRecalc) {
        updateSnapPositions(layouts, nodes, edges, moved, gridSize, skipBendPointRecalc);
    };

    auto recalcFunc = [this](
        EdgeLayout& layout,
        const std::unordered_map<NodeId, NodeLayout>& nodes,
        float gridSize,
        const std::unordered_map<EdgeId, EdgeLayout>* otherEdges) {
        recalculateBendPoints(layout, nodes, gridSize, otherEdges);
    };

    // Delegate to DragOptimizationHandler
    DragOptimizationHandler handler(pathFinder_, snapUpdateFunc, recalcFunc);
    handler.updateEdgeRoutingWithOptimization(
        edgeLayouts, nodeLayouts, affectedEdges, options, movedNodes, edgeOptimizer_);
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
