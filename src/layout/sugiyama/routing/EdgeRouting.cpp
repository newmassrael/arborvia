#include "EdgeRouting.h"
#include "PathCalculator.h"
#include "EdgePathFixer.h"
#include "../../interactive/DragOptimizationHandler.h"
#include "../../snap/SnapPositionUpdater.h"
#include "../../snap/SnapDistributor.h"
#include "RoutingOptimizer.h"
#include "EdgeRoutingUtils.h"
#include "../../snap/GridSnapCalculator.h"
#include "../../snap/GridCoordinates.h"
#include "ChannelRouter.h"
#include "../../snap/SnapIndexManager.h"
#include "../../pathfinding/ObstacleMap.h"
#include "../../pathfinding/AStarPathFinder.h"
#include "../../optimization/astar/AStarEdgeOptimizer.h"
#include "../../optimization/geometric/GeometricEdgeOptimizer.h"
#include "EdgeValidator.h"
#include "SelfLoopRouter.h"
#include "PathCleanup.h"
#include "arborvia/core/GeometryUtils.h"
#include "arborvia/layout/api/IEdgeOptimizer.h"
#include "arborvia/layout/config/LayoutTypes.h"
#include "arborvia/layout/util/LayoutUtils.h"
#include "arborvia/layout/interactive/PathRoutingCoordinator.h"
#include "layout/optimization/OptimizerRegistry.h"
#include "arborvia/layout/config/OptimizerConfig.h"
#include "arborvia/layout/api/EdgePenaltySystem.h"
#include "arborvia/common/Logger.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <set>
#include <unordered_map>

// Debug flag for edge routing

namespace arborvia {


// =============================================================================
// Routing Constants
// =============================================================================

namespace {
    /// Tolerance for floating point comparisons in path calculations
    constexpr float EPSILON = 0.1f;
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
    // Use LayoutUtils with default gridSize (20.0f) for legacy compatibility
    // New code should use GridSnapCalculator directly for explicit grid control
    return LayoutUtils::calculateSnapPointFromRatio(node, edge, position, constants::effectiveGridSize(0.0f));
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
    const std::unordered_map<EdgeId, EdgeLayout>* otherEdges,
    const std::unordered_set<NodeId>* movedNodes) {
    
    // Delegate to PathCalculator which encapsulates bend point calculation logic
    PathCalculator calculator(activePathFinder());
    calculator.recalculateBendPoints(layout, nodeLayouts, gridSize, otherEdges, movedNodes);
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
            // Pass already-routed edges so A* can avoid overlaps during initial routing
            auto channelIt = channelAssignments.find(edgeId);
            if (channelIt != channelAssignments.end()) {
                layout = channelRouter_->routeChannelOrthogonal(edge, fromIt->second, toIt->second,
                                               isReversed, channelIt->second, options, &nodeLayouts,
                                               &result.edgeLayouts);
            } else {
                // Fallback: create simple orthogonal routing without channel
                layout = channelRouter_->routeChannelOrthogonal(edge, fromIt->second, toIt->second,
                                               isReversed, ChannelAssignment{}, options, &nodeLayouts,
                                               &result.edgeLayouts);
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
    
    // Get gridSize from LayoutOptions (single source of truth)
    const float gridSize = constants::effectiveGridSize(options.gridConfig.cellSize);

    // Use injected optimizer, or create based on postDragAlgorithm
    if (!optimizer && options.optimizationOptions.postDragAlgorithm != PostDragAlgorithm::None) {
        switch (options.optimizationOptions.postDragAlgorithm) {
            case PostDragAlgorithm::AStar: {
                // Create optimizer with penalty system for unified constraint handling
                OptimizerConfig config = OptimizerConfig::balanced();
                config.pathFinder = pathFinder_;  // Share pathfinder with optimizer
                config.penaltySystem = EdgePenaltySystem::createDefault();
                fallbackOptimizer = OptimizerRegistry::instance().create("AStar", config);
                break;
            }
            case PostDragAlgorithm::Geometric: {
                OptimizerConfig config = OptimizerConfig::aggressive();
                config.penaltySystem = EdgePenaltySystem::createDefault();
                fallbackOptimizer = OptimizerRegistry::instance().create("Geometric", config);
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
        auto optimizedLayouts = optimizer->optimize(edgeIds, result.edgeLayouts, nodeLayouts, gridSize);

        // Merge optimized layouts (bend points already calculated by optimizer)
        // NOTE: Snap indices are no longer stored - they're computed from positions
        for (auto& [edgeId, layout] : optimizedLayouts) {
            result.edgeLayouts[edgeId] = std::move(layout);
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

    // Create callback for bend point recalculation
    auto recalcFunc = [this](EdgeLayout& layout,
                             const std::unordered_map<NodeId, NodeLayout>& nodes,
                             float gs) {
        recalculateBendPoints(layout, nodes, gs);
    };

    // Delegate to SnapDistributor
    SnapDistributor distributor(recalcFunc);
    distributor.distribute(result, nodeLayouts, gridSize, sortSnapPoints);
}

void EdgeRouting::optimizeRouting(
    Result& result,
    const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
    const LayoutOptions& options) {

    // Delegate to RoutingOptimizer
    RoutingOptimizer optimizer(pathFinder_, edgeOptimizer_.get());
    optimizer.optimize(result, nodeLayouts, options);
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
                             const std::unordered_map<EdgeId, EdgeLayout>* otherEdges,
                             const std::unordered_set<NodeId>* moved) {
        recalculateBendPoints(layout, nodes, gs, otherEdges, moved);
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
        const std::unordered_map<EdgeId, EdgeLayout>* otherEdges,
        const std::unordered_set<NodeId>* moved) {
        recalculateBendPoints(layout, nodes, gridSize, otherEdges, moved);
    };

    // Delegate to DragOptimizationHandler
    DragOptimizationHandler handler(pathFinder_, snapUpdateFunc, recalcFunc);
    handler.updateEdgeRoutingWithOptimization(
        edgeLayouts, nodeLayouts, affectedEdges, options, movedNodes, edgeOptimizer_);
}

void EdgeRouting::regenerateBendPointsOnly(
    std::unordered_map<EdgeId, EdgeLayout>& edgeLayouts,
    const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
    const std::vector<EdgeId>& affectedEdges,
    const LayoutOptions& options) {

    if (affectedEdges.empty()) {
        return;
    }

    // Get or create optimizer
    IEdgeOptimizer* optimizer = edgeOptimizer_.get();
    std::unique_ptr<IEdgeOptimizer> fallbackOptimizer;

    // Get gridSize from LayoutOptions (single source of truth)
    float gridSize = constants::effectiveGridSize(options.gridConfig.cellSize);

    if (!optimizer && options.optimizationOptions.postDragAlgorithm != PostDragAlgorithm::None) {
        switch (options.optimizationOptions.postDragAlgorithm) {
            case PostDragAlgorithm::AStar: {
                OptimizerConfig config = OptimizerConfig::balanced();
                config.pathFinder = pathFinder_;
                config.penaltySystem = EdgePenaltySystem::createDefault();
                fallbackOptimizer = OptimizerRegistry::instance().create("AStar", config);
                break;
            }
            case PostDragAlgorithm::Geometric: {
                OptimizerConfig config = OptimizerConfig::aggressive();
                config.penaltySystem = EdgePenaltySystem::createDefault();
                fallbackOptimizer = OptimizerRegistry::instance().create("Geometric", config);
                break;
            }
            case PostDragAlgorithm::None:
                break;
        }
        optimizer = fallbackOptimizer.get();
    }

    if (!optimizer) {
        return;
    }

    // Use regenerateBendPoints which preserves sourceEdge/targetEdge and sourcePoint/targetPoint
    // This is the key difference from updateEdgeRoutingWithOptimization:
    // - updateEdgeRoutingWithOptimization: optimizer may change everything
    // - regenerateBendPointsOnly: only bendPoints are recalculated
    
    // [SNAP-SYNC-DEBUG] Log before regenerateBendPoints
    LOG_DEBUG("[SNAP-SYNC-DEBUG] regenerateBendPointsOnly: affectedEdges={} gridSize={}", affectedEdges.size(), gridSize);
    for (EdgeId edgeId : affectedEdges) {
        auto it = edgeLayouts.find(edgeId);
        if (it != edgeLayouts.end()) {
            const auto& layout = it->second;
            auto srcIt = nodeLayouts.find(layout.from);
            auto tgtIt = nodeLayouts.find(layout.to);
            LOG_DEBUG("[SNAP-SYNC-DEBUG] Edge {} from={} to={}", edgeId, layout.from, layout.to);
            if (srcIt != nodeLayouts.end()) {
                LOG_DEBUG("[SNAP-SYNC-DEBUG]   srcNode pos=({},{}) size=({},{})", 
                          srcIt->second.position.x, srcIt->second.position.y,
                          srcIt->second.size.width, srcIt->second.size.height);
            }
            // NOTE: snapIndex computed from position, not stored
            LOG_DEBUG("[SNAP-SYNC-DEBUG]   srcPoint=({},{}) srcEdge={}", 
                      layout.sourcePoint.x, layout.sourcePoint.y,
                      static_cast<int>(layout.sourceEdge));
            if (tgtIt != nodeLayouts.end()) {
                LOG_DEBUG("[SNAP-SYNC-DEBUG]   tgtNode pos=({},{}) size=({},{})", 
                          tgtIt->second.position.x, tgtIt->second.position.y,
                          tgtIt->second.size.width, tgtIt->second.size.height);
            }
            LOG_DEBUG("[SNAP-SYNC-DEBUG]   tgtPoint=({},{}) tgtEdge={}", 
                      layout.targetPoint.x, layout.targetPoint.y,
                      static_cast<int>(layout.targetEdge));
        }
    }
    
    optimizer->regenerateBendPoints(affectedEdges, edgeLayouts, nodeLayouts, gridSize);

    // Update label positions
    for (EdgeId edgeId : affectedEdges) {
        auto it = edgeLayouts.find(edgeId);
        if (it != edgeLayouts.end()) {
            it->second.labelPosition = LayoutUtils::calculateEdgeLabelPosition(it->second);
        }
    }
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
