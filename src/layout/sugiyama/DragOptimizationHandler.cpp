#include "DragOptimizationHandler.h"
#include "DragOptimizationHandler.h"
#include "EdgeRoutingUtils.h"
#include "arborvia/layout/OptimizerRegistry.h"
#include "arborvia/layout/OptimizerConfig.h"
#include "arborvia/layout/EdgePenaltySystem.h"
#include "arborvia/layout/EdgeNudger.h"
#include <algorithm>
#include <cmath>
#include <iostream>

#ifndef EDGE_ROUTING_DEBUG
#define EDGE_ROUTING_DEBUG 0
#endif

namespace arborvia {

namespace constants {
    constexpr int SNAP_INDEX_UNASSIGNED = -1;
}

DragOptimizationHandler::DragOptimizationHandler(
    std::shared_ptr<IPathFinder> pathFinder,
    SnapUpdateFunc snapUpdateFunc,
    RecalcBendPointsFunc recalcFunc)
    : pathFinder_(std::move(pathFinder))
    , snapUpdateFunc_(std::move(snapUpdateFunc))
    , recalcFunc_(std::move(recalcFunc)) {
}

bool DragOptimizationHandler::segmentPenetratesNodeInterior(
    const Point& p1, const Point& p2, const NodeLayout& node) {
    return EdgeRoutingUtils::segmentPenetratesNodeInterior(p1, p2, node);
}

std::vector<EdgeId> DragOptimizationHandler::collectPenetratingEdges(
    const std::unordered_map<EdgeId, EdgeLayout>& edgeLayouts,
    const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
    const std::unordered_set<NodeId>& nodesToCheck,
    const std::vector<EdgeId>& excludeEdges) {

    std::vector<EdgeId> penetratingEdges;

    for (const auto& [edgeId, edgeLayout] : edgeLayouts) {
        // Skip excluded edges
        if (std::find(excludeEdges.begin(), excludeEdges.end(), edgeId) != excludeEdges.end()) {
            continue;
        }
        // Skip self-loops
        if (edgeLayout.from == edgeLayout.to) {
            continue;
        }

        bool penetrates = false;
        for (NodeId nodeId : nodesToCheck) {
            if (penetrates) break;

            // Skip if edge connects to this node
            if (edgeLayout.from == nodeId || edgeLayout.to == nodeId) continue;

            auto nodeIt = nodeLayouts.find(nodeId);
            if (nodeIt == nodeLayouts.end()) continue;
            const NodeLayout& node = nodeIt->second;

            // Build path
            std::vector<Point> path;
            path.push_back(edgeLayout.sourcePoint);
            for (const auto& bp : edgeLayout.bendPoints) {
                path.push_back(bp.position);
            }
            path.push_back(edgeLayout.targetPoint);

            // Check each segment
            for (size_t i = 0; i + 1 < path.size(); ++i) {
                if (segmentPenetratesNodeInterior(path[i], path[i + 1], node)) {
                    penetrates = true;
                    break;
                }
            }
        }

        if (penetrates) {
            penetratingEdges.push_back(edgeId);
        }
    }

    return penetratingEdges;
}

std::vector<EdgeId> DragOptimizationHandler::collectAllPenetratingEdges(
    const std::unordered_map<EdgeId, EdgeLayout>& edgeLayouts,
    const std::unordered_map<NodeId, NodeLayout>& nodeLayouts) {

    std::vector<EdgeId> penetratingEdges;

    for (const auto& [edgeId, edgeLayout] : edgeLayouts) {
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
            if (nodeId == edgeLayout.from || nodeId == edgeLayout.to) continue;

            for (size_t i = 0; i + 1 < path.size(); ++i) {
                if (segmentPenetratesNodeInterior(path[i], path[i + 1], nodeLayout)) {
                    penetrates = true;
                    break;
                }
            }
        }

        if (penetrates) {
            penetratingEdges.push_back(edgeId);
        }
    }

    return penetratingEdges;
}

void DragOptimizationHandler::applyPostNudging(
    std::unordered_map<EdgeId, EdgeLayout>& edgeLayouts,
    const LayoutOptions& options) {

    if (!options.optimizationOptions.enablePostNudging) return;

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

    // Apply nudged positions
    for (const auto& nudged : nudgeResult.nudgedSegments) {
        auto it = edgeLayouts.find(nudged.edgeId);
        if (it == edgeLayouts.end()) continue;

        EdgeLayout& layout = it->second;

        if (nudged.segmentIndex == 0) {
            layout.sourcePoint = nudged.p1;
            if (layout.bendPoints.empty()) {
                layout.targetPoint = nudged.p2;
            } else {
                layout.bendPoints[0].position = nudged.p2;
            }
        } else if (nudged.segmentIndex < layout.bendPoints.size()) {
            layout.bendPoints[nudged.segmentIndex - 1].position = nudged.p1;
            layout.bendPoints[nudged.segmentIndex].position = nudged.p2;
        } else if (nudged.segmentIndex == layout.bendPoints.size()) {
            if (!layout.bendPoints.empty()) {
                layout.bendPoints.back().position = nudged.p1;
            }
            layout.targetPoint = nudged.p2;
        }
    }
}

void DragOptimizationHandler::updateEdgeRoutingWithOptimization(
    std::unordered_map<EdgeId, EdgeLayout>& edgeLayouts,
    const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
    const std::vector<EdgeId>& affectedEdges,
    const LayoutOptions& options,
    const std::unordered_set<NodeId>& movedNodes,
    std::shared_ptr<IEdgeOptimizer>& edgeOptimizer) {

#if EDGE_ROUTING_DEBUG
    std::cout << "\n[DragOptimizationHandler] updateEdgeRoutingWithOptimization called" << std::endl;
    std::cout << "  affectedEdges: " << affectedEdges.size() << std::endl;
    std::cout << "  movedNodes: " << movedNodes.size() << std::endl;
#endif

    float gridSize = options.gridConfig.cellSize;
    bool usedDragOptimizer = false;

    // Apply drag algorithm if enabled
    if (options.optimizationOptions.dragAlgorithm != DragAlgorithm::None && !affectedEdges.empty()) {
        std::unique_ptr<IEdgeOptimizer> optimizer;

        switch (options.optimizationOptions.dragAlgorithm) {
            case DragAlgorithm::Geometric: {
                OptimizerConfig config = OptimizerConfig::aggressive();
                config.preserveDirections = true;
                config.penaltySystem = EdgePenaltySystem::createDefault();
                optimizer = OptimizerRegistry::instance().create("Geometric", config);
                break;
            }
            case DragAlgorithm::AStar: {
                OptimizerConfig config = OptimizerConfig::aggressive();
                config.preserveDirections = false;
                config.gridSize = options.gridConfig.cellSize;
                config.pathFinder = pathFinder_;
                config.penaltySystem = EdgePenaltySystem::createDefault();
                optimizer = OptimizerRegistry::instance().create("AStar", config);
                break;
            }
            case DragAlgorithm::None:
                break;
            case DragAlgorithm::HideUntilDrop:
                return;
        }

        if (optimizer) {
            edgeOptimizer = std::move(optimizer);

            std::vector<EdgeId> edgesToOptimize;
            edgesToOptimize.reserve(affectedEdges.size());
            for (EdgeId edgeId : affectedEdges) {
                if (edgeLayouts.find(edgeId) != edgeLayouts.end()) {
                    edgesToOptimize.push_back(edgeId);
                }
            }

            auto optimizedLayouts = edgeOptimizer->optimize(edgesToOptimize, edgeLayouts, nodeLayouts);

            for (auto& [edgeId, layout] : optimizedLayouts) {
                auto it = edgeLayouts.find(edgeId);
                if (it != edgeLayouts.end()) {
                    it->second.sourceEdge = layout.sourceEdge;
                    it->second.targetEdge = layout.targetEdge;
                    it->second.sourcePoint = layout.sourcePoint;
                    it->second.targetPoint = layout.targetPoint;
                    it->second.bendPoints = layout.bendPoints;
                    it->second.sourceSnapIndex = constants::SNAP_INDEX_UNASSIGNED;
                    it->second.targetSnapIndex = constants::SNAP_INDEX_UNASSIGNED;
                }
            }

            snapUpdateFunc_(edgeLayouts, nodeLayouts, edgesToOptimize, movedNodes, gridSize, true);
            usedDragOptimizer = true;
        }
    } else {
        snapUpdateFunc_(edgeLayouts, nodeLayouts, affectedEdges, movedNodes, gridSize, false);
    }

    // Skip A* post-processing when drag optimizer was used
    if (usedDragOptimizer) {
#if EDGE_ROUTING_DEBUG
        std::cout << "[DragOptimizationHandler] Skipping A* post-processing (drag optimizer mode)" << std::endl;
#endif
        return;
    }

    // Check for edges penetrating moved nodes
    if (!movedNodes.empty()) {
        auto penetratingEdges = collectPenetratingEdges(edgeLayouts, nodeLayouts, movedNodes, affectedEdges);
        if (!penetratingEdges.empty()) {
#if EDGE_ROUTING_DEBUG
            std::cout << "[DragOptimizationHandler] " << penetratingEdges.size()
                      << " non-affected edges penetrate moved nodes, re-routing" << std::endl;
#endif
            snapUpdateFunc_(edgeLayouts, nodeLayouts, penetratingEdges, movedNodes, gridSize, false);
        }
    }

    // Final validation: check ALL edges against ALL nodes
    {
        auto finalPenetratingEdges = collectAllPenetratingEdges(edgeLayouts, nodeLayouts);
        if (!finalPenetratingEdges.empty()) {
#if EDGE_ROUTING_DEBUG
            std::cout << "[DragOptimizationHandler] Final validation found " << finalPenetratingEdges.size()
                      << " edges still penetrating nodes, forcing A* recalculation" << std::endl;
#endif
            for (EdgeId edgeId : finalPenetratingEdges) {
                auto it = edgeLayouts.find(edgeId);
                if (it != edgeLayouts.end()) {
                    recalcFunc_(it->second, nodeLayouts, gridSize, nullptr);
                }
            }
        }
    }

    // Post-nudging
    applyPostNudging(edgeLayouts, options);

#if EDGE_ROUTING_DEBUG
    std::cout << "[DragOptimizationHandler] updateEdgeRoutingWithOptimization DONE" << std::endl;
#endif
}

} // namespace arborvia
