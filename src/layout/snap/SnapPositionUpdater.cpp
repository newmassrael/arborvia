#include "SnapPositionUpdater.h"
#include "../sugiyama/routing/EdgeRoutingUtils.h"
#include "arborvia/core/GeometryUtils.h"
#include "GridSnapCalculator.h"
#include "SnapIndexManager.h"
#include "../sugiyama/routing/SelfLoopRouter.h"
#include "arborvia/layout/util/LayoutUtils.h"
#include "arborvia/layout/api/EdgeLayoutResolver.h"
#include "arborvia/common/Logger.h"
#include <algorithm>


namespace arborvia {

namespace {
    template <typename HashType = EdgeRoutingUtils::PairHash>
    std::unordered_map<std::pair<NodeId, NodeId>, EdgeId, HashType> buildEdgeMapFromLayouts(
        const std::unordered_map<EdgeId, EdgeLayout>& edgeLayouts) {
        std::unordered_map<std::pair<NodeId, NodeId>, EdgeId, HashType> edgeMap;
        for (const auto& [edgeId, layout] : edgeLayouts) {
            edgeMap[{layout.from, layout.to}] = edgeId;
        }
        return edgeMap;
    }
}

SnapPositionUpdater::SnapPositionUpdater(
    std::shared_ptr<IPathFinder> pathFinder,
    RecalcBendPointsFunc recalcFunc,
    DetectFixDiagonalsFunc detectFixFunc,
    ValidateFixDirectionFunc validateFixFunc)
    : pathFinder_(std::move(pathFinder))
    , recalcFunc_(std::move(recalcFunc))
    , detectFixFunc_(std::move(detectFixFunc))
    , validateFixFunc_(std::move(validateFixFunc)) {
}

bool SnapPositionUpdater::hasFreshBendPoints(const EdgeLayout& layout, float gridSize) {
    return EdgeRoutingUtils::hasFreshBendPoints(layout, gridSize);
}

SnapPositionUpdater::AffectedConnectionsMap SnapPositionUpdater::collectAffectedConnections(
    const std::unordered_map<EdgeId, EdgeLayout>& edgeLayouts,
    const std::vector<EdgeId>& affectedEdges) {

    AffectedConnectionsMap result;
    for (EdgeId edgeId : affectedEdges) {
        auto it = edgeLayouts.find(edgeId);
        if (it == edgeLayouts.end()) continue;

        const EdgeLayout& layout = it->second;
        result[{layout.from, layout.sourceEdge}].push_back({edgeId, true});
        result[{layout.to, layout.targetEdge}].push_back({edgeId, false});
    }
    return result;
}

void SnapPositionUpdater::calculateSnapPositionsForNodeEdge(
    const std::pair<NodeId, NodeEdge>& key,
    const std::vector<std::pair<EdgeId, bool>>& connections,
    std::unordered_map<EdgeId, EdgeLayout>& edgeLayouts,
    const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
    const std::unordered_map<NodeId, NodeLayout>& oldNodeLayouts,
    const std::unordered_set<NodeId>& movedNodes,
    float effectiveGridSize,
    SnapUpdateResult& result) {

    auto [nodeId, nodeEdge] = key;

    // Use EdgeLayoutResolver::isNodeMoved() for consistent movedNodes check
    auto shouldUpdateNode = [&movedNodes](NodeId nid) -> bool {
        return EdgeLayoutResolver::isNodeMoved(&movedNodes, nid);
    };

    auto nodeIt = nodeLayouts.find(nodeId);
    if (nodeIt == nodeLayouts.end()) return;
    const NodeLayout& node = nodeIt->second;
    int candidateCount = GridSnapCalculator::getCandidateCount(node, nodeEdge, effectiveGridSize);

    // Get old node layout for computing candidateIdx from old snap positions
    // For moved nodes, old position is needed to correctly interpret old snap positions
    auto oldNodeIt = oldNodeLayouts.find(nodeId);
    const NodeLayout& oldNode = (oldNodeIt != oldNodeLayouts.end()) ? oldNodeIt->second : node;

    // For unmoved nodes, verify positions are on valid grid points
    // NOTE: snapIndex is computed from position, not stored
    if (!shouldUpdateNode(nodeId)) {
        for (const auto& [edgeId, isSource] : connections) {
            auto it = edgeLayouts.find(edgeId);
            if (it != edgeLayouts.end()) {
                Point& snapPoint = isSource ? it->second.sourcePoint : it->second.targetPoint;
                
                // Compute current index from position
                int computedIdx = GridSnapCalculator::getCandidateIndexFromPosition(
                    node, nodeEdge, snapPoint, effectiveGridSize);
                
                // Clamp to valid range if needed
                if (computedIdx >= candidateCount) {
                    computedIdx = candidateCount - 1;
                    snapPoint = GridSnapCalculator::getPositionFromCandidateIndex(
                        node, nodeEdge, computedIdx, effectiveGridSize);
                }
            }
        }
        return;
    }

    // MOVED UP: Get all connections BEFORE collision detection
    // This allows detecting collisions with sibling edges that aren't in 'connections'
    auto allConnections = SnapIndexManager::getConnections(edgeLayouts, nodeId, nodeEdge);
    int totalCount = static_cast<int>(allConnections.incoming.size() + allConnections.outgoing.size());

    // Build set of affected edge IDs for quick lookup
    std::set<EdgeId> affectedEdgeIds;
    for (const auto& [edgeId, isSource] : connections) {
        affectedEdgeIds.insert(edgeId);
    }

    // Check if redistribution is needed by computing indices from positions
    // FIXED: Check against ALL edges on this NodeEdge, not just affected ones
    bool needsRedistribution = false;
    std::set<int> usedIndices;

    // First: collect indices from non-affected sibling edges (existing edges on this NodeEdge)
    for (EdgeId edgeId : allConnections.incoming) {
        if (affectedEdgeIds.count(edgeId) == 0) {
            auto it = edgeLayouts.find(edgeId);
            if (it != edgeLayouts.end()) {
                int snapIdx = GridSnapCalculator::getCandidateIndexFromPosition(
                    node, nodeEdge, it->second.targetPoint, effectiveGridSize);
                if (snapIdx >= 0 && snapIdx < candidateCount) {
                    usedIndices.insert(snapIdx);
                }
            }
        }
    }
    for (EdgeId edgeId : allConnections.outgoing) {
        if (affectedEdgeIds.count(edgeId) == 0) {
            auto it = edgeLayouts.find(edgeId);
            if (it != edgeLayouts.end()) {
                int snapIdx = GridSnapCalculator::getCandidateIndexFromPosition(
                    node, nodeEdge, it->second.sourcePoint, effectiveGridSize);
                if (snapIdx >= 0 && snapIdx < candidateCount) {
                    usedIndices.insert(snapIdx);
                }
            }
        }
    }

    // Then: check if affected edges collide with existing sibling indices
    for (const auto& [edgeId, isSource] : connections) {
        auto it = edgeLayouts.find(edgeId);
        if (it != edgeLayouts.end()) {
            Point snapPoint = isSource ? it->second.sourcePoint : it->second.targetPoint;
            int snapIdx = GridSnapCalculator::getCandidateIndexFromPosition(
                oldNode, nodeEdge, snapPoint, effectiveGridSize);
            if (snapIdx < 0 || snapIdx >= candidateCount) {
                needsRedistribution = true;
                break;
            }
            if (usedIndices.count(snapIdx) > 0) {
                LOG_DEBUG("[CONSTRAINT-DEBUG] Snap collision detected! Edge {} index {} already used, triggering redistribution",
                          edgeId, snapIdx);
                needsRedistribution = true;  // Collision with sibling detected!
                break;
            }
            usedIndices.insert(snapIdx);
        }
    }

    // [CONSTRAINT-DEBUG] Log redistribution decision
    if (needsRedistribution) {
        LOG_DEBUG("[CONSTRAINT-DEBUG] Node {} edge {} needs redistribution, total edges to process: {}",
                  nodeId, static_cast<int>(nodeEdge), 
                  allConnections.incoming.size() + allConnections.outgoing.size());
    }

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

    SelfLoopRouter::applySelfLoopCornerPositioning(edgesToProcess, edgeLayouts, nodeEdge);

    std::set<EdgeId> edgesNeedingNewIndex;
    if (needsRedistribution) {
        std::map<int, EdgeId> indexToFirstEdge;
        for (const auto& [edgeId, isSource] : edgesToProcess) {
            const EdgeLayout& el = edgeLayouts[edgeId];
            Point snapPoint = isSource ? el.sourcePoint : el.targetPoint;
            // Use oldNode to correctly interpret old snap positions
            int idx = GridSnapCalculator::getCandidateIndexFromPosition(
                oldNode, nodeEdge, snapPoint, effectiveGridSize);
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

    std::set<int> usedCandidateIndices;
    for (const auto& [edgeId, isSource] : edgesToProcess) {
        if (edgesNeedingNewIndex.count(edgeId) > 0) continue;
        const EdgeLayout& el = edgeLayouts[edgeId];
        Point snapPoint = isSource ? el.sourcePoint : el.targetPoint;
        // Use oldNode to correctly interpret old snap positions
        int idx = GridSnapCalculator::getCandidateIndexFromPosition(
            oldNode, nodeEdge, snapPoint, effectiveGridSize);
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
            // Compute candidateIndex from old snap position using old node layout
            // Then compute new snap position using new node layout
            Point currentSnapPoint = isSource ? layout.sourcePoint : layout.targetPoint;
            candidateIndex = GridSnapCalculator::getCandidateIndexFromPosition(
                oldNode, nodeEdge, currentSnapPoint, effectiveGridSize);
            if (candidateIndex < 0 || candidateIndex >= candidateCount) {
                snapPoint = GridSnapCalculator::calculateSnapPosition(node, nodeEdge, static_cast<int>(connIdx), totalCount, effectiveGridSize, &candidateIndex);
            } else {
                // Use new node layout to compute new position from preserved candidateIndex
                snapPoint = GridSnapCalculator::getPositionFromStoredIndex(node, nodeEdge, candidateIndex, effectiveGridSize);
            }
        }

        // [SNAP-SYNC-DEBUG] Log before update
        Point oldSnapPoint = isSource ? layout.sourcePoint : layout.targetPoint;
        int oldSnapIndex = GridSnapCalculator::getCandidateIndexFromPosition(
            oldNode, nodeEdge, oldSnapPoint, effectiveGridSize);
        LOG_DEBUG("[SNAP-SYNC-DEBUG] Edge {} {} nodeId={} nodeHasMoved={} thisEdgeNeedsNewIndex={} candidateCount={}",
                  edgeId, isSource ? "SOURCE" : "TARGET", nodeId, nodeHasMoved, thisEdgeNeedsNewIndex, candidateCount);
        LOG_DEBUG("[SNAP-SYNC-DEBUG]   node pos=({},{}) size=({},{})",
                  node.position.x, node.position.y, node.size.width, node.size.height);
        LOG_DEBUG("[SNAP-SYNC-DEBUG]   BEFORE: snapPoint=({},{}) computedIndex={}",
                  oldSnapPoint.x, oldSnapPoint.y, oldSnapIndex);
        LOG_DEBUG("[SNAP-SYNC-DEBUG]   CALCULATED: snapPoint=({},{}) candidateIndex={}",
                  snapPoint.x, snapPoint.y, candidateIndex);

        // SSOT Architecture: Only update snapIndex here.
        // snapPoint will be computed later by EdgeLayoutResolver::resolveSnapPointsOnly()
        // This ensures snapIndex is the Single Source of Truth.
        if (!nodeHasMoved) {
            if (thisEdgeNeedsNewIndex) {
                if (isSource) {
                    layout.sourceSnapIndex = candidateIndex;
                } else {
                    layout.targetSnapIndex = candidateIndex;
                }
                LOG_DEBUG("[SNAP-SYNC-DEBUG]   UPDATED snapIndex only (needsNewIndex): candidateIndex={}",
                          candidateIndex);
            } else {
                LOG_DEBUG("[SNAP-SYNC-DEBUG]   SKIPPED: nodeHasMoved=false and !thisEdgeNeedsNewIndex");
            }
        } else {
            if (isSource) {
                layout.sourceSnapIndex = candidateIndex;
            } else {
                layout.targetSnapIndex = candidateIndex;
            }
            LOG_DEBUG("[SNAP-SYNC-DEBUG]   UPDATED snapIndex only (nodeHasMoved): candidateIndex={}",
                      candidateIndex);
        }

        if (needsRedistribution) {
            result.redistributedEdges.insert(edgeId);
        }
    }
}

SnapUpdateResult SnapPositionUpdater::updateSnapPositions(
    std::unordered_map<EdgeId, EdgeLayout>& edgeLayouts,
    const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
    const std::unordered_map<NodeId, NodeLayout>& oldNodeLayouts,
    const std::vector<EdgeId>& affectedEdges,
    const std::unordered_set<NodeId>& movedNodes,
    float gridSize,
    bool skipBendPointRecalc,
    IEdgeOptimizer* edgeOptimizer) {

    SnapUpdateResult result;
    float gridSizeToUse = constants::effectiveGridSize(gridSize);

    // [SNAP-SYNC-DEBUG] Log entry parameters
    LOG_DEBUG("[SNAP-SYNC-DEBUG] updateSnapPositions called: affectedEdges={} movedNodes={} gridSize={} skipBendPointRecalc={}",
              affectedEdges.size(), movedNodes.size(), gridSizeToUse, skipBendPointRecalc);
    for (NodeId nid : movedNodes) {
        auto it = nodeLayouts.find(nid);
        if (it != nodeLayouts.end()) {
            LOG_DEBUG("[SNAP-SYNC-DEBUG] movedNode {} pos=({},{})", nid, it->second.position.x, it->second.position.y);
        }
    }
    for (EdgeId eid : affectedEdges) {
        auto it = edgeLayouts.find(eid);
        if (it != edgeLayouts.end()) {
            LOG_DEBUG("[SNAP-SYNC-DEBUG] affectedEdge {} from={} to={} srcPoint=({},{}) tgtPoint=({},{})",
                      eid, it->second.from, it->second.to,
                      it->second.sourcePoint.x, it->second.sourcePoint.y,
                      it->second.targetPoint.x, it->second.targetPoint.y);
        }
    }

    // Phase 0: Recalculate sourceEdge/targetEdge for Point nodes
    // Point nodes have no physical edge, so their exit direction should match target direction
    // Uses LayoutUtils::calculateSourceEdgeForPointNode/calculateTargetEdgeForPointNode
    for (EdgeId edgeId : affectedEdges) {
        auto it = edgeLayouts.find(edgeId);
        if (it == edgeLayouts.end()) continue;
        EdgeLayout& layout = it->second;

        auto srcNodeIt = nodeLayouts.find(layout.from);
        auto tgtNodeIt = nodeLayouts.find(layout.to);
        if (srcNodeIt == nodeLayouts.end() || tgtNodeIt == nodeLayouts.end()) continue;

        const NodeLayout& srcNode = srcNodeIt->second;
        const NodeLayout& tgtNode = tgtNodeIt->second;

        // Source is Point node → recalculate sourceEdge
        if (srcNode.isPointNode()) {
            NodeEdge oldSourceEdge = layout.sourceEdge;
            layout.sourceEdge = LayoutUtils::calculateSourceEdgeForPointNode(srcNode, tgtNode);

            if (layout.sourceEdge != oldSourceEdge) {
                LOG_DEBUG("[SNAP-SYNC-DEBUG] Point node sourceEdge recalculated: edge={} from={} "
                          "oldEdge={} newEdge={}",
                          edgeId, layout.from, static_cast<int>(oldSourceEdge),
                          static_cast<int>(layout.sourceEdge));
            }
        }

        // Target is Point node → recalculate targetEdge
        if (tgtNode.isPointNode()) {
            NodeEdge oldTargetEdge = layout.targetEdge;
            layout.targetEdge = LayoutUtils::calculateTargetEdgeForPointNode(srcNode, tgtNode);

            if (layout.targetEdge != oldTargetEdge) {
                LOG_DEBUG("[SNAP-SYNC-DEBUG] Point node targetEdge recalculated: edge={} to={} "
                          "oldEdge={} newEdge={}",
                          edgeId, layout.to, static_cast<int>(oldTargetEdge),
                          static_cast<int>(layout.targetEdge));
            }
        }
    }

    // Phase 1: Collect affected connections
    auto affectedConnections = collectAffectedConnections(edgeLayouts, affectedEdges);

    // Phase 2: Calculate snap positions
    for (auto& [key, connections] : affectedConnections) {
        calculateSnapPositionsForNodeEdge(key, connections, edgeLayouts, nodeLayouts, oldNodeLayouts, movedNodes, gridSizeToUse, result);
    }

    // Phase 3: Merge affected and redistributed edges
    result.processedEdges.insert(affectedEdges.begin(), affectedEdges.end());
    result.processedEdges.insert(result.redistributedEdges.begin(), result.redistributedEdges.end());

    // ==========================================================================
    // Phase 4: Resolve derived fields from snapIndex (SSOT Architecture)
    // ==========================================================================
    //
    // Two execution paths based on skipBendPointRecalc:
    //
    // Path A (skipBendPointRecalc=false):
    //   Full atomic resolution via EdgeLayoutResolver
    //   → snapPoint + bendPoints + labelPosition computed together
    //   → Used for final edge updates (drop events, initial layout)
    //
    // Path B (skipBendPointRecalc=true):
    //   SnapPoints only + optimizer handles bendPoints
    //   → Used during drag (optimizer provides real-time feedback)
    //
    // ==========================================================================

    if (!skipBendPointRecalc) {
        // Path A: Full atomic resolution
        // EdgeLayoutResolver computes all derived fields atomically
        std::vector<EdgeId> processedEdgesList(result.processedEdges.begin(), result.processedEdges.end());
        auto failedEdges = EdgeLayoutResolver::resolveEdgesWithRetry(
            edgeLayouts, processedEdgesList, nodeLayouts,
            gridSizeToUse, pathFinder_, &movedNodes);

        if (!failedEdges.empty()) {
            LOG_WARN("[SNAP-SYNC] {} edges failed atomic resolution", failedEdges.size());
        }
        LOG_DEBUG("[SNAP-SYNC] Atomically resolved {} edges via EdgeLayoutResolver",
                  result.processedEdges.size() - failedEdges.size());

        return result;
    }

    // Path B: SnapPoints only (optimizer handles bendPoints)
    // Step 1: Resolve snapPoints from snapIndex
    int resolvedCount = 0;
    int failedCount = 0;
    for (EdgeId edgeId : result.processedEdges) {
        auto it = edgeLayouts.find(edgeId);
        if (it == edgeLayouts.end()) continue;

        auto resolveResult = EdgeLayoutResolver::resolveSnapPointsOnly(
            it->second, nodeLayouts, gridSizeToUse, &movedNodes);
        if (resolveResult.success) {
            ++resolvedCount;
        } else {
            ++failedCount;
            LOG_WARN("[SNAP-SYNC] Edge {} failed to resolve snapPoints: {}",
                     edgeId, resolveResult.error);
        }
    }
    LOG_DEBUG("[SNAP-SYNC] Resolved snapPoints only for {} edges ({} failed)",
              resolvedCount, failedCount);

    // Step 2: Regenerate bendPoints via optimizer
    if (edgeOptimizer) {
        std::unordered_set<EdgeId> affectedSet(affectedEdges.begin(), affectedEdges.end());
        std::vector<EdgeId> edgesToRegenerate;

        for (EdgeId edgeId : affectedEdges) {
            edgesToRegenerate.push_back(edgeId);
        }
        for (EdgeId edgeId : result.redistributedEdges) {
            if (affectedSet.find(edgeId) == affectedSet.end()) {
                edgesToRegenerate.push_back(edgeId);
            }
        }

        if (!edgesToRegenerate.empty()) {
            edgeOptimizer->regenerateBendPoints(edgesToRegenerate, edgeLayouts, nodeLayouts, gridSizeToUse);
        }
    }

    // Step 3: Update label positions
    for (EdgeId edgeId : result.processedEdges) {
        auto it = edgeLayouts.find(edgeId);
        if (it != edgeLayouts.end()) {
            it->second.labelPosition = LayoutUtils::calculateEdgeLabelPosition(it->second);
        }
    }

    return result;
}

} // namespace arborvia
