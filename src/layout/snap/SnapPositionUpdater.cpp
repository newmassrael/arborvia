#include "SnapPositionUpdater.h"
#include "../sugiyama/routing/EdgeRoutingUtils.h"
#include "arborvia/core/GeometryUtils.h"
#include "GridSnapCalculator.h"
#include "SnapIndexManager.h"
#include "../sugiyama/routing/SelfLoopRouter.h"
#include "../sugiyama/routing/PathCleanup.h"
#include "arborvia/layout/util/LayoutUtils.h"
#include "arborvia/common/Logger.h"
#include <algorithm>
#include <cmath>
#include <iostream>


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
    const std::unordered_set<NodeId>& movedNodes,
    float effectiveGridSize,
    SnapUpdateResult& result) {

    auto [nodeId, nodeEdge] = key;

    auto shouldUpdateNode = [&movedNodes](NodeId nid) -> bool {
        return movedNodes.empty() || movedNodes.count(nid) > 0;
    };

    auto nodeIt = nodeLayouts.find(nodeId);
    if (nodeIt == nodeLayouts.end()) return;
    const NodeLayout& node = nodeIt->second;
    int candidateCount = GridSnapCalculator::getCandidateCount(node, nodeEdge, effectiveGridSize);

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

    // Check if redistribution is needed by computing indices from positions
    bool needsRedistribution = false;
    std::set<int> usedIndices;
    for (const auto& [edgeId, isSource] : connections) {
        auto it = edgeLayouts.find(edgeId);
        if (it != edgeLayouts.end()) {
            Point snapPoint = isSource ? it->second.sourcePoint : it->second.targetPoint;
            int snapIdx = GridSnapCalculator::getCandidateIndexFromPosition(
                node, nodeEdge, snapPoint, effectiveGridSize);
            if (snapIdx < 0 || snapIdx >= candidateCount) {
                needsRedistribution = true;
                break;
            }
            if (usedIndices.count(snapIdx) > 0) {
                needsRedistribution = true;
                break;
            }
            usedIndices.insert(snapIdx);
        }
    }

    auto allConnections = SnapIndexManager::getConnections(edgeLayouts, nodeId, nodeEdge);
    int totalCount = static_cast<int>(allConnections.incoming.size() + allConnections.outgoing.size());

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
            int idx = GridSnapCalculator::getCandidateIndexFromPosition(
                node, nodeEdge, snapPoint, effectiveGridSize);
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
        int idx = GridSnapCalculator::getCandidateIndexFromPosition(
            node, nodeEdge, snapPoint, effectiveGridSize);
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
            // Compute current index from position
            Point currentSnapPoint = isSource ? layout.sourcePoint : layout.targetPoint;
            candidateIndex = GridSnapCalculator::getCandidateIndexFromPosition(
                node, nodeEdge, currentSnapPoint, effectiveGridSize);
            if (candidateIndex < 0 || candidateIndex >= candidateCount) {
                snapPoint = GridSnapCalculator::calculateSnapPosition(node, nodeEdge, static_cast<int>(connIdx), totalCount, effectiveGridSize, &candidateIndex);
            } else {
                snapPoint = GridSnapCalculator::getPositionFromStoredIndex(node, nodeEdge, candidateIndex, effectiveGridSize);
            }
        }

        // [SNAP-SYNC-DEBUG] Log before update
        Point oldSnapPoint = isSource ? layout.sourcePoint : layout.targetPoint;
        int oldSnapIndex = GridSnapCalculator::getCandidateIndexFromPosition(
            node, nodeEdge, oldSnapPoint, effectiveGridSize);
        LOG_DEBUG("[SNAP-SYNC-DEBUG] Edge {} {} nodeId={} nodeHasMoved={} thisEdgeNeedsNewIndex={} candidateCount={}",
                  edgeId, isSource ? "SOURCE" : "TARGET", nodeId, nodeHasMoved, thisEdgeNeedsNewIndex, candidateCount);
        LOG_DEBUG("[SNAP-SYNC-DEBUG]   node pos=({},{}) size=({},{})",
                  node.position.x, node.position.y, node.size.width, node.size.height);
        LOG_DEBUG("[SNAP-SYNC-DEBUG]   BEFORE: snapPoint=({},{}) computedIndex={}",
                  oldSnapPoint.x, oldSnapPoint.y, oldSnapIndex);
        LOG_DEBUG("[SNAP-SYNC-DEBUG]   CALCULATED: snapPoint=({},{}) candidateIndex={}",
                  snapPoint.x, snapPoint.y, candidateIndex);

        // NOTE: snapIndex is no longer stored - position is the source of truth
        if (!nodeHasMoved) {
            if (thisEdgeNeedsNewIndex) {
                if (isSource) {
                    layout.sourcePoint = snapPoint;
                } else {
                    layout.targetPoint = snapPoint;
                }
                LOG_DEBUG("[SNAP-SYNC-DEBUG]   UPDATED (needsNewIndex): snapPoint=({},{}) candidateIndex={}",
                          snapPoint.x, snapPoint.y, candidateIndex);
            } else {
                LOG_DEBUG("[SNAP-SYNC-DEBUG]   SKIPPED: nodeHasMoved=false and !thisEdgeNeedsNewIndex");
            }
        } else {
            if (isSource) {
                layout.sourcePoint = snapPoint;
            } else {
                layout.targetPoint = snapPoint;
            }
            LOG_DEBUG("[SNAP-SYNC-DEBUG]   UPDATED (nodeHasMoved): snapPoint=({},{}) candidateIndex={}",
                      snapPoint.x, snapPoint.y, candidateIndex);
        }

        if (needsRedistribution) {
            result.redistributedEdges.insert(edgeId);
        }
    }
}

SnapUpdateResult SnapPositionUpdater::updateSnapPositions(
    std::unordered_map<EdgeId, EdgeLayout>& edgeLayouts,
    const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
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

    // Phase 1: Collect affected connections
    auto affectedConnections = collectAffectedConnections(edgeLayouts, affectedEdges);

    // Phase 2: Calculate snap positions
    for (auto& [key, connections] : affectedConnections) {
        calculateSnapPositionsForNodeEdge(key, connections, edgeLayouts, nodeLayouts, movedNodes, gridSizeToUse, result);
    }

    // Phase 3: Merge affected and redistributed edges
    result.processedEdges.insert(affectedEdges.begin(), affectedEdges.end());
    result.processedEdges.insert(result.redistributedEdges.begin(), result.redistributedEdges.end());

    // Check for bidirectional edges
    std::unordered_set<EdgeId> bidirectionalEdges;
    auto edgeMap = buildEdgeMapFromLayouts<EdgeRoutingUtils::PairHash>(edgeLayouts);

    for (EdgeId edgeId : result.processedEdges) {
        auto it = edgeLayouts.find(edgeId);
        if (it == edgeLayouts.end()) continue;

        auto reverseIt = edgeMap.find({it->second.to, it->second.from});
        if (reverseIt != edgeMap.end()) {
            bidirectionalEdges.insert(edgeId);
        }
    }

    // Phase 4: Recalculate bend points
    if (skipBendPointRecalc) {
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

        if (edgeOptimizer && !edgesToRegenerate.empty()) {
            edgeOptimizer->regenerateBendPoints(edgesToRegenerate, edgeLayouts, nodeLayouts, gridSizeToUse);
        }

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
        if (result.processedEdges.find(edgeId) == result.processedEdges.end()) {
            otherEdges[edgeId] = layout;
        }
    }

    // Process each edge - movedNodes constraint is enforced at source level in UnifiedRetryChain
    for (EdgeId edgeId : result.processedEdges) {
        auto it = edgeLayouts.find(edgeId);
        if (it == edgeLayouts.end()) continue;

        // 1. A* pathfinding (movedNodes constraint enforced at source level)
        recalcFunc_(it->second, nodeLayouts, gridSizeToUse, &otherEdges, &movedNodes);

        // 2. Diagonal detection and fix
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
            bool srcMoved = movedNodes.empty() || movedNodes.count(it->second.from) > 0;
            bool tgtMoved = movedNodes.empty() || movedNodes.count(it->second.to) > 0;
            if (srcMoved || tgtMoved) {
                detectFixFunc_(edgeId, edgeLayouts, nodeLayouts, movedNodes, gridSizeToUse, otherEdges);
            }
        }

        // 3. PathCleanup
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


        // 4. Direction validation
        bool directionWasFixed = validateFixFunc_(it->second, gridSizeToUse);
        if (directionWasFixed) {
            // Recalculate with movedNodes constraint enforced at source level
            recalcFunc_(it->second, nodeLayouts, gridSizeToUse, &otherEdges, &movedNodes);
        }

        // 5. Update other edges
        otherEdges[edgeId] = it->second;

        // 6. Update label
        it->second.labelPosition = LayoutUtils::calculateEdgeLabelPosition(it->second);
    }

    // Note: endpoint restore removed - movedNodes constraint is now enforced at source level
    // in UnifiedRetryChain, ensuring endpoints are never modified for unmoved nodes.

    // Full re-route handling - delegate to optimizer instead of direct pathfinding
    if (result.needsFullReroute && !result.edgesNeedingReroute.empty() && edgeOptimizer) {
        std::vector<EdgeId> edgesToReroute;
        for (EdgeId edgeId : affectedEdges) {
            if (edgeLayouts.find(edgeId) != edgeLayouts.end()) {
                edgesToReroute.push_back(edgeId);
            }
        }

        if (!edgesToReroute.empty()) {
            // Use optimizer for path regeneration (single source of truth)
            edgeOptimizer->regenerateBendPoints(edgesToReroute, edgeLayouts, nodeLayouts, gridSizeToUse);

            // Update label positions after regeneration
            for (EdgeId edgeId : edgesToReroute) {
                auto it = edgeLayouts.find(edgeId);
                if (it != edgeLayouts.end()) {
                    it->second.labelPosition = LayoutUtils::calculateEdgeLabelPosition(it->second);
                }
            }
        }
    }

    return result;
}

} // namespace arborvia
