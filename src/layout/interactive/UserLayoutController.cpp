#include "arborvia/layout/interactive/UserLayoutController.h"
#include "arborvia/layout/util/LayoutSerializer.h"
#include "arborvia/layout/util/LayoutUtils.h"
#include "arborvia/layout/config/LayoutTypes.h"
#include "arborvia/core/GeometryUtils.h"
#include "snap/SnapPointCalculator.h"
#include "snap/GridSnapCalculator.h"
#include <fstream>
#include <sstream>
#include <cmath>
#include <map>
#include <unordered_map>

namespace arborvia {

// =============================================================================
// Node Position Management
// =============================================================================

void UserLayoutController::setNodePosition(NodeId id, const Point& position) {
    manualState_.nodePositions[id] = position;
}

Point UserLayoutController::getNodePosition(NodeId id) const {
    auto it = manualState_.nodePositions.find(id);
    if (it != manualState_.nodePositions.end()) {
        return it->second;
    }
    return {0, 0};
}

bool UserLayoutController::hasNodePosition(NodeId id) const {
    return manualState_.hasNodePosition(id);
}

// =============================================================================
// Snap Point Configuration
// =============================================================================

void UserLayoutController::setSnapPointCount(NodeId id, NodeEdge edge, int count) {
    manualState_.snapConfigs[id].setCount(edge, count);
}

int UserLayoutController::getSnapPointCount(NodeId id, NodeEdge edge) const {
    auto it = manualState_.snapConfigs.find(id);
    if (it != manualState_.snapConfigs.end()) {
        return it->second.getCount(edge);
    }
    return 1;  // Default
}

void UserLayoutController::setSnapConfig(NodeId id, const SnapPointConfig& config) {
    manualState_.snapConfigs[id] = config;
}

SnapPointConfig UserLayoutController::getSnapConfig(NodeId id) const {
    auto it = manualState_.snapConfigs.find(id);
    if (it != manualState_.snapConfigs.end()) {
        return it->second;
    }
    return defaultSnapConfig();
}

// =============================================================================
// Edge Routing Configuration
// =============================================================================

void UserLayoutController::setEdgeRouting(EdgeId id, const EdgeRoutingConfig& config) {
    manualState_.edgeRoutings[id] = config;
}

void UserLayoutController::setEdgeSourceEdge(EdgeId id, NodeEdge edge, int snapIndex) {
    auto& routing = manualState_.edgeRoutings[id];
    routing.sourceEdge = edge;
    routing.sourceSnapIndex = snapIndex;
}

void UserLayoutController::setEdgeTargetEdge(EdgeId id, NodeEdge edge, int snapIndex) {
    auto& routing = manualState_.edgeRoutings[id];
    routing.targetEdge = edge;
    routing.targetSnapIndex = snapIndex;
}

EdgeRoutingConfig UserLayoutController::getEdgeRouting(EdgeId id) const {
    auto it = manualState_.edgeRoutings.find(id);
    if (it != manualState_.edgeRoutings.end()) {
        return it->second;
    }
    return EdgeRoutingConfig{};
}

bool UserLayoutController::hasEdgeRouting(EdgeId id) const {
    return manualState_.hasEdgeRouting(id);
}

// =============================================================================
// Bend Point Management
// =============================================================================

void UserLayoutController::addBendPoint(EdgeId edgeId, size_t index, const GridPoint& gridPosition, float gridSize) {
    auto& config = manualState_.edgeRoutings[edgeId];
    BendPoint bp{gridPosition.toPixel(gridSize), false};
    if (index >= config.manualBendPoints.size()) {
        config.manualBendPoints.push_back(bp);
    } else {
        config.manualBendPoints.insert(config.manualBendPoints.begin() + static_cast<std::ptrdiff_t>(index), bp);
    }
}

void UserLayoutController::appendBendPoint(EdgeId edgeId, const GridPoint& gridPosition, float gridSize) {
    auto& config = manualState_.edgeRoutings[edgeId];
    config.manualBendPoints.push_back({gridPosition.toPixel(gridSize), false});
}

void UserLayoutController::removeBendPoint(EdgeId edgeId, size_t index) {
    auto it = manualState_.edgeRoutings.find(edgeId);
    if (it != manualState_.edgeRoutings.end() &&
        index < it->second.manualBendPoints.size()) {
        it->second.manualBendPoints.erase(
            it->second.manualBendPoints.begin() + static_cast<std::ptrdiff_t>(index));
    }
}

void UserLayoutController::moveBendPoint(EdgeId edgeId, size_t index, const GridPoint& gridPosition, float gridSize) {
    auto it = manualState_.edgeRoutings.find(edgeId);
    if (it != manualState_.edgeRoutings.end() &&
        index < it->second.manualBendPoints.size()) {
        it->second.manualBendPoints[index].position = gridPosition.toPixel(gridSize);
    }
}

const std::vector<BendPoint>& UserLayoutController::getBendPoints(EdgeId edgeId) const {
    static const std::vector<BendPoint> empty;
    auto it = manualState_.edgeRoutings.find(edgeId);
    if (it != manualState_.edgeRoutings.end()) {
        return it->second.manualBendPoints;
    }
    return empty;
}

bool UserLayoutController::hasManualBendPoints(EdgeId edgeId) const {
    auto it = manualState_.edgeRoutings.find(edgeId);
    return it != manualState_.edgeRoutings.end() &&
           !it->second.manualBendPoints.empty();
}

void UserLayoutController::clearBendPoints(EdgeId edgeId) {
    auto it = manualState_.edgeRoutings.find(edgeId);
    if (it != manualState_.edgeRoutings.end()) {
        it->second.manualBendPoints.clear();
    }
}

void UserLayoutController::setBendPoints(EdgeId edgeId, const std::vector<GridPoint>& gridPositions, float gridSize) {
    auto& bendPoints = manualState_.edgeRoutings[edgeId].manualBendPoints;
    bendPoints.clear();
    bendPoints.reserve(gridPositions.size());
    for (const auto& gp : gridPositions) {
        bendPoints.push_back({gp.toPixel(gridSize), false});
    }
}

void UserLayoutController::clearAllEdgeRoutings() {
    manualState_.edgeRoutings.clear();
}

// =============================================================================
// Orthogonal Routing Calculations
// =============================================================================

BendPointPairResult UserLayoutController::calculateBendPointPair(
    const EdgeLayout& edgeLayout,
    const std::vector<BendPoint>& existingBendPoints,
    const Point& clickPosition,
    int segmentIndex)
{
    BendPointPairResult result;
    Point bp1, bp2;

    if (existingBendPoints.empty()) {
        // First insertion - calculate relative to source and target only
        Point source = edgeLayout.sourcePoint;
        Point target = edgeLayout.targetPoint;

        float dx = std::abs(target.x - source.x);
        float dy = std::abs(target.y - source.y);

        if (dx > dy) {
            // More horizontal - create vertical step
            bp1 = {clickPosition.x, source.y};
            bp2 = {clickPosition.x, target.y};
        } else {
            // More vertical - create horizontal step
            bp1 = {source.x, clickPosition.y};
            bp2 = {target.x, clickPosition.y};
        }
        result.insertIndex = 0;
    } else {
        // Existing manual bends - use current path structure
        // Build path: source -> [manual bends] -> target
        std::vector<Point> path;
        path.push_back(edgeLayout.sourcePoint);
        for (const auto& bp : existingBendPoints) {
            path.push_back(bp.position);
        }
        path.push_back(edgeLayout.targetPoint);

        // Validate segmentIndex range
        int maxSegmentIndex = static_cast<int>(path.size()) - 2;
        if (segmentIndex < 0 || segmentIndex > maxSegmentIndex) {
            // Invalid segment index - use first segment as fallback
            segmentIndex = 0;
        }

        Point prevPoint = path[segmentIndex];
        Point nextPoint = path[segmentIndex + 1];

        float dx = std::abs(nextPoint.x - prevPoint.x);
        float dy = std::abs(nextPoint.y - prevPoint.y);

        if (dx > dy) {
            // More horizontal segment
            bp1 = {clickPosition.x, prevPoint.y};
            bp2 = {clickPosition.x, nextPoint.y};
            if (std::abs(prevPoint.y - nextPoint.y) < OrthogonalRouting::COLLINEAR_THRESHOLD) {
                bp2 = {clickPosition.x, clickPosition.y};
            }
        } else {
            // More vertical segment
            bp1 = {prevPoint.x, clickPosition.y};
            bp2 = {nextPoint.x, clickPosition.y};
            if (std::abs(prevPoint.x - nextPoint.x) < OrthogonalRouting::COLLINEAR_THRESHOLD) {
                bp2 = {clickPosition.x, clickPosition.y};
            }
        }
        result.insertIndex = static_cast<size_t>(segmentIndex);
    }

    result.first = bp1;
    result.second = bp2;
    return result;
}

OrthogonalDragResult UserLayoutController::calculateOrthogonalDrag(
    const Point& prevPoint,
    const Point& currentPos,
    const Point& nextPoint,
    const Point& dragTarget,
    bool hasNextBend,
    bool isLastBend)
{
    OrthogonalDragResult result;

    // Determine incoming segment direction
    // Using EPSILON to handle edge case where dx == dy (45 degrees)
    // In that case, we default to treating it as horizontal
    float incomingDx = std::abs(currentPos.x - prevPoint.x);
    float incomingDy = std::abs(currentPos.y - prevPoint.y);
    bool incomingHorizontal = (incomingDx > incomingDy + OrthogonalRouting::EPSILON) ||
                               (std::abs(incomingDx - incomingDy) <= OrthogonalRouting::EPSILON);

    if (isLastBend) {
        // For the last bend point, must maintain orthogonality with BOTH prev and target
        float outgoingDx = std::abs(currentPos.x - nextPoint.x);
        float outgoingDy = std::abs(currentPos.y - nextPoint.y);
        bool outgoingHorizontal = (outgoingDx > outgoingDy + OrthogonalRouting::EPSILON) ||
                                   (std::abs(outgoingDx - outgoingDy) <= OrthogonalRouting::EPSILON);

        if (incomingHorizontal && !outgoingHorizontal) {
            // Incoming horizontal, outgoing vertical
            // Keep Y = prev.y (for horizontal) and X = target.x (for vertical)
            result.newCurrentPos = {nextPoint.x, prevPoint.y};
        } else if (!incomingHorizontal && outgoingHorizontal) {
            // Incoming vertical, outgoing horizontal
            // Keep X = prev.x (for vertical) and Y = target.y (for horizontal)
            result.newCurrentPos = {prevPoint.x, nextPoint.y};
        } else if (incomingHorizontal && outgoingHorizontal) {
            // Both horizontal - only change X freely
            result.newCurrentPos = {dragTarget.x, prevPoint.y};
        } else {
            // Both vertical - only change Y freely
            result.newCurrentPos = {prevPoint.x, dragTarget.y};
        }
    } else {
        // Not the last bend - constrain based on incoming direction only
        if (incomingHorizontal) {
            // Incoming is horizontal - keep Y same as prevPoint
            result.newCurrentPos = {dragTarget.x, prevPoint.y};
        } else {
            // Incoming is vertical - keep X same as prevPoint
            result.newCurrentPos = {prevPoint.x, dragTarget.y};
        }
    }

    // Adjust the next bend point to maintain orthogonality on outgoing segment
    if (hasNextBend) {
        if (incomingHorizontal) {
            // Outgoing should be vertical, so next.x = current.x
            result.adjustedNextPos = {result.newCurrentPos.x, nextPoint.y};
        } else {
            // Outgoing should be horizontal, so next.y = current.y
            result.adjustedNextPos = {nextPoint.x, result.newCurrentPos.y};
        }
        result.nextAdjusted = true;
    }

    return result;
}

// =============================================================================
// Apply/Capture Manual State
// =============================================================================

void UserLayoutController::applyManualState(LayoutResult& result, [[maybe_unused]] const Graph& graph, float gridSize) const {
    applyManualNodePositions(result);
    applyManualEdgeRoutings(result, gridSize);
}

void UserLayoutController::applyManualNodePositions(LayoutResult& result) const {
    for (const auto& [nodeId, position] : manualState_.nodePositions) {
        NodeLayout* layout = result.getNodeLayout(nodeId);
        if (layout) {
            layout->position = position;
        }
    }
}

void UserLayoutController::applyManualEdgeRoutings(LayoutResult& result, float gridSize) const {
    for (const auto& [edgeId, routing] : manualState_.edgeRoutings) {
        EdgeLayout* layout = result.getEdgeLayout(edgeId);
        if (!layout) continue;

        const NodeLayout* fromNode = result.getNodeLayout(layout->from);
        const NodeLayout* toNode = result.getNodeLayout(layout->to);
        if (!fromNode || !toNode) continue;

        // Get snap configs
        SnapPointConfig fromConfig = getSnapConfig(layout->from);
        SnapPointConfig toConfig = getSnapConfig(layout->to);

        int sourceSnapCount = fromConfig.getCount(routing.sourceEdge);
        int targetSnapCount = toConfig.getCount(routing.targetEdge);

        // Calculate grid-aligned snap points (A* standard)
        layout->sourcePoint = SnapPointCalculator::calculateFromIndex(
            *fromNode, routing.sourceEdge, routing.sourceSnapIndex, sourceSnapCount, gridSize);
        layout->targetPoint = SnapPointCalculator::calculateFromIndex(
            *toNode, routing.targetEdge, routing.targetSnapIndex, targetSnapCount, gridSize);

        // Store routing info in edge layout
        layout->sourceEdge = routing.sourceEdge;
        layout->targetEdge = routing.targetEdge;
        // NOTE: snapIndex is no longer stored - computed from position as needed

        // Apply bend points - only override if user set manual bends
        // Otherwise preserve optimizer output (do NOT overwrite with midY)
        if (routing.hasManualBendPoints()) {
            layout->bendPoints.clear();
            layout->bendPoints = routing.manualBendPoints;
        }
        // When no manual bends: keep existing bendPoints from optimizer
    }
}

void UserLayoutController::captureFromResult(const LayoutResult& result) {
    // Capture node positions
    for (const auto& [id, layout] : result.nodeLayouts()) {
        manualState_.nodePositions[id] = layout.position;
    }

    // Count snap points needed per node edge
    // Key: (nodeId, edge) -> max snap index + 1
    std::map<std::pair<NodeId, NodeEdge>, int> snapCounts;

    // Capture edge routings and track snap point usage
    // NOTE: snapIndex is computed from position, need to look up node layouts
    for (const auto& [id, layout] : result.edgeLayouts()) {
        EdgeRoutingConfig routing;
        routing.sourceEdge = layout.sourceEdge;
        routing.targetEdge = layout.targetEdge;

        // Compute snap indices from positions
        const NodeLayout* srcNode = result.getNodeLayout(layout.from);
        const NodeLayout* tgtNode = result.getNodeLayout(layout.to);
        float gs = constants::effectiveGridSize(0.0f);  // Use default

        if (srcNode) {
            routing.sourceSnapIndex = GridSnapCalculator::getCandidateIndexFromPosition(
                *srcNode, layout.sourceEdge, layout.sourcePoint, gs);
        }
        if (tgtNode) {
            routing.targetSnapIndex = GridSnapCalculator::getCandidateIndexFromPosition(
                *tgtNode, layout.targetEdge, layout.targetPoint, gs);
        }
        manualState_.edgeRoutings[id] = routing;

        // Track max snap index for each node edge
        auto& srcCount = snapCounts[{layout.from, layout.sourceEdge}];
        srcCount = std::max(srcCount, routing.sourceSnapIndex + 1);

        auto& tgtCount = snapCounts[{layout.to, layout.targetEdge}];
        tgtCount = std::max(tgtCount, routing.targetSnapIndex + 1);
    }

    // Update snap configs based on captured edge routings
    for (const auto& [key, count] : snapCounts) {
        auto [nodeId, edge] = key;
        auto& config = manualState_.snapConfigs[nodeId];
        int currentCount = config.getCount(edge);
        if (count > currentCount) {
            config.setCount(edge, count);
        }
    }
}

void UserLayoutController::clearManualState() {
    manualState_.clear();
}

SnapPointConfig UserLayoutController::defaultSnapConfig() {
    return SnapPointConfig{1, 1, 1, 1};
}

// =============================================================================
// Serialization
// =============================================================================

std::string UserLayoutController::toJson() const {
    return LayoutSerializer::toJson(*this);
}

bool UserLayoutController::fromJson(const std::string& json) {
    return LayoutSerializer::fromJson(*this, json);
}

bool UserLayoutController::saveToFile(const std::string& path) const {
    return LayoutSerializer::saveToFile(*this, path);
}

bool UserLayoutController::loadFromFile(const std::string& path) {
    return LayoutSerializer::loadFromFile(*this, path);
}

}  // namespace arborvia
