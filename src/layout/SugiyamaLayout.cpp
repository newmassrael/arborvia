#include "arborvia/layout/SugiyamaLayout.h"
#include "arborvia/layout/api/ICycleRemoval.h"
#include "arborvia/layout/api/ILayerAssignment.h"
#include "arborvia/layout/api/ICrossingMinimization.h"
#include "arborvia/layout/api/ICoordinateAssignment.h"
#include "arborvia/layout/api/IPathFinder.h"
#include "arborvia/core/GeometryUtils.h"
#include "sugiyama/phases/CycleRemoval.h"
#include "sugiyama/phases/LayerAssignment.h"
#include "sugiyama/phases/CrossingMinimization.h"
#include "sugiyama/phases/CoordinateAssignment.h"
#include "pathfinding/AStarPathFinder.h"
#include "sugiyama/routing/EdgeRouting.h"
#include "sugiyama/routing/PathCleanup.h"
#include "optimization/geometric/GeometricEdgeOptimizer.h"
#include "optimization/astar/AStarEdgeOptimizer.h"

#include <unordered_set>

namespace arborvia {

namespace {
    /// Snap a value to the nearest grid point (returns value unchanged if gridSize <= 0)
    inline float snapToGrid(float value, float gridSize) {
        if (gridSize <= 0.0f) return value;
        return std::round(value / gridSize) * gridSize;
    }
}

struct SugiyamaLayout::LayoutState {
    const Graph* graph = nullptr;
    const CompoundGraph* compoundGraph = nullptr;

    std::unordered_set<EdgeId> reversedEdges;
    std::vector<std::vector<NodeId>> layers;
    std::unordered_map<NodeId, int> nodeLayer;
    std::unordered_map<NodeId, Size> nodeSizes;
    std::unordered_map<NodeId, Point> nodePositions;

    LayoutResult result;
};

SugiyamaLayout::SugiyamaLayout()
    : SugiyamaLayout(LayoutOptions{}) {}

SugiyamaLayout::SugiyamaLayout(const LayoutOptions& options)
    : options_(options)
    , cycleRemoval_(std::make_shared<CycleRemoval>())
    , layerAssignment_(std::make_shared<LongestPathLayerAssignment>())
    , crossingMinimization_(std::make_shared<BarycenterCrossingMinimization>())
    , coordinateAssignment_(std::make_shared<SimpleCoordinateAssignment>())
    , pathFinder_(std::make_shared<AStarPathFinder>())
    , state_(std::make_unique<LayoutState>()) {}

SugiyamaLayout::~SugiyamaLayout() = default;

SugiyamaLayout::SugiyamaLayout(SugiyamaLayout&&) noexcept = default;
SugiyamaLayout& SugiyamaLayout::operator=(SugiyamaLayout&&) noexcept = default;

void SugiyamaLayout::setOptions(const LayoutOptions& options) {
    options_ = options;
}

void SugiyamaLayout::setCycleRemoval(std::shared_ptr<ICycleRemoval> impl) {
    if (impl) cycleRemoval_ = std::move(impl);
}

void SugiyamaLayout::setLayerAssignment(std::shared_ptr<ILayerAssignment> impl) {
    if (impl) layerAssignment_ = std::move(impl);
}

void SugiyamaLayout::setCrossingMinimization(std::shared_ptr<ICrossingMinimization> impl) {
    if (impl) crossingMinimization_ = std::move(impl);
}

void SugiyamaLayout::setCoordinateAssignment(std::shared_ptr<ICoordinateAssignment> impl) {
    if (impl) coordinateAssignment_ = std::move(impl);
}

void SugiyamaLayout::setPathFinder(std::shared_ptr<IPathFinder> impl) {
    if (impl) pathFinder_ = std::move(impl);
}

LayoutResult SugiyamaLayout::layout(const Graph& graph) {
    state_ = std::make_unique<LayoutState>();
    state_->graph = &graph;
    state_->result.clear();
    stats_ = LayoutStats{};

    if (graph.nodeCount() == 0) {
        return state_->result;
    }

    // Build node sizes map
    for (NodeId id : graph.nodes()) {
        state_->nodeSizes[id] = graph.getNode(id).size;
    }

    // Phase 1: Cycle Removal
    removeCycles();

    // Phase 2: Layer Assignment
    assignLayers();

    // Phase 3: Crossing Minimization
    minimizeCrossings();

    // Phase 4: Coordinate Assignment
    assignCoordinates();

    // Apply manual node positions BEFORE edge routing (so edges route to correct positions)
    if (manualManager_) {
        manualManager_->applyManualNodePositions(state_->result);
    }

    // Phase 5: Edge Routing
    routeEdges();

    // Apply manual edge routings (to override auto-routing if set)
    if (manualManager_) {
        manualManager_->applyManualEdgeRoutings(state_->result, options_.gridConfig.cellSize);
    }

    applyFinalCleanup();

    return state_->result;
}

LayoutResult SugiyamaLayout::layout(const CompoundGraph& graph) {
    state_ = std::make_unique<LayoutState>();
    state_->graph = &graph;
    state_->compoundGraph = &graph;
    state_->result.clear();
    stats_ = LayoutStats{};

    if (graph.nodeCount() == 0) {
        return state_->result;
    }

    // Build node sizes map
    for (NodeId id : graph.nodes()) {
        state_->nodeSizes[id] = graph.getNode(id).size;
    }

    // For compound graphs, we need to layout bottom-up
    // 1. First, find all root nodes
    std::vector<NodeId> roots = graph.rootNodes();

    // 2. Layout each root's subtree
    for (NodeId root : roots) {
        layoutCompoundNode(root, graph);
    }

    // 3. Layout the roots as a flat graph
    // Build a subgraph of just root-level nodes
    // For now, treat visible nodes at root level as a flat graph

    // Collect root-level visible nodes and their edges
    std::vector<NodeId> rootLevel;
    for (NodeId id : graph.nodes()) {
        if (graph.isRoot(id) && graph.isVisible(id)) {
            rootLevel.push_back(id);
        }
    }

    if (!rootLevel.empty()) {
        // Simple layout for root nodes
        removeCycles();
        assignLayers();
        minimizeCrossings();
        assignCoordinates();

        // Apply manual node positions BEFORE edge routing
        if (manualManager_) {
            manualManager_->applyManualNodePositions(state_->result);
        }

        routeEdges();
    }

    // Apply manual edge routings (to override auto-routing if set)
    if (manualManager_) {
        manualManager_->applyManualEdgeRoutings(state_->result, options_.gridConfig.cellSize);
    }

    applyFinalCleanup();

    return state_->result;
}

void SugiyamaLayout::applyFinalCleanup() {
    // Helper: apply cleanup operations to an edge layout
    // NOTE: moveBendsOutsideNode was removed because it's a post-processing step
    // that can break constraints (e.g., overlap resolution) that the optimizer
    // carefully satisfied. A* pathfinding already avoids nodes as obstacles,
    // so bends will never be INSIDE nodes. Being ON the boundary is acceptable.
    auto applyCleanup = [this](EdgeLayout& layout) {
        PathCleanup::removeConsecutiveDuplicates(layout);
    };

    std::vector<EdgeId> needsRegeneration;

    // First pass: cleanup and identify edges needing regeneration
    for (const auto& [edgeId, _] : state_->result.edgeLayouts()) {
        if (EdgeLayout* layout = state_->result.getEdgeLayout(edgeId)) {
            if (!PathCleanup::removeEndpointDuplicates(*layout)) {
                needsRegeneration.push_back(edgeId);
                continue;
            }
            applyCleanup(*layout);
        }
    }

    // Second pass: regenerate invalid paths via optimizer
    if (!needsRegeneration.empty()) {
        // Pass ALL edge layouts so A* can see them as obstacles
        // (not just the edges being regenerated)
        std::unordered_map<EdgeId, EdgeLayout> allEdgeLayouts;
        for (const auto& [edgeId, _] : state_->result.edgeLayouts()) {
            if (const EdgeLayout* layout = state_->result.getEdgeLayout(edgeId)) {
                allEdgeLayouts[edgeId] = *layout;
            }
        }

        // Use AStarEdgeOptimizer to consider other edges as obstacles
        // (GeometricEdgeOptimizer would ignore already-routed edges, causing overlaps)
        AStarEdgeOptimizer optimizer;
        float gridSize = constants::effectiveGridSize(options_.gridConfig.cellSize);
        optimizer.regenerateBendPoints(needsRegeneration, allEdgeLayouts, state_->result.nodeLayouts(), gridSize);

        // Update ALL edges from allEdgeLayouts (not just needsRegeneration)
        // Phase 2 (rip-up-and-reroute) may have modified other edges too
        for (const auto& [edgeId, layout] : allEdgeLayouts) {
            state_->result.setEdgeLayout(edgeId, layout);
        }

        // Apply cleanup to all edges
        for (const auto& [edgeId, _] : allEdgeLayouts) {
            if (EdgeLayout* layout = state_->result.getEdgeLayout(edgeId)) {
                applyCleanup(*layout);
            }
        }
    }
}

std::pair<LayoutResult, bool> SugiyamaLayout::layoutIncremental(Graph& graph) {
    // Check if we can use cached result
    if (hasCachedResult(graph.version())) {
        return {cachedResult_, false};  // Return cached, no new computation
    }

    // Compute new layout
    LayoutResult result = layout(graph);

    // Cache the result and mark graph as clean
    cachedVersion_ = graph.version();
    cachedResult_ = result;
    graph.markClean();

    return {result, true};  // New computation was performed
}

std::pair<LayoutResult, bool> SugiyamaLayout::layoutIncremental(CompoundGraph& graph) {
    // Check if we can use cached result
    if (hasCachedResult(graph.version())) {
        return {cachedResult_, false};  // Return cached, no new computation
    }

    // Compute new layout
    LayoutResult result = layout(graph);

    // Cache the result and mark graph as clean
    cachedVersion_ = graph.version();
    cachedResult_ = result;
    graph.markClean();

    return {result, true};  // New computation was performed
}

void SugiyamaLayout::removeCycles() {
    auto result = cycleRemoval_->findEdgesToReverse(*state_->graph);

    state_->reversedEdges.clear();
    for (EdgeId id : result.reversedEdges) {
        state_->reversedEdges.insert(id);
    }

    stats_.reversedEdges = static_cast<int>(result.reversedEdges.size());
}

void SugiyamaLayout::assignLayers() {
    auto result = layerAssignment_->assignLayers(
        *state_->graph, state_->reversedEdges);

    state_->layers = std::move(result.layers);
    state_->nodeLayer = std::move(result.nodeLayer);

    stats_.layerCount = result.layerCount;
    state_->result.setLayerCount(result.layerCount);
}

void SugiyamaLayout::minimizeCrossings() {
    auto result = crossingMinimization_->minimize(*state_->graph,
                                 state_->layers,
                                 state_->reversedEdges,
                                 options_.crossingMinimization,
                                 options_.crossingMinimizationPasses);

    state_->layers = std::move(result.layers);
    stats_.edgeCrossings = result.crossingCount;

    // Update max layer width
    for (const auto& layer : state_->layers) {
        stats_.maxLayerWidth = std::max(stats_.maxLayerWidth,
                                        static_cast<int>(layer.size()));
    }
}

void SugiyamaLayout::assignCoordinates() {
    auto result = coordinateAssignment_->assignWithSizes(*state_->graph,
                                        state_->layers,
                                        state_->nodeSizes,
                                        options_);

    state_->nodePositions = std::move(result.positions);

    // Build node layouts
    for (size_t layerIdx = 0; layerIdx < state_->layers.size(); ++layerIdx) {
        const auto& layer = state_->layers[layerIdx];
        for (size_t order = 0; order < layer.size(); ++order) {
            NodeId id = layer[order];

            NodeLayout layout;
            layout.id = id;
            // Apply grid snap to node position
            const float gridSize = options_.gridConfig.cellSize;
            Point pos = state_->nodePositions[id];
            layout.position = {snapToGrid(pos.x, gridSize), snapToGrid(pos.y, gridSize)};
            layout.size = state_->nodeSizes[id];
            layout.layer = static_cast<int>(layerIdx);
            layout.order = static_cast<int>(order);
            
            // Copy NodeType from NodeData (explicit, not inferred from size)
            layout.nodeType = state_->graph->getNode(id).nodeType;

            state_->result.setNodeLayout(id, layout);
        }
    }
}

void SugiyamaLayout::routeEdges() {
    // Create EdgeRouting with injected pathfinder if available
    EdgeRouting routing(pathFinder_);

    // === Integrated Pipeline for Constraint Satisfaction ===
    // 1. Create initial layouts WITHOUT optimization (snap positions not final yet)
    auto result = routing.route(*state_->graph,
                               state_->result.nodeLayouts(),
                               state_->reversedEdges,
                               options_,
                               true);  // skipOptimization=true

    // 2. Distribute snap points (assigns snapIndex - the Single Source of Truth)
    //    Optimizer will preserve these indices when possible
    if (options_.autoSnapPoints) {
        routing.distributeAutoSnapPoints(
            result, state_->result.nodeLayouts(),
            options_.gridConfig.cellSize);
    }

    // 3. Run optimizer (may change NodeEdge, sets snapIndex=-1 when NodeEdge changes)
    //    When snapIndex is valid, optimizer preserves it and derives positions from it
    routing.optimizeRouting(result, state_->result.nodeLayouts(), options_);

    for (auto& [id, layout] : result.edgeLayouts) {
        state_->result.setEdgeLayout(id, layout);

        // Calculate edge length for stats
        auto points = layout.allPoints();
        for (size_t i = 1; i < points.size(); ++i) {
            stats_.totalEdgeLength += points[i].distanceTo(points[i - 1]);
        }
    }
}

void SugiyamaLayout::layoutCompoundNode(NodeId id, const CompoundGraph& graph) {
    // Skip if not visible or atomic
    if (!graph.isVisible(id)) return;

    std::vector<NodeId> children = graph.getChildren(id);
    if (children.empty()) return;

    // Check if this is a parallel node
    if (graph.isParallel(id)) {
        layoutParallelRegions(id, graph);
    } else {
        // Regular compound node - layout children as a subgraph
        // Recursively layout each child first
        for (NodeId child : children) {
            if (graph.hasChildren(child)) {
                layoutCompoundNode(child, graph);
            }
        }

        // Now layout children of this compound node
        // (This would require creating a subgraph and running layout on it)
        // For MVP, just stack them vertically
        float currentY = options_.compoundPadding();
        float maxWidth = 0.0f;

        for (NodeId child : children) {
            const float gridSize = options_.gridConfig.cellSize;
            Point pos = {snapToGrid(options_.compoundPadding(), gridSize),
                         snapToGrid(currentY, gridSize)};
            Size size = state_->nodeSizes[child];

            NodeLayout layout;
            layout.id = child;
            layout.position = pos;
            layout.size = size;
            layout.nodeType = state_->graph->getNode(child).nodeType;

            state_->result.setNodeLayout(child, layout);

            currentY += size.height + options_.nodeSpacingVertical();
            maxWidth = std::max(maxWidth, size.width);
        }

        // Update compound node size
        state_->nodeSizes[id] = {
            maxWidth + 2 * options_.compoundPadding(),
            currentY - options_.nodeSpacingVertical() + options_.compoundPadding()
        };
    }
}

void SugiyamaLayout::layoutParallelRegions(NodeId id, const CompoundGraph& graph) {
    std::vector<NodeId> children = graph.getChildren(id);
    if (children.empty()) return;

    // Layout children side by side (horizontally)
    float currentX = options_.compoundPadding();
    float maxHeight = 0.0f;

    for (NodeId child : children) {
        // Recursively layout child if it has children
        if (graph.hasChildren(child)) {
            layoutCompoundNode(child, graph);
        }

        Size size = state_->nodeSizes[child];
        const float gridSize = options_.gridConfig.cellSize;
        Point pos = {snapToGrid(currentX, gridSize),
                     snapToGrid(options_.compoundPadding(), gridSize)};

        NodeLayout layout;
        layout.id = child;
        layout.position = pos;
        layout.size = size;
        layout.nodeType = state_->graph->getNode(child).nodeType;

        state_->result.setNodeLayout(child, layout);

        currentX += size.width + options_.parallelSpacing();
        maxHeight = std::max(maxHeight, size.height);
    }

    // Update parallel node size
    state_->nodeSizes[id] = {
        currentX - options_.parallelSpacing() + options_.compoundPadding(),
        maxHeight + 2 * options_.compoundPadding()
    };
}

void SugiyamaLayout::updateCompoundBounds(NodeId id, const CompoundGraph& graph) {
    std::vector<NodeId> children = graph.getChildren(id);
    if (children.empty()) return;

    Rect bounds{0, 0, 0, 0};
    bool first = true;

    for (NodeId child : children) {
        const NodeLayout* childLayout = state_->result.getNodeLayout(child);
        if (childLayout) {
            Rect childBounds = childLayout->bounds();
            if (first) {
                bounds = childBounds;
                first = false;
            } else {
                bounds = bounds.united(childBounds);
            }
        }
    }

    // Add padding
    bounds = bounds.expanded(options_.compoundPadding());

    state_->nodeSizes[id] = bounds.size();
}

}  // namespace arborvia
