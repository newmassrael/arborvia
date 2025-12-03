#include "arborvia/layout/SugiyamaLayout.h"
#include "arborvia/layout/ICycleRemoval.h"
#include "arborvia/layout/ILayerAssignment.h"
#include "arborvia/layout/ICrossingMinimization.h"
#include "arborvia/layout/ICoordinateAssignment.h"
#include "arborvia/layout/IPathFinder.h"
#include "sugiyama/CycleRemoval.h"
#include "sugiyama/LayerAssignment.h"
#include "sugiyama/CrossingMinimization.h"
#include "sugiyama/CoordinateAssignment.h"
#include "sugiyama/AStarPathFinder.h"
#include "sugiyama/EdgeRouting.h"

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
    , cycleRemoval_(std::make_shared<algorithms::CycleRemoval>())
    , layerAssignment_(std::make_shared<algorithms::LongestPathLayerAssignment>())
    , crossingMinimization_(std::make_shared<algorithms::BarycenterCrossingMinimization>())
    , coordinateAssignment_(std::make_shared<algorithms::SimpleCoordinateAssignment>())
    , pathFinder_(std::make_shared<algorithms::AStarPathFinder>())
    , state_(std::make_unique<LayoutState>()) {}

SugiyamaLayout::~SugiyamaLayout() = default;

SugiyamaLayout::SugiyamaLayout(SugiyamaLayout&&) noexcept = default;
SugiyamaLayout& SugiyamaLayout::operator=(SugiyamaLayout&&) noexcept = default;

void SugiyamaLayout::setOptions(const LayoutOptions& options) {
    options_ = options;
}

void SugiyamaLayout::setCycleRemoval(std::shared_ptr<algorithms::ICycleRemoval> impl) {
    if (impl) cycleRemoval_ = std::move(impl);
}

void SugiyamaLayout::setLayerAssignment(std::shared_ptr<algorithms::ILayerAssignment> impl) {
    if (impl) layerAssignment_ = std::move(impl);
}

void SugiyamaLayout::setCrossingMinimization(std::shared_ptr<algorithms::ICrossingMinimization> impl) {
    if (impl) crossingMinimization_ = std::move(impl);
}

void SugiyamaLayout::setCoordinateAssignment(std::shared_ptr<algorithms::ICoordinateAssignment> impl) {
    if (impl) coordinateAssignment_ = std::move(impl);
}

void SugiyamaLayout::setPathFinder(std::shared_ptr<algorithms::IPathFinder> impl) {
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
    
    // Phase 5: Edge Routing
    routeEdges();
    
    // Apply manual layout state if in manual mode
    if (manualManager_ && manualManager_->getMode() == LayoutMode::Manual) {
        manualManager_->applyManualState(state_->result, graph);
        
        // Update edge positions to match manual node positions
        updateEdgePositionsAfterManualState();
    }
    
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
        routeEdges();
    }
    
    // Apply manual layout state if in manual mode
    if (manualManager_ && manualManager_->getMode() == LayoutMode::Manual) {
        manualManager_->applyManualState(state_->result, graph);
        
        // Update edge positions to match manual node positions
        updateEdgePositionsAfterManualState();
    }
    
    return state_->result;
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
        *state_->graph, state_->reversedEdges, options_.layerAssignment);

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

            state_->result.setNodeLayout(id, layout);
        }
    }
}

void SugiyamaLayout::routeEdges() {
    // Create EdgeRouting with injected pathfinder if available
    algorithms::EdgeRouting routing(pathFinder_);
    auto result = routing.route(*state_->graph,
                               state_->result.nodeLayouts(),
                               state_->reversedEdges,
                               options_);
    
    // Apply auto snap point distribution if enabled
    if (options_.autoSnapPoints && options_.mode == LayoutMode::Auto) {
        routing.distributeAutoSnapPoints(
            result, state_->result.nodeLayouts(), options_.snapDistribution,
            options_.gridConfig.cellSize);
    }
    
    for (auto& [id, layout] : result.edgeLayouts) {
        state_->result.setEdgeLayout(id, layout);
        
        // Calculate edge length for stats
        auto points = layout.allPoints();
        for (size_t i = 1; i < points.size(); ++i) {
            stats_.totalEdgeLength += points[i].distanceTo(points[i - 1]);
        }
    }
}

void SugiyamaLayout::updateEdgePositionsAfterManualState() {
    // Get all edge IDs
    std::vector<EdgeId> allEdges;
    std::unordered_map<EdgeId, EdgeLayout> edgeLayouts;
    for (const auto& [id, layout] : state_->result.edgeLayouts()) {
        allEdges.push_back(id);
        edgeLayouts[id] = layout;
    }
    
    // Update edge positions based on current (manual) node positions
    algorithms::EdgeRouting routing;
    routing.updateEdgePositions(
        edgeLayouts,
        state_->result.nodeLayouts(),
        allEdges,
        options_.snapDistribution,
        {},  // Empty set = update all nodes
        options_.gridConfig.cellSize
    );
    
    // Apply updated edge layouts back to result
    for (const auto& [id, layout] : edgeLayouts) {
        state_->result.setEdgeLayout(id, layout);
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
        float currentY = options_.compoundPadding;
        float maxWidth = 0.0f;
        
        for (NodeId child : children) {
            const float gridSize = options_.gridConfig.cellSize;
            Point pos = {snapToGrid(options_.compoundPadding, gridSize),
                         snapToGrid(currentY, gridSize)};
            Size size = state_->nodeSizes[child];

            NodeLayout layout;
            layout.id = child;
            layout.position = pos;
            layout.size = size;

            state_->result.setNodeLayout(child, layout);

            currentY += size.height + options_.nodeSpacingVertical;
            maxWidth = std::max(maxWidth, size.width);
        }
        
        // Update compound node size
        state_->nodeSizes[id] = {
            maxWidth + 2 * options_.compoundPadding,
            currentY - options_.nodeSpacingVertical + options_.compoundPadding
        };
    }
}

void SugiyamaLayout::layoutParallelRegions(NodeId id, const CompoundGraph& graph) {
    std::vector<NodeId> children = graph.getChildren(id);
    if (children.empty()) return;
    
    // Layout children side by side (horizontally)
    float currentX = options_.compoundPadding;
    float maxHeight = 0.0f;
    
    for (NodeId child : children) {
        // Recursively layout child if it has children
        if (graph.hasChildren(child)) {
            layoutCompoundNode(child, graph);
        }
        
        Size size = state_->nodeSizes[child];
        const float gridSize = options_.gridConfig.cellSize;
        Point pos = {snapToGrid(currentX, gridSize),
                     snapToGrid(options_.compoundPadding, gridSize)};

        NodeLayout layout;
        layout.id = child;
        layout.position = pos;
        layout.size = size;

        state_->result.setNodeLayout(child, layout);

        currentX += size.width + options_.parallelSpacing;
        maxHeight = std::max(maxHeight, size.height);
    }
    
    // Update parallel node size
    state_->nodeSizes[id] = {
        currentX - options_.parallelSpacing + options_.compoundPadding,
        maxHeight + 2 * options_.compoundPadding
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
    bounds = bounds.expanded(options_.compoundPadding);
    
    state_->nodeSizes[id] = bounds.size();
}

}  // namespace arborvia
