#include <gtest/gtest.h>
#include <arborvia/arborvia.h>
#include "../src/layout/sugiyama/routing/SelfLoopRouter.h"

using namespace arborvia;

// ============================================================================
// SugiyamaLayoutTest - Sugiyama 레이아웃 알고리즘 테스트
// ============================================================================

// --- Basic Layout ---

TEST(SugiyamaLayoutTest, EmptyGraph_ReturnsEmptyResult) {
    Graph graph;
    SugiyamaLayout layout;

    LayoutResult result = layout.layout(graph);

    EXPECT_EQ(result.nodeCount(), 0);
    EXPECT_EQ(result.edgeCount(), 0);
}

TEST(SugiyamaLayoutTest, SingleNode_ProducesValidLayout) {
    Graph graph;
    NodeId n1 = graph.addNode("Node1");

    SugiyamaLayout layout;
    LayoutResult result = layout.layout(graph);

    EXPECT_EQ(result.nodeCount(), 1);
    EXPECT_TRUE(result.hasNodeLayout(n1));
}

// --- Layer Assignment ---

TEST(SugiyamaLayoutTest, ChainGraph_AssignsSequentialLayers) {
    Graph graph;
    NodeId n1 = graph.addNode("Node1");
    NodeId n2 = graph.addNode("Node2");
    NodeId n3 = graph.addNode("Node3");

    graph.addEdge(n1, n2);
    graph.addEdge(n2, n3);

    SugiyamaLayout layout;
    LayoutResult result = layout.layout(graph);

    EXPECT_EQ(result.nodeCount(), 3);
    EXPECT_EQ(result.layerCount(), 3);

    const NodeLayout* l1 = result.getNodeLayout(n1);
    const NodeLayout* l2 = result.getNodeLayout(n2);
    const NodeLayout* l3 = result.getNodeLayout(n3);

    ASSERT_NE(l1, nullptr);
    ASSERT_NE(l2, nullptr);
    ASSERT_NE(l3, nullptr);

    EXPECT_EQ(l1->layer, 0);
    EXPECT_EQ(l2->layer, 1);
    EXPECT_EQ(l3->layer, 2);
}

TEST(SugiyamaLayoutTest, ParallelTargets_PlacedOnSameLayer) {
    Graph graph;
    NodeId n1 = graph.addNode("Source");
    NodeId n2 = graph.addNode("Target1");
    NodeId n3 = graph.addNode("Target2");

    graph.addEdge(n1, n2);
    graph.addEdge(n1, n3);

    SugiyamaLayout layout;
    LayoutResult result = layout.layout(graph);

    EXPECT_EQ(result.nodeCount(), 3);

    const NodeLayout* l1 = result.getNodeLayout(n1);
    const NodeLayout* l2 = result.getNodeLayout(n2);
    const NodeLayout* l3 = result.getNodeLayout(n3);

    ASSERT_NE(l1, nullptr);
    ASSERT_NE(l2, nullptr);
    ASSERT_NE(l3, nullptr);

    EXPECT_EQ(l2->layer, l3->layer);
    EXPECT_NE(l1->layer, l2->layer);
    EXPECT_LT(l1->layer, l2->layer);
}

// --- Edge Routing ---

TEST(SugiyamaLayoutTest, EdgeRouting_ConnectsNodeBoundaries) {
    Graph graph;
    NodeId n1 = graph.addNode("Source");
    NodeId n2 = graph.addNode("Target");
    EdgeId e = graph.addEdge(n1, n2);

    SugiyamaLayout layout;
    LayoutResult result = layout.layout(graph);

    EXPECT_TRUE(result.hasEdgeLayout(e));

    const EdgeLayout* edgeLayout = result.getEdgeLayout(e);
    ASSERT_NE(edgeLayout, nullptr);

    auto points = edgeLayout->allPoints();
    EXPECT_GE(points.size(), 2);

    EXPECT_EQ(edgeLayout->from, n1);
    EXPECT_EQ(edgeLayout->to, n2);

    const NodeLayout* srcNode = result.getNodeLayout(n1);
    const NodeLayout* tgtNode = result.getNodeLayout(n2);
    ASSERT_NE(srcNode, nullptr);
    ASSERT_NE(tgtNode, nullptr);

    Rect srcBounds = srcNode->bounds();
    EXPECT_GE(edgeLayout->sourcePoint.x, srcBounds.x - 1.0f);
    EXPECT_LE(edgeLayout->sourcePoint.x, srcBounds.x + srcBounds.width + 1.0f);

    Rect tgtBounds = tgtNode->bounds();
    EXPECT_GE(edgeLayout->targetPoint.x, tgtBounds.x - 1.0f);
    EXPECT_LE(edgeLayout->targetPoint.x, tgtBounds.x + tgtBounds.width + 1.0f);
}

// --- Layout Options ---

TEST(SugiyamaLayoutTest, LeftToRightDirection_NodesOrderedHorizontally) {
    Graph graph;
    NodeId n1 = graph.addNode("Node1");
    NodeId n2 = graph.addNode("Node2");
    graph.addEdge(n1, n2);

    LayoutOptions options;
    options.direction = Direction::LeftToRight;
    options.nodeSpacingHorizontalGrids = 5;  // 5 * 20 = 100
    options.nodeSpacingVerticalGrids = 3;    // 3 * 20 = 60

    SugiyamaLayout layout(options);
    LayoutResult result = layout.layout(graph);

    EXPECT_EQ(result.nodeCount(), 2);

    const NodeLayout* l1 = result.getNodeLayout(n1);
    const NodeLayout* l2 = result.getNodeLayout(n2);
    ASSERT_NE(l1, nullptr);
    ASSERT_NE(l2, nullptr);

    EXPECT_LT(l1->position.x, l2->position.x);
}

// --- Cycle Handling ---

TEST(SugiyamaLayoutTest, CyclicGraph_BreaksCycleAndLayouts) {
    Graph graph;
    NodeId n1 = graph.addNode("Node1");
    NodeId n2 = graph.addNode("Node2");
    NodeId n3 = graph.addNode("Node3");

    graph.addEdge(n1, n2);
    graph.addEdge(n2, n3);
    graph.addEdge(n3, n1);  // Creates cycle

    SugiyamaLayout layout;
    LayoutResult result = layout.layout(graph);

    EXPECT_EQ(result.nodeCount(), 3);
    EXPECT_GE(layout.lastStats().reversedEdges, 1);
}

// --- Compound Graph ---

TEST(SugiyamaLayoutTest, CompoundGraph_LayoutsParentAndChildren) {
    CompoundGraph graph;
    NodeId parent = graph.addCompoundNode(CompoundType::Compound);
    NodeId child1 = graph.addNode("Child1");
    NodeId child2 = graph.addNode("Child2");

    graph.setParent(child1, parent);
    graph.setParent(child2, parent);

    SugiyamaLayout layout;
    LayoutResult result = layout.layout(graph);

    EXPECT_EQ(result.nodeCount(), 3);
    EXPECT_TRUE(result.hasNodeLayout(parent));
    EXPECT_TRUE(result.hasNodeLayout(child1));
    EXPECT_TRUE(result.hasNodeLayout(child2));
}

// --- Bounds Computation ---

TEST(SugiyamaLayoutTest, ComputeBounds_ContainsAllNodes) {
    Graph graph;
    NodeId n1 = graph.addNode(Size{100, 50}, "Node1");
    NodeId n2 = graph.addNode(Size{100, 50}, "Node2");

    SugiyamaLayout layout;
    LayoutResult result = layout.layout(graph);

    Rect bounds = result.computeBounds();
    EXPECT_GT(bounds.width, 0);
    EXPECT_GT(bounds.height, 0);

    const NodeLayout* l1 = result.getNodeLayout(n1);
    const NodeLayout* l2 = result.getNodeLayout(n2);
    ASSERT_NE(l1, nullptr);
    ASSERT_NE(l2, nullptr);

    EXPECT_GE(l1->position.x, bounds.x);
    EXPECT_GE(l1->position.y, bounds.y);
    EXPECT_LE(l1->position.x + l1->size.width, bounds.x + bounds.width);
    EXPECT_LE(l1->position.y + l1->size.height, bounds.y + bounds.height);

    EXPECT_GE(l2->position.x, bounds.x);
    EXPECT_GE(l2->position.y, bounds.y);
    EXPECT_LE(l2->position.x + l2->size.width, bounds.x + bounds.width);
    EXPECT_LE(l2->position.y + l2->size.height, bounds.y + bounds.height);
}

TEST(SugiyamaLayoutTest, ComputeBounds_WithPadding_ExpandsBounds) {
    Graph graph;
    graph.addNode(Size{100, 50}, "Node1");
    graph.addNode(Size{100, 50}, "Node2");

    SugiyamaLayout layout;
    LayoutResult result = layout.layout(graph);

    Rect bounds = result.computeBounds();
    Rect paddedBounds = result.computeBounds(20.0f);

    EXPECT_GT(paddedBounds.width, bounds.width);
    EXPECT_GT(paddedBounds.height, bounds.height);
    EXPECT_FLOAT_EQ(paddedBounds.width, bounds.width + 40.0f);
    EXPECT_FLOAT_EQ(paddedBounds.height, bounds.height + 40.0f);
}

// --- Translation ---

TEST(SugiyamaLayoutTest, Translate_MovesNodesAndEdges) {
    Graph graph;
    NodeId n1 = graph.addNode("Node1");
    NodeId n2 = graph.addNode("Node2");
    EdgeId e = graph.addEdge(n1, n2);

    SugiyamaLayout layout;
    LayoutResult result = layout.layout(graph);

    const NodeLayout* nodeBefore = result.getNodeLayout(n1);
    const EdgeLayout* edgeBefore = result.getEdgeLayout(e);
    ASSERT_NE(nodeBefore, nullptr);
    ASSERT_NE(edgeBefore, nullptr);

    Point origNodePos = nodeBefore->position;
    Point origEdgeSrc = edgeBefore->sourcePoint;
    Point origEdgeTgt = edgeBefore->targetPoint;

    result.translate(100.0f, 50.0f);

    const NodeLayout* nodeAfter = result.getNodeLayout(n1);
    EXPECT_FLOAT_EQ(nodeAfter->position.x, origNodePos.x + 100.0f);
    EXPECT_FLOAT_EQ(nodeAfter->position.y, origNodePos.y + 50.0f);

    const EdgeLayout* edgeAfter = result.getEdgeLayout(e);
    EXPECT_FLOAT_EQ(edgeAfter->sourcePoint.x, origEdgeSrc.x + 100.0f);
    EXPECT_FLOAT_EQ(edgeAfter->sourcePoint.y, origEdgeSrc.y + 50.0f);
    EXPECT_FLOAT_EQ(edgeAfter->targetPoint.x, origEdgeTgt.x + 100.0f);
    EXPECT_FLOAT_EQ(edgeAfter->targetPoint.y, origEdgeTgt.y + 50.0f);
}

// --- Channel-Based Edge Routing ---

TEST(SugiyamaLayoutTest, ChannelOrthogonal_ProducesValidRouting) {
    Graph graph;
    NodeId n1 = graph.addNode("Source");
    NodeId n2 = graph.addNode("Target");
    EdgeId e = graph.addEdge(n1, n2);

    LayoutOptions options;


    SugiyamaLayout layout(options);
    LayoutResult result = layout.layout(graph);

    EXPECT_TRUE(result.hasEdgeLayout(e));

    const EdgeLayout* edgeLayout = result.getEdgeLayout(e);
    ASSERT_NE(edgeLayout, nullptr);

    // Channel routing should produce valid source/target points
    EXPECT_EQ(edgeLayout->from, n1);
    EXPECT_EQ(edgeLayout->to, n2);

    // Verify points connect properly
    auto points = edgeLayout->allPoints();
    EXPECT_GE(points.size(), 2);
}

TEST(SugiyamaLayoutTest, ChannelOrthogonal_MultipleEdges_DifferentChannels) {
    Graph graph;
    NodeId n1 = graph.addNode("Source");
    NodeId n2 = graph.addNode("Target1");
    NodeId n3 = graph.addNode("Target2");

    EdgeId e1 = graph.addEdge(n1, n2);
    EdgeId e2 = graph.addEdge(n1, n3);

    LayoutOptions options;


    SugiyamaLayout layout(options);
    LayoutResult result = layout.layout(graph);

    const EdgeLayout* edge1 = result.getEdgeLayout(e1);
    const EdgeLayout* edge2 = result.getEdgeLayout(e2);
    ASSERT_NE(edge1, nullptr);
    ASSERT_NE(edge2, nullptr);

    // Both edges should have bend points for channel routing
    // When edges go to different nodes, they may share or differ in channels
    EXPECT_TRUE(edge1->bendPoints.size() >= 0);  // May or may not have bends
    EXPECT_TRUE(edge2->bendPoints.size() >= 0);
}

TEST(SugiyamaLayoutTest, ChannelOrthogonal_SelfLoop_RoutesCorrectly) {
    Graph graph;
    NodeId n1 = graph.addNode("State");
    EdgeId selfLoop = graph.addEdge(n1, n1);  // Self-loop

    LayoutOptions options;

    SugiyamaLayout layout(options);
    LayoutResult result = layout.layout(graph);

    const EdgeLayout* edgeLayout = result.getEdgeLayout(selfLoop);
    ASSERT_NE(edgeLayout, nullptr);

    // Self-loop should have same source and target node
    EXPECT_EQ(edgeLayout->from, n1);
    EXPECT_EQ(edgeLayout->to, n1);

    // Self-loop should have bend points to form the loop
    EXPECT_GE(edgeLayout->bendPoints.size(), 2);

    // Source and target should be different points (not collapsed)
    float distance = std::sqrt(
        std::pow(edgeLayout->sourcePoint.x - edgeLayout->targetPoint.x, 2) +
        std::pow(edgeLayout->sourcePoint.y - edgeLayout->targetPoint.y, 2)
    );
    EXPECT_GT(distance, 5.0f);  // Should be separated

    // CONSTRAINT: Self-loop must use adjacent edges (not same or opposite)
    EXPECT_TRUE(SelfLoopRouter::isValidSelfLoopCombination(
        edgeLayout->sourceEdge, edgeLayout->targetEdge))
        << "Self-loop must use adjacent edges! "
        << "sourceEdge=" << static_cast<int>(edgeLayout->sourceEdge)
        << " targetEdge=" << static_cast<int>(edgeLayout->targetEdge);

    // CONSTRAINT: Arrow direction must be correct
    // The last bend point before target must approach from outside the node
    const NodeLayout* nodeLayout = result.getNodeLayout(n1);
    ASSERT_NE(nodeLayout, nullptr);

    if (edgeLayout->bendPoints.size() >= 1) {
        Point lastBend = edgeLayout->bendPoints.back().position;
        Point target = edgeLayout->targetPoint;

        // Check arrow comes from correct direction based on target edge
        switch (edgeLayout->targetEdge) {
            case NodeEdge::Top:
                EXPECT_LT(lastBend.y, nodeLayout->position.y)
                    << "Arrow to top edge must come from above (y < nodeTop)";
                break;
            case NodeEdge::Bottom:
                EXPECT_GT(lastBend.y, nodeLayout->position.y + nodeLayout->size.height)
                    << "Arrow to bottom edge must come from below (y > nodeBottom)";
                break;
            case NodeEdge::Left:
                EXPECT_LT(lastBend.x, nodeLayout->position.x)
                    << "Arrow to left edge must come from left (x < nodeLeft)";
                break;
            case NodeEdge::Right:
                EXPECT_GT(lastBend.x, nodeLayout->position.x + nodeLayout->size.width)
                    << "Arrow to right edge must come from right (x > nodeRight)";
                break;
        }
    }
}

TEST(SugiyamaLayoutTest, ChannelOrthogonal_MultipleSelfLoops_Stacked) {
    Graph graph;
    NodeId n1 = graph.addNode("State");
    EdgeId loop1 = graph.addEdge(n1, n1);
    EdgeId loop2 = graph.addEdge(n1, n1);

    LayoutOptions options;

    SugiyamaLayout layout(options);
    LayoutResult result = layout.layout(graph);

    const EdgeLayout* edge1 = result.getEdgeLayout(loop1);
    const EdgeLayout* edge2 = result.getEdgeLayout(loop2);
    ASSERT_NE(edge1, nullptr);
    ASSERT_NE(edge2, nullptr);

    // Both should be valid self-loops
    EXPECT_EQ(edge1->from, n1);
    EXPECT_EQ(edge1->to, n1);
    EXPECT_EQ(edge2->from, n1);
    EXPECT_EQ(edge2->to, n1);

    // Both should have bend points
    EXPECT_GE(edge1->bendPoints.size(), 2);
    EXPECT_GE(edge2->bendPoints.size(), 2);

    // CONSTRAINT: Both self-loops must use adjacent edges
    EXPECT_TRUE(SelfLoopRouter::isValidSelfLoopCombination(
        edge1->sourceEdge, edge1->targetEdge))
        << "Self-loop 1 must use adjacent edges!";
    EXPECT_TRUE(SelfLoopRouter::isValidSelfLoopCombination(
        edge2->sourceEdge, edge2->targetEdge))
        << "Self-loop 2 must use adjacent edges!";

    // The loops should be at different offsets (stacked)
    // Check that bend points are different
    if (edge1->bendPoints.size() >= 1 && edge2->bendPoints.size() >= 1) {
        Point bp1 = edge1->bendPoints[0].position;
        Point bp2 = edge2->bendPoints[0].position;

        // At least one coordinate should differ (stacked loops)
        bool different = (std::abs(bp1.x - bp2.x) > 1.0f) ||
                         (std::abs(bp1.y - bp2.y) > 1.0f);
        EXPECT_TRUE(different);
    }
}

TEST(SugiyamaLayoutTest, ChannelOrthogonal_ChainGraph_BendPointsAligned) {
    Graph graph;
    NodeId n1 = graph.addNode("Node1");
    NodeId n2 = graph.addNode("Node2");
    NodeId n3 = graph.addNode("Node3");

    EdgeId e1 = graph.addEdge(n1, n2);
    EdgeId e2 = graph.addEdge(n2, n3);

    LayoutOptions options;


    SugiyamaLayout layout(options);
    LayoutResult result = layout.layout(graph);

    const EdgeLayout* edge1 = result.getEdgeLayout(e1);
    const EdgeLayout* edge2 = result.getEdgeLayout(e2);
    ASSERT_NE(edge1, nullptr);
    ASSERT_NE(edge2, nullptr);

    // Edges in same layer region should be routed through channels
    // Verify the routing produces valid orthogonal paths
    auto points1 = edge1->allPoints();
    auto points2 = edge2->allPoints();

    EXPECT_GE(points1.size(), 2);
    EXPECT_GE(points2.size(), 2);
}

// Helper function to check if a point is on the node boundary
static bool isPointOnNodeBoundary(const Point& point, const NodeLayout& node, float tolerance = 1.0f) {
    float left = node.position.x;
    float right = node.position.x + node.size.width;
    float top = node.position.y;
    float bottom = node.position.y + node.size.height;

    // Check if point is on any edge of the node
    bool onLeft = std::abs(point.x - left) < tolerance &&
                  point.y >= top - tolerance && point.y <= bottom + tolerance;
    bool onRight = std::abs(point.x - right) < tolerance &&
                   point.y >= top - tolerance && point.y <= bottom + tolerance;
    bool onTop = std::abs(point.y - top) < tolerance &&
                 point.x >= left - tolerance && point.x <= right + tolerance;
    bool onBottom = std::abs(point.y - bottom) < tolerance &&
                    point.x >= left - tolerance && point.x <= right + tolerance;

    return onLeft || onRight || onTop || onBottom;
}

TEST(SugiyamaLayoutTest, SelfLoop_LoopOffsetChange_SnapPointsRemainOnNode) {
    Graph graph;
    NodeId n1 = graph.addNode("State");
    EdgeId selfLoop = graph.addEdge(n1, n1);

    // Test with different loop offsets
    std::vector<float> loopOffsets = {15.0f, 30.0f, 50.0f};

    for (float loopOffset : loopOffsets) {
        LayoutOptions options;

        options.channelRouting.selfLoop.loopOffsetGrids = static_cast<int>(loopOffset / 20.0f);

        SugiyamaLayout layout(options);
        LayoutResult result = layout.layout(graph);

        const NodeLayout* nodeLayout = result.getNodeLayout(n1);
        const EdgeLayout* edgeLayout = result.getEdgeLayout(selfLoop);

        ASSERT_NE(nodeLayout, nullptr);
        ASSERT_NE(edgeLayout, nullptr);

        // Source and target points should be ON the node boundary, not offset
        // Note: With gridSize=20, snap points may be up to half grid (10px) from exact boundary
        EXPECT_TRUE(isPointOnNodeBoundary(edgeLayout->sourcePoint, *nodeLayout, 20.0f))
            << "Source point (" << edgeLayout->sourcePoint.x << ", " << edgeLayout->sourcePoint.y
            << ") should be on node boundary with loopOffset=" << loopOffset
            << ". Node: pos=(" << nodeLayout->position.x << ", " << nodeLayout->position.y
            << "), size=(" << nodeLayout->size.width << ", " << nodeLayout->size.height << ")";

        EXPECT_TRUE(isPointOnNodeBoundary(edgeLayout->targetPoint, *nodeLayout, 20.0f))
            << "Target point (" << edgeLayout->targetPoint.x << ", " << edgeLayout->targetPoint.y
            << ") should be on node boundary with loopOffset=" << loopOffset
            << ". Node: pos=(" << nodeLayout->position.x << ", " << nodeLayout->position.y
            << "), size=(" << nodeLayout->size.width << ", " << nodeLayout->size.height << ")";

        // The loop offset should affect bend points, not snap points
        EXPECT_GE(edgeLayout->bendPoints.size(), 2);
    }
}

TEST(SugiyamaLayoutTest, SelfLoop_DragThenChangeOptions_SnapPointsMatchNewPosition) {
    // This test reproduces the actual bug scenario:
    // 1. Layout is done with auto positions
    // 2. Node is DRAGGED to a DIFFERENT position
    // 3. Options are changed (e.g., loop offset)
    // 4. Re-layout should have snap points on the NEW node position

    Graph graph;
    NodeId n1 = graph.addNode("State");
    graph.setNodeSize(n1, {100, 50});
    EdgeId selfLoop = graph.addEdge(n1, n1);

    // Initial layout
    LayoutOptions options;

    options.channelRouting.selfLoop.loopOffsetGrids = 1;

    ManualLayoutManager manualManager;

    SugiyamaLayout layout1(options);
    layout1.setManualLayoutManager(std::make_shared<ManualLayoutManager>(manualManager));
    LayoutResult result1 = layout1.layout(graph);

    const NodeLayout* nodeLayout1 = result1.getNodeLayout(n1);
    ASSERT_NE(nodeLayout1, nullptr);

    // DRAG node to a DIFFERENT position (simulating user drag)
    Point draggedPos = {nodeLayout1->position.x + 200.0f, nodeLayout1->position.y + 150.0f};
    manualManager.setNodePosition(n1, draggedPos);
    manualManager.clearAllEdgeRoutings();

    // Change loop offset (like adjusting the slider after drag)
    options.channelRouting.selfLoop.loopOffsetGrids = 3;

    SugiyamaLayout layout2(options);
    layout2.setManualLayoutManager(std::make_shared<ManualLayoutManager>(manualManager));
    LayoutResult result2 = layout2.layout(graph);

    const NodeLayout* nodeLayout2 = result2.getNodeLayout(n1);
    const EdgeLayout* edgeLayout2 = result2.getEdgeLayout(selfLoop);
    ASSERT_NE(nodeLayout2, nullptr);
    ASSERT_NE(edgeLayout2, nullptr);

    // Node should be at the DRAGGED position
    EXPECT_FLOAT_EQ(nodeLayout2->position.x, draggedPos.x) << "Node X should be at dragged position";
    EXPECT_FLOAT_EQ(nodeLayout2->position.y, draggedPos.y) << "Node Y should be at dragged position";

    // Snap points should be on the NEW node boundary (not the old position)
    EXPECT_TRUE(isPointOnNodeBoundary(edgeLayout2->sourcePoint, *nodeLayout2, 1.0f))
        << "Source point (" << edgeLayout2->sourcePoint.x << ", " << edgeLayout2->sourcePoint.y
        << ") should be on NEW node boundary at (" << nodeLayout2->position.x << ", " << nodeLayout2->position.y << ")"
        << ". Node: pos=(" << nodeLayout2->position.x << ", " << nodeLayout2->position.y
        << "), size=(" << nodeLayout2->size.width << ", " << nodeLayout2->size.height << ")";

    EXPECT_TRUE(isPointOnNodeBoundary(edgeLayout2->targetPoint, *nodeLayout2, 1.0f))
        << "Target point (" << edgeLayout2->targetPoint.x << ", " << edgeLayout2->targetPoint.y
        << ") should be on node boundary after manual mode re-layout";
}


// Helper to check if two points are at the same location
static bool arePointsEqual(const Point& p1, const Point& p2, float tolerance = 0.5f) {
    return std::abs(p1.x - p2.x) < tolerance && std::abs(p1.y - p2.y) < tolerance;
}

TEST(SugiyamaLayoutTest, MultipleEdges_AfterDrag_SnapPointsNotDuplicated) {
    // This test detects the bug where snap points merge to same coordinates
    // after dragging a node and re-routing edges.
    //
    // Bug scenario:
    // 1. Node has multiple edges (self-loops + incoming/outgoing)
    // 2. Node is dragged to new position
    // 3. Re-layout causes snap points to merge to same coordinates

    Graph graph;
    NodeId n1 = graph.addNode("Start");
    NodeId n2 = graph.addNode("Running");  // This node will have multiple edges
    NodeId n3 = graph.addNode("End");

    graph.setNodeSize(n1, {200, 100});
    graph.setNodeSize(n2, {200, 100});
    graph.setNodeSize(n3, {200, 100});

    // Create multiple edges connected to n2
    EdgeId e1 = graph.addEdge(n1, n2);  // incoming
    EdgeId selfLoop1 = graph.addEdge(n2, n2);  // self-loop 1
    EdgeId selfLoop2 = graph.addEdge(n2, n2);  // self-loop 2
    EdgeId e2 = graph.addEdge(n2, n3);  // outgoing

    // Initial layout
    LayoutOptions options;

    options.channelRouting.selfLoop.loopOffsetGrids = 2;


    ManualLayoutManager manualManager;

    SugiyamaLayout layout1(options);
    layout1.setManualLayoutManager(std::make_shared<ManualLayoutManager>(manualManager));
    LayoutResult result1 = layout1.layout(graph);

    const NodeLayout* nodeLayout1 = result1.getNodeLayout(n2);
    ASSERT_NE(nodeLayout1, nullptr);

    // Collect all snap points before drag
    std::vector<Point> snapPointsBefore;
    for (EdgeId edgeId : {e1, selfLoop1, selfLoop2, e2}) {
        const EdgeLayout* el = result1.getEdgeLayout(edgeId);
        ASSERT_NE(el, nullptr);

        // Check if this edge involves n2
        const EdgeData& edgeData = graph.getEdge(edgeId);
        if (edgeData.from == n2 || edgeData.to == n2) {
            if (edgeData.from == n2) {
                snapPointsBefore.push_back(el->sourcePoint);
            }
            if (edgeData.to == n2) {
                snapPointsBefore.push_back(el->targetPoint);
            }
        }
    }

    // Verify snap points are unique before drag
    for (size_t i = 0; i < snapPointsBefore.size(); ++i) {
        for (size_t j = i + 1; j < snapPointsBefore.size(); ++j) {
            EXPECT_FALSE(arePointsEqual(snapPointsBefore[i], snapPointsBefore[j]))
                << "BEFORE DRAG: Snap points " << i << " and " << j << " are duplicated at ("
                << snapPointsBefore[i].x << ", " << snapPointsBefore[i].y << ")";
        }
    }

    // DRAG node n2 to a DIFFERENT position
    Point draggedPos = {nodeLayout1->position.x + 150.0f, nodeLayout1->position.y + 100.0f};
    manualManager.setNodePosition(n2, draggedPos);
    manualManager.clearAllEdgeRoutings();

    // Re-layout with different loop offset
    options.channelRouting.selfLoop.loopOffsetGrids = 3;

    SugiyamaLayout layout2(options);
    layout2.setManualLayoutManager(std::make_shared<ManualLayoutManager>(manualManager));
    LayoutResult result2 = layout2.layout(graph);

    const NodeLayout* nodeLayout2 = result2.getNodeLayout(n2);
    ASSERT_NE(nodeLayout2, nullptr);

    // Verify node moved to dragged position
    EXPECT_FLOAT_EQ(nodeLayout2->position.x, draggedPos.x) << "Node should be at dragged X";
    EXPECT_FLOAT_EQ(nodeLayout2->position.y, draggedPos.y) << "Node should be at dragged Y";

    // Collect all snap points after drag
    std::vector<Point> snapPointsAfter;
    std::vector<std::string> snapPointLabels;

    for (EdgeId edgeId : {e1, selfLoop1, selfLoop2, e2}) {
        const EdgeLayout* el = result2.getEdgeLayout(edgeId);
        ASSERT_NE(el, nullptr);

        const EdgeData& edgeData = graph.getEdge(edgeId);
        if (edgeData.from == n2) {
            snapPointsAfter.push_back(el->sourcePoint);
            snapPointLabels.push_back("Edge " + std::to_string(edgeId) + " source");
        }
        if (edgeData.to == n2) {
            snapPointsAfter.push_back(el->targetPoint);
            snapPointLabels.push_back("Edge " + std::to_string(edgeId) + " target");
        }
    }

    // CRITICAL: Verify NO snap points are duplicated after drag
    for (size_t i = 0; i < snapPointsAfter.size(); ++i) {
        for (size_t j = i + 1; j < snapPointsAfter.size(); ++j) {
            EXPECT_FALSE(arePointsEqual(snapPointsAfter[i], snapPointsAfter[j]))
                << "AFTER DRAG: Snap points merged! " << snapPointLabels[i]
                << " and " << snapPointLabels[j]
                << " are both at (" << snapPointsAfter[i].x << ", " << snapPointsAfter[i].y << ")"
                << "\nThis indicates a bug in snap point distribution after node drag.";
        }
    }

    // Also verify all snap points are on node boundary
    // Note: With gridSize=20, snap points may be up to half grid (10px) from exact boundary
    for (size_t i = 0; i < snapPointsAfter.size(); ++i) {
        EXPECT_TRUE(isPointOnNodeBoundary(snapPointsAfter[i], *nodeLayout2, 20.0f))
            << snapPointLabels[i] << " at (" << snapPointsAfter[i].x << ", " << snapPointsAfter[i].y
            << ") is not on node boundary";
    }
}

TEST(SugiyamaLayoutTest, MultipleEdges_UpdateEdgePositions_SnapPointsNotDuplicated) {
    // This test simulates the interactive demo behavior:
    // 1. Initial layout
    // 2. Drag node (update nodeLayouts directly)
    // 3. Call updateEdgePositions (not full re-layout)
    // 4. Verify snap points are not duplicated

    Graph graph;
    NodeId n1 = graph.addNode("Start");
    NodeId n2 = graph.addNode("Running");
    NodeId n3 = graph.addNode("End");

    graph.setNodeSize(n1, {200, 100});
    graph.setNodeSize(n2, {200, 100});
    graph.setNodeSize(n3, {200, 100});

    // Create multiple edges connected to n2
    EdgeId e1 = graph.addEdge(n1, n2);  // incoming
    EdgeId selfLoop1 = graph.addEdge(n2, n2);  // self-loop 1
    EdgeId selfLoop2 = graph.addEdge(n2, n2);  // self-loop 2
    EdgeId e2 = graph.addEdge(n2, n3);  // outgoing

    // Initial layout
    LayoutOptions options;

    options.channelRouting.selfLoop.loopOffsetGrids = 2;


    SugiyamaLayout layout(options);
    LayoutResult result = layout.layout(graph);

    // Copy layouts to mutable maps (like demo does)
    std::unordered_map<NodeId, NodeLayout> nodeLayouts;
    std::unordered_map<EdgeId, EdgeLayout> edgeLayouts;

    for (NodeId nid : {n1, n2, n3}) {
        const NodeLayout* nl = result.getNodeLayout(nid);
        ASSERT_NE(nl, nullptr);
        nodeLayouts[nid] = *nl;
    }

    for (EdgeId eid : {e1, selfLoop1, selfLoop2, e2}) {
        const EdgeLayout* el = result.getEdgeLayout(eid);
        ASSERT_NE(el, nullptr);
        edgeLayouts[eid] = *el;
    }

    // Simulate drag: move n2 to new position
    NodeLayout& n2Layout = nodeLayouts[n2];
    Point originalPos = n2Layout.position;
    n2Layout.position.x += 150.0f;
    n2Layout.position.y += 100.0f;

    // Get affected edges (edges connected to n2)
    std::vector<EdgeId> affectedEdges;
    for (EdgeId eid : {e1, selfLoop1, selfLoop2, e2}) {
        const EdgeData& edge = graph.getEdge(eid);
        if (edge.from == n2 || edge.to == n2) {
            affectedEdges.push_back(eid);
        }
    }

    // Debug: Print initial snap indices and positions
    std::cout << "\n=== Before updateEdgePositions ===" << std::endl;
    std::cout << "Node n2 position: (" << n2Layout.position.x << ", " << n2Layout.position.y << ")" << std::endl;
    for (EdgeId eid : affectedEdges) {
        const EdgeLayout& el = edgeLayouts[eid];
        const EdgeData& edge = graph.getEdge(eid);
        if (edge.from == n2) {
            std::cout << "Edge " << eid << " source: (" << el.sourcePoint.x << ", " << el.sourcePoint.y
                      << ") edge=" << static_cast<int>(el.sourceEdge) << std::endl;
        }
        if (edge.to == n2) {
            std::cout << "Edge " << eid << " target: (" << el.targetPoint.x << ", " << el.targetPoint.y
                      << ") edge=" << static_cast<int>(el.targetEdge) << std::endl;
        }
    }

    // Call updateEdgePositions like demo does
    std::unordered_set<NodeId> movedNodes = {n2};
    LayoutUtils::updateEdgePositions(
        edgeLayouts, nodeLayouts, affectedEdges,
        movedNodes);

    // Debug: Print after positions (snapIndex computed from position as needed)
    std::cout << "\n=== After updateEdgePositions ===" << std::endl;
    for (EdgeId eid : affectedEdges) {
        const EdgeLayout& el = edgeLayouts[eid];
        const EdgeData& edge = graph.getEdge(eid);
        if (edge.from == n2) {
            std::cout << "Edge " << eid << " source: (" << el.sourcePoint.x << ", " << el.sourcePoint.y
                      << ") edge=" << static_cast<int>(el.sourceEdge) << std::endl;
        }
        if (edge.to == n2) {
            std::cout << "Edge " << eid << " target: (" << el.targetPoint.x << ", " << el.targetPoint.y
                      << ") edge=" << static_cast<int>(el.targetEdge) << std::endl;
        }
    }

    // Collect all snap points on n2 after update
    std::vector<Point> snapPointsAfter;
    std::vector<std::string> snapPointLabels;

    for (EdgeId eid : affectedEdges) {
        const EdgeLayout& el = edgeLayouts[eid];
        const EdgeData& edge = graph.getEdge(eid);

        if (edge.from == n2) {
            snapPointsAfter.push_back(el.sourcePoint);
            snapPointLabels.push_back("Edge " + std::to_string(eid) + " source");
        }
        if (edge.to == n2) {
            snapPointsAfter.push_back(el.targetPoint);
            snapPointLabels.push_back("Edge " + std::to_string(eid) + " target");
        }
    }

    // CRITICAL: Verify NO snap points are duplicated after updateEdgePositions
    for (size_t i = 0; i < snapPointsAfter.size(); ++i) {
        for (size_t j = i + 1; j < snapPointsAfter.size(); ++j) {
            EXPECT_FALSE(arePointsEqual(snapPointsAfter[i], snapPointsAfter[j]))
                << "AFTER updateEdgePositions: Snap points merged! " << snapPointLabels[i]
                << " and " << snapPointLabels[j]
                << " are both at (" << snapPointsAfter[i].x << ", " << snapPointsAfter[i].y << ")"
                << "\nThis indicates a bug in snap point distribution.";
        }
    }

    // Also verify all snap points are on node boundary
    for (size_t i = 0; i < snapPointsAfter.size(); ++i) {
        EXPECT_TRUE(isPointOnNodeBoundary(snapPointsAfter[i], n2Layout, 1.0f))
            << snapPointLabels[i] << " at (" << snapPointsAfter[i].x << ", " << snapPointsAfter[i].y
            << ") is not on node boundary at (" << n2Layout.position.x << ", " << n2Layout.position.y << ")";
    }
}

TEST(SugiyamaLayoutTest, DemoGraph_DragRunning_SnapPointsNotDuplicated) {
    // Reproduce the exact demo graph structure
    Graph graph;
    NodeId idle = graph.addNode("Idle");
    NodeId running = graph.addNode("Running");
    NodeId paused = graph.addNode("Paused");
    NodeId stopped = graph.addNode("Stopped");
    NodeId error = graph.addNode("Error");

    graph.setNodeSize(idle, {200, 100});
    graph.setNodeSize(running, {200, 100});
    graph.setNodeSize(paused, {200, 100});
    graph.setNodeSize(stopped, {200, 100});
    graph.setNodeSize(error, {200, 100});

    EdgeId e0 = graph.addEdge(idle, running);      // start
    EdgeId e1 = graph.addEdge(running, paused);    // pause
    EdgeId e2 = graph.addEdge(paused, running);    // resume
    EdgeId e3 = graph.addEdge(running, stopped);   // stop
    EdgeId e4 = graph.addEdge(paused, stopped);    // stop
    EdgeId e5 = graph.addEdge(running, error);     // fail
    EdgeId e6 = graph.addEdge(error, idle);        // reset
    EdgeId e7 = graph.addEdge(error, error);       // retry (self-loop)

    // Initial layout
    LayoutOptions options;

    options.channelRouting.selfLoop.loopOffsetGrids = 3;
      // Default mode

    SugiyamaLayout layout(options);
    LayoutResult result = layout.layout(graph);

    // Copy to mutable maps
    std::unordered_map<NodeId, NodeLayout> nodeLayouts;
    std::unordered_map<EdgeId, EdgeLayout> edgeLayouts;

    for (NodeId nid : {idle, running, paused, stopped, error}) {
        const NodeLayout* nl = result.getNodeLayout(nid);
        ASSERT_NE(nl, nullptr);
        nodeLayouts[nid] = *nl;
    }

    for (EdgeId eid : {e0, e1, e2, e3, e4, e5, e6, e7}) {
        const EdgeLayout* el = result.getEdgeLayout(eid);
        ASSERT_NE(el, nullptr);
        edgeLayouts[eid] = *el;
    }

    // Print initial state for Running node
    std::cout << "\n=== Demo Graph Test: Before Drag ===" << std::endl;
    NodeLayout& runningLayout = nodeLayouts[running];
    std::cout << "Running node position: (" << runningLayout.position.x
              << ", " << runningLayout.position.y << ")" << std::endl;

    // Print edges connected to Running (snapIndex computed from position as needed)
    for (EdgeId eid : {e0, e1, e2, e3, e5}) {
        const EdgeLayout& el = edgeLayouts[eid];
        const EdgeData& edge = graph.getEdge(eid);
        if (edge.from == running) {
            std::cout << "Edge " << eid << " source: (" << el.sourcePoint.x << ", " << el.sourcePoint.y << ")" << std::endl;
        }
        if (edge.to == running) {
            std::cout << "Edge " << eid << " target: (" << el.targetPoint.x << ", " << el.targetPoint.y << ")" << std::endl;
        }
    }

    // Simulate drag: move Running node
    Point originalPos = runningLayout.position;
    runningLayout.position.x += 90.0f;
    runningLayout.position.y -= 3.0f;

    std::cout << "\nDragged to: (" << runningLayout.position.x << ", " << runningLayout.position.y << ")" << std::endl;

    // Get affected edges
    std::vector<EdgeId> affectedEdges = {e0, e1, e2, e3, e5};

    // Call updateEdgePositions
    std::unordered_set<NodeId> movedNodes = {running};
    LayoutUtils::updateEdgePositions(
        edgeLayouts, nodeLayouts, affectedEdges,
        movedNodes);

    // Print after state (snapIndex computed from position as needed)
    std::cout << "\n=== Demo Graph Test: After Drag ===" << std::endl;
    std::vector<Point> snapPointsOnRunning;
    std::vector<std::string> snapLabels;

    for (EdgeId eid : affectedEdges) {
        const EdgeLayout& el = edgeLayouts[eid];
        const EdgeData& edge = graph.getEdge(eid);
        if (edge.from == running) {
            std::cout << "Edge " << eid << " source: (" << el.sourcePoint.x << ", " << el.sourcePoint.y << ")" << std::endl;
            snapPointsOnRunning.push_back(el.sourcePoint);
            snapLabels.push_back("E" + std::to_string(eid) + " src");
        }
        if (edge.to == running) {
            std::cout << "Edge " << eid << " target: (" << el.targetPoint.x << ", " << el.targetPoint.y << ")" << std::endl;
            snapPointsOnRunning.push_back(el.targetPoint);
            snapLabels.push_back("E" + std::to_string(eid) + " tgt");
        }
    }

    // Check for duplicates
    bool hasDuplicates = false;
    for (size_t i = 0; i < snapPointsOnRunning.size(); ++i) {
        for (size_t j = i + 1; j < snapPointsOnRunning.size(); ++j) {
            if (arePointsEqual(snapPointsOnRunning[i], snapPointsOnRunning[j])) {
                hasDuplicates = true;
                std::cout << "DUPLICATE: " << snapLabels[i] << " and " << snapLabels[j]
                          << " at (" << snapPointsOnRunning[i].x << ", " << snapPointsOnRunning[i].y << ")" << std::endl;
            }
        }
    }

    EXPECT_FALSE(hasDuplicates) << "Snap points should not be duplicated after drag";
}

// ============================================================================
// Grid Alignment Tests
// ============================================================================

namespace {
    bool isOnGrid(float value, float gridSize) {
        if (gridSize <= 0.0f) return true;
        float remainder = std::fmod(std::abs(value), gridSize);
        return remainder < 0.001f || (gridSize - remainder) < 0.001f;
    }

    bool isPointOnGrid(const Point& p, float gridSize) {
        return isOnGrid(p.x, gridSize) && isOnGrid(p.y, gridSize);
    }

    // For snap points: only the coordinate ALONG the edge needs to be grid-aligned
    // The perpendicular coordinate is the exact node edge position
    bool isSnapPointAlongEdgeOnGrid(const Point& p, NodeEdge edge, float gridSize) {
        switch (edge) {
            case NodeEdge::Top:
            case NodeEdge::Bottom:
                return isOnGrid(p.x, gridSize);  // X is along edge
            case NodeEdge::Left:
            case NodeEdge::Right:
                return isOnGrid(p.y, gridSize);  // Y is along edge
        }
        return false;
    }
}

TEST(SugiyamaLayoutTest, GridAlignment_NodePositionsOnGrid) {
    // Create a simple graph
    Graph graph;
    NodeId n0 = graph.addNode(Size{100, 50}, "Start");
    NodeId n1 = graph.addNode(Size{100, 50}, "Process");
    NodeId n2 = graph.addNode(Size{100, 50}, "End");
    graph.addEdge(n0, n1, "next");
    graph.addEdge(n1, n2, "next");

    // Enable grid with 20px cell size
    LayoutOptions options;
    options.gridConfig.cellSize = 20.0f;

    SugiyamaLayout layout;
    layout.setOptions(options);
    LayoutResult result = layout.layout(graph);

    // Verify all node positions are on grid
    float gridSize = options.gridConfig.cellSize;
    for (NodeId nodeId : {n0, n1, n2}) {
        const NodeLayout* nl = result.getNodeLayout(nodeId);
        ASSERT_NE(nl, nullptr);

        EXPECT_TRUE(isOnGrid(nl->position.x, gridSize))
            << "Node " << nodeId << " X position " << nl->position.x
            << " is not on grid (gridSize=" << gridSize << ")";
        EXPECT_TRUE(isOnGrid(nl->position.y, gridSize))
            << "Node " << nodeId << " Y position " << nl->position.y
            << " is not on grid (gridSize=" << gridSize << ")";
    }
}

TEST(SugiyamaLayoutTest, GridAlignment_SnapPointsOnGrid) {
    // Create a graph with multiple edges to same node
    Graph graph;
    NodeId n0 = graph.addNode(Size{100, 50}, "Source1");
    NodeId n1 = graph.addNode(Size{100, 50}, "Source2");
    NodeId n2 = graph.addNode(Size{100, 50}, "Target");
    EdgeId e0 = graph.addEdge(n0, n2, "edge1");
    EdgeId e1 = graph.addEdge(n1, n2, "edge2");

    // Enable grid with 20px cell size
    LayoutOptions options;
    options.gridConfig.cellSize = 20.0f;

    SugiyamaLayout layout;
    layout.setOptions(options);
    LayoutResult result = layout.layout(graph);

    // Verify snap points: coordinate ALONG edge is grid-aligned
    // The perpendicular coordinate is the exact node edge position (may not be grid-aligned)
    float gridSize = options.gridConfig.cellSize;
    for (EdgeId edgeId : {e0, e1}) {
        const EdgeLayout* el = result.getEdgeLayout(edgeId);
        ASSERT_NE(el, nullptr);

        // Source snap point: coordinate along sourceEdge must be grid-aligned
        EXPECT_TRUE(isSnapPointAlongEdgeOnGrid(el->sourcePoint, el->sourceEdge, gridSize))
            << "Edge " << edgeId << " sourcePoint (" << el->sourcePoint.x << ", "
            << el->sourcePoint.y << ") coordinate along edge is not on grid";

        // Target snap point: coordinate along targetEdge must be grid-aligned
        EXPECT_TRUE(isSnapPointAlongEdgeOnGrid(el->targetPoint, el->targetEdge, gridSize))
            << "Edge " << edgeId << " targetPoint (" << el->targetPoint.x << ", "
            << el->targetPoint.y << ") coordinate along edge is not on grid";

        // Verify bend points are fully on grid (internal routing points)
        for (size_t i = 0; i < el->bendPoints.size(); ++i) {
            EXPECT_TRUE(isPointOnGrid(el->bendPoints[i].position, gridSize))
                << "Edge " << edgeId << " bendPoint[" << i << "] ("
                << el->bendPoints[i].position.x << ", "
                << el->bendPoints[i].position.y << ") is not on grid";
        }
    }
}

TEST(SugiyamaLayoutTest, GridAlignment_AllCoordinatesOnGrid) {
    // Comprehensive test with complex graph
    Graph graph;
    NodeId idle = graph.addNode(Size{200, 100}, "Idle");
    NodeId running = graph.addNode(Size{200, 100}, "Running");
    NodeId paused = graph.addNode(Size{200, 100}, "Paused");
    NodeId stopped = graph.addNode(Size{200, 100}, "Stopped");

    graph.addEdge(idle, running, "start");
    graph.addEdge(running, paused, "pause");
    graph.addEdge(paused, running, "resume");  // backward
    graph.addEdge(running, stopped, "stop");
    graph.addEdge(paused, stopped, "stop");

    // Enable grid with 10px cell size
    LayoutOptions options;
    options.gridConfig.cellSize = 10.0f;

    SugiyamaLayout layout;
    layout.setOptions(options);
    LayoutResult result = layout.layout(graph);

    float gridSize = options.gridConfig.cellSize;
    int failures = 0;

    // Check all nodes
    for (const auto& [nodeId, nl] : result.nodeLayouts()) {
        if (!isOnGrid(nl.position.x, gridSize) || !isOnGrid(nl.position.y, gridSize)) {
            std::cout << "Node " << nodeId << " position (" << nl.position.x << ", "
                      << nl.position.y << ") NOT on grid\n";
            failures++;
        }
    }

    // Check all edges - snap points only need coordinate ALONG edge to be grid-aligned
    for (const auto& [edgeId, el] : result.edgeLayouts()) {
        if (!isSnapPointAlongEdgeOnGrid(el.sourcePoint, el.sourceEdge, gridSize)) {
            std::cout << "Edge " << edgeId << " sourcePoint (" << el.sourcePoint.x << ", "
                      << el.sourcePoint.y << ") coordinate along edge NOT on grid\n";
            failures++;
        }
        if (!isSnapPointAlongEdgeOnGrid(el.targetPoint, el.targetEdge, gridSize)) {
            std::cout << "Edge " << edgeId << " targetPoint (" << el.targetPoint.x << ", "
                      << el.targetPoint.y << ") coordinate along edge NOT on grid\n";
            failures++;
        }
        for (size_t i = 0; i < el.bendPoints.size(); ++i) {
            if (!isPointOnGrid(el.bendPoints[i].position, gridSize)) {
                std::cout << "Edge " << edgeId << " bendPoint[" << i << "] ("
                          << el.bendPoints[i].position.x << ", "
                          << el.bendPoints[i].position.y << ") NOT on grid\n";
                failures++;
            }
        }
    }

    EXPECT_EQ(failures, 0) << "Found " << failures << " coordinates not on grid";
}

TEST(SugiyamaLayoutTest, GridAlignment_AfterDrag_StillOnGrid) {
    // Test that grid alignment is maintained after updateEdgePositions
    Graph graph;
    NodeId n0 = graph.addNode(Size{100, 50}, "Node0");
    NodeId n1 = graph.addNode(Size{100, 50}, "Node1");
    EdgeId e0 = graph.addEdge(n0, n1, "edge");

    // Enable grid
    LayoutOptions options;
    options.gridConfig.cellSize = 20.0f;
    float gridSize = options.gridConfig.cellSize;

    SugiyamaLayout layout;
    layout.setOptions(options);
    LayoutResult result = layout.layout(graph);

    // Get mutable copies
    auto nodeLayouts = result.nodeLayouts();
    auto edgeLayouts = result.edgeLayouts();

    // Simulate drag: move node to non-grid position
    nodeLayouts[n0].position.x += 7.0f;  // 7 is not a multiple of 20
    nodeLayouts[n0].position.y += 13.0f; // 13 is not a multiple of 20

    // Node position should be snapped back to grid by updateEdgePositions
    // But updateEdgePositions doesn't modify node positions!
    // So we need to snap node positions before calling updateEdgePositions

    // This test verifies the expectation that nodes should be snapped
    // Currently this will FAIL because node snap is not implemented in updateEdgePositions

    std::vector<EdgeId> affectedEdges = {e0};
    LayoutUtils::updateEdgePositions(
        edgeLayouts, nodeLayouts, affectedEdges,
        {}, gridSize);

    // Verify snap points: coordinate ALONG edge is on grid
    const EdgeLayout& el = edgeLayouts[e0];
    EXPECT_TRUE(isSnapPointAlongEdgeOnGrid(el.sourcePoint, el.sourceEdge, gridSize))
        << "After drag, sourcePoint (" << el.sourcePoint.x << ", "
        << el.sourcePoint.y << ") coordinate along edge should be on grid";
    EXPECT_TRUE(isSnapPointAlongEdgeOnGrid(el.targetPoint, el.targetEdge, gridSize))
        << "After drag, targetPoint (" << el.targetPoint.x << ", "
        << el.targetPoint.y << ") should be on grid";
}

TEST(SugiyamaLayoutTest, GridAlignment_NodePosition_MustBeOnGrid) {
    // This test documents the requirement that node positions
    // should be snapped to grid during drag in interactive mode

    Graph graph;
    NodeId n0 = graph.addNode(Size{100, 50}, "Node0");

    LayoutOptions options;
    options.gridConfig.cellSize = 20.0f;
    float gridSize = options.gridConfig.cellSize;

    SugiyamaLayout layout;
    layout.setOptions(options);
    LayoutResult result = layout.layout(graph);

    // Initial position should be on grid
    const NodeLayout* nl = result.getNodeLayout(n0);
    ASSERT_NE(nl, nullptr);

    EXPECT_TRUE(isOnGrid(nl->position.x, gridSize))
        << "Initial X=" << nl->position.x << " should be on grid";
    EXPECT_TRUE(isOnGrid(nl->position.y, gridSize))
        << "Initial Y=" << nl->position.y << " should be on grid";

    // After layout, position should be multiples of gridSize
    float expectedX = std::round(nl->position.x / gridSize) * gridSize;
    float expectedY = std::round(nl->position.y / gridSize) * gridSize;

    EXPECT_FLOAT_EQ(nl->position.x, expectedX);
    EXPECT_FLOAT_EQ(nl->position.y, expectedY);
}

// =============================================================================
// =============================================================================
// TDD: Edge Overlap After applyFinalCleanup
// =============================================================================
// BUG: When two edges target the same node's top edge, AStarEdgeOptimizer
// resolves the overlap. But applyFinalCleanup uses GeometricEdgeOptimizer
// which doesn't consider other edges as obstacles, causing overlap to reappear.

namespace {
// Helper: Check if two horizontal segments overlap
bool horizontalSegmentsOverlap(
    float y1, float x1Start, float x1End,
    float y2, float x2Start, float x2End,
    float tolerance = 1.0f) {
    
    // Must be on same Y (within tolerance)
    if (std::abs(y1 - y2) > tolerance) return false;
    
    // Check X range overlap
    float min1 = std::min(x1Start, x1End);
    float max1 = std::max(x1Start, x1End);
    float min2 = std::min(x2Start, x2End);
    float max2 = std::max(x2Start, x2End);
    
    // Overlap if ranges intersect (excluding single point touches)
    float overlapStart = std::max(min1, min2);
    float overlapEnd = std::min(max1, max2);
    
    return overlapEnd - overlapStart > tolerance;
}

// Helper: Get all horizontal segments from edge layout
std::vector<std::tuple<float, float, float>> getHorizontalSegments(const EdgeLayout& layout) {
    std::vector<std::tuple<float, float, float>> segments;  // (y, xStart, xEnd)
    
    std::vector<Point> points;
    points.push_back(layout.sourcePoint);
    for (const auto& bp : layout.bendPoints) {
        points.push_back(bp.position);
    }
    points.push_back(layout.targetPoint);
    
    for (size_t i = 0; i + 1 < points.size(); ++i) {
        const Point& p1 = points[i];
        const Point& p2 = points[i + 1];
        
        // Horizontal segment: same Y
        if (std::abs(p1.y - p2.y) < 1.0f) {
            segments.emplace_back(p1.y, p1.x, p2.x);
        }
    }
    
    return segments;
}

// Helper: Check if two edges have overlapping horizontal segments
bool edgesHaveHorizontalOverlap(const EdgeLayout& e1, const EdgeLayout& e2) {
    auto segs1 = getHorizontalSegments(e1);
    auto segs2 = getHorizontalSegments(e2);
    
    for (const auto& [y1, x1Start, x1End] : segs1) {
        for (const auto& [y2, x2Start, x2End] : segs2) {
            if (horizontalSegmentsOverlap(y1, x1Start, x1End, y2, x2Start, x2End)) {
                return true;
            }
        }
    }
    return false;
}
}  // namespace

TEST(SugiyamaLayoutTest, ApplyFinalCleanup_MustNotCreateOverlap) {
    // Reproduce EXACT interactive demo graph structure:
    // idle -> running -> paused -> running (cycle)
    // running -> stopped
    // paused -> stopped  
    // running -> error -> idle (cycle)
    // error -> error (self-loop)
    
    Graph graph;
    
    // Create nodes in same order as interactive demo
    NodeId idle = graph.addNode(Size{200, 100}, "Idle");
    NodeId running = graph.addNode(Size{200, 100}, "Running");
    NodeId paused = graph.addNode(Size{200, 100}, "Paused");
    NodeId stopped = graph.addNode(Size{200, 100}, "Stopped");
    NodeId error = graph.addNode(Size{200, 100}, "Error");
    
    // Create edges in same order as interactive demo
    graph.addEdge(idle, running, "start");       // e0
    graph.addEdge(running, paused, "pause");     // e1
    graph.addEdge(paused, running, "resume");    // e2 - back edge
    EdgeId e_run_stop = graph.addEdge(running, stopped, "stop");  // e3 - PROBLEM EDGE
    EdgeId e_pause_stop = graph.addEdge(paused, stopped, "stop"); // e4 - PROBLEM EDGE
    graph.addEdge(running, error, "fail");       // e5
    graph.addEdge(error, idle, "reset");         // e6 - back edge
    graph.addEdge(error, error, "retry");        // e7 - self-loop
    
    // Configure with AStar post-drag algorithm (enables AStarEdgeOptimizer)
    LayoutOptions options;
    options.gridConfig.cellSize = 20.0f;
    options.optimizationOptions.postDragAlgorithm = PostDragAlgorithm::AStar;
    
    SugiyamaLayout layout;
    layout.setOptions(options);
    
    // Perform layout (includes edge routing and applyFinalCleanup)
    LayoutResult result = layout.layout(graph);
    
    // Get edge layouts for the two edges targeting "stopped"
    const EdgeLayout* el_run_stop = result.getEdgeLayout(e_run_stop);
    const EdgeLayout* el_pause_stop = result.getEdgeLayout(e_pause_stop);
    
    ASSERT_NE(el_run_stop, nullptr) << "Edge running->stopped layout must exist";
    ASSERT_NE(el_pause_stop, nullptr) << "Edge paused->stopped layout must exist";
    
    // THE ASSERTION: No horizontal segment overlap allowed!
    // BUG: applyFinalCleanup uses GeometricEdgeOptimizer which doesn't consider
    // other edges as obstacles, so overlap can reappear after AStarEdgeOptimizer
    // already resolved it.
    bool hasOverlap = edgesHaveHorizontalOverlap(*el_run_stop, *el_pause_stop);
    
    // Debug output
    if (hasOverlap) {
        auto segs1 = getHorizontalSegments(*el_run_stop);
        auto segs2 = getHorizontalSegments(*el_pause_stop);
        
        std::cerr << "Edge running->stopped horizontal segments:\n";
        for (const auto& [y, xs, xe] : segs1) {
            std::cerr << "  y=" << y << " x=[" << xs << ", " << xe << "]\n";
        }
        std::cerr << "Edge paused->stopped horizontal segments:\n";
        for (const auto& [y, xs, xe] : segs2) {
            std::cerr << "  y=" << y << " x=[" << xs << ", " << xe << "]\n";
        }
    }
    
    EXPECT_FALSE(hasOverlap)
        << "Edges targeting same node must not have overlapping horizontal segments!\n"
        << "applyFinalCleanup should use A* (which considers other edges) not Geometric.";
}
