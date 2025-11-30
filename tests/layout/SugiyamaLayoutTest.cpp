#include <gtest/gtest.h>
#include <arborvia/arborvia.h>

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
    options.nodeSpacingHorizontal = 100.0f;
    options.nodeSpacingVertical = 50.0f;

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
