#include <gtest/gtest.h>
#include <arborvia/arborvia.h>

using namespace arborvia;

TEST(SugiyamaLayoutTest, EmptyGraph) {
    Graph graph;
    SugiyamaLayout layout;
    
    LayoutResult result = layout.layout(graph);
    
    EXPECT_EQ(result.nodeCount(), 0);
    EXPECT_EQ(result.edgeCount(), 0);
}

TEST(SugiyamaLayoutTest, SingleNode) {
    Graph graph;
    NodeId n1 = graph.addNode("Node1");
    
    SugiyamaLayout layout;
    LayoutResult result = layout.layout(graph);
    
    EXPECT_EQ(result.nodeCount(), 1);
    EXPECT_TRUE(result.hasNodeLayout(n1));
}

TEST(SugiyamaLayoutTest, SimpleChain) {
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
    
    // Check that nodes are in different layers
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

TEST(SugiyamaLayoutTest, ParallelNodes) {
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

    // n2 and n3 should be in the same layer
    EXPECT_EQ(l2->layer, l3->layer);

    // n1 should be in a different (earlier) layer than n2/n3
    EXPECT_NE(l1->layer, l2->layer);
    EXPECT_LT(l1->layer, l2->layer);
}

TEST(SugiyamaLayoutTest, EdgeRouting) {
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
    EXPECT_GE(points.size(), 2);  // At least source and target

    // Verify edge connects the correct nodes
    EXPECT_EQ(edgeLayout->from, n1);
    EXPECT_EQ(edgeLayout->to, n2);

    // Verify source point is near source node and target point is near target node
    const NodeLayout* srcNode = result.getNodeLayout(n1);
    const NodeLayout* tgtNode = result.getNodeLayout(n2);
    ASSERT_NE(srcNode, nullptr);
    ASSERT_NE(tgtNode, nullptr);

    // Source point should be within or on boundary of source node
    Rect srcBounds = srcNode->bounds();
    EXPECT_GE(edgeLayout->sourcePoint.x, srcBounds.x - 1.0f);
    EXPECT_LE(edgeLayout->sourcePoint.x, srcBounds.x + srcBounds.width + 1.0f);

    // Target point should be within or on boundary of target node
    Rect tgtBounds = tgtNode->bounds();
    EXPECT_GE(edgeLayout->targetPoint.x, tgtBounds.x - 1.0f);
    EXPECT_LE(edgeLayout->targetPoint.x, tgtBounds.x + tgtBounds.width + 1.0f);
}

TEST(SugiyamaLayoutTest, LayoutOptions) {
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

    // Verify LeftToRight direction is applied (x increases for successors)
    const NodeLayout* l1 = result.getNodeLayout(n1);
    const NodeLayout* l2 = result.getNodeLayout(n2);
    ASSERT_NE(l1, nullptr);
    ASSERT_NE(l2, nullptr);
    EXPECT_LT(l1->position.x, l2->position.x);  // n1 should be left of n2
}

TEST(SugiyamaLayoutTest, CycleHandling) {
    Graph graph;
    NodeId n1 = graph.addNode("Node1");
    NodeId n2 = graph.addNode("Node2");
    NodeId n3 = graph.addNode("Node3");
    
    graph.addEdge(n1, n2);
    graph.addEdge(n2, n3);
    graph.addEdge(n3, n1);  // Creates cycle
    
    SugiyamaLayout layout;
    LayoutResult result = layout.layout(graph);
    
    // Should still layout successfully
    EXPECT_EQ(result.nodeCount(), 3);
    
    // Check stats for reversed edge
    EXPECT_GE(layout.lastStats().reversedEdges, 1);
}

TEST(SugiyamaLayoutTest, CompoundGraphLayout) {
    CompoundGraph graph;
    NodeId parent = graph.addCompoundNode(CompoundType::Compound);
    NodeId child1 = graph.addNode("Child1");
    NodeId child2 = graph.addNode("Child2");

    graph.setParent(child1, parent);
    graph.setParent(child2, parent);

    SugiyamaLayout layout;
    LayoutResult result = layout.layout(graph);

    // Verify exact count and that specific nodes have layouts
    EXPECT_EQ(result.nodeCount(), 3);  // parent + 2 children
    EXPECT_TRUE(result.hasNodeLayout(parent));
    EXPECT_TRUE(result.hasNodeLayout(child1));
    EXPECT_TRUE(result.hasNodeLayout(child2));
}

TEST(SugiyamaLayoutTest, ComputeBounds) {
    Graph graph;
    NodeId n1 = graph.addNode(Size{100, 50}, "Node1");
    NodeId n2 = graph.addNode(Size{100, 50}, "Node2");

    SugiyamaLayout layout;
    LayoutResult result = layout.layout(graph);

    Rect bounds = result.computeBounds();
    EXPECT_GT(bounds.width, 0);
    EXPECT_GT(bounds.height, 0);

    // Verify bounds actually contain all nodes
    const NodeLayout* l1 = result.getNodeLayout(n1);
    const NodeLayout* l2 = result.getNodeLayout(n2);
    ASSERT_NE(l1, nullptr);
    ASSERT_NE(l2, nullptr);

    // Each node's bounding box should be within computed bounds
    EXPECT_GE(l1->position.x, bounds.x);
    EXPECT_GE(l1->position.y, bounds.y);
    EXPECT_LE(l1->position.x + l1->size.width, bounds.x + bounds.width);
    EXPECT_LE(l1->position.y + l1->size.height, bounds.y + bounds.height);

    EXPECT_GE(l2->position.x, bounds.x);
    EXPECT_GE(l2->position.y, bounds.y);
    EXPECT_LE(l2->position.x + l2->size.width, bounds.x + bounds.width);
    EXPECT_LE(l2->position.y + l2->size.height, bounds.y + bounds.height);

    Rect paddedBounds = result.computeBounds(20.0f);
    EXPECT_GT(paddedBounds.width, bounds.width);
    EXPECT_GT(paddedBounds.height, bounds.height);

    // Padding should add 40 to each dimension (20 on each side)
    EXPECT_FLOAT_EQ(paddedBounds.width, bounds.width + 40.0f);
    EXPECT_FLOAT_EQ(paddedBounds.height, bounds.height + 40.0f);
}

TEST(SugiyamaLayoutTest, Translate) {
    Graph graph;
    NodeId n1 = graph.addNode("Node1");
    NodeId n2 = graph.addNode("Node2");
    EdgeId e = graph.addEdge(n1, n2);

    SugiyamaLayout layout;
    LayoutResult result = layout.layout(graph);

    // Store original positions
    const NodeLayout* nodeBefore = result.getNodeLayout(n1);
    const EdgeLayout* edgeBefore = result.getEdgeLayout(e);
    ASSERT_NE(nodeBefore, nullptr);
    ASSERT_NE(edgeBefore, nullptr);

    Point origNodePos = nodeBefore->position;
    Point origEdgeSrc = edgeBefore->sourcePoint;
    Point origEdgeTgt = edgeBefore->targetPoint;

    result.translate(100.0f, 50.0f);

    // Verify node is translated
    const NodeLayout* nodeAfter = result.getNodeLayout(n1);
    EXPECT_FLOAT_EQ(nodeAfter->position.x, origNodePos.x + 100.0f);
    EXPECT_FLOAT_EQ(nodeAfter->position.y, origNodePos.y + 50.0f);

    // Verify edge is also translated
    const EdgeLayout* edgeAfter = result.getEdgeLayout(e);
    EXPECT_FLOAT_EQ(edgeAfter->sourcePoint.x, origEdgeSrc.x + 100.0f);
    EXPECT_FLOAT_EQ(edgeAfter->sourcePoint.y, origEdgeSrc.y + 50.0f);
    EXPECT_FLOAT_EQ(edgeAfter->targetPoint.x, origEdgeTgt.x + 100.0f);
    EXPECT_FLOAT_EQ(edgeAfter->targetPoint.y, origEdgeTgt.y + 50.0f);
}

// JSON Serialization tests
TEST(LayoutResultTest, JsonSerialization) {
    Graph graph;
    NodeId n1 = graph.addNode("Node1");
    NodeId n2 = graph.addNode("Node2");
    EdgeId e = graph.addEdge(n1, n2, "edge");

    SugiyamaLayout layout;
    LayoutResult original = layout.layout(graph);

    // Serialize to JSON
    std::string json = original.toJson();
    EXPECT_FALSE(json.empty());
    EXPECT_NE(json.find("layerCount"), std::string::npos);
    EXPECT_NE(json.find("nodeLayouts"), std::string::npos);
    EXPECT_NE(json.find("edgeLayouts"), std::string::npos);

    // Deserialize from JSON
    LayoutResult restored = LayoutResult::fromJson(json);

    // Verify restored data matches original
    EXPECT_EQ(restored.nodeCount(), original.nodeCount());
    EXPECT_EQ(restored.edgeCount(), original.edgeCount());
    EXPECT_EQ(restored.layerCount(), original.layerCount());

    // Check node layouts are restored correctly
    const NodeLayout* origNode1 = original.getNodeLayout(n1);
    const NodeLayout* restNode1 = restored.getNodeLayout(n1);
    ASSERT_NE(origNode1, nullptr);
    ASSERT_NE(restNode1, nullptr);
    EXPECT_FLOAT_EQ(origNode1->position.x, restNode1->position.x);
    EXPECT_FLOAT_EQ(origNode1->position.y, restNode1->position.y);
    EXPECT_EQ(origNode1->layer, restNode1->layer);

    // Check edge layouts are restored correctly
    const EdgeLayout* origEdge = original.getEdgeLayout(e);
    const EdgeLayout* restEdge = restored.getEdgeLayout(e);
    ASSERT_NE(origEdge, nullptr);
    ASSERT_NE(restEdge, nullptr);
    EXPECT_EQ(origEdge->from, restEdge->from);
    EXPECT_EQ(origEdge->to, restEdge->to);
}

TEST(LayoutResultTest, IncrementalLayout) {
    Graph graph;
    NodeId n1 = graph.addNode("Node1");
    NodeId n2 = graph.addNode("Node2");
    EdgeId e = graph.addEdge(n1, n2);

    SugiyamaLayout layout;

    // First layout should compute new result
    auto [result1, computed1] = layout.layoutIncremental(graph);
    EXPECT_TRUE(computed1);
    EXPECT_EQ(result1.nodeCount(), 2);

    // Store original positions for comparison
    const NodeLayout* orig1 = result1.getNodeLayout(n1);
    const NodeLayout* orig2 = result1.getNodeLayout(n2);
    ASSERT_NE(orig1, nullptr);
    ASSERT_NE(orig2, nullptr);
    Point origPos1 = orig1->position;
    Point origPos2 = orig2->position;

    // Graph is now clean, second call should use cached result
    auto [result2, computed2] = layout.layoutIncremental(graph);
    EXPECT_FALSE(computed2);  // Should use cached
    EXPECT_EQ(result2.nodeCount(), 2);

    // Verify cached result has identical content to original
    const NodeLayout* cached1 = result2.getNodeLayout(n1);
    const NodeLayout* cached2 = result2.getNodeLayout(n2);
    ASSERT_NE(cached1, nullptr);
    ASSERT_NE(cached2, nullptr);
    EXPECT_FLOAT_EQ(cached1->position.x, origPos1.x);
    EXPECT_FLOAT_EQ(cached1->position.y, origPos1.y);
    EXPECT_FLOAT_EQ(cached2->position.x, origPos2.x);
    EXPECT_FLOAT_EQ(cached2->position.y, origPos2.y);

    // Verify edge layout is also cached correctly
    const EdgeLayout* origEdge = result1.getEdgeLayout(e);
    const EdgeLayout* cachedEdge = result2.getEdgeLayout(e);
    ASSERT_NE(origEdge, nullptr);
    ASSERT_NE(cachedEdge, nullptr);
    EXPECT_FLOAT_EQ(cachedEdge->sourcePoint.x, origEdge->sourcePoint.x);
    EXPECT_FLOAT_EQ(cachedEdge->sourcePoint.y, origEdge->sourcePoint.y);

    // Modify graph - should become dirty
    NodeId n3 = graph.addNode("Node3");
    EXPECT_TRUE(graph.isDirty());

    // Now layout should compute new result
    auto [result3, computed3] = layout.layoutIncremental(graph);
    EXPECT_TRUE(computed3);  // Should recompute
    EXPECT_EQ(result3.nodeCount(), 3);
    EXPECT_FALSE(graph.isDirty());  // Should be clean after layout

    // Verify new node is in the result
    const NodeLayout* new3 = result3.getNodeLayout(n3);
    ASSERT_NE(new3, nullptr);

    // Again should use cached
    auto [result4, computed4] = layout.layoutIncremental(graph);
    EXPECT_FALSE(computed4);
    EXPECT_EQ(result4.nodeCount(), 3);
}

TEST(LayoutResultTest, DirtyFlagTracking) {
    Graph graph;

    // Initial state - not dirty (nothing added yet)
    EXPECT_FALSE(graph.isDirty());
    EXPECT_EQ(graph.version(), 0);

    // Add node - becomes dirty
    NodeId n1 = graph.addNode("Node1");
    EXPECT_TRUE(graph.isDirty());
    EXPECT_EQ(graph.version(), 1);

    // Add edge - stays dirty, version increases
    NodeId n2 = graph.addNode("Node2");
    EXPECT_EQ(graph.version(), 2);
    graph.addEdge(n1, n2);
    EXPECT_EQ(graph.version(), 3);

    // Mark clean
    graph.markClean();
    EXPECT_FALSE(graph.isDirty());
    EXPECT_EQ(graph.version(), 3);  // Version stays same

    // Modify size - becomes dirty again
    graph.setNodeSize(n1, Size{200, 100});
    EXPECT_TRUE(graph.isDirty());
    EXPECT_EQ(graph.version(), 4);
}

TEST(LayoutResultTest, JsonRoundTrip) {
    // Test with more complex graph
    Graph graph;
    NodeId n1 = graph.addNode(Size{100, 50}, "Source");
    NodeId n2 = graph.addNode(Size{80, 40}, "Middle");
    NodeId n3 = graph.addNode(Size{120, 60}, "Target");

    EdgeId e1 = graph.addEdge(n1, n2);
    EdgeId e2 = graph.addEdge(n2, n3);

    LayoutOptions options;
    options.direction = Direction::LeftToRight;
    options.edgeRouting = EdgeRouting::Orthogonal;

    SugiyamaLayout layout(options);
    LayoutResult original = layout.layout(graph);

    // Round-trip
    std::string json = original.toJson();
    LayoutResult restored = LayoutResult::fromJson(json);

    // Verify all nodes preserved with full attributes
    for (NodeId id : {n1, n2, n3}) {
        const NodeLayout* orig = original.getNodeLayout(id);
        const NodeLayout* rest = restored.getNodeLayout(id);
        ASSERT_NE(orig, nullptr);
        ASSERT_NE(rest, nullptr);
        EXPECT_FLOAT_EQ(orig->size.width, rest->size.width);
        EXPECT_FLOAT_EQ(orig->size.height, rest->size.height);
        EXPECT_FLOAT_EQ(orig->position.x, rest->position.x);
        EXPECT_FLOAT_EQ(orig->position.y, rest->position.y);
        EXPECT_EQ(orig->layer, rest->layer);
        EXPECT_EQ(orig->order, rest->order);
    }

    // Verify edges preserved with bendPoints
    for (EdgeId id : {e1, e2}) {
        const EdgeLayout* orig = original.getEdgeLayout(id);
        const EdgeLayout* rest = restored.getEdgeLayout(id);
        ASSERT_NE(orig, nullptr);
        ASSERT_NE(rest, nullptr);
        EXPECT_EQ(orig->from, rest->from);
        EXPECT_EQ(orig->to, rest->to);
        EXPECT_FLOAT_EQ(orig->sourcePoint.x, rest->sourcePoint.x);
        EXPECT_FLOAT_EQ(orig->sourcePoint.y, rest->sourcePoint.y);
        EXPECT_FLOAT_EQ(orig->targetPoint.x, rest->targetPoint.x);
        EXPECT_FLOAT_EQ(orig->targetPoint.y, rest->targetPoint.y);
        EXPECT_EQ(orig->bendPoints.size(), rest->bendPoints.size());
        for (size_t i = 0; i < orig->bendPoints.size(); ++i) {
            EXPECT_FLOAT_EQ(orig->bendPoints[i].position.x, rest->bendPoints[i].position.x);
            EXPECT_FLOAT_EQ(orig->bendPoints[i].position.y, rest->bendPoints[i].position.y);
        }
    }
}

// SVG Export tests (only when export module is enabled)
#ifdef ARBORVIA_ENABLE_SVG_EXPORT
#include <arborvia/export/SvgExport.h>

TEST(SvgExportTest, ExportToString) {
    Graph graph;
    NodeId n1 = graph.addNode("Source");
    NodeId n2 = graph.addNode("Target");
    graph.addEdge(n1, n2, "edge");

    SugiyamaLayout layout;
    LayoutResult result = layout.layout(graph);

    SvgExport svg;
    std::string output = svg.exportToString(graph, result);

    EXPECT_FALSE(output.empty());
    EXPECT_NE(output.find("<svg"), std::string::npos);
    EXPECT_NE(output.find("</svg>"), std::string::npos);
    EXPECT_NE(output.find("<rect"), std::string::npos);

    // Verify all nodes are rendered (should have rect for each node + background)
    size_t rectCount = 0;
    size_t pos = 0;
    while ((pos = output.find("<rect", pos)) != std::string::npos) {
        ++rectCount;
        ++pos;
    }
    EXPECT_EQ(rectCount, 3);  // 1 background + 2 nodes

    // Verify edge is rendered (should have path or line element)
    EXPECT_TRUE(output.find("<path") != std::string::npos ||
                output.find("<line") != std::string::npos ||
                output.find("<polyline") != std::string::npos);
}

TEST(SvgExportTest, ExportCompoundGraph) {
    CompoundGraph graph;
    NodeId parent = graph.addCompoundNode("Parent", CompoundType::Compound);
    NodeId child = graph.addNode("Child");
    graph.setParent(child, parent);
    
    SugiyamaLayout layout;
    LayoutResult result = layout.layout(graph);
    
    SvgExport svg;
    std::string output = svg.exportToString(graph, result);
    
    EXPECT_FALSE(output.empty());
    EXPECT_NE(output.find("compound"), std::string::npos);
}

#endif // ARBORVIA_ENABLE_SVG_EXPORT
