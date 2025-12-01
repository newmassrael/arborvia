#include <gtest/gtest.h>
#include <arborvia/arborvia.h>

using namespace arborvia;

// ============================================================================
// LayoutResultTest - 레이아웃 결과 및 직렬화 테스트
// ============================================================================

// --- JSON Serialization ---

TEST(LayoutResultTest, ToJson_ContainsRequiredFields) {
    Graph graph;
    NodeId n1 = graph.addNode("Node1");
    NodeId n2 = graph.addNode("Node2");
    EdgeId e = graph.addEdge(n1, n2, "edge");

    SugiyamaLayout layout;
    LayoutResult original = layout.layout(graph);

    std::string json = original.toJson();
    
    EXPECT_FALSE(json.empty());
    EXPECT_NE(json.find("layerCount"), std::string::npos);
    EXPECT_NE(json.find("nodeLayouts"), std::string::npos);
    EXPECT_NE(json.find("edgeLayouts"), std::string::npos);
}

TEST(LayoutResultTest, FromJson_RestoresNodeLayouts) {
    Graph graph;
    NodeId n1 = graph.addNode("Node1");
    NodeId n2 = graph.addNode("Node2");
    graph.addEdge(n1, n2, "edge");

    SugiyamaLayout layout;
    LayoutResult original = layout.layout(graph);
    
    std::string json = original.toJson();
    LayoutResult restored = LayoutResult::fromJson(json);

    EXPECT_EQ(restored.nodeCount(), original.nodeCount());
    EXPECT_EQ(restored.layerCount(), original.layerCount());

    const NodeLayout* origNode1 = original.getNodeLayout(n1);
    const NodeLayout* restNode1 = restored.getNodeLayout(n1);
    ASSERT_NE(origNode1, nullptr);
    ASSERT_NE(restNode1, nullptr);
    EXPECT_FLOAT_EQ(origNode1->position.x, restNode1->position.x);
    EXPECT_FLOAT_EQ(origNode1->position.y, restNode1->position.y);
    EXPECT_EQ(origNode1->layer, restNode1->layer);
}

TEST(LayoutResultTest, FromJson_RestoresEdgeLayouts) {
    Graph graph;
    NodeId n1 = graph.addNode("Node1");
    NodeId n2 = graph.addNode("Node2");
    EdgeId e = graph.addEdge(n1, n2, "edge");

    SugiyamaLayout layout;
    LayoutResult original = layout.layout(graph);
    
    std::string json = original.toJson();
    LayoutResult restored = LayoutResult::fromJson(json);

    EXPECT_EQ(restored.edgeCount(), original.edgeCount());

    const EdgeLayout* origEdge = original.getEdgeLayout(e);
    const EdgeLayout* restEdge = restored.getEdgeLayout(e);
    ASSERT_NE(origEdge, nullptr);
    ASSERT_NE(restEdge, nullptr);
    EXPECT_EQ(origEdge->from, restEdge->from);
    EXPECT_EQ(origEdge->to, restEdge->to);
}

TEST(LayoutResultTest, JsonRoundTrip_PreservesAllAttributes) {
    Graph graph;
    NodeId n1 = graph.addNode(Size{100, 50}, "Source");
    NodeId n2 = graph.addNode(Size{80, 40}, "Middle");
    NodeId n3 = graph.addNode(Size{120, 60}, "Target");

    EdgeId e1 = graph.addEdge(n1, n2);
    EdgeId e2 = graph.addEdge(n2, n3);

    LayoutOptions options;
    options.direction = Direction::LeftToRight;

    SugiyamaLayout layout(options);
    LayoutResult original = layout.layout(graph);

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

// --- Incremental Layout ---

TEST(LayoutResultTest, IncrementalLayout_FirstCall_ComputesNewResult) {
    Graph graph;
    NodeId n1 = graph.addNode("Node1");
    NodeId n2 = graph.addNode("Node2");
    graph.addEdge(n1, n2);

    SugiyamaLayout layout;

    auto [result, computed] = layout.layoutIncremental(graph);
    
    EXPECT_TRUE(computed);
    EXPECT_EQ(result.nodeCount(), 2);
}

TEST(LayoutResultTest, IncrementalLayout_CleanGraph_UsesCachedResult) {
    Graph graph;
    NodeId n1 = graph.addNode("Node1");
    NodeId n2 = graph.addNode("Node2");
    graph.addEdge(n1, n2);

    SugiyamaLayout layout;

    auto [result1, computed1] = layout.layoutIncremental(graph);
    EXPECT_TRUE(computed1);
    
    Point origPos1 = result1.getNodeLayout(n1)->position;

    auto [result2, computed2] = layout.layoutIncremental(graph);
    
    EXPECT_FALSE(computed2);  // Should use cached
    EXPECT_EQ(result2.nodeCount(), 2);
    
    // Verify cached result has identical content
    EXPECT_FLOAT_EQ(result2.getNodeLayout(n1)->position.x, origPos1.x);
    EXPECT_FLOAT_EQ(result2.getNodeLayout(n1)->position.y, origPos1.y);
}

TEST(LayoutResultTest, IncrementalLayout_ModifiedGraph_RecomputesResult) {
    Graph graph;
    NodeId n1 = graph.addNode("Node1");
    NodeId n2 = graph.addNode("Node2");
    graph.addEdge(n1, n2);

    SugiyamaLayout layout;

    auto [result1, computed1] = layout.layoutIncremental(graph);
    EXPECT_TRUE(computed1);
    EXPECT_EQ(result1.nodeCount(), 2);

    // Modify graph
    NodeId n3 = graph.addNode("Node3");
    EXPECT_TRUE(graph.isDirty());

    auto [result2, computed2] = layout.layoutIncremental(graph);
    
    EXPECT_TRUE(computed2);  // Should recompute
    EXPECT_EQ(result2.nodeCount(), 3);
    EXPECT_FALSE(graph.isDirty());
    
    ASSERT_NE(result2.getNodeLayout(n3), nullptr);
}

// --- Dirty Flag Tracking ---

TEST(LayoutResultTest, DirtyFlag_InitialState_IsClean) {
    Graph graph;
    
    EXPECT_FALSE(graph.isDirty());
    EXPECT_EQ(graph.version(), 0);
}

TEST(LayoutResultTest, DirtyFlag_AfterAddNode_IsDirty) {
    Graph graph;
    
    graph.addNode("Node1");
    
    EXPECT_TRUE(graph.isDirty());
    EXPECT_EQ(graph.version(), 1);
}

TEST(LayoutResultTest, DirtyFlag_AfterMultipleChanges_VersionIncreases) {
    Graph graph;
    
    NodeId n1 = graph.addNode("Node1");
    EXPECT_EQ(graph.version(), 1);
    
    NodeId n2 = graph.addNode("Node2");
    EXPECT_EQ(graph.version(), 2);
    
    graph.addEdge(n1, n2);
    EXPECT_EQ(graph.version(), 3);
}

TEST(LayoutResultTest, DirtyFlag_AfterMarkClean_IsClean) {
    Graph graph;
    NodeId n1 = graph.addNode("Node1");
    
    EXPECT_TRUE(graph.isDirty());
    
    graph.markClean();
    
    EXPECT_FALSE(graph.isDirty());
    EXPECT_EQ(graph.version(), 1);  // Version stays same
}

TEST(LayoutResultTest, DirtyFlag_AfterSetNodeSize_IsDirty) {
    Graph graph;
    NodeId n1 = graph.addNode("Node1");
    graph.markClean();
    
    graph.setNodeSize(n1, Size{200, 100});
    
    EXPECT_TRUE(graph.isDirty());
    EXPECT_EQ(graph.version(), 2);
}
