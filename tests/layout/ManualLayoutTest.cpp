#include <gtest/gtest.h>
#include <arborvia/arborvia.h>
#include <cmath>
#include <vector>

using namespace arborvia;

class ManualLayoutTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Create a simple test graph
        n1_ = graph_.addNode(Size{100, 50}, "Node1");
        n2_ = graph_.addNode(Size{100, 50}, "Node2");
        n3_ = graph_.addNode(Size{100, 50}, "Node3");

        e1_ = graph_.addEdge(n1_, n2_, "edge1");
        e2_ = graph_.addEdge(n1_, n3_, "edge2");
        e3_ = graph_.addEdge(n2_, n3_, "edge3");
    }

    Graph graph_;
    NodeId n1_, n2_, n3_;
    EdgeId e1_, e2_, e3_;
};

// ============== Node Position Tests ==============

TEST_F(ManualLayoutTest, SetNodePosition) {
    ManualLayoutManager manager;

    manager.setNodePosition(n1_, {100.0f, 50.0f});

    EXPECT_TRUE(manager.hasNodePosition(n1_));
    Point pos = manager.getNodePosition(n1_);
    EXPECT_FLOAT_EQ(pos.x, 100.0f);
    EXPECT_FLOAT_EQ(pos.y, 50.0f);
}

TEST_F(ManualLayoutTest, GetNodePositionDefault) {
    ManualLayoutManager manager;

    EXPECT_FALSE(manager.hasNodePosition(n1_));
    Point pos = manager.getNodePosition(n1_);
    EXPECT_FLOAT_EQ(pos.x, 0.0f);
    EXPECT_FLOAT_EQ(pos.y, 0.0f);
}

// ============== Snap Point Config Tests ==============

TEST_F(ManualLayoutTest, SetSnapPointCount) {
    ManualLayoutManager manager;

    manager.setSnapPointCount(n1_, NodeEdge::Top, 3);
    manager.setSnapPointCount(n1_, NodeEdge::Bottom, 2);

    EXPECT_EQ(manager.getSnapPointCount(n1_, NodeEdge::Top), 3);
    EXPECT_EQ(manager.getSnapPointCount(n1_, NodeEdge::Bottom), 2);
    EXPECT_EQ(manager.getSnapPointCount(n1_, NodeEdge::Left), 1);  // Default
    EXPECT_EQ(manager.getSnapPointCount(n1_, NodeEdge::Right), 1); // Default
}

TEST_F(ManualLayoutTest, SetSnapConfig) {
    ManualLayoutManager manager;

    SnapPointConfig config{2, 3, 4, 5};
    manager.setSnapConfig(n1_, config);

    SnapPointConfig result = manager.getSnapConfig(n1_);
    EXPECT_EQ(result.topCount, 2);
    EXPECT_EQ(result.bottomCount, 3);
    EXPECT_EQ(result.leftCount, 4);
    EXPECT_EQ(result.rightCount, 5);
}

TEST_F(ManualLayoutTest, SnapPointConfigGetCount) {
    SnapPointConfig config{1, 2, 3, 4};

    EXPECT_EQ(config.getCount(NodeEdge::Top), 1);
    EXPECT_EQ(config.getCount(NodeEdge::Bottom), 2);
    EXPECT_EQ(config.getCount(NodeEdge::Left), 3);
    EXPECT_EQ(config.getCount(NodeEdge::Right), 4);
}

TEST_F(ManualLayoutTest, SnapPointConfigSetCount) {
    SnapPointConfig config;

    config.setCount(NodeEdge::Top, 5);
    config.setCount(NodeEdge::Bottom, 6);

    EXPECT_EQ(config.topCount, 5);
    EXPECT_EQ(config.bottomCount, 6);
}

// ============== Edge Routing Config Tests ==============

TEST_F(ManualLayoutTest, SetEdgeRouting) {
    ManualLayoutManager manager;

    EdgeRoutingConfig config{NodeEdge::Right, NodeEdge::Left, 1, 2};
    manager.setEdgeRouting(e1_, config);

    EXPECT_TRUE(manager.hasEdgeRouting(e1_));
    EdgeRoutingConfig result = manager.getEdgeRouting(e1_);
    EXPECT_EQ(result.sourceEdge, NodeEdge::Right);
    EXPECT_EQ(result.targetEdge, NodeEdge::Left);
    EXPECT_EQ(result.sourceSnapIndex, 1);
    EXPECT_EQ(result.targetSnapIndex, 2);
}

TEST_F(ManualLayoutTest, SetEdgeSourceEdge) {
    ManualLayoutManager manager;

    manager.setEdgeSourceEdge(e1_, NodeEdge::Left, 2);

    EdgeRoutingConfig result = manager.getEdgeRouting(e1_);
    EXPECT_EQ(result.sourceEdge, NodeEdge::Left);
    EXPECT_EQ(result.sourceSnapIndex, 2);
}

TEST_F(ManualLayoutTest, SetEdgeTargetEdge) {
    ManualLayoutManager manager;

    manager.setEdgeTargetEdge(e1_, NodeEdge::Right, 3);

    EdgeRoutingConfig result = manager.getEdgeRouting(e1_);
    EXPECT_EQ(result.targetEdge, NodeEdge::Right);
    EXPECT_EQ(result.targetSnapIndex, 3);
}

// ============== Calculate Snap Point Tests ==============

TEST_F(ManualLayoutTest, CalculateSnapPointTop) {
    NodeLayout node;
    node.position = {0, 0};
    node.size = {100, 50};

    constexpr float gridSize = 10.0f;

    // Single snap point on top edge
    // 50 stays at 50 (already on 10px grid)
    Point p = LayoutUtils::calculateSnapPoint(node, NodeEdge::Top, 0, 1, gridSize);
    EXPECT_FLOAT_EQ(p.x, 50.0f);  // Center, already on 10px grid
    EXPECT_FLOAT_EQ(p.y, 0.0f);   // Top edge

    // Two snap points on top edge
    // Formula: (snapIndex+1) / (totalSnapPoints+1) * width → grid-aligned
    // With 10px gridSize:
    //   snapIndex 0: 100 * 1/3 = 33.33 → round(33.33/10)*10 = round(3.33)*10 = 30
    //   snapIndex 1: 100 * 2/3 = 66.66 → round(66.66/10)*10 = round(6.67)*10 = 70
    Point p1 = LayoutUtils::calculateSnapPoint(node, NodeEdge::Top, 0, 2, gridSize);
    Point p2 = LayoutUtils::calculateSnapPoint(node, NodeEdge::Top, 1, 2, gridSize);
    EXPECT_FLOAT_EQ(p1.x, 30.0f);  // Grid-aligned (33.33 → 30)
    EXPECT_FLOAT_EQ(p2.x, 70.0f);  // Grid-aligned (66.66 → 70)
}

TEST_F(ManualLayoutTest, CalculateSnapPointBottom) {
    NodeLayout node;
    node.position = {0, 0};
    node.size = {100, 50};

    constexpr float gridSize = 10.0f;

    // With grid quantization, 50 (center) snaps to nearest grid line
    Point p = LayoutUtils::calculateSnapPoint(node, NodeEdge::Bottom, 0, 1, gridSize);
    EXPECT_FLOAT_EQ(p.x, 50.0f);  // Center, exactly on 10px grid
    EXPECT_FLOAT_EQ(p.y, 50.0f);  // Bottom edge (already on grid)
}

TEST_F(ManualLayoutTest, CalculateSnapPointLeft) {
    NodeLayout node;
    node.position = {0, 0};
    node.size = {100, 50};

    constexpr float gridSize = 10.0f;

    // With grid quantization, 25 (center of 50 height) snaps to nearest grid line
    Point p = LayoutUtils::calculateSnapPoint(node, NodeEdge::Left, 0, 1, gridSize);
    EXPECT_FLOAT_EQ(p.x, 0.0f);   // Left edge
    EXPECT_FLOAT_EQ(p.y, 30.0f);  // Center 25 → round(25/10)*10 = 30
}

TEST_F(ManualLayoutTest, CalculateSnapPointRight) {
    NodeLayout node;
    node.position = {0, 0};
    node.size = {100, 50};

    constexpr float gridSize = 10.0f;

    // With grid quantization, values are snapped to grid lines
    Point p = LayoutUtils::calculateSnapPoint(node, NodeEdge::Right, 0, 1, gridSize);
    EXPECT_FLOAT_EQ(p.x, 100.0f); // Right edge (already on grid)
    EXPECT_FLOAT_EQ(p.y, 30.0f);  // Center 25 → round(25/10)*10 = 30
}

// ============== JSON Serialization Tests ==============

TEST_F(ManualLayoutTest, ToJsonBasic) {
    ManualLayoutManager manager;
    manager.setNodePosition(n1_, {100.0f, 50.0f});

    std::string json = manager.toJson();

    EXPECT_NE(json.find("\"version\": 1"), std::string::npos);
    EXPECT_NE(json.find("\"nodePositions\""), std::string::npos);
}

TEST_F(ManualLayoutTest, FromJsonNodePositions) {
    ManualLayoutManager manager;

    std::string json = R"({
        "version": 1,
        "nodePositions": {
            "0": {"x": 100.0, "y": 50.0},
            "1": {"x": 200.0, "y": 150.0}
        },
        "snapConfigs": {},
        "edgeRoutings": {}
    })";

    EXPECT_TRUE(manager.fromJson(json));

    Point p0 = manager.getNodePosition(0);
    EXPECT_FLOAT_EQ(p0.x, 100.0f);
    EXPECT_FLOAT_EQ(p0.y, 50.0f);

    Point p1 = manager.getNodePosition(1);
    EXPECT_FLOAT_EQ(p1.x, 200.0f);
    EXPECT_FLOAT_EQ(p1.y, 150.0f);
}

TEST_F(ManualLayoutTest, FromJsonSnapConfigs) {
    ManualLayoutManager manager;

    std::string json = R"({
        "version": 1,
        "mode": "manual",
        "nodePositions": {},
        "snapConfigs": {
            "0": {"top": 2, "bottom": 3, "left": 1, "right": 4}
        },
        "edgeRoutings": {}
    })";

    EXPECT_TRUE(manager.fromJson(json));

    SnapPointConfig config = manager.getSnapConfig(0);
    EXPECT_EQ(config.topCount, 2);
    EXPECT_EQ(config.bottomCount, 3);
    EXPECT_EQ(config.leftCount, 1);
    EXPECT_EQ(config.rightCount, 4);
}

TEST_F(ManualLayoutTest, FromJsonEdgeRoutings) {
    ManualLayoutManager manager;

    std::string json = R"({
        "version": 1,
        "mode": "manual",
        "nodePositions": {},
        "snapConfigs": {},
        "edgeRoutings": {
            "0": {"sourceEdge": "right", "targetEdge": "left", "sourceSnapIndex": 1, "targetSnapIndex": 2}
        }
    })";

    EXPECT_TRUE(manager.fromJson(json));

    EdgeRoutingConfig routing = manager.getEdgeRouting(0);
    EXPECT_EQ(routing.sourceEdge, NodeEdge::Right);
    EXPECT_EQ(routing.targetEdge, NodeEdge::Left);
    EXPECT_EQ(routing.sourceSnapIndex, 1);
    EXPECT_EQ(routing.targetSnapIndex, 2);
}

TEST_F(ManualLayoutTest, JsonRoundTrip) {
    ManualLayoutManager manager1;
    manager1.setNodePosition(n1_, {100.0f, 50.0f});
    manager1.setSnapConfig(n1_, {2, 3, 1, 4});
    manager1.setEdgeRouting(e1_, {NodeEdge::Right, NodeEdge::Left, 1, 2});

    std::string json = manager1.toJson();

    ManualLayoutManager manager2;
    EXPECT_TRUE(manager2.fromJson(json));

    Point pos = manager2.getNodePosition(n1_);
    EXPECT_FLOAT_EQ(pos.x, 100.0f);
    EXPECT_FLOAT_EQ(pos.y, 50.0f);

    SnapPointConfig config = manager2.getSnapConfig(n1_);
    EXPECT_EQ(config.topCount, 2);
    EXPECT_EQ(config.bottomCount, 3);

    EdgeRoutingConfig routing = manager2.getEdgeRouting(e1_);
    EXPECT_EQ(routing.sourceEdge, NodeEdge::Right);
    EXPECT_EQ(routing.targetEdge, NodeEdge::Left);
}

// ============== Clear and State Tests ==============

TEST_F(ManualLayoutTest, ClearManualState) {
    ManualLayoutManager manager;
    manager.setNodePosition(n1_, {100.0f, 50.0f});
    manager.setSnapConfig(n1_, {2, 3, 1, 4});
    manager.setEdgeRouting(e1_, {NodeEdge::Right, NodeEdge::Left, 1, 2});

    manager.clearManualState();

    EXPECT_FALSE(manager.hasNodePosition(n1_));
    EXPECT_FALSE(manager.hasEdgeRouting(e1_));
    EXPECT_TRUE(manager.getManualState().isEmpty());
}

TEST_F(ManualLayoutTest, ManualStateIsEmpty) {
    ManualLayoutManager manager;
    EXPECT_TRUE(manager.getManualState().isEmpty());

    manager.setNodePosition(n1_, {100.0f, 50.0f});
    EXPECT_FALSE(manager.getManualState().isEmpty());
}

// ============== Integration Tests ==============

TEST_F(ManualLayoutTest, SugiyamaLayoutWithManualManager) {
    ManualLayoutManager manager;
    manager.setNodePosition(n1_, {50.0f, 0.0f});
    manager.setNodePosition(n2_, {0.0f, 100.0f});
    manager.setNodePosition(n3_, {100.0f, 100.0f});

    SugiyamaLayout layout;
    layout.setManualLayoutManager(std::make_shared<ManualLayoutManager>(manager));

    LayoutResult result = layout.layout(graph_);

    // Verify manual positions are applied
    const NodeLayout* layout1 = result.getNodeLayout(n1_);
    ASSERT_NE(layout1, nullptr);
    EXPECT_FLOAT_EQ(layout1->position.x, 50.0f);
    EXPECT_FLOAT_EQ(layout1->position.y, 0.0f);
}

TEST_F(ManualLayoutTest, CaptureFromResult) {
    SugiyamaLayout layout;
    LayoutResult result = layout.layout(graph_);

    ManualLayoutManager manager;
    manager.captureFromResult(result);

    // Verify positions were captured
    EXPECT_TRUE(manager.hasNodePosition(n1_));
    EXPECT_TRUE(manager.hasNodePosition(n2_));
    EXPECT_TRUE(manager.hasNodePosition(n3_));
}

TEST_F(ManualLayoutTest, AutoSnapPointDistribution) {
    // Create graph where multiple edges connect to same node edge
    Graph graph;
    auto center = graph.addNode(Size{100, 50}, "Center");
    auto left1 = graph.addNode(Size{100, 50}, "Left1");
    auto left2 = graph.addNode(Size{100, 50}, "Left2");

    graph.addEdge(left1, center, "e1");
    graph.addEdge(left2, center, "e2");

    SugiyamaLayout layout;
    LayoutOptions options;
    options.autoSnapPoints = true;
    layout.setOptions(options);

    LayoutResult result = layout.layout(graph);

    // Both edges should connect to center node
    // Auto distribution should spread them out
    EXPECT_EQ(result.edgeCount(), 2u);
}
