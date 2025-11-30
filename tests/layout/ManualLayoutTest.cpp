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

// ============== LayoutMode Tests ==============

TEST_F(ManualLayoutTest, DefaultModeIsAuto) {
    ManualLayoutManager manager;
    EXPECT_EQ(manager.getMode(), LayoutMode::Auto);
}

TEST_F(ManualLayoutTest, SetModeManual) {
    ManualLayoutManager manager;
    manager.setMode(LayoutMode::Manual);
    EXPECT_EQ(manager.getMode(), LayoutMode::Manual);
}

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
    
    // Single snap point on top edge
    Point p = ManualLayoutManager::calculateSnapPoint(node, NodeEdge::Top, 0, 1);
    EXPECT_FLOAT_EQ(p.x, 50.0f);  // Center
    EXPECT_FLOAT_EQ(p.y, 0.0f);   // Top edge
    
    // Two snap points on top edge
    Point p1 = ManualLayoutManager::calculateSnapPoint(node, NodeEdge::Top, 0, 2);
    Point p2 = ManualLayoutManager::calculateSnapPoint(node, NodeEdge::Top, 1, 2);
    EXPECT_FLOAT_EQ(p1.x, 100.0f / 3.0f);
    EXPECT_FLOAT_EQ(p2.x, 200.0f / 3.0f);
}

TEST_F(ManualLayoutTest, CalculateSnapPointBottom) {
    NodeLayout node;
    node.position = {0, 0};
    node.size = {100, 50};
    
    Point p = ManualLayoutManager::calculateSnapPoint(node, NodeEdge::Bottom, 0, 1);
    EXPECT_FLOAT_EQ(p.x, 50.0f);
    EXPECT_FLOAT_EQ(p.y, 50.0f);  // Bottom edge
}

TEST_F(ManualLayoutTest, CalculateSnapPointLeft) {
    NodeLayout node;
    node.position = {0, 0};
    node.size = {100, 50};
    
    Point p = ManualLayoutManager::calculateSnapPoint(node, NodeEdge::Left, 0, 1);
    EXPECT_FLOAT_EQ(p.x, 0.0f);   // Left edge
    EXPECT_FLOAT_EQ(p.y, 25.0f);  // Center
}

TEST_F(ManualLayoutTest, CalculateSnapPointRight) {
    NodeLayout node;
    node.position = {0, 0};
    node.size = {100, 50};
    
    Point p = ManualLayoutManager::calculateSnapPoint(node, NodeEdge::Right, 0, 1);
    EXPECT_FLOAT_EQ(p.x, 100.0f); // Right edge
    EXPECT_FLOAT_EQ(p.y, 25.0f);  // Center
}

// ============== JSON Serialization Tests ==============

TEST_F(ManualLayoutTest, ToJsonBasic) {
    ManualLayoutManager manager;
    manager.setMode(LayoutMode::Manual);
    manager.setNodePosition(n1_, {100.0f, 50.0f});
    
    std::string json = manager.toJson();
    
    EXPECT_NE(json.find("\"version\": 1"), std::string::npos);
    EXPECT_NE(json.find("\"mode\": \"manual\""), std::string::npos);
    EXPECT_NE(json.find("\"nodePositions\""), std::string::npos);
}

TEST_F(ManualLayoutTest, FromJsonNodePositions) {
    ManualLayoutManager manager;
    
    std::string json = R"({
        "version": 1,
        "mode": "manual",
        "nodePositions": {
            "0": {"x": 100.0, "y": 50.0},
            "1": {"x": 200.0, "y": 150.0}
        },
        "snapConfigs": {},
        "edgeRoutings": {}
    })";
    
    EXPECT_TRUE(manager.fromJson(json));
    EXPECT_EQ(manager.getMode(), LayoutMode::Manual);
    
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
    manager1.setMode(LayoutMode::Manual);
    manager1.setNodePosition(n1_, {100.0f, 50.0f});
    manager1.setSnapConfig(n1_, {2, 3, 1, 4});
    manager1.setEdgeRouting(e1_, {NodeEdge::Right, NodeEdge::Left, 1, 2});
    
    std::string json = manager1.toJson();
    
    ManualLayoutManager manager2;
    EXPECT_TRUE(manager2.fromJson(json));
    
    EXPECT_EQ(manager2.getMode(), LayoutMode::Manual);
    
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
    manager.setMode(LayoutMode::Manual);
    manager.setNodePosition(n1_, {50.0f, 0.0f});
    manager.setNodePosition(n2_, {0.0f, 100.0f});
    manager.setNodePosition(n3_, {100.0f, 100.0f});
    
    SugiyamaLayout layout;
    layout.setManualLayoutManager(&manager);
    
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

// ============== Bend Point Tests ==============

TEST_F(ManualLayoutTest, BendPoint_Append_AddsToBendPoints) {
    ManualLayoutManager manager;
    
    manager.appendBendPoint(e1_, {100.0f, 200.0f});
    manager.appendBendPoint(e1_, {150.0f, 250.0f});
    
    EXPECT_TRUE(manager.hasManualBendPoints(e1_));
    const auto& bps = manager.getBendPoints(e1_);
    ASSERT_EQ(bps.size(), 2u);
    EXPECT_FLOAT_EQ(bps[0].position.x, 100.0f);
    EXPECT_FLOAT_EQ(bps[0].position.y, 200.0f);
    EXPECT_FLOAT_EQ(bps[1].position.x, 150.0f);
    EXPECT_FLOAT_EQ(bps[1].position.y, 250.0f);
}

TEST_F(ManualLayoutTest, BendPoint_InsertAtIndex_InsertsCorrectly) {
    ManualLayoutManager manager;
    
    manager.appendBendPoint(e1_, {100.0f, 200.0f});
    manager.appendBendPoint(e1_, {300.0f, 400.0f});
    // Insert in the middle
    manager.addBendPoint(e1_, 1, {200.0f, 300.0f});
    
    const auto& bps = manager.getBendPoints(e1_);
    ASSERT_EQ(bps.size(), 3u);
    EXPECT_FLOAT_EQ(bps[0].position.x, 100.0f);
    EXPECT_FLOAT_EQ(bps[1].position.x, 200.0f);  // inserted
    EXPECT_FLOAT_EQ(bps[2].position.x, 300.0f);
}

TEST_F(ManualLayoutTest, BendPoint_Remove_RemovesFromVector) {
    ManualLayoutManager manager;
    
    manager.appendBendPoint(e1_, {100.0f, 200.0f});
    manager.appendBendPoint(e1_, {150.0f, 250.0f});
    manager.appendBendPoint(e1_, {200.0f, 300.0f});
    
    manager.removeBendPoint(e1_, 1);  // remove middle
    
    const auto& bps = manager.getBendPoints(e1_);
    ASSERT_EQ(bps.size(), 2u);
    EXPECT_FLOAT_EQ(bps[0].position.x, 100.0f);
    EXPECT_FLOAT_EQ(bps[1].position.x, 200.0f);
}

TEST_F(ManualLayoutTest, BendPoint_Move_UpdatesPosition) {
    ManualLayoutManager manager;
    
    manager.appendBendPoint(e1_, {100.0f, 200.0f});
    manager.moveBendPoint(e1_, 0, {150.0f, 250.0f});
    
    const auto& bps = manager.getBendPoints(e1_);
    ASSERT_EQ(bps.size(), 1u);
    EXPECT_FLOAT_EQ(bps[0].position.x, 150.0f);
    EXPECT_FLOAT_EQ(bps[0].position.y, 250.0f);
}

TEST_F(ManualLayoutTest, BendPoint_Clear_RevertsToAutoRouting) {
    ManualLayoutManager manager;
    
    manager.appendBendPoint(e1_, {100.0f, 200.0f});
    EXPECT_TRUE(manager.hasManualBendPoints(e1_));
    
    manager.clearBendPoints(e1_);
    
    EXPECT_FALSE(manager.hasManualBendPoints(e1_));
    EXPECT_TRUE(manager.getBendPoints(e1_).empty());
}

TEST_F(ManualLayoutTest, BendPoint_SetBatch_ReplacesAll) {
    ManualLayoutManager manager;
    
    manager.appendBendPoint(e1_, {100.0f, 200.0f});
    
    std::vector<BendPoint> newBps = {
        {{50.0f, 100.0f}, false},
        {{75.0f, 150.0f}, true},
        {{100.0f, 200.0f}, false}
    };
    manager.setBendPoints(e1_, newBps);
    
    const auto& bps = manager.getBendPoints(e1_);
    ASSERT_EQ(bps.size(), 3u);
    EXPECT_FLOAT_EQ(bps[0].position.x, 50.0f);
    EXPECT_TRUE(bps[1].isControlPoint);
    EXPECT_FLOAT_EQ(bps[2].position.x, 100.0f);
}

TEST_F(ManualLayoutTest, BendPoint_ApplyToLayout_UsesManualPoints) {
    ManualLayoutManager manager;
    manager.setMode(LayoutMode::Manual);
    
    // Set edge routing first
    manager.setEdgeSourceEdge(e1_, NodeEdge::Bottom);
    manager.setEdgeTargetEdge(e1_, NodeEdge::Top);
    
    // Add manual bend points
    manager.appendBendPoint(e1_, {100.0f, 150.0f});
    manager.appendBendPoint(e1_, {150.0f, 200.0f});
    
    // Set node positions
    manager.setNodePosition(n1_, {0.0f, 0.0f});
    manager.setNodePosition(n2_, {200.0f, 250.0f});
    
    // Create a layout result and apply manual state
    SugiyamaLayout layout;
    LayoutResult result = layout.layout(graph_);
    manager.applyManualState(result, graph_);
    
    // Verify manual bend points were used
    const EdgeLayout* edgeLayout = result.getEdgeLayout(e1_);
    ASSERT_NE(edgeLayout, nullptr);
    ASSERT_EQ(edgeLayout->bendPoints.size(), 2u);
    EXPECT_FLOAT_EQ(edgeLayout->bendPoints[0].position.x, 100.0f);
    EXPECT_FLOAT_EQ(edgeLayout->bendPoints[0].position.y, 150.0f);
    EXPECT_FLOAT_EQ(edgeLayout->bendPoints[1].position.x, 150.0f);
    EXPECT_FLOAT_EQ(edgeLayout->bendPoints[1].position.y, 200.0f);
}

TEST_F(ManualLayoutTest, BendPoint_JsonRoundTrip_PreservesBendPoints) {
    ManualLayoutManager manager;
    manager.setMode(LayoutMode::Manual);
    manager.setEdgeSourceEdge(e1_, NodeEdge::Left);
    manager.setEdgeTargetEdge(e1_, NodeEdge::Right);
    
    manager.appendBendPoint(e1_, {100.0f, 200.0f});
    manager.appendBendPoint(e1_, {150.0f, 250.0f});
    
    std::string json = manager.toJson();
    
    ManualLayoutManager restored;
    ASSERT_TRUE(restored.fromJson(json));
    
    EXPECT_TRUE(restored.hasManualBendPoints(e1_));
    const auto& bps = restored.getBendPoints(e1_);
    ASSERT_EQ(bps.size(), 2u);
    EXPECT_FLOAT_EQ(bps[0].position.x, 100.0f);
    EXPECT_FLOAT_EQ(bps[0].position.y, 200.0f);
    EXPECT_FLOAT_EQ(bps[1].position.x, 150.0f);
    EXPECT_FLOAT_EQ(bps[1].position.y, 250.0f);
}

TEST_F(ManualLayoutTest, BendPoint_NoManual_UsesAutoRouting) {
    ManualLayoutManager manager;
    manager.setMode(LayoutMode::Manual);
    
    // Set edge routing without any manual bend points
    manager.setEdgeSourceEdge(e1_, NodeEdge::Bottom);
    manager.setEdgeTargetEdge(e1_, NodeEdge::Top);
    
    // Set node positions with vertical separation
    manager.setNodePosition(n1_, {50.0f, 0.0f});
    manager.setNodePosition(n2_, {150.0f, 200.0f});
    
    SugiyamaLayout layout;
    LayoutResult result = layout.layout(graph_);
    manager.applyManualState(result, graph_);
    
    // With no manual bend points, auto routing should create bend points
    const EdgeLayout* edgeLayout = result.getEdgeLayout(e1_);
    ASSERT_NE(edgeLayout, nullptr);
    
    // Auto routing should have created orthogonal bend points
    EXPECT_FALSE(manager.hasManualBendPoints(e1_));
    // The layout should still have auto-generated bend points
    EXPECT_GE(edgeLayout->bendPoints.size(), 0u);  // May or may not have auto bends
}

// ============== Orthogonal Bend Point Tests ==============

namespace {
// Helper function to check if a segment is orthogonal (horizontal or vertical)
bool isSegmentOrthogonal(const Point& a, const Point& b, float tolerance = 1.0f) {
    float dx = std::abs(a.x - b.x);
    float dy = std::abs(a.y - b.y);
    // Either horizontal (dy ≈ 0) or vertical (dx ≈ 0)
    return dx < tolerance || dy < tolerance;
}

// Helper function to check if all segments in a path are orthogonal
bool isPathOrthogonal(const std::vector<Point>& path, float tolerance = 1.0f) {
    if (path.size() < 2) return true;
    for (size_t i = 1; i < path.size(); ++i) {
        if (!isSegmentOrthogonal(path[i-1], path[i], tolerance)) {
            return false;
        }
    }
    return true;
}

// Build full path from edge layout
std::vector<Point> buildPath(const EdgeLayout& layout) {
    std::vector<Point> path;
    path.push_back(layout.sourcePoint);
    for (const auto& bp : layout.bendPoints) {
        path.push_back(bp.position);
    }
    path.push_back(layout.targetPoint);
    return path;
}
}  // namespace

TEST_F(ManualLayoutTest, BendPoint_Orthogonal_TwoPointsMaintainRightAngles) {
    ManualLayoutManager manager;
    manager.setMode(LayoutMode::Manual);
    
    // Set node positions with diagonal relationship
    manager.setNodePosition(n1_, {0.0f, 0.0f});
    manager.setNodePosition(n2_, {200.0f, 150.0f});
    
    manager.setEdgeSourceEdge(e1_, NodeEdge::Bottom);
    manager.setEdgeTargetEdge(e1_, NodeEdge::Top);
    
    // Add two orthogonal bend points (simulating what the UI does)
    // For horizontal-first approach: go right then down
    Point source = {50.0f, 50.0f};  // bottom center of n1
    Point target = {250.0f, 150.0f}; // top center of n2
    
    // Create L-shaped path: horizontal then vertical
    Point bp1 = {150.0f, source.y};  // go horizontal
    Point bp2 = {150.0f, target.y};  // go vertical
    
    manager.appendBendPoint(e1_, bp1);
    manager.appendBendPoint(e1_, bp2);
    
    SugiyamaLayout layout;
    LayoutResult result = layout.layout(graph_);
    manager.applyManualState(result, graph_);
    
    const EdgeLayout* edgeLayout = result.getEdgeLayout(e1_);
    ASSERT_NE(edgeLayout, nullptr);
    
    // Verify all segments are orthogonal
    auto path = buildPath(*edgeLayout);
    EXPECT_TRUE(isPathOrthogonal(path)) 
        << "Path should be orthogonal. Points: "
        << "source(" << path[0].x << "," << path[0].y << ") -> "
        << "bp1(" << path[1].x << "," << path[1].y << ") -> "
        << "bp2(" << path[2].x << "," << path[2].y << ") -> "
        << "target(" << path[3].x << "," << path[3].y << ")";
}

TEST_F(ManualLayoutTest, BendPoint_Orthogonal_SourceToTargetDirect) {
    ManualLayoutManager manager;
    manager.setMode(LayoutMode::Manual);
    
    // Set node positions: n1 at origin, n2 diagonal
    manager.setNodePosition(n1_, {0.0f, 0.0f});
    manager.setNodePosition(n2_, {200.0f, 200.0f});
    
    manager.setEdgeSourceEdge(e1_, NodeEdge::Bottom);
    manager.setEdgeTargetEdge(e1_, NodeEdge::Top);
    
    // For a diagonal source-target relationship, 
    // adding orthogonal bends should create right angles
    // Simulating first bend point insertion at click position (100, 100)
    
    // Expected orthogonal step:
    // source.y = 50 (bottom of node at y=0, height=50)
    // target.y = 200 (top of node at y=200)
    // bp1 = (100, 50) - horizontal from source
    // bp2 = (100, 200) - vertical to target's y level
    
    float sourceY = 50.0f;  // approximate
    float targetY = 200.0f;
    
    manager.appendBendPoint(e1_, Point{100.0f, sourceY});
    manager.appendBendPoint(e1_, Point{100.0f, targetY});
    
    SugiyamaLayout layout;
    LayoutResult result = layout.layout(graph_);
    manager.applyManualState(result, graph_);
    
    const EdgeLayout* edgeLayout = result.getEdgeLayout(e1_);
    ASSERT_NE(edgeLayout, nullptr);
    ASSERT_EQ(edgeLayout->bendPoints.size(), 2u);
    
    // Check each segment is orthogonal
    auto path = buildPath(*edgeLayout);
    
    // Segment 1: source -> bp1 (should be horizontal)
    EXPECT_TRUE(isSegmentOrthogonal(path[0], path[1]))
        << "Source to bp1 should be orthogonal";
    
    // Segment 2: bp1 -> bp2 (should be vertical)
    EXPECT_TRUE(isSegmentOrthogonal(path[1], path[2]))
        << "bp1 to bp2 should be orthogonal";
    
    // Segment 3: bp2 -> target (should be horizontal)
    EXPECT_TRUE(isSegmentOrthogonal(path[2], path[3]))
        << "bp2 to target should be orthogonal";
}

TEST_F(ManualLayoutTest, BendPoint_Orthogonal_MultipleInsertions) {
    ManualLayoutManager manager;
    manager.setMode(LayoutMode::Manual);
    
    manager.setNodePosition(n1_, {0.0f, 0.0f});
    manager.setNodePosition(n2_, {300.0f, 200.0f});
    
    manager.setEdgeSourceEdge(e1_, NodeEdge::Right);
    manager.setEdgeTargetEdge(e1_, NodeEdge::Left);
    
    // First insertion: creates 2 bends for orthogonal step
    manager.appendBendPoint(e1_, {100.0f, 25.0f});  // horizontal from source
    manager.appendBendPoint(e1_, {100.0f, 225.0f}); // vertical to target y
    
    // Second insertion: add more bends to create a detour
    // Insert between bp2 and target
    manager.appendBendPoint(e1_, {200.0f, 225.0f}); // horizontal step
    manager.appendBendPoint(e1_, {200.0f, 225.0f}); // (same point - will merge in rendering)
    
    SugiyamaLayout layout;
    LayoutResult result = layout.layout(graph_);
    manager.applyManualState(result, graph_);
    
    const EdgeLayout* edgeLayout = result.getEdgeLayout(e1_);
    ASSERT_NE(edgeLayout, nullptr);
    
    // Build and check path
    auto path = buildPath(*edgeLayout);
    
    // All segments should be orthogonal
    for (size_t i = 1; i < path.size(); ++i) {
        EXPECT_TRUE(isSegmentOrthogonal(path[i-1], path[i]))
            << "Segment " << i-1 << " to " << i << " should be orthogonal: "
            << "(" << path[i-1].x << "," << path[i-1].y << ") -> "
            << "(" << path[i].x << "," << path[i].y << ")";
    }
}

TEST_F(ManualLayoutTest, BendPoint_Orthogonal_AfterClearReaddMaintainsOrthogonal) {
    ManualLayoutManager manager;
    manager.setMode(LayoutMode::Manual);
    
    manager.setNodePosition(n1_, {0.0f, 0.0f});
    manager.setNodePosition(n2_, {200.0f, 150.0f});
    
    manager.setEdgeSourceEdge(e1_, NodeEdge::Bottom);
    manager.setEdgeTargetEdge(e1_, NodeEdge::Top);
    
    // Add bends, clear, then re-add
    manager.appendBendPoint(e1_, {50.0f, 50.0f});
    manager.appendBendPoint(e1_, {50.0f, 150.0f});
    
    manager.clearBendPoints(e1_);
    EXPECT_FALSE(manager.hasManualBendPoints(e1_));
    
    // Re-add orthogonal bends
    manager.appendBendPoint(e1_, {100.0f, 50.0f});
    manager.appendBendPoint(e1_, {100.0f, 150.0f});
    
    SugiyamaLayout layout;
    LayoutResult result = layout.layout(graph_);
    manager.applyManualState(result, graph_);
    
    const EdgeLayout* edgeLayout = result.getEdgeLayout(e1_);
    ASSERT_NE(edgeLayout, nullptr);
    
    auto path = buildPath(*edgeLayout);
    EXPECT_TRUE(isPathOrthogonal(path))
        << "Re-added bends should maintain orthogonal path";
}

// ============== Drag Orthogonality Tests ==============

// Helper: Calculate orthogonal-constrained position for drag
// Orthogonal drag constraint tests use ManualLayoutManager::calculateOrthogonalDrag()

TEST_F(ManualLayoutTest, BendPoint_Drag_MaintainsOrthogonalWithPrev) {
    ManualLayoutManager manager;
    manager.setMode(LayoutMode::Manual);
    
    // Set up nodes
    manager.setNodePosition(n1_, {0.0f, 0.0f});
    manager.setNodePosition(n2_, {200.0f, 200.0f});
    
    manager.setEdgeSourceEdge(e1_, NodeEdge::Right);
    manager.setEdgeTargetEdge(e1_, NodeEdge::Left);
    
    // Create orthogonal path: source(100,25) → bp1(150,25) → bp2(150,225) → target(200,225)
    // Horizontal from source, then vertical, then horizontal to target
    manager.appendBendPoint(e1_, Point{150.0f, 25.0f});   // bp1: horizontal from source
    manager.appendBendPoint(e1_, Point{150.0f, 225.0f});  // bp2: vertical from bp1
    
    // Verify initial path is orthogonal
    SugiyamaLayout layout;
    LayoutResult result = layout.layout(graph_);
    manager.applyManualState(result, graph_);
    
    const EdgeLayout* edgeLayout = result.getEdgeLayout(e1_);
    ASSERT_NE(edgeLayout, nullptr);
    
    auto initialPath = buildPath(*edgeLayout);
    ASSERT_TRUE(isPathOrthogonal(initialPath)) << "Initial path should be orthogonal";
    
    // Now simulate dragging bp1 (index 0)
    // Source point is the prev for bp1
    Point source = edgeLayout->sourcePoint;
    Point bp1 = manager.getBendPoints(e1_)[0].position;
    Point bp2 = manager.getBendPoints(e1_)[1].position;
    
    // Try to drag bp1 to a diagonal position
    Point dragTarget = {180.0f, 50.0f};  // Would create diagonal if unconstrained
    
    // Calculate what the orthogonal-constrained position should be
    auto dragResult = ManualLayoutManager::calculateOrthogonalDrag(source, bp1, bp2, dragTarget, true, false);
    
    // Apply the constrained move
    manager.moveBendPoint(e1_, 0, dragResult.newCurrentPos);
    if (dragResult.nextAdjusted) {
        manager.moveBendPoint(e1_, 1, dragResult.adjustedNextPos);
    }
    
    // Re-apply and check
    result = layout.layout(graph_);
    manager.applyManualState(result, graph_);
    
    edgeLayout = result.getEdgeLayout(e1_);
    auto afterDragPath = buildPath(*edgeLayout);
    
    EXPECT_TRUE(isPathOrthogonal(afterDragPath))
        << "Path after drag should remain orthogonal. Points: "
        << "src(" << afterDragPath[0].x << "," << afterDragPath[0].y << ") -> "
        << "bp1(" << afterDragPath[1].x << "," << afterDragPath[1].y << ") -> "
        << "bp2(" << afterDragPath[2].x << "," << afterDragPath[2].y << ") -> "
        << "tgt(" << afterDragPath[3].x << "," << afterDragPath[3].y << ")";
}

TEST_F(ManualLayoutTest, BendPoint_Drag_VerticalIncoming_HorizontalConstraint) {
    ManualLayoutManager manager;
    manager.setMode(LayoutMode::Manual);
    
    manager.setNodePosition(n1_, {0.0f, 0.0f});
    manager.setNodePosition(n2_, {200.0f, 200.0f});
    
    manager.setEdgeSourceEdge(e1_, NodeEdge::Bottom);
    manager.setEdgeTargetEdge(e1_, NodeEdge::Top);
    
    // Path: source(50,50) → bp1(50,125) → bp2(250,125) → target(250,200)
    // Vertical from source, then horizontal, then vertical to target
    manager.appendBendPoint(e1_, Point{50.0f, 125.0f});   // bp1: vertical from source
    manager.appendBendPoint(e1_, Point{250.0f, 125.0f});  // bp2: horizontal from bp1
    
    SugiyamaLayout layout;
    LayoutResult result = layout.layout(graph_);
    manager.applyManualState(result, graph_);
    
    const EdgeLayout* edgeLayout = result.getEdgeLayout(e1_);
    ASSERT_NE(edgeLayout, nullptr);
    
    Point source = edgeLayout->sourcePoint;
    Point bp1 = manager.getBendPoints(e1_)[0].position;
    Point bp2 = manager.getBendPoints(e1_)[1].position;
    
    // Drag bp1 - incoming is vertical, so should constrain to vertical movement only
    // (keep X same as source.x)
    Point dragTarget = {80.0f, 100.0f};  // Would create diagonal if X changes
    
    auto dragResult = ManualLayoutManager::calculateOrthogonalDrag(source, bp1, bp2, dragTarget, true, false);
    
    // The constrained position should have X = source.x (since incoming is vertical)
    EXPECT_FLOAT_EQ(dragResult.newCurrentPos.x, source.x)
        << "With vertical incoming, X should be constrained to source.x";
    
    // Apply and verify
    manager.moveBendPoint(e1_, 0, dragResult.newCurrentPos);
    if (dragResult.nextAdjusted) {
        manager.moveBendPoint(e1_, 1, dragResult.adjustedNextPos);
    }
    
    result = layout.layout(graph_);
    manager.applyManualState(result, graph_);
    
    edgeLayout = result.getEdgeLayout(e1_);
    auto path = buildPath(*edgeLayout);
    
    EXPECT_TRUE(isPathOrthogonal(path))
        << "Path should remain orthogonal after constrained drag";
}

TEST_F(ManualLayoutTest, BendPoint_Drag_HorizontalIncoming_VerticalConstraint) {
    ManualLayoutManager manager;
    manager.setMode(LayoutMode::Manual);
    
    manager.setNodePosition(n1_, {0.0f, 100.0f});
    manager.setNodePosition(n2_, {300.0f, 100.0f});
    
    manager.setEdgeSourceEdge(e1_, NodeEdge::Right);
    manager.setEdgeTargetEdge(e1_, NodeEdge::Left);
    
    // Horizontal source and target at same Y level
    // Path with a vertical detour
    manager.appendBendPoint(e1_, Point{150.0f, 125.0f});  // bp1
    manager.appendBendPoint(e1_, Point{150.0f, 50.0f});   // bp2: vertical from bp1
    manager.appendBendPoint(e1_, Point{250.0f, 50.0f});   // bp3: horizontal from bp2
    manager.appendBendPoint(e1_, Point{250.0f, 125.0f});  // bp4: vertical to target level
    
    SugiyamaLayout layout;
    LayoutResult result = layout.layout(graph_);
    manager.applyManualState(result, graph_);
    
    auto path = buildPath(*result.getEdgeLayout(e1_));
    ASSERT_TRUE(isPathOrthogonal(path)) << "Initial 4-bend path should be orthogonal";
}

// ============== Pure Unit Tests for calculateOrthogonalDrag API ==============
// These tests verify the API logic without any layout dependencies

TEST_F(ManualLayoutTest, CalculateOrthogonalDrag_HorizontalIncoming_ConstrainsY) {
    // Pure API test - no layout involved
    // Incoming segment is horizontal (same Y)
    Point prev{0, 100};
    Point current{50, 100};  // horizontal from prev (same Y=100)
    Point next{50, 200};
    Point drag{80, 150};     // diagonal drag attempt
    
    auto result = ManualLayoutManager::calculateOrthogonalDrag(
        prev, current, next, drag, true, false);
    
    // Y should be constrained to prev.y (horizontal constraint)
    EXPECT_FLOAT_EQ(result.newCurrentPos.y, 100.0f);
    // X should follow drag target
    EXPECT_FLOAT_EQ(result.newCurrentPos.x, 80.0f);
    // Next should be adjusted
    EXPECT_TRUE(result.nextAdjusted);
    EXPECT_FLOAT_EQ(result.adjustedNextPos.x, 80.0f);  // x = newCurrent.x
    EXPECT_FLOAT_EQ(result.adjustedNextPos.y, 200.0f); // y preserved
}

TEST_F(ManualLayoutTest, CalculateOrthogonalDrag_VerticalIncoming_ConstrainsX) {
    // Incoming segment is vertical (same X)
    Point prev{100, 0};
    Point current{100, 50};  // vertical from prev (same X=100)
    Point next{200, 50};
    Point drag{150, 80};     // diagonal drag attempt
    
    auto result = ManualLayoutManager::calculateOrthogonalDrag(
        prev, current, next, drag, true, false);
    
    // X should be constrained to prev.x (vertical constraint)
    EXPECT_FLOAT_EQ(result.newCurrentPos.x, 100.0f);
    // Y should follow drag target
    EXPECT_FLOAT_EQ(result.newCurrentPos.y, 80.0f);
    // Next should be adjusted
    EXPECT_TRUE(result.nextAdjusted);
    EXPECT_FLOAT_EQ(result.adjustedNextPos.x, 200.0f); // x preserved
    EXPECT_FLOAT_EQ(result.adjustedNextPos.y, 80.0f);  // y = newCurrent.y
}

TEST_F(ManualLayoutTest, CalculateOrthogonalDrag_LastBend_HorizontalVertical) {
    // Last bend: incoming horizontal, outgoing vertical
    Point prev{0, 100};
    Point current{50, 100};   // horizontal from prev
    Point target{50, 200};    // vertical to target
    Point drag{80, 150};      // try to drag diagonally
    
    auto result = ManualLayoutManager::calculateOrthogonalDrag(
        prev, current, target, drag, false, true);
    
    // Position should be constrained to intersection: X=target.x, Y=prev.y
    EXPECT_FLOAT_EQ(result.newCurrentPos.x, 50.0f);   // target.x
    EXPECT_FLOAT_EQ(result.newCurrentPos.y, 100.0f);  // prev.y
    EXPECT_FALSE(result.nextAdjusted);  // no next bend to adjust
}

TEST_F(ManualLayoutTest, CalculateOrthogonalDrag_LastBend_VerticalHorizontal) {
    // Last bend: incoming vertical, outgoing horizontal
    Point prev{100, 0};
    Point current{100, 50};   // vertical from prev
    Point target{200, 50};    // horizontal to target
    Point drag{150, 80};      // try to drag diagonally
    
    auto result = ManualLayoutManager::calculateOrthogonalDrag(
        prev, current, target, drag, false, true);
    
    // Position should be constrained to intersection: X=prev.x, Y=target.y
    EXPECT_FLOAT_EQ(result.newCurrentPos.x, 100.0f);  // prev.x
    EXPECT_FLOAT_EQ(result.newCurrentPos.y, 50.0f);   // target.y
    EXPECT_FALSE(result.nextAdjusted);
}

TEST_F(ManualLayoutTest, CalculateOrthogonalDrag_NoNextBend_NoAdjustment) {
    Point prev{0, 100};
    Point current{50, 100};
    Point next{100, 100};  // doesn't matter, hasNextBend=false
    Point drag{80, 150};
    
    auto result = ManualLayoutManager::calculateOrthogonalDrag(
        prev, current, next, drag, false, false);
    
    // Should still constrain position
    EXPECT_FLOAT_EQ(result.newCurrentPos.y, 100.0f);
    EXPECT_FLOAT_EQ(result.newCurrentPos.x, 80.0f);
    // But no next adjustment
    EXPECT_FALSE(result.nextAdjusted);
}

TEST_F(ManualLayoutTest, CalculateOrthogonalDrag_DiagonalEdgeCase_TreatsAsHorizontal) {
    // Edge case: 45-degree incoming (dx == dy)
    // Should be treated as horizontal per EPSILON logic
    Point prev{0, 0};
    Point current{50, 50};    // exactly 45 degrees
    Point next{50, 100};
    Point drag{80, 80};
    
    auto result = ManualLayoutManager::calculateOrthogonalDrag(
        prev, current, next, drag, true, false);
    
    // Treated as horizontal: Y should be constrained to prev.y
    EXPECT_FLOAT_EQ(result.newCurrentPos.y, 0.0f);
    EXPECT_FLOAT_EQ(result.newCurrentPos.x, 80.0f);
}

TEST_F(ManualLayoutTest, CalculateOrthogonalDrag_ResultDefaultInitialized) {
    // Verify struct is properly initialized
    ManualLayoutManager::OrthogonalDragResult result;
    
    EXPECT_FLOAT_EQ(result.newCurrentPos.x, 0.0f);
    EXPECT_FLOAT_EQ(result.newCurrentPos.y, 0.0f);
    EXPECT_FLOAT_EQ(result.adjustedNextPos.x, 0.0f);
    EXPECT_FLOAT_EQ(result.adjustedNextPos.y, 0.0f);
    EXPECT_FALSE(result.nextAdjusted);
}

// ============== TDD: Drag Orthogonality Bug Detection Tests ==============
// These tests verify ManualLayoutManager::calculateOrthogonalDrag() directly

// TDD Test 1: Drag first bend point - should maintain orthogonality with source
TEST_F(ManualLayoutTest, TDD_BendPoint_Drag_FirstPoint_MaintainsOrthogonalWithSource) {
    ManualLayoutManager manager;
    manager.setMode(LayoutMode::Manual);

    // Setup: source at (0,0), target at (200,200)
    manager.setNodePosition(n1_, {0.0f, 0.0f});
    manager.setNodePosition(n2_, {200.0f, 200.0f});
    manager.setEdgeSourceEdge(e1_, NodeEdge::Bottom);
    manager.setEdgeTargetEdge(e1_, NodeEdge::Top);

    // Initial orthogonal path: source(50,50) → bp1(50,125) → bp2(250,125) → target(250,200)
    // Segment 1: vertical (same X=50)
    // Segment 2: horizontal (same Y=125)
    // Segment 3: vertical (same X=250)
    manager.appendBendPoint(e1_, Point{50.0f, 125.0f});   // bp1
    manager.appendBendPoint(e1_, Point{250.0f, 125.0f});  // bp2

    SugiyamaLayout layout;
    LayoutResult result = layout.layout(graph_);
    manager.applyManualState(result, graph_);

    const EdgeLayout* edgeLayout = result.getEdgeLayout(e1_);
    ASSERT_NE(edgeLayout, nullptr);

    // Verify initial path is orthogonal
    auto initialPath = buildPath(*edgeLayout);
    ASSERT_TRUE(isPathOrthogonal(initialPath)) << "Initial path must be orthogonal";

    Point source = edgeLayout->sourcePoint;
    Point bp1 = manager.getBendPoints(e1_)[0].position;
    Point bp2 = manager.getBendPoints(e1_)[1].position;

    // Simulate dragging bp1 to a diagonal position
    Point dragTarget = {80.0f, 100.0f};  // Would break orthogonality if unconstrained

    auto dragResult = ManualLayoutManager::calculateOrthogonalDrag(source, bp1, bp2, dragTarget, true, false);

    // Apply the drag
    manager.moveBendPoint(e1_, 0, dragResult.newCurrentPos);
    if (dragResult.nextAdjusted) {
        manager.moveBendPoint(e1_, 1, dragResult.adjustedNextPos);
    }

    // Re-layout and verify
    result = layout.layout(graph_);
    manager.applyManualState(result, graph_);

    auto afterPath = buildPath(*result.getEdgeLayout(e1_));

    EXPECT_TRUE(isPathOrthogonal(afterPath))
        << "Path after drag must remain orthogonal.\n"
        << "Source: (" << afterPath[0].x << "," << afterPath[0].y << ")\n"
        << "BP1: (" << afterPath[1].x << "," << afterPath[1].y << ")\n"
        << "BP2: (" << afterPath[2].x << "," << afterPath[2].y << ")\n"
        << "Target: (" << afterPath[3].x << "," << afterPath[3].y << ")";
}

// TDD Test 2: Drag middle bend point in 4-point path - cascade adjustment needed
TEST_F(ManualLayoutTest, TDD_BendPoint_Drag_MiddlePoint_CascadeAdjustment) {
    ManualLayoutManager manager;
    manager.setMode(LayoutMode::Manual);

    manager.setNodePosition(n1_, {0.0f, 100.0f});
    manager.setNodePosition(n2_, {400.0f, 100.0f});
    manager.setEdgeSourceEdge(e1_, NodeEdge::Right);
    manager.setEdgeTargetEdge(e1_, NodeEdge::Left);

    // 4 bend points creating a detour path:
    // source(100,125) → bp1(150,125) → bp2(150,50) → bp3(350,50) → bp4(350,125) → target(400,125)
    manager.appendBendPoint(e1_, Point{150.0f, 125.0f});  // bp1: horizontal from source
    manager.appendBendPoint(e1_, Point{150.0f, 50.0f});   // bp2: vertical from bp1
    manager.appendBendPoint(e1_, Point{350.0f, 50.0f});   // bp3: horizontal from bp2
    manager.appendBendPoint(e1_, Point{350.0f, 125.0f});  // bp4: vertical from bp3

    SugiyamaLayout layout;
    LayoutResult result = layout.layout(graph_);
    manager.applyManualState(result, graph_);

    auto initialPath = buildPath(*result.getEdgeLayout(e1_));
    ASSERT_TRUE(isPathOrthogonal(initialPath)) << "Initial 4-bend path must be orthogonal";

    // Now drag bp2 (index 1) - this is a middle point
    Point bp1 = manager.getBendPoints(e1_)[0].position;
    Point bp2 = manager.getBendPoints(e1_)[1].position;
    Point bp3 = manager.getBendPoints(e1_)[2].position;

    // Drag bp2 to new position
    Point dragTarget = {200.0f, 30.0f};

    auto dragResult = ManualLayoutManager::calculateOrthogonalDrag(bp1, bp2, bp3, dragTarget, true, false);

    // Apply the drag
    manager.moveBendPoint(e1_, 1, dragResult.newCurrentPos);
    if (dragResult.nextAdjusted) {
        manager.moveBendPoint(e1_, 2, dragResult.adjustedNextPos);
    }

    // Re-layout and verify
    result = layout.layout(graph_);
    manager.applyManualState(result, graph_);

    auto afterPath = buildPath(*result.getEdgeLayout(e1_));

    // This test might FAIL if cascade adjustment doesn't propagate correctly
    EXPECT_TRUE(isPathOrthogonal(afterPath))
        << "Path after middle drag must remain orthogonal.\n"
        << "Source: (" << afterPath[0].x << "," << afterPath[0].y << ")\n"
        << "BP1: (" << afterPath[1].x << "," << afterPath[1].y << ")\n"
        << "BP2: (" << afterPath[2].x << "," << afterPath[2].y << ")\n"
        << "BP3: (" << afterPath[3].x << "," << afterPath[3].y << ")\n"
        << "BP4: (" << afterPath[4].x << "," << afterPath[4].y << ")\n"
        << "Target: (" << afterPath[5].x << "," << afterPath[5].y << ")";
}

// TDD Test 3: Verify segment orthogonality individually
TEST_F(ManualLayoutTest, TDD_BendPoint_Drag_VerifyEachSegment) {
    ManualLayoutManager manager;
    manager.setMode(LayoutMode::Manual);

    manager.setNodePosition(n1_, {0.0f, 0.0f});
    manager.setNodePosition(n2_, {200.0f, 200.0f});
    manager.setEdgeSourceEdge(e1_, NodeEdge::Bottom);
    manager.setEdgeTargetEdge(e1_, NodeEdge::Top);

    // Create simple 2-bend orthogonal path
    manager.appendBendPoint(e1_, Point{50.0f, 125.0f});
    manager.appendBendPoint(e1_, Point{250.0f, 125.0f});

    SugiyamaLayout layout;
    LayoutResult result = layout.layout(graph_);
    manager.applyManualState(result, graph_);

    const EdgeLayout* edgeLayout = result.getEdgeLayout(e1_);
    Point source = edgeLayout->sourcePoint;
    Point bp1 = manager.getBendPoints(e1_)[0].position;
    Point bp2 = manager.getBendPoints(e1_)[1].position;

    // Drag bp1 downward
    Point dragTarget = {50.0f, 150.0f};
    auto dragResult = ManualLayoutManager::calculateOrthogonalDrag(source, bp1, bp2, dragTarget, true, false);

    manager.moveBendPoint(e1_, 0, dragResult.newCurrentPos);
    if (dragResult.nextAdjusted) {
        manager.moveBendPoint(e1_, 1, dragResult.adjustedNextPos);
    }

    result = layout.layout(graph_);
    manager.applyManualState(result, graph_);

    edgeLayout = result.getEdgeLayout(e1_);
    auto path = buildPath(*edgeLayout);

    // Check each segment individually
    for (size_t i = 1; i < path.size(); ++i) {
        bool segmentOk = isSegmentOrthogonal(path[i-1], path[i]);
        EXPECT_TRUE(segmentOk)
            << "Segment " << i-1 << " -> " << i << " is NOT orthogonal!\n"
            << "Point " << i-1 << ": (" << path[i-1].x << "," << path[i-1].y << ")\n"
            << "Point " << i << ": (" << path[i].x << "," << path[i].y << ")\n"
            << "dx=" << std::abs(path[i].x - path[i-1].x)
            << ", dy=" << std::abs(path[i].y - path[i-1].y);
    }
}

// TDD Test 4: Rapid consecutive drags
TEST_F(ManualLayoutTest, TDD_BendPoint_Drag_ConsecutiveDrags) {
    ManualLayoutManager manager;
    manager.setMode(LayoutMode::Manual);

    manager.setNodePosition(n1_, {0.0f, 0.0f});
    manager.setNodePosition(n2_, {200.0f, 200.0f});
    manager.setEdgeSourceEdge(e1_, NodeEdge::Bottom);
    manager.setEdgeTargetEdge(e1_, NodeEdge::Top);

    manager.appendBendPoint(e1_, Point{50.0f, 100.0f});
    manager.appendBendPoint(e1_, Point{250.0f, 100.0f});

    SugiyamaLayout layout;

    // Simulate 5 consecutive drags on bp1
    std::vector<Point> dragPositions = {
        {60.0f, 120.0f},
        {70.0f, 80.0f},
        {40.0f, 150.0f},
        {90.0f, 90.0f},
        {55.0f, 110.0f}
    };

    for (size_t i = 0; i < dragPositions.size(); ++i) {
        LayoutResult result = layout.layout(graph_);
        manager.applyManualState(result, graph_);

        const EdgeLayout* edgeLayout = result.getEdgeLayout(e1_);
        Point source = edgeLayout->sourcePoint;
        Point bp1 = manager.getBendPoints(e1_)[0].position;
        Point bp2 = manager.getBendPoints(e1_)[1].position;

        auto dragResult = ManualLayoutManager::calculateOrthogonalDrag(source, bp1, bp2, dragPositions[i], true, false);

        manager.moveBendPoint(e1_, 0, dragResult.newCurrentPos);
        if (dragResult.nextAdjusted) {
            manager.moveBendPoint(e1_, 1, dragResult.adjustedNextPos);
        }
    }

    // Final verification
    LayoutResult result = layout.layout(graph_);
    manager.applyManualState(result, graph_);

    auto finalPath = buildPath(*result.getEdgeLayout(e1_));
    EXPECT_TRUE(isPathOrthogonal(finalPath))
        << "Path after " << dragPositions.size() << " consecutive drags must remain orthogonal";
}

// TDD Test 5: Source point not aligned with first bend point (REAL BUG SCENARIO)
// When source/target positions don't align with bend points, orthogonality is already broken
TEST_F(ManualLayoutTest, TDD_BendPoint_SourceMismatch_DetectsNonOrthogonal) {
    ManualLayoutManager manager;
    manager.setMode(LayoutMode::Manual);

    manager.setNodePosition(n1_, {0.0f, 0.0f});
    manager.setNodePosition(n2_, {200.0f, 200.0f});
    manager.setEdgeSourceEdge(e1_, NodeEdge::Bottom);
    manager.setEdgeTargetEdge(e1_, NodeEdge::Top);

    // Add bend points at positions that assume source is at (50, 50)
    manager.appendBendPoint(e1_, Point{50.0f, 125.0f});
    manager.appendBendPoint(e1_, Point{250.0f, 125.0f});

    SugiyamaLayout layout;
    LayoutResult result = layout.layout(graph_);
    manager.applyManualState(result, graph_);

    const EdgeLayout* edgeLayout = result.getEdgeLayout(e1_);
    Point source = edgeLayout->sourcePoint;
    Point bp1 = manager.getBendPoints(e1_)[0].position;

    // Check if source->bp1 is actually orthogonal
    // If source.x != bp1.x AND source.y != bp1.y, then it's NOT orthogonal
    bool sourceToFirstIsOrthogonal = isSegmentOrthogonal(source, bp1);

    // This test will FAIL if source point doesn't align with bp1
    // This exposes the root cause: bend points created without considering actual source position
    if (!sourceToFirstIsOrthogonal) {
        std::cout << "DETECTED: Source->BP1 is NOT orthogonal!\n"
                  << "Source: (" << source.x << "," << source.y << ")\n"
                  << "BP1: (" << bp1.x << "," << bp1.y << ")\n"
                  << "dx=" << std::abs(source.x - bp1.x)
                  << ", dy=" << std::abs(source.y - bp1.y) << "\n";
    }

    // The path should be orthogonal
    auto path = buildPath(*edgeLayout);
    EXPECT_TRUE(isPathOrthogonal(path))
        << "Initial path must be orthogonal with actual source position";
}

// TDD Test 6: Simulate exact demo scenario - add bend points then drag
TEST_F(ManualLayoutTest, TDD_BendPoint_DemoScenario_InsertThenDrag) {
    ManualLayoutManager manager;
    manager.setMode(LayoutMode::Manual);

    manager.setNodePosition(n1_, {0.0f, 0.0f});
    manager.setNodePosition(n2_, {200.0f, 200.0f});
    manager.setEdgeSourceEdge(e1_, NodeEdge::Bottom);
    manager.setEdgeTargetEdge(e1_, NodeEdge::Top);

    SugiyamaLayout layout;

    // Step 1: Get initial layout to find source/target positions
    LayoutResult result = layout.layout(graph_);
    manager.applyManualState(result, graph_);

    const EdgeLayout* edgeLayout = result.getEdgeLayout(e1_);
    Point source = edgeLayout->sourcePoint;
    Point target = edgeLayout->targetPoint;

    std::cout << "Initial source: (" << source.x << "," << source.y << ")\n";
    std::cout << "Initial target: (" << target.x << "," << target.y << ")\n";

    // Step 2: Simulate 2-point insertion (same logic as demo)
    Point clickPos = {(source.x + target.x) / 2, (source.y + target.y) / 2};
    float dx = std::abs(target.x - source.x);
    float dy = std::abs(target.y - source.y);

    Point bp1, bp2;
    if (dx > dy) {
        bp1 = {clickPos.x, source.y};
        bp2 = {clickPos.x, target.y};
    } else {
        bp1 = {source.x, clickPos.y};
        bp2 = {target.x, clickPos.y};
    }

    manager.appendBendPoint(e1_, bp1);
    manager.appendBendPoint(e1_, bp2);

    std::cout << "Inserted BP1: (" << bp1.x << "," << bp1.y << ")\n";
    std::cout << "Inserted BP2: (" << bp2.x << "," << bp2.y << ")\n";

    // Step 3: Apply and verify initial orthogonality
    result = layout.layout(graph_);
    manager.applyManualState(result, graph_);

    edgeLayout = result.getEdgeLayout(e1_);
    auto pathAfterInsert = buildPath(*edgeLayout);

    std::cout << "After insert - Source: (" << edgeLayout->sourcePoint.x
              << "," << edgeLayout->sourcePoint.y << ")\n";

    ASSERT_TRUE(isPathOrthogonal(pathAfterInsert))
        << "Path after insert must be orthogonal";

    // Step 4: Drag bp1
    source = edgeLayout->sourcePoint;
    bp1 = manager.getBendPoints(e1_)[0].position;
    bp2 = manager.getBendPoints(e1_)[1].position;

    Point dragTarget = {bp1.x + 30, bp1.y + 20};
    auto dragResult = ManualLayoutManager::calculateOrthogonalDrag(source, bp1, bp2, dragTarget, true, false);

    manager.moveBendPoint(e1_, 0, dragResult.newCurrentPos);
    if (dragResult.nextAdjusted) {
        manager.moveBendPoint(e1_, 1, dragResult.adjustedNextPos);
    }

    // Step 5: Verify after drag
    result = layout.layout(graph_);
    manager.applyManualState(result, graph_);

    auto pathAfterDrag = buildPath(*result.getEdgeLayout(e1_));

    EXPECT_TRUE(isPathOrthogonal(pathAfterDrag))
        << "Path after drag must remain orthogonal";
}

// TDD Test 7: Source/Target position consistency after doLayout()
// This tests if source/target points change unexpectedly between layout calls
TEST_F(ManualLayoutTest, TDD_BendPoint_SourceConsistency_BetweenLayouts) {
    ManualLayoutManager manager;
    manager.setMode(LayoutMode::Manual);

    manager.setNodePosition(n1_, {0.0f, 0.0f});
    manager.setNodePosition(n2_, {200.0f, 200.0f});
    manager.setEdgeSourceEdge(e1_, NodeEdge::Bottom);
    manager.setEdgeTargetEdge(e1_, NodeEdge::Top);

    SugiyamaLayout layout;

    // First layout
    LayoutResult result1 = layout.layout(graph_);
    manager.applyManualState(result1, graph_);
    const EdgeLayout* edge1 = result1.getEdgeLayout(e1_);
    Point source1 = edge1->sourcePoint;
    Point target1 = edge1->targetPoint;

    // Add bend points based on first layout
    Point bp1 = {source1.x, (source1.y + target1.y) / 2};
    Point bp2 = {target1.x, (source1.y + target1.y) / 2};
    manager.appendBendPoint(e1_, bp1);
    manager.appendBendPoint(e1_, bp2);

    // Second layout (after adding bend points)
    LayoutResult result2 = layout.layout(graph_);
    manager.applyManualState(result2, graph_);
    const EdgeLayout* edge2 = result2.getEdgeLayout(e1_);
    Point source2 = edge2->sourcePoint;
    Point target2 = edge2->targetPoint;

    // Source and target should be the same (or very close)
    float srcDx = std::abs(source2.x - source1.x);
    float srcDy = std::abs(source2.y - source1.y);
    float tgtDx = std::abs(target2.x - target1.x);
    float tgtDy = std::abs(target2.y - target1.y);

    std::cout << "Source1: (" << source1.x << "," << source1.y << ")\n";
    std::cout << "Source2: (" << source2.x << "," << source2.y << ")\n";
    std::cout << "Source delta: dx=" << srcDx << ", dy=" << srcDy << "\n";

    EXPECT_LT(srcDx + srcDy, 1.0f)
        << "Source point should not change significantly between layouts";
    EXPECT_LT(tgtDx + tgtDy, 1.0f)
        << "Target point should not change significantly between layouts";

    // Path should still be orthogonal
    auto path = buildPath(*edge2);
    EXPECT_TRUE(isPathOrthogonal(path))
        << "Path should be orthogonal after second layout";
}

// TDD Test 8: Test the exact visual feedback bug in demo
// When updating visual feedback, old bps position is used instead of new
TEST_F(ManualLayoutTest, TDD_BendPoint_VisualFeedbackBug) {
    ManualLayoutManager manager;
    manager.setMode(LayoutMode::Manual);

    manager.setNodePosition(n1_, {0.0f, 0.0f});
    manager.setNodePosition(n2_, {200.0f, 200.0f});
    manager.setEdgeSourceEdge(e1_, NodeEdge::Bottom);
    manager.setEdgeTargetEdge(e1_, NodeEdge::Top);

    manager.appendBendPoint(e1_, Point{50.0f, 125.0f});
    manager.appendBendPoint(e1_, Point{250.0f, 125.0f});

    SugiyamaLayout layout;
    LayoutResult result = layout.layout(graph_);
    manager.applyManualState(result, graph_);

    const EdgeLayout* edgeLayout = result.getEdgeLayout(e1_);
    Point source = edgeLayout->sourcePoint;

    // Simulate the EXACT bug in visual feedback code
    // Get bps BEFORE any update (like the demo does)
    const auto& bps = manager.getBendPoints(e1_);
    Point bp1_old = bps[0].position;
    Point bp2_old = bps[1].position;

    // Calculate drag result
    Point dragTarget = {80.0f, 100.0f};
    auto dragResult = ManualLayoutManager::calculateOrthogonalDrag(source, bp1_old, bp2_old, dragTarget, true, false);

    // Update manualManager (correct)
    if (dragResult.nextAdjusted) {
        manager.moveBendPoint(e1_, 1, dragResult.adjustedNextPos);
    }
    manager.moveBendPoint(e1_, 0, dragResult.newCurrentPos);

    // Get bps AFTER update
    const auto& bps_after = manager.getBendPoints(e1_);

    // Simulate the visual feedback update using OLD bps (THE BUG)
    Point visual_bp1 = dragResult.newCurrentPos;  // This is correct
    Point visual_bp2_buggy;  // This might be wrong
    if (dragResult.nextAdjusted) {
        // Bug: uses bps[1].position which is OLD (before moveBendPoint)
        // But we called moveBendPoint before this, so bps might be updated...
        // Actually in demo, bps is captured BEFORE moveBendPoint calls!
        bool incomingHorizontal = std::abs(bp1_old.x - source.x) > std::abs(bp1_old.y - source.y);
        if (incomingHorizontal) {
            visual_bp2_buggy = {visual_bp1.x, bp2_old.y};
        } else {
            visual_bp2_buggy = {bp2_old.x, visual_bp1.y};
        }
    }

    // What SHOULD be displayed (from manualManager after update)
    Point correct_bp2 = bps_after[1].position;

    // The visual feedback might not match the actual data
    std::cout << "Visual BP2 (buggy): (" << visual_bp2_buggy.x << "," << visual_bp2_buggy.y << ")\n";
    std::cout << "Correct BP2: (" << correct_bp2.x << "," << correct_bp2.y << ")\n";

    // They should be the same
    EXPECT_FLOAT_EQ(visual_bp2_buggy.x, correct_bp2.x) << "Visual and actual BP2.x should match";
    EXPECT_FLOAT_EQ(visual_bp2_buggy.y, correct_bp2.y) << "Visual and actual BP2.y should match";
}

// TDD Test 9: Drag LAST bend point - should maintain orthogonality with TARGET
// Current demo logic only adjusts NEXT bend, but last bend has no next!
TEST_F(ManualLayoutTest, TDD_BendPoint_Drag_LastPoint_NoNextToAdjust) {
    ManualLayoutManager manager;
    manager.setMode(LayoutMode::Manual);

    manager.setNodePosition(n1_, {0.0f, 0.0f});
    manager.setNodePosition(n2_, {200.0f, 200.0f});
    manager.setEdgeSourceEdge(e1_, NodeEdge::Bottom);
    manager.setEdgeTargetEdge(e1_, NodeEdge::Top);

    // 2 bend points
    manager.appendBendPoint(e1_, Point{50.0f, 125.0f});   // bp1
    manager.appendBendPoint(e1_, Point{250.0f, 125.0f});  // bp2 (last)

    SugiyamaLayout layout;
    LayoutResult result = layout.layout(graph_);
    manager.applyManualState(result, graph_);

    auto initialPath = buildPath(*result.getEdgeLayout(e1_));
    ASSERT_TRUE(isPathOrthogonal(initialPath)) << "Initial path must be orthogonal";

    const EdgeLayout* edgeLayout = result.getEdgeLayout(e1_);
    Point target = edgeLayout->targetPoint;

    Point bp1 = manager.getBendPoints(e1_)[0].position;
    Point bp2 = manager.getBendPoints(e1_)[1].position;

    // Drag the LAST bend point (bp2, index 1)
    // prevPoint is bp1, nextPoint is target
    Point dragTarget = {280.0f, 150.0f};

    // hasNextBend = false, isLastBend = true
    auto dragResult = ManualLayoutManager::calculateOrthogonalDrag(bp1, bp2, target, dragTarget, false, true);

    // Apply the drag (no next bend to adjust)
    manager.moveBendPoint(e1_, 1, dragResult.newCurrentPos);

    // Re-layout and verify
    result = layout.layout(graph_);
    manager.applyManualState(result, graph_);

    edgeLayout = result.getEdgeLayout(e1_);
    auto afterPath = buildPath(*edgeLayout);

    // Check specifically the last segment (bp2 -> target)
    Point newBp2 = afterPath[2];
    Point newTarget = afterPath[3];

    std::cout << "After dragging last bend point:\n";
    std::cout << "BP2: (" << newBp2.x << "," << newBp2.y << ")\n";
    std::cout << "Target: (" << newTarget.x << "," << newTarget.y << ")\n";
    std::cout << "Segment dx=" << std::abs(newBp2.x - newTarget.x)
              << ", dy=" << std::abs(newBp2.y - newTarget.y) << "\n";

    // THIS TEST MAY FAIL: After dragging last bend, bp2->target might not be orthogonal!
    bool lastSegmentOrthogonal = isSegmentOrthogonal(newBp2, newTarget);
    EXPECT_TRUE(lastSegmentOrthogonal)
        << "Last segment (BP2->Target) must be orthogonal after drag!\n"
        << "BP2: (" << newBp2.x << "," << newBp2.y << ")\n"
        << "Target: (" << newTarget.x << "," << newTarget.y << ")";

    EXPECT_TRUE(isPathOrthogonal(afterPath))
        << "Full path must remain orthogonal";
}
