#include <gtest/gtest.h>
#include <arborvia/arborvia.h>
#include <cmath>
#include <vector>

using namespace arborvia;

class LayoutUtilsTest : public ::testing::Test {
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

// ============== PointToSegmentDistance Tests ==============

TEST_F(LayoutUtilsTest, PointToSegmentDistance_PointOnSegment) {
    Point p = {50.0f, 0.0f};
    Point segStart = {0.0f, 0.0f};
    Point segEnd = {100.0f, 0.0f};
    Point closest;

    float dist = LayoutUtils::pointToSegmentDistance(p, segStart, segEnd, closest);

    EXPECT_FLOAT_EQ(dist, 0.0f);
    EXPECT_FLOAT_EQ(closest.x, 50.0f);
    EXPECT_FLOAT_EQ(closest.y, 0.0f);
}

TEST_F(LayoutUtilsTest, PointToSegmentDistance_PointAboveSegment) {
    Point p = {50.0f, 10.0f};
    Point segStart = {0.0f, 0.0f};
    Point segEnd = {100.0f, 0.0f};
    Point closest;

    float dist = LayoutUtils::pointToSegmentDistance(p, segStart, segEnd, closest);

    EXPECT_FLOAT_EQ(dist, 10.0f);
    EXPECT_FLOAT_EQ(closest.x, 50.0f);
    EXPECT_FLOAT_EQ(closest.y, 0.0f);
}

TEST_F(LayoutUtilsTest, PointToSegmentDistance_PointBeyondStart) {
    Point p = {-10.0f, 0.0f};
    Point segStart = {0.0f, 0.0f};
    Point segEnd = {100.0f, 0.0f};
    Point closest;

    float dist = LayoutUtils::pointToSegmentDistance(p, segStart, segEnd, closest);

    EXPECT_FLOAT_EQ(dist, 10.0f);
    EXPECT_FLOAT_EQ(closest.x, 0.0f);  // Clamped to start
    EXPECT_FLOAT_EQ(closest.y, 0.0f);
}

TEST_F(LayoutUtilsTest, PointToSegmentDistance_PointBeyondEnd) {
    Point p = {110.0f, 0.0f};
    Point segStart = {0.0f, 0.0f};
    Point segEnd = {100.0f, 0.0f};
    Point closest;

    float dist = LayoutUtils::pointToSegmentDistance(p, segStart, segEnd, closest);

    EXPECT_FLOAT_EQ(dist, 10.0f);
    EXPECT_FLOAT_EQ(closest.x, 100.0f);  // Clamped to end
    EXPECT_FLOAT_EQ(closest.y, 0.0f);
}

TEST_F(LayoutUtilsTest, PointToSegmentDistance_ZeroLengthSegment) {
    Point p = {10.0f, 10.0f};
    Point segStart = {0.0f, 0.0f};
    Point segEnd = {0.0f, 0.0f};  // Zero-length segment
    Point closest;

    float dist = LayoutUtils::pointToSegmentDistance(p, segStart, segEnd, closest);

    // Distance should be to the point itself
    float expected = std::sqrt(10.0f * 10.0f + 10.0f * 10.0f);
    EXPECT_NEAR(dist, expected, 0.001f);
    EXPECT_FLOAT_EQ(closest.x, 0.0f);
    EXPECT_FLOAT_EQ(closest.y, 0.0f);
}

// ============== HitTestEdge Tests ==============

TEST_F(LayoutUtilsTest, HitTestEdge_Hit_ReturnsClosestSegment) {
    // Create an edge layout with multiple segments
    EdgeLayout edge;
    edge.sourcePoint = {0.0f, 0.0f};
    edge.targetPoint = {100.0f, 100.0f};
    edge.bendPoints = {
        {{50.0f, 0.0f}},    // First bend
        {{50.0f, 100.0f}}   // Second bend
    };
    // Path: (0,0) -> (50,0) -> (50,100) -> (100,100)

    // Test point near the vertical segment (should be closest)
    Point testPoint = {45.0f, 50.0f};  // Near vertical segment at x=50

    auto result = LayoutUtils::hitTestEdge(testPoint, edge, 10.0f);

    EXPECT_TRUE(result.hit);
    EXPECT_EQ(result.segmentIndex, 1);  // Vertical segment (50,0) -> (50,100)
    EXPECT_NEAR(result.closestPoint.x, 50.0f, 0.1f);
    EXPECT_NEAR(result.closestPoint.y, 50.0f, 0.1f);
    EXPECT_NEAR(result.distance, 5.0f, 0.1f);
}

TEST_F(LayoutUtilsTest, HitTestEdge_Miss_ReturnsNoHit) {
    EdgeLayout edge;
    edge.sourcePoint = {0.0f, 0.0f};
    edge.targetPoint = {100.0f, 0.0f};

    // Test point far from the edge
    Point testPoint = {50.0f, 100.0f};

    auto result = LayoutUtils::hitTestEdge(testPoint, edge, 10.0f);

    EXPECT_FALSE(result.hit);
    EXPECT_EQ(result.segmentIndex, -1);
}

TEST_F(LayoutUtilsTest, HitTestEdge_MultipleSegmentsInRange_ReturnsClosest) {
    // Edge with a tight corner where two segments are both within threshold
    EdgeLayout edge;
    edge.sourcePoint = {0.0f, 0.0f};
    edge.targetPoint = {10.0f, 10.0f};
    edge.bendPoints = {
        {{10.0f, 0.0f}}  // Corner at (10, 0)
    };
    // Path: (0,0) -> (10,0) -> (10,10)

    // Point near the corner, closer to vertical segment
    Point testPoint = {12.0f, 5.0f};

    auto result = LayoutUtils::hitTestEdge(testPoint, edge, 5.0f);

    EXPECT_TRUE(result.hit);
    EXPECT_EQ(result.segmentIndex, 1);  // Vertical segment is closer
    EXPECT_NEAR(result.closestPoint.x, 10.0f, 0.1f);
}

// ============== calculateBendPointPair Tests ==============

TEST_F(LayoutUtilsTest, CalculateBendPointPair_EmptyBends_HorizontalEdge) {
    EdgeLayout edge;
    edge.sourcePoint = {0.0f, 50.0f};
    edge.targetPoint = {200.0f, 50.0f};  // Horizontal edge

    std::vector<BendPoint> emptyBends;
    Point clickPos = {100.0f, 50.0f};

    auto result = OrthogonalRouter::calculateBendPointPair(edge, emptyBends, clickPos, 0);

    // Horizontal edge -> vertical step
    EXPECT_EQ(result.insertIndex, 0u);
    EXPECT_FLOAT_EQ(result.first.x, 100.0f);
    EXPECT_FLOAT_EQ(result.first.y, 50.0f);   // source.y
    EXPECT_FLOAT_EQ(result.second.x, 100.0f);
    EXPECT_FLOAT_EQ(result.second.y, 50.0f);  // target.y (same as source in this case)
}

TEST_F(LayoutUtilsTest, CalculateBendPointPair_EmptyBends_VerticalEdge) {
    EdgeLayout edge;
    edge.sourcePoint = {50.0f, 0.0f};
    edge.targetPoint = {50.0f, 200.0f};  // Vertical edge

    std::vector<BendPoint> emptyBends;
    Point clickPos = {50.0f, 100.0f};

    auto result = OrthogonalRouter::calculateBendPointPair(edge, emptyBends, clickPos, 0);

    // Vertical edge -> horizontal step
    EXPECT_EQ(result.insertIndex, 0u);
    EXPECT_FLOAT_EQ(result.first.x, 50.0f);   // source.x
    EXPECT_FLOAT_EQ(result.first.y, 100.0f);
    EXPECT_FLOAT_EQ(result.second.x, 50.0f);  // target.x (same as source)
    EXPECT_FLOAT_EQ(result.second.y, 100.0f);
}

TEST_F(LayoutUtilsTest, CalculateBendPointPair_EmptyBends_DiagonalEdge) {
    EdgeLayout edge;
    edge.sourcePoint = {0.0f, 0.0f};
    edge.targetPoint = {200.0f, 100.0f};  // More horizontal than vertical

    std::vector<BendPoint> emptyBends;
    Point clickPos = {100.0f, 50.0f};

    auto result = OrthogonalRouter::calculateBendPointPair(edge, emptyBends, clickPos, 0);

    // dx(200) > dy(100) -> vertical step
    EXPECT_EQ(result.insertIndex, 0u);
    EXPECT_FLOAT_EQ(result.first.x, 100.0f);
    EXPECT_FLOAT_EQ(result.first.y, 0.0f);    // source.y
    EXPECT_FLOAT_EQ(result.second.x, 100.0f);
    EXPECT_FLOAT_EQ(result.second.y, 100.0f); // target.y
}

TEST_F(LayoutUtilsTest, CalculateBendPointPair_ExistingBends_MiddleSegment) {
    EdgeLayout edge;
    edge.sourcePoint = {0.0f, 0.0f};
    edge.targetPoint = {200.0f, 200.0f};

    std::vector<BendPoint> existingBends = {
        {{100.0f, 0.0f}},
        {{100.0f, 200.0f}}
    };
    // Path: (0,0) -> (100,0) -> (100,200) -> (200,200)
    // Segment 1: (100,0) -> (100,200) is vertical

    Point clickPos = {100.0f, 100.0f};

    auto result = OrthogonalRouter::calculateBendPointPair(edge, existingBends, clickPos, 1);

    EXPECT_EQ(result.insertIndex, 1u);
    // Vertical segment -> horizontal step
    EXPECT_FLOAT_EQ(result.first.x, 100.0f);
    EXPECT_FLOAT_EQ(result.first.y, 100.0f);
}

TEST_F(LayoutUtilsTest, CalculateBendPointPair_InvalidSegmentIndex_UsesFallback) {
    EdgeLayout edge;
    edge.sourcePoint = {0.0f, 0.0f};
    edge.targetPoint = {100.0f, 100.0f};

    std::vector<BendPoint> existingBends = {
        {{50.0f, 0.0f}}
    };
    // Path: (0,0) -> (50,0) -> (100,100)
    // Valid segment indices: 0, 1

    Point clickPos = {50.0f, 50.0f};

    // Test with invalid index (too large)
    auto result = OrthogonalRouter::calculateBendPointPair(edge, existingBends, clickPos, 100);

    // Should fall back to segment 0
    EXPECT_EQ(result.insertIndex, 0u);
}

TEST_F(LayoutUtilsTest, CalculateBendPointPair_NegativeSegmentIndex_UsesFallback) {
    EdgeLayout edge;
    edge.sourcePoint = {0.0f, 0.0f};
    edge.targetPoint = {100.0f, 100.0f};

    std::vector<BendPoint> existingBends = {
        {{50.0f, 0.0f}}
    };

    Point clickPos = {50.0f, 50.0f};

    // Test with negative index
    auto result = OrthogonalRouter::calculateBendPointPair(edge, existingBends, clickPos, -5);

    // Should fall back to segment 0
    EXPECT_EQ(result.insertIndex, 0u);
}

// ============== syncSnapConfigsFromEdgeLayouts Tests ==============

TEST_F(LayoutUtilsTest, SyncSnapConfigs_UpdatesFromEdgeLayouts) {
    ManualLayoutManager manager;

    std::unordered_map<EdgeId, EdgeLayout> edgeLayouts;

    // Edge using snap index 2 on node n1_'s bottom edge
    EdgeLayout layout1;
    layout1.from = n1_;
    layout1.to = n2_;
    layout1.sourceEdge = NodeEdge::Bottom;
    layout1.targetEdge = NodeEdge::Top;
    layout1.sourceSnapIndex = 2;  // Needs at least 3 snap points
    layout1.targetSnapIndex = 1;  // Needs at least 2 snap points
    edgeLayouts[e1_] = layout1;

    manager.syncSnapConfigsFromEdgeLayouts(edgeLayouts);

    // Check snap configs were updated
    SnapPointConfig n1Config = manager.getSnapConfig(n1_);
    SnapPointConfig n2Config = manager.getSnapConfig(n2_);

    EXPECT_GE(n1Config.bottomCount, 3);  // At least 3 for snapIndex 2
    EXPECT_GE(n2Config.topCount, 2);     // At least 2 for snapIndex 1
}

TEST_F(LayoutUtilsTest, SyncSnapConfigs_PreservesHigherManualCounts) {
    ManualLayoutManager manager;

    // Set higher manual count first
    SnapPointConfig manualConfig;
    manualConfig.bottomCount = 5;
    manager.setSnapConfig(n1_, manualConfig);

    std::unordered_map<EdgeId, EdgeLayout> edgeLayouts;
    EdgeLayout layout1;
    layout1.from = n1_;
    layout1.to = n2_;
    layout1.sourceEdge = NodeEdge::Bottom;
    layout1.targetEdge = NodeEdge::Top;
    layout1.sourceSnapIndex = 1;  // Would need only 2
    layout1.targetSnapIndex = 0;
    edgeLayouts[e1_] = layout1;

    manager.syncSnapConfigsFromEdgeLayouts(edgeLayouts);

    // Manual count (5) should be preserved
    SnapPointConfig n1Config = manager.getSnapConfig(n1_);
    EXPECT_EQ(n1Config.bottomCount, 5);
}

TEST_F(LayoutUtilsTest, SyncSnapConfigs_EmptyEdgeLayouts) {
    ManualLayoutManager manager;

    // Set initial config
    SnapPointConfig initialConfig;
    initialConfig.topCount = 3;
    manager.setSnapConfig(n1_, initialConfig);

    std::unordered_map<EdgeId, EdgeLayout> emptyLayouts;

    manager.syncSnapConfigsFromEdgeLayouts(emptyLayouts);

    // Config should remain unchanged
    SnapPointConfig config = manager.getSnapConfig(n1_);
    EXPECT_EQ(config.topCount, 3);
}
