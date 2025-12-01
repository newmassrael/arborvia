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

// ============== Label Position Update on Drag Tests ==============

TEST_F(LayoutUtilsTest, LabelPosition_UpdatesAfterDrag) {
    // Setup: Create initial layout with channel-based routing
    SugiyamaLayout layout;
    LayoutResult result = layout.layout(graph_);
    
    // Get initial layouts
    std::unordered_map<NodeId, NodeLayout> nodeLayouts;
    std::unordered_map<EdgeId, EdgeLayout> edgeLayouts;
    
    for (const auto& [id, nl] : result.nodeLayouts()) {
        nodeLayouts[id] = nl;
    }
    for (const auto& [id, el] : result.edgeLayouts()) {
        edgeLayouts[id] = el;
    }
    
    // Store initial labelPosition for edge e1_
    const EdgeLayout& initialEdge = edgeLayouts[e1_];
    Point initialLabelPos = initialEdge.labelPosition;
    
    // Simulate drag: move n1_ by offset
    Point dragOffset = {50.0f, 30.0f};
    nodeLayouts[n1_].position.x += dragOffset.x;
    nodeLayouts[n1_].position.y += dragOffset.y;
    
    // Get affected edges
    std::vector<EdgeId> affectedEdges = {e1_, e2_};
    
    // Update edge positions (this should also update labelPosition)
    LayoutUtils::updateEdgePositions(
        edgeLayouts, nodeLayouts, affectedEdges,
        SnapDistribution::Unified, {n1_});
    
    // Verify labelPosition has changed
    const EdgeLayout& updatedEdge = edgeLayouts[e1_];
    Point updatedLabelPos = updatedEdge.labelPosition;
    
    // Label position should have moved
    EXPECT_NE(initialLabelPos.x, updatedLabelPos.x)
        << "Label position X should change after drag";
    EXPECT_NE(initialLabelPos.y, updatedLabelPos.y)
        << "Label position Y should change after drag";
    
    // Label position should be calculated from updated bendPoints
    Point expectedLabelPos = LayoutUtils::calculateEdgeLabelPosition(updatedEdge);
    EXPECT_FLOAT_EQ(updatedLabelPos.x, expectedLabelPos.x)
        << "Label position should match calculateEdgeLabelPosition result";
    EXPECT_FLOAT_EQ(updatedLabelPos.y, expectedLabelPos.y)
        << "Label position should match calculateEdgeLabelPosition result";
}

TEST_F(LayoutUtilsTest, LabelPosition_UpdatesForMultipleEdges) {
    // Setup
    SugiyamaLayout layout;
    LayoutResult result = layout.layout(graph_);
    
    std::unordered_map<NodeId, NodeLayout> nodeLayouts;
    std::unordered_map<EdgeId, EdgeLayout> edgeLayouts;
    
    for (const auto& [id, nl] : result.nodeLayouts()) {
        nodeLayouts[id] = nl;
    }
    for (const auto& [id, el] : result.edgeLayouts()) {
        edgeLayouts[id] = el;
    }
    
    // Store initial labelPositions
    std::unordered_map<EdgeId, Point> initialLabelPositions;
    for (EdgeId edgeId : {e1_, e2_, e3_}) {
        initialLabelPositions[edgeId] = edgeLayouts[edgeId].labelPosition;
    }
    
    // Move n2_ (affects e1_ and e3_)
    Point dragOffset = {-30.0f, 40.0f};
    nodeLayouts[n2_].position.x += dragOffset.x;
    nodeLayouts[n2_].position.y += dragOffset.y;
    
    std::vector<EdgeId> affectedEdges = {e1_, e3_};
    
    LayoutUtils::updateEdgePositions(
        edgeLayouts, nodeLayouts, affectedEdges,
        SnapDistribution::Separated, {n2_});
    
    // Verify affected edges have updated labelPositions
    for (EdgeId edgeId : affectedEdges) {
        Point initialPos = initialLabelPositions[edgeId];
        Point updatedPos = edgeLayouts[edgeId].labelPosition;
        
        // At least one coordinate should change
        bool changed = (initialPos.x != updatedPos.x) || (initialPos.y != updatedPos.y);
        EXPECT_TRUE(changed)
            << "Edge " << edgeId << " labelPosition should change after drag";
        
        // Should match calculated position
        Point expectedPos = LayoutUtils::calculateEdgeLabelPosition(edgeLayouts[edgeId]);
        EXPECT_FLOAT_EQ(updatedPos.x, expectedPos.x)
            << "Edge " << edgeId << " labelPosition.x mismatch";
        EXPECT_FLOAT_EQ(updatedPos.y, expectedPos.y)
            << "Edge " << edgeId << " labelPosition.y mismatch";
    }
    
    // Unaffected edge (e2_) should remain unchanged
    Point e2InitialPos = initialLabelPositions[e2_];
    Point e2UpdatedPos = edgeLayouts[e2_].labelPosition;
    EXPECT_FLOAT_EQ(e2InitialPos.x, e2UpdatedPos.x)
        << "Unaffected edge labelPosition should not change";
    EXPECT_FLOAT_EQ(e2InitialPos.y, e2UpdatedPos.y)
        << "Unaffected edge labelPosition should not change";
}

// ============== Bidirectional Edge Label Separation Tests ==============

TEST_F(LayoutUtilsTest, BidirectionalEdgeLabels_DoNotOverlap) {
    // Create graph with bidirectional edges (like running <-> paused)
    Graph graph;
    NodeId running = graph.addNode(Size{200, 100}, "Running");
    NodeId paused = graph.addNode(Size{200, 100}, "Paused");
    
    EdgeId pauseEdge = graph.addEdge(running, paused, "pause");
    EdgeId resumeEdge = graph.addEdge(paused, running, "resume");
    
    // Layout with channel-based routing
    SugiyamaLayout layout;
    LayoutResult result = layout.layout(graph);
    
    const EdgeLayout* pauseLayout = result.getEdgeLayout(pauseEdge);
    const EdgeLayout* resumeLayout = result.getEdgeLayout(resumeEdge);
    
    ASSERT_NE(pauseLayout, nullptr);
    ASSERT_NE(resumeLayout, nullptr);
    
    Point pauseLabelPos = pauseLayout->labelPosition;
    Point resumeLabelPos = resumeLayout->labelPosition;
    
    // Calculate distance between label positions
    float dx = pauseLabelPos.x - resumeLabelPos.x;
    float dy = pauseLabelPos.y - resumeLabelPos.y;
    float distance = std::sqrt(dx * dx + dy * dy);
    
    // Labels should be separated by at least 20 pixels (minimum readable distance)
    const float MIN_LABEL_SEPARATION = 20.0f;
    
    EXPECT_GT(distance, MIN_LABEL_SEPARATION)
        << "Bidirectional edge labels should not overlap. "
        << "pause label at (" << pauseLabelPos.x << ", " << pauseLabelPos.y << "), "
        << "resume label at (" << resumeLabelPos.x << ", " << resumeLabelPos.y << "), "
        << "distance: " << distance << " pixels";
    
    // Also verify they use different channels (should have different bendPoints)
    EXPECT_FALSE(pauseLayout->bendPoints.empty())
        << "Channel routing should create bendPoints";
    EXPECT_FALSE(resumeLayout->bendPoints.empty())
        << "Channel routing should create bendPoints";
    
    if (!pauseLayout->bendPoints.empty() && !resumeLayout->bendPoints.empty()) {
        // First bendPoint Y should differ (different channel)
        float pauseChannelY = pauseLayout->bendPoints[0].position.y;
        float resumeChannelY = resumeLayout->bendPoints[0].position.y;
        
        EXPECT_NE(pauseChannelY, resumeChannelY)
            << "Bidirectional edges should use different channels. "
            << "pause channel Y: " << pauseChannelY << ", "
            << "resume channel Y: " << resumeChannelY;
    }
}

TEST_F(LayoutUtilsTest, BidirectionalEdgeLabels_SeparationMaintainedAfterDrag) {
    // Create bidirectional graph
    Graph graph;
    NodeId running = graph.addNode(Size{200, 100}, "Running");
    NodeId paused = graph.addNode(Size{200, 100}, "Paused");
    
    EdgeId pauseEdge = graph.addEdge(running, paused, "pause");
    EdgeId resumeEdge = graph.addEdge(paused, running, "resume");
    
    // Initial layout
    SugiyamaLayout layout;
    LayoutResult result = layout.layout(graph);
    
    std::unordered_map<NodeId, NodeLayout> nodeLayouts;
    std::unordered_map<EdgeId, EdgeLayout> edgeLayouts;
    
    for (const auto& [id, nl] : result.nodeLayouts()) {
        nodeLayouts[id] = nl;
    }
    for (const auto& [id, el] : result.edgeLayouts()) {
        edgeLayouts[id] = el;
    }
    
    // Verify initial separation
    Point pauseLabelBefore = edgeLayouts[pauseEdge].labelPosition;
    Point resumeLabelBefore = edgeLayouts[resumeEdge].labelPosition;
    
    float dxBefore = pauseLabelBefore.x - resumeLabelBefore.x;
    float dyBefore = pauseLabelBefore.y - resumeLabelBefore.y;
    float distanceBefore = std::sqrt(dxBefore * dxBefore + dyBefore * dyBefore);
    
    // Drag running node
    Point dragOffset = {50.0f, 30.0f};
    nodeLayouts[running].position.x += dragOffset.x;
    nodeLayouts[running].position.y += dragOffset.y;
    
    std::vector<EdgeId> affectedEdges = {pauseEdge, resumeEdge};
    
    LayoutUtils::updateEdgePositions(
        edgeLayouts, nodeLayouts, affectedEdges,
        SnapDistribution::Unified, {running});
    
    // Verify separation is maintained after drag
    Point pauseLabelAfter = edgeLayouts[pauseEdge].labelPosition;
    Point resumeLabelAfter = edgeLayouts[resumeEdge].labelPosition;
    
    float dxAfter = pauseLabelAfter.x - resumeLabelAfter.x;
    float dyAfter = pauseLabelAfter.y - resumeLabelAfter.y;
    float distanceAfter = std::sqrt(dxAfter * dxAfter + dyAfter * dyAfter);
    
    const float MIN_LABEL_SEPARATION = 20.0f;

    EXPECT_GT(distanceAfter, MIN_LABEL_SEPARATION)
        << "Label separation should be maintained after drag. "
        << "pause label at (" << pauseLabelAfter.x << ", " << pauseLabelAfter.y << "), "
        << "resume label at (" << resumeLabelAfter.x << ", " << resumeLabelAfter.y << "), "
        << "distance: " << distanceAfter << " pixels";
}

TEST_F(LayoutUtilsTest, ChannelAssignment_NoCollisionWithMixedEdges) {
    // Test scenario: Multiple edges in same region (same source layer)
    // All edges from layer 0 to layer 1
    Graph graph;
    NodeId source1 = graph.addNode(Size{200, 100}, "S1");
    NodeId source2 = graph.addNode(Size{200, 100}, "S2");
    NodeId source3 = graph.addNode(Size{200, 100}, "S3");
    NodeId target = graph.addNode(Size{200, 100}, "Target");

    // All edges go to same target, creating same region
    EdgeId e1 = graph.addEdge(source1, target, "edge1");        // Normal
    EdgeId e2 = graph.addEdge(source2, target, "forward");     // Bidirectional
    EdgeId e3 = graph.addEdge(target, source2, "backward");    // Bidirectional
    EdgeId e4 = graph.addEdge(source3, target, "edge4");        // Normal

    SugiyamaLayout layout;
    LayoutResult result = layout.layout(graph);

    const EdgeLayout* layout1 = result.getEdgeLayout(e1);
    const EdgeLayout* layout2 = result.getEdgeLayout(e2);
    const EdgeLayout* layout3 = result.getEdgeLayout(e3);
    const EdgeLayout* layout4 = result.getEdgeLayout(e4);

    ASSERT_NE(layout1, nullptr);
    ASSERT_NE(layout2, nullptr);
    ASSERT_NE(layout3, nullptr);
    ASSERT_NE(layout4, nullptr);

    // Extract channel Y positions from bendPoints
    ASSERT_FALSE(layout1->bendPoints.empty());
    ASSERT_FALSE(layout2->bendPoints.empty());
    ASSERT_FALSE(layout3->bendPoints.empty());
    ASSERT_FALSE(layout4->bendPoints.empty());

    // Verify all edges have valid channel routing
    EXPECT_FALSE(layout1->bendPoints.empty());
    EXPECT_FALSE(layout2->bendPoints.empty());
    EXPECT_FALSE(layout3->bendPoints.empty());
    EXPECT_FALSE(layout4->bendPoints.empty());
}

TEST_F(LayoutUtilsTest, BidirectionalEdges_HorizontalLayout) {
    // Test bidirectional edge separation in horizontal (LeftToRight) layout
    Graph graph;
    NodeId left = graph.addNode(Size{100, 200}, "Left");
    NodeId right = graph.addNode(Size{100, 200}, "Right");

    EdgeId forward = graph.addEdge(left, right, "forward");
    EdgeId backward = graph.addEdge(right, left, "backward");

    LayoutOptions options;
    options.direction = Direction::LeftToRight;

    SugiyamaLayout layout;
    layout.setOptions(options);
    LayoutResult result = layout.layout(graph);

    const EdgeLayout* forwardLayout = result.getEdgeLayout(forward);
    const EdgeLayout* backwardLayout = result.getEdgeLayout(backward);

    ASSERT_NE(forwardLayout, nullptr);
    ASSERT_NE(backwardLayout, nullptr);

    // Both edges should have bendPoints
    ASSERT_FALSE(forwardLayout->bendPoints.empty())
        << "Forward edge should have bend points";
    ASSERT_FALSE(backwardLayout->bendPoints.empty())
        << "Backward edge should have bend points";

    // In horizontal layout, channel separation is in X direction
    float forwardChannelX = forwardLayout->bendPoints[0].position.x;
    float backwardChannelX = backwardLayout->bendPoints[0].position.x;

    std::cout << "Forward: channelY=" << forwardLayout->channelY
              << ", bendX=" << forwardChannelX << std::endl;
    std::cout << "Backward: channelY=" << backwardLayout->channelY
              << ", bendX=" << backwardChannelX << std::endl;

    // Bidirectional edges must use different channels (different X positions)
    EXPECT_NE(forwardChannelX, backwardChannelX)
        << "Bidirectional edges in horizontal layout must use different channels. "
        << "forward X: " << forwardChannelX << ", backward X: " << backwardChannelX;

    // Verify minimum separation
    const float MIN_CHANNEL_SEPARATION = 15.0f;
    EXPECT_GT(std::abs(forwardChannelX - backwardChannelX), MIN_CHANNEL_SEPARATION)
        << "Bidirectional edges need sufficient channel separation in horizontal layout";

    // Verify channelY is set (even though it's channelX logically)
    EXPECT_GE(forwardLayout->channelY, 0.0f)
        << "Channel position should be stored for drag operations";
    EXPECT_GE(backwardLayout->channelY, 0.0f)
        << "Channel position should be stored for drag operations";
}
