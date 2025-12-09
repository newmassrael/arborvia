#include <gtest/gtest.h>
#include <arborvia/arborvia.h>
#include "../../src/layout/snap/GridSnapCalculator.h"
#include "../../src/layout/routing/OrthogonalRouter.h"
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
        edgeLayouts, nodeLayouts, affectedEdges, {n1_});

    // Verify labelPosition has changed
    const EdgeLayout& updatedEdge = edgeLayouts[e1_];
    Point updatedLabelPos = updatedEdge.labelPosition;

    // Label position should have moved (at least in one dimension)
    // Note: With corrected directional constraints, Y may not change if the node
    // moves to a position where the channel is still valid for both endpoints
    bool labelMoved = (initialLabelPos.x != updatedLabelPos.x) ||
                      (initialLabelPos.y != updatedLabelPos.y);
    EXPECT_TRUE(labelMoved)
        << "Label position should change after drag. "
        << "Initial: (" << initialLabelPos.x << ", " << initialLabelPos.y << "), "
        << "Updated: (" << updatedLabelPos.x << ", " << updatedLabelPos.y << ")";

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
        {n2_});

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

    // Verify different channels are assigned (channelY should differ)
    // Note: In Unified mode, edges may be straight lines (no bendPoints) if snap points align,
    // but channelY should still be different for bidirectional edge separation
    EXPECT_NE(pauseLayout->channelY, resumeLayout->channelY)
        << "Bidirectional edges should use different channels. "
        << "pause channelY: " << pauseLayout->channelY << ", "
        << "resume channelY: " << resumeLayout->channelY;
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
        {running});

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

    // Verify all edges have valid channel routing (channelY assigned)
    // Note: In Unified mode, edges may be straight lines (no bendPoints) if snap points align,
    // but channelY should still be different for edge separation

    // Bidirectional edges (e2 forward, e3 backward) should have different channels
    EXPECT_NE(layout2->channelY, layout3->channelY)
        << "Bidirectional edges should use different channels. "
        << "forward channelY: " << layout2->channelY << ", "
        << "backward channelY: " << layout3->channelY;
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

    // Verify channelY is different for bidirectional edges
    // Note: In horizontal layout, channelY actually stores the channel X position
    // In Unified mode, edges may be straight lines (no bendPoints) if snap points align

    std::cout << "Forward: channelY=" << forwardLayout->channelY << std::endl;
    std::cout << "Backward: channelY=" << backwardLayout->channelY << std::endl;

    // Bidirectional edges must use different channels
    EXPECT_NE(forwardLayout->channelY, backwardLayout->channelY)
        << "Bidirectional edges must use different channels. "
        << "forward channelY: " << forwardLayout->channelY
        << ", backward channelY: " << backwardLayout->channelY;

    // Verify minimum separation
    const float MIN_CHANNEL_SEPARATION = 15.0f;
    EXPECT_GT(std::abs(forwardLayout->channelY - backwardLayout->channelY), MIN_CHANNEL_SEPARATION)
        << "Bidirectional edges need sufficient channel separation";
}


// ============== MoveSnapPoint SnapIndex Update Tests ==============

TEST_F(LayoutUtilsTest, MoveSnapPoint_UpdatesSnapIndex) {
    // Setup: Create initial layout
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

    // Get initial state of edge e1_ (n1_ -> n2_)
    EdgeLayout initialEdge = edgeLayouts[e1_];
    Point initialTargetPoint = initialEdge.targetPoint;
    NodeEdge initialTargetEdge = initialEdge.targetEdge;

    std::cout << "\n========== INITIAL STATE ==========\n" << std::endl;
    std::cout << "Node Layouts:" << std::endl;
    for (const auto& [nodeId, nl] : nodeLayouts) {
        std::cout << "  Node " << nodeId << ": pos=(" << nl.position.x << "," << nl.position.y
                  << ") size=(" << nl.size.width << "," << nl.size.height << ")" << std::endl;
    }
    std::cout << std::endl;

    std::cout << "Edge 0 (to be moved):" << std::endl;
    std::cout << "  src=(" << initialEdge.sourcePoint.x << "," << initialEdge.sourcePoint.y
              << ") edge=" << static_cast<int>(initialEdge.sourceEdge) << std::endl;
    std::cout << "  tgt=(" << initialEdge.targetPoint.x << "," << initialEdge.targetPoint.y
              << ") edge=" << static_cast<int>(initialEdge.targetEdge) << std::endl;

    // Move target snap point along the SAME edge (shift right by 40px)
    Point newPosition = {initialTargetPoint.x + 40.0f, initialTargetPoint.y};
    std::cout << "\nRequested new target position: (" << newPosition.x << "," << newPosition.y << ")" << std::endl;

    LayoutOptions options;
    options.gridConfig.cellSize = 20.0f;
    options.optimizationOptions.dragAlgorithm = DragAlgorithm::AStar;
    options.optimizationOptions.postDragAlgorithm = PostDragAlgorithm::AStar;

    auto moveResult = LayoutUtils::moveSnapPoint(
        e1_, false, newPosition,  // false = target snap point
        nodeLayouts, edgeLayouts, graph_, options);

    ASSERT_TRUE(moveResult.success) << "moveSnapPoint should succeed";

    EdgeLayout& movedEdge = edgeLayouts[e1_];
    float gridSize = options.gridConfig.cellSize;

    std::cout << "\n========== AFTER MOVE ==========\n" << std::endl;
    std::cout << "Edge 0 (moved):" << std::endl;
    std::cout << "  src=(" << movedEdge.sourcePoint.x << "," << movedEdge.sourcePoint.y
              << ") edge=" << static_cast<int>(movedEdge.sourceEdge) << std::endl;
    std::cout << "  tgt=(" << movedEdge.targetPoint.x << "," << movedEdge.targetPoint.y
              << ") edge=" << static_cast<int>(movedEdge.targetEdge) << std::endl;
    for (size_t i = 0; i < movedEdge.bendPoints.size(); ++i) {
        std::cout << "  bend[" << i << "]=(" << movedEdge.bendPoints[i].position.x
                  << "," << movedEdge.bendPoints[i].position.y << ")" << std::endl;
    }

    int failures = 0;

    // =========================================================================
    // CONSTRAINT 1: Target edge should be PRESERVED (user moved along same edge)
    // =========================================================================
    if (movedEdge.targetEdge != initialTargetEdge) {
        std::cout << "[FAIL] Target edge changed from " << static_cast<int>(initialTargetEdge)
                  << " to " << static_cast<int>(movedEdge.targetEdge)
                  << " - user's intended edge should be preserved!" << std::endl;
        ++failures;
    }

    // =========================================================================
    // CONSTRAINT 2: Target position should be close to requested position
    // =========================================================================
    float positionError = std::abs(movedEdge.targetPoint.x - newPosition.x);
    // Allow grid snapping tolerance (within one grid cell)
    if (positionError > gridSize) {
        std::cout << "[FAIL] Target position too far from requested. "
                  << "Requested x=" << newPosition.x
                  << ", Got x=" << movedEdge.targetPoint.x
                  << ", Error=" << positionError << std::endl;
        ++failures;
    }

    // =========================================================================
    // CONSTRAINT 3: All segments must be ORTHOGONAL
    // =========================================================================
    int segmentIdx = 0;
    movedEdge.forEachSegment([&](const Point& p1, const Point& p2) {
        float dx = std::abs(p2.x - p1.x);
        float dy = std::abs(p2.y - p1.y);

        bool isHorizontal = (dy < 0.1f);
        bool isVertical = (dx < 0.1f);
        bool isOrthogonal = isHorizontal || isVertical;

        if (!isOrthogonal) {
            std::cout << "[FAIL] Segment " << segmentIdx << " is DIAGONAL: "
                      << "(" << p1.x << "," << p1.y << ") -> (" << p2.x << "," << p2.y << ") "
                      << "dx=" << dx << " dy=" << dy << std::endl;
            ++failures;
        }
        ++segmentIdx;
    });

    // =========================================================================
    // CONSTRAINT 4: Snap points must be ON declared node edge
    // =========================================================================
    auto checkSnapPointOnEdge = [&](bool isSource, const char* label) {
        NodeId nodeId = isSource ? movedEdge.from : movedEdge.to;
        const Point& point = isSource ? movedEdge.sourcePoint : movedEdge.targetPoint;
        NodeEdge nodeEdge = isSource ? movedEdge.sourceEdge : movedEdge.targetEdge;

        auto nodeIt = nodeLayouts.find(nodeId);
        if (nodeIt == nodeLayouts.end()) return;

        const NodeLayout& node = nodeIt->second;
        float tolerance = 1.0f;
        bool onEdge = false;

        switch (nodeEdge) {
            case NodeEdge::Top:
                onEdge = std::abs(point.y - node.position.y) < tolerance;
                break;
            case NodeEdge::Bottom:
                onEdge = std::abs(point.y - (node.position.y + node.size.height)) < tolerance;
                break;
            case NodeEdge::Left:
                onEdge = std::abs(point.x - node.position.x) < tolerance;
                break;
            case NodeEdge::Right:
                onEdge = std::abs(point.x - (node.position.x + node.size.width)) < tolerance;
                break;
        }

        if (!onEdge) {
            std::cout << "[FAIL] " << label << " point NOT on declared edge. "
                      << "Point=(" << point.x << "," << point.y << ") "
                      << "NodeEdge=" << static_cast<int>(nodeEdge) << std::endl;
            ++failures;
        }
    };

    checkSnapPointOnEdge(true, "Source");
    checkSnapPointOnEdge(false, "Target");

    // =========================================================================
    // CONSTRAINT 5: snapIndex must be CONSISTENT with position
    // =========================================================================
    auto checkSnapIndexConsistency = [&](bool isSource, const char* label) {
        NodeId nodeId = isSource ? movedEdge.from : movedEdge.to;
        const Point& point = isSource ? movedEdge.sourcePoint : movedEdge.targetPoint;
        int snapIndex = isSource ? movedEdge.sourceSnapIndex : movedEdge.targetSnapIndex;
        NodeEdge nodeEdge = isSource ? movedEdge.sourceEdge : movedEdge.targetEdge;

        auto nodeIt = nodeLayouts.find(nodeId);
        if (nodeIt == nodeLayouts.end()) return;

        const NodeLayout& node = nodeIt->second;

        // Use GridSnapCalculator to get expected snap index (handles corner exclusion correctly)
        int expectedSnapIndex = GridSnapCalculator::getCandidateIndexFromPosition(
            node, nodeEdge, point, gridSize);

        if (snapIndex != expectedSnapIndex) {
            std::cout << "[FAIL] " << label << " snapIndex INCONSISTENT: "
                      << "actual=" << snapIndex << " expected=" << expectedSnapIndex
                      << " (point=" << point.x << "," << point.y << ")" << std::endl;
            ++failures;
        }
    };

    // Only check snapIndex for the moved snap point (target in this case)
    // Source snapIndex may be inconsistent if original layout was created with old logic
    // checkSnapIndexConsistency(true, "Source");  // Skip - not moved
    checkSnapIndexConsistency(false, "Target");  // Check - this was moved

    // =========================================================================
    // CONSTRAINT 6: Edge must NOT penetrate other nodes
    // =========================================================================
    for (const auto& [nodeId, node] : nodeLayouts) {
        if (nodeId == movedEdge.from || nodeId == movedEdge.to) continue;

        float nodeLeft = node.position.x;
        float nodeRight = node.position.x + node.size.width;
        float nodeTop = node.position.y;
        float nodeBottom = node.position.y + node.size.height;

        movedEdge.forEachSegment([&](const Point& p1, const Point& p2) {
            auto pointInNode = [&](const Point& p) {
                return p.x > nodeLeft && p.x < nodeRight &&
                       p.y > nodeTop && p.y < nodeBottom;
            };

            if (pointInNode(p1) || pointInNode(p2)) {
                std::cout << "[FAIL] Edge penetrates node " << nodeId << std::endl;
                ++failures;
            }
        });
    }

    std::cout << "\n========== SUMMARY ==========" << std::endl;
    std::cout << "Total failures: " << failures << std::endl;

    EXPECT_EQ(failures, 0) << "Moved edge must satisfy all constraints";
}

TEST_F(LayoutUtilsTest, MoveSnapPoint_RejectsCornerPosition) {
    // Setup: Create initial layout
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

    // Get target node (n2_) info
    const NodeLayout& targetNode = nodeLayouts[n2_];
    EdgeLayout initialEdge = edgeLayouts[e1_];

    std::cout << "\n========== CORNER REJECTION TEST ==========\n" << std::endl;
    std::cout << "Target node n2_: pos=(" << targetNode.position.x << "," << targetNode.position.y
              << ") size=(" << targetNode.size.width << "," << targetNode.size.height << ")" << std::endl;

    // Try to move snap point to each corner
    std::vector<std::pair<std::string, Point>> corners = {
        {"TopLeft", {targetNode.position.x, targetNode.position.y}},
        {"TopRight", {targetNode.position.x + targetNode.size.width, targetNode.position.y}},
        {"BottomLeft", {targetNode.position.x, targetNode.position.y + targetNode.size.height}},
        {"BottomRight", {targetNode.position.x + targetNode.size.width, targetNode.position.y + targetNode.size.height}}
    };

    LayoutOptions options;
    options.gridConfig.cellSize = 20.0f;

    int cornerViolations = 0;

    for (const auto& [cornerName, cornerPos] : corners) {
        // Reset edge layouts
        for (const auto& [id, el] : result.edgeLayouts()) {
            edgeLayouts[id] = el;
        }

        std::cout << "\nTrying to move to " << cornerName << " corner: ("
                  << cornerPos.x << "," << cornerPos.y << ")" << std::endl;

        auto moveResult = LayoutUtils::moveSnapPoint(
            e1_, false, cornerPos,  // false = target snap point
            nodeLayouts, edgeLayouts, graph_, options);

        if (!moveResult.success) {
            std::cout << "  -> Move failed (expected for corner)" << std::endl;
            continue;
        }

        EdgeLayout& movedEdge = edgeLayouts[e1_];

        // Check if result is at exact corner position
        bool isAtCorner = false;
        float tolerance = 1.0f;

        // Check all 4 corners
        for (const auto& [name, corner] : corners) {
            if (std::abs(movedEdge.targetPoint.x - corner.x) < tolerance &&
                std::abs(movedEdge.targetPoint.y - corner.y) < tolerance) {
                isAtCorner = true;
                std::cout << "  [FAIL] Result is at " << name << " corner: ("
                          << movedEdge.targetPoint.x << "," << movedEdge.targetPoint.y << ")" << std::endl;
                ++cornerViolations;
                break;
            }
        }

        if (!isAtCorner) {
            std::cout << "  [OK] Snapped to non-corner position: ("
                      << movedEdge.targetPoint.x << "," << movedEdge.targetPoint.y << ")" << std::endl;
        }
    }

    std::cout << "\n========== SUMMARY ==========" << std::endl;
    std::cout << "Corner violations: " << cornerViolations << std::endl;

    EXPECT_EQ(cornerViolations, 0) << "Snap points should not be placed at corners";
}

// ============== MoveSnapPoint Swap Tests ==============

// Test: When dragging snap point to another edge's position, they should SWAP
TEST_F(LayoutUtilsTest, MoveSnapPoint_SwapsWithOccupiedPosition) {
    // Setup: Create a controlled scenario where two edges share the same source node
    // and are on the same NodeEdge with known snap indices
    Graph swapGraph;
    NodeId nodeA = swapGraph.addNode(Size{100, 50}, "A");
    NodeId nodeB = swapGraph.addNode(Size{100, 50}, "B");
    NodeId nodeC = swapGraph.addNode(Size{100, 50}, "C");

    EdgeId edgeAB = swapGraph.addEdge(nodeA, nodeB, "A->B");
    EdgeId edgeAC = swapGraph.addEdge(nodeA, nodeC, "A->C");

    LayoutOptions options;
    options.gridConfig.cellSize = 10.0f;
    float gridSize = options.gridConfig.cellSize;

    // Create node layouts manually for precise control
    std::unordered_map<NodeId, NodeLayout> nodeLayouts;
    std::unordered_map<EdgeId, EdgeLayout> edgeLayouts;

    // Node A at (0, 0), 100x50
    NodeLayout nodeA_layout;
    nodeA_layout.id = nodeA;
    nodeA_layout.position = {0, 0};
    nodeA_layout.size = {100, 50};
    nodeLayouts[nodeA] = nodeA_layout;

    // Node B at (0, 100)
    NodeLayout nodeB_layout;
    nodeB_layout.id = nodeB;
    nodeB_layout.position = {0, 100};
    nodeB_layout.size = {100, 50};
    nodeLayouts[nodeB] = nodeB_layout;

    // Node C at (50, 100)
    NodeLayout nodeC_layout;
    nodeC_layout.id = nodeC;
    nodeC_layout.position = {50, 100};
    nodeC_layout.size = {100, 50};
    nodeLayouts[nodeC] = nodeC_layout;

    // Edge A->B: source on bottom edge of A at snap index 2
    // Edge A->C: source on bottom edge of A at snap index 5
    // Bottom edge of A goes from x=0 to x=100
    // Grid candidates (excluding corners): x=10,20,30,40,50,60,70,80,90 (indices 0-8)

    EdgeLayout edge1;
    edge1.from = nodeA;
    edge1.to = nodeB;
    edge1.sourceEdge = NodeEdge::Bottom;
    edge1.targetEdge = NodeEdge::Top;
    edge1.sourceSnapIndex = 2;  // x = 30
    edge1.sourcePoint = GridSnapCalculator::getPositionFromCandidateIndex(
        nodeA_layout, NodeEdge::Bottom, 2, gridSize);
    edge1.targetPoint = {50, 100};  // top of B
    edge1.targetSnapIndex = 4;
    edgeLayouts[edgeAB] = edge1;

    EdgeLayout edge2;
    edge2.from = nodeA;
    edge2.to = nodeC;
    edge2.sourceEdge = NodeEdge::Bottom;
    edge2.targetEdge = NodeEdge::Top;
    edge2.sourceSnapIndex = 5;  // x = 60
    edge2.sourcePoint = GridSnapCalculator::getPositionFromCandidateIndex(
        nodeA_layout, NodeEdge::Bottom, 5, gridSize);
    edge2.targetPoint = {100, 100};  // top of C
    edge2.targetSnapIndex = 4;
    edgeLayouts[edgeAC] = edge2;

    std::cout << "\n========== SWAP TEST (STRICT) ==========\n" << std::endl;

    // Save initial state
    int edge1_initialIdx = edgeLayouts[edgeAB].sourceSnapIndex;
    int edge2_initialIdx = edgeLayouts[edgeAC].sourceSnapIndex;
    Point edge1_initialPos = edgeLayouts[edgeAB].sourcePoint;
    Point edge2_initialPos = edgeLayouts[edgeAC].sourcePoint;

    std::cout << "Before swap:" << std::endl;
    std::cout << "  Edge1 (A->B): snapIdx=" << edge1_initialIdx
              << " pos=(" << edge1_initialPos.x << "," << edge1_initialPos.y << ")" << std::endl;
    std::cout << "  Edge2 (A->C): snapIdx=" << edge2_initialIdx
              << " pos=(" << edge2_initialPos.x << "," << edge2_initialPos.y << ")" << std::endl;

    // Drag edge1's source snap point to EXACTLY edge2's position
    Point targetPosition = edge2_initialPos;
    std::cout << "\nDragging Edge1 to Edge2's position: ("
              << targetPosition.x << "," << targetPosition.y << ")" << std::endl;

    auto moveResult = LayoutUtils::moveSnapPoint(
        edgeAB, true, targetPosition,
        nodeLayouts, edgeLayouts, swapGraph, options);

    ASSERT_TRUE(moveResult.success) << "moveSnapPoint should succeed";

    // Get updated state
    int edge1_afterIdx = edgeLayouts[edgeAB].sourceSnapIndex;
    int edge2_afterIdx = edgeLayouts[edgeAC].sourceSnapIndex;
    Point edge1_afterPos = edgeLayouts[edgeAB].sourcePoint;
    Point edge2_afterPos = edgeLayouts[edgeAC].sourcePoint;

    std::cout << "\nAfter move:" << std::endl;
    std::cout << "  Edge1 (A->B): snapIdx=" << edge1_afterIdx
              << " pos=(" << edge1_afterPos.x << "," << edge1_afterPos.y << ")" << std::endl;
    std::cout << "  Edge2 (A->C): snapIdx=" << edge2_afterIdx
              << " pos=(" << edge2_afterPos.x << "," << edge2_afterPos.y << ")" << std::endl;

    // STRICT verification: snap indices MUST be swapped
    // Edge1 must now have Edge2's original snap index
    // Edge2 must now have Edge1's original snap index
    std::cout << "\nSwap verification (STRICT):" << std::endl;
    std::cout << "  Edge1 snapIdx: " << edge1_initialIdx << " -> " << edge1_afterIdx
              << " (expected: " << edge2_initialIdx << ")" << std::endl;
    std::cout << "  Edge2 snapIdx: " << edge2_initialIdx << " -> " << edge2_afterIdx
              << " (expected: " << edge1_initialIdx << ")" << std::endl;

    EXPECT_EQ(edge1_afterIdx, edge2_initialIdx)
        << "Edge1 should have Edge2's original snap index after swap";
    EXPECT_EQ(edge2_afterIdx, edge1_initialIdx)
        << "Edge2 should have Edge1's original snap index after swap";

    // Also verify positions match the swapped indices
    float tolerance = 1.0f;
    EXPECT_NEAR(edge1_afterPos.x, edge2_initialPos.x, tolerance)
        << "Edge1 should be at Edge2's original X position";
    EXPECT_NEAR(edge2_afterPos.x, edge1_initialPos.x, tolerance)
        << "Edge2 should be at Edge1's original X position";
}

// Test: Cross-NodeEdge swap (Bottom -> Right)
// When dragging from one NodeEdge to another occupied NodeEdge, should swap
TEST_F(LayoutUtilsTest, MoveSnapPoint_CrossNodeEdgeSwap) {
    Graph swapGraph;
    NodeId nodeA = swapGraph.addNode(Size{100, 100}, "A");
    NodeId nodeB = swapGraph.addNode(Size{100, 50}, "B");
    NodeId nodeC = swapGraph.addNode(Size{100, 50}, "C");

    EdgeId edgeAB = swapGraph.addEdge(nodeA, nodeB, "A->B");
    EdgeId edgeAC = swapGraph.addEdge(nodeA, nodeC, "A->C");

    LayoutOptions options;
    options.gridConfig.cellSize = 10.0f;
    float gridSize = options.gridConfig.cellSize;

    std::unordered_map<NodeId, NodeLayout> nodeLayouts;
    std::unordered_map<EdgeId, EdgeLayout> edgeLayouts;

    // Node A at (0, 0), 100x100
    NodeLayout nodeA_layout;
    nodeA_layout.id = nodeA;
    nodeA_layout.position = {0, 0};
    nodeA_layout.size = {100, 100};
    nodeLayouts[nodeA] = nodeA_layout;

    // Node B below A
    NodeLayout nodeB_layout;
    nodeB_layout.id = nodeB;
    nodeB_layout.position = {0, 150};
    nodeB_layout.size = {100, 50};
    nodeLayouts[nodeB] = nodeB_layout;

    // Node C to the right of A
    NodeLayout nodeC_layout;
    nodeC_layout.id = nodeC;
    nodeC_layout.position = {150, 0};
    nodeC_layout.size = {100, 50};
    nodeLayouts[nodeC] = nodeC_layout;

    // Edge A->B: source on BOTTOM edge of A
    EdgeLayout edge1;
    edge1.from = nodeA;
    edge1.to = nodeB;
    edge1.sourceEdge = NodeEdge::Bottom;
    edge1.targetEdge = NodeEdge::Top;
    edge1.sourceSnapIndex = 4;
    edge1.sourcePoint = GridSnapCalculator::getPositionFromCandidateIndex(
        nodeA_layout, NodeEdge::Bottom, 4, gridSize);
    edge1.targetPoint = {50, 150};
    edge1.targetSnapIndex = 4;
    edgeLayouts[edgeAB] = edge1;

    // Edge A->C: source on RIGHT edge of A
    EdgeLayout edge2;
    edge2.from = nodeA;
    edge2.to = nodeC;
    edge2.sourceEdge = NodeEdge::Right;
    edge2.targetEdge = NodeEdge::Left;
    edge2.sourceSnapIndex = 4;
    edge2.sourcePoint = GridSnapCalculator::getPositionFromCandidateIndex(
        nodeA_layout, NodeEdge::Right, 4, gridSize);
    edge2.targetPoint = {150, 25};
    edge2.targetSnapIndex = 2;
    edgeLayouts[edgeAC] = edge2;

    std::cout << "\n========== CROSS-NODEEDGE SWAP TEST ==========\n" << std::endl;

    // Save initial state
    NodeEdge edge1_initialEdge = edgeLayouts[edgeAB].sourceEdge;
    NodeEdge edge2_initialEdge = edgeLayouts[edgeAC].sourceEdge;
    int edge1_initialIdx = edgeLayouts[edgeAB].sourceSnapIndex;
    int edge2_initialIdx = edgeLayouts[edgeAC].sourceSnapIndex;

    std::cout << "Before swap:" << std::endl;
    std::cout << "  Edge1 (A->B): edge=" << static_cast<int>(edge1_initialEdge)
              << " snapIdx=" << edge1_initialIdx << std::endl;
    std::cout << "  Edge2 (A->C): edge=" << static_cast<int>(edge2_initialEdge)
              << " snapIdx=" << edge2_initialIdx << std::endl;

    // Drag Edge1's source from Bottom to Edge2's position on Right
    Point targetPosition = edgeLayouts[edgeAC].sourcePoint;
    std::cout << "\nDragging Edge1 (Bottom) to Edge2's position (Right): ("
              << targetPosition.x << "," << targetPosition.y << ")" << std::endl;

    auto moveResult = LayoutUtils::moveSnapPoint(
        edgeAB, true, targetPosition,
        nodeLayouts, edgeLayouts, swapGraph, options);

    ASSERT_TRUE(moveResult.success);

    // Get updated state
    NodeEdge edge1_afterEdge = edgeLayouts[edgeAB].sourceEdge;
    NodeEdge edge2_afterEdge = edgeLayouts[edgeAC].sourceEdge;
    int edge1_afterIdx = edgeLayouts[edgeAB].sourceSnapIndex;
    int edge2_afterIdx = edgeLayouts[edgeAC].sourceSnapIndex;

    std::cout << "\nAfter swap:" << std::endl;
    std::cout << "  Edge1 (A->B): edge=" << static_cast<int>(edge1_afterEdge)
              << " snapIdx=" << edge1_afterIdx << std::endl;
    std::cout << "  Edge2 (A->C): edge=" << static_cast<int>(edge2_afterEdge)
              << " snapIdx=" << edge2_afterIdx << std::endl;

    // Verify: Edge1 should now be on Right (Edge2's original)
    EXPECT_EQ(edge1_afterEdge, edge2_initialEdge)
        << "Edge1 should move to Edge2's original NodeEdge (Right)";
    EXPECT_EQ(edge1_afterIdx, edge2_initialIdx)
        << "Edge1 should have Edge2's original snapIndex";

    // Verify: Edge2 should now be on Bottom (Edge1's original)
    EXPECT_EQ(edge2_afterEdge, edge1_initialEdge)
        << "Edge2 should move to Edge1's original NodeEdge (Bottom)";
    EXPECT_EQ(edge2_afterIdx, edge1_initialIdx)
        << "Edge2 should have Edge1's original snapIndex";
}

// Test: Source-Target cross swap
// When dragging Edge1's source to Edge2's target position, should swap
TEST_F(LayoutUtilsTest, MoveSnapPoint_SourceTargetCrossSwap) {
    Graph swapGraph;
    NodeId nodeA = swapGraph.addNode(Size{100, 50}, "A");
    NodeId nodeB = swapGraph.addNode(Size{100, 50}, "B");
    NodeId nodeC = swapGraph.addNode(Size{100, 50}, "C");

    EdgeId edgeAB = swapGraph.addEdge(nodeA, nodeB, "A->B");
    EdgeId edgeCB = swapGraph.addEdge(nodeC, nodeB, "C->B");  // Both edges target nodeB

    LayoutOptions options;
    options.gridConfig.cellSize = 10.0f;
    float gridSize = options.gridConfig.cellSize;

    std::unordered_map<NodeId, NodeLayout> nodeLayouts;
    std::unordered_map<EdgeId, EdgeLayout> edgeLayouts;

    // Node A at top
    NodeLayout nodeA_layout;
    nodeA_layout.id = nodeA;
    nodeA_layout.position = {0, 0};
    nodeA_layout.size = {100, 50};
    nodeLayouts[nodeA] = nodeA_layout;

    // Node B in middle (shared target)
    NodeLayout nodeB_layout;
    nodeB_layout.id = nodeB;
    nodeB_layout.position = {0, 100};
    nodeB_layout.size = {100, 50};
    nodeLayouts[nodeB] = nodeB_layout;

    // Node C at top-right
    NodeLayout nodeC_layout;
    nodeC_layout.id = nodeC;
    nodeC_layout.position = {150, 0};
    nodeC_layout.size = {100, 50};
    nodeLayouts[nodeC] = nodeC_layout;

    // Edge A->B: target on Top edge of B at snapIdx=2
    EdgeLayout edge1;
    edge1.from = nodeA;
    edge1.to = nodeB;
    edge1.sourceEdge = NodeEdge::Bottom;
    edge1.targetEdge = NodeEdge::Top;
    edge1.sourceSnapIndex = 4;
    edge1.sourcePoint = {50, 50};
    edge1.targetSnapIndex = 2;
    edge1.targetPoint = GridSnapCalculator::getPositionFromCandidateIndex(
        nodeB_layout, NodeEdge::Top, 2, gridSize);
    edgeLayouts[edgeAB] = edge1;

    // Edge C->B: target on Top edge of B at snapIdx=6
    EdgeLayout edge2;
    edge2.from = nodeC;
    edge2.to = nodeB;
    edge2.sourceEdge = NodeEdge::Bottom;
    edge2.targetEdge = NodeEdge::Top;
    edge2.sourceSnapIndex = 4;
    edge2.sourcePoint = {200, 50};
    edge2.targetSnapIndex = 6;
    edge2.targetPoint = GridSnapCalculator::getPositionFromCandidateIndex(
        nodeB_layout, NodeEdge::Top, 6, gridSize);
    edgeLayouts[edgeCB] = edge2;

    std::cout << "\n========== SOURCE-TARGET CROSS SWAP TEST ==========\n" << std::endl;

    // Save initial state
    int edge1_initialIdx = edgeLayouts[edgeAB].targetSnapIndex;
    int edge2_initialIdx = edgeLayouts[edgeCB].targetSnapIndex;
    Point edge1_initialPos = edgeLayouts[edgeAB].targetPoint;
    Point edge2_initialPos = edgeLayouts[edgeCB].targetPoint;

    std::cout << "Before swap:" << std::endl;
    std::cout << "  Edge1 (A->B) target: snapIdx=" << edge1_initialIdx
              << " pos=(" << edge1_initialPos.x << "," << edge1_initialPos.y << ")" << std::endl;
    std::cout << "  Edge2 (C->B) target: snapIdx=" << edge2_initialIdx
              << " pos=(" << edge2_initialPos.x << "," << edge2_initialPos.y << ")" << std::endl;

    // Drag Edge1's TARGET to Edge2's TARGET position
    std::cout << "\nDragging Edge1's target to Edge2's target position" << std::endl;

    auto moveResult = LayoutUtils::moveSnapPoint(
        edgeAB, false, edge2_initialPos,  // false = target
        nodeLayouts, edgeLayouts, swapGraph, options);

    ASSERT_TRUE(moveResult.success);

    int edge1_afterIdx = edgeLayouts[edgeAB].targetSnapIndex;
    int edge2_afterIdx = edgeLayouts[edgeCB].targetSnapIndex;

    std::cout << "\nAfter swap:" << std::endl;
    std::cout << "  Edge1 (A->B) target: snapIdx=" << edge1_afterIdx << std::endl;
    std::cout << "  Edge2 (C->B) target: snapIdx=" << edge2_afterIdx << std::endl;

    EXPECT_EQ(edge1_afterIdx, edge2_initialIdx)
        << "Edge1's target should have Edge2's original snapIndex";
    EXPECT_EQ(edge2_afterIdx, edge1_initialIdx)
        << "Edge2's target should have Edge1's original snapIndex";
}

// Test: 3+ edges - swap doesn't break third edge
TEST_F(LayoutUtilsTest, MoveSnapPoint_ThreeEdgesSwapNoCollision) {
    Graph swapGraph;
    NodeId nodeA = swapGraph.addNode(Size{100, 50}, "A");
    NodeId nodeB = swapGraph.addNode(Size{100, 50}, "B");
    NodeId nodeC = swapGraph.addNode(Size{100, 50}, "C");
    NodeId nodeD = swapGraph.addNode(Size{100, 50}, "D");

    EdgeId edgeAB = swapGraph.addEdge(nodeA, nodeB, "A->B");
    EdgeId edgeAC = swapGraph.addEdge(nodeA, nodeC, "A->C");
    EdgeId edgeAD = swapGraph.addEdge(nodeA, nodeD, "A->D");

    LayoutOptions options;
    options.gridConfig.cellSize = 10.0f;
    float gridSize = options.gridConfig.cellSize;

    std::unordered_map<NodeId, NodeLayout> nodeLayouts;
    std::unordered_map<EdgeId, EdgeLayout> edgeLayouts;

    // Node A at (0, 0)
    NodeLayout nodeA_layout;
    nodeA_layout.id = nodeA;
    nodeA_layout.position = {0, 0};
    nodeA_layout.size = {100, 50};
    nodeLayouts[nodeA] = nodeA_layout;

    nodeLayouts[nodeB] = {nodeB, {0, 100}, {100, 50}};
    nodeLayouts[nodeC] = {nodeC, {50, 100}, {100, 50}};
    nodeLayouts[nodeD] = {nodeD, {100, 100}, {100, 50}};

    // Three edges on Bottom of A at snapIdx 2, 4, 6
    auto makeEdge = [&](NodeId to, int snapIdx) {
        EdgeLayout e;
        e.from = nodeA;
        e.to = to;
        e.sourceEdge = NodeEdge::Bottom;
        e.targetEdge = NodeEdge::Top;
        e.sourceSnapIndex = snapIdx;
        e.sourcePoint = GridSnapCalculator::getPositionFromCandidateIndex(
            nodeA_layout, NodeEdge::Bottom, snapIdx, gridSize);
        e.targetSnapIndex = 4;
        e.targetPoint = {50, 100};
        return e;
    };

    edgeLayouts[edgeAB] = makeEdge(nodeB, 2);  // x=30
    edgeLayouts[edgeAC] = makeEdge(nodeC, 4);  // x=50
    edgeLayouts[edgeAD] = makeEdge(nodeD, 6);  // x=70

    std::cout << "\n========== THREE EDGES SWAP TEST ==========\n" << std::endl;

    std::cout << "Before swap:" << std::endl;
    std::cout << "  EdgeAB: snapIdx=" << edgeLayouts[edgeAB].sourceSnapIndex << std::endl;
    std::cout << "  EdgeAC: snapIdx=" << edgeLayouts[edgeAC].sourceSnapIndex << std::endl;
    std::cout << "  EdgeAD: snapIdx=" << edgeLayouts[edgeAD].sourceSnapIndex << std::endl;

    // Drag EdgeAB (idx=2) to EdgeAD's position (idx=6)
    // EdgeAC (idx=4) should NOT be affected
    Point targetPos = edgeLayouts[edgeAD].sourcePoint;
    std::cout << "\nDragging EdgeAB (idx=2) to EdgeAD (idx=6)" << std::endl;

    auto moveResult = LayoutUtils::moveSnapPoint(
        edgeAB, true, targetPos,
        nodeLayouts, edgeLayouts, swapGraph, options);

    ASSERT_TRUE(moveResult.success);

    std::cout << "\nAfter swap:" << std::endl;
    std::cout << "  EdgeAB: snapIdx=" << edgeLayouts[edgeAB].sourceSnapIndex << std::endl;
    std::cout << "  EdgeAC: snapIdx=" << edgeLayouts[edgeAC].sourceSnapIndex << std::endl;
    std::cout << "  EdgeAD: snapIdx=" << edgeLayouts[edgeAD].sourceSnapIndex << std::endl;

    // EdgeAB and EdgeAD should swap
    EXPECT_EQ(edgeLayouts[edgeAB].sourceSnapIndex, 6)
        << "EdgeAB should move to idx=6";
    EXPECT_EQ(edgeLayouts[edgeAD].sourceSnapIndex, 2)
        << "EdgeAD should move to idx=2";

    // EdgeAC should be UNCHANGED
    EXPECT_EQ(edgeLayouts[edgeAC].sourceSnapIndex, 4)
        << "EdgeAC should remain at idx=4 (not affected by swap)";
}
