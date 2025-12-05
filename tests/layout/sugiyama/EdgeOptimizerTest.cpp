#include <gtest/gtest.h>

#include "arborvia/core/Types.h"
#include "arborvia/layout/LayoutResult.h"
#include "arborvia/layout/LayoutOptions.h"

// Internal headers
#include "../../../src/layout/sugiyama/EdgeScorer.h"
#include "../../../src/layout/sugiyama/AStarEdgeOptimizer.h"
#include "../../../src/layout/sugiyama/PathIntersection.h"

using namespace arborvia;

// =============================================================================
// PathIntersection Tests
// =============================================================================

TEST(PathIntersectionTest, SegmentsIntersect_CrossingSegments_ReturnsTrue) {
    // X pattern
    bool result = PathIntersection::segmentsIntersect(
        Point{0, 0}, Point{10, 10},
        Point{0, 10}, Point{10, 0});
    EXPECT_TRUE(result);
}

TEST(PathIntersectionTest, SegmentsIntersect_ParallelSegments_ReturnsFalse) {
    // Parallel horizontal lines
    bool result = PathIntersection::segmentsIntersect(
        Point{0, 0}, Point{10, 0},
        Point{0, 5}, Point{10, 5});
    EXPECT_FALSE(result);
}

TEST(PathIntersectionTest, SegmentsIntersect_NonIntersecting_ReturnsFalse) {
    // L shape that doesn't touch
    bool result = PathIntersection::segmentsIntersect(
        Point{0, 0}, Point{5, 0},
        Point{10, 0}, Point{10, 5});
    EXPECT_FALSE(result);
}

TEST(PathIntersectionTest, SegmentsIntersect_SharedEndpoint_ReturnsFalse) {
    // Segments sharing an endpoint should not count as intersection
    bool result = PathIntersection::segmentsIntersect(
        Point{0, 0}, Point{5, 5},
        Point{5, 5}, Point{10, 0});
    EXPECT_FALSE(result);
}

TEST(PathIntersectionTest, SegmentsIntersect_TShape_ReturnsTrue) {
    // T intersection
    bool result = PathIntersection::segmentsIntersect(
        Point{0, 5}, Point{10, 5},
        Point{5, 0}, Point{5, 10});
    EXPECT_TRUE(result);
}

TEST(PathIntersectionTest, CountPathIntersections_NoIntersections) {
    EdgeLayout e1, e2;
    e1.sourcePoint = Point{0, 0};
    e1.targetPoint = Point{0, 100};

    e2.sourcePoint = Point{50, 0};
    e2.targetPoint = Point{50, 100};

    EXPECT_EQ(0, PathIntersection::countPathIntersections(e1, e2));
}

TEST(PathIntersectionTest, CountPathIntersections_OneIntersection) {
    EdgeLayout e1, e2;
    e1.sourcePoint = Point{0, 50};
    e1.targetPoint = Point{100, 50};

    e2.sourcePoint = Point{50, 0};
    e2.targetPoint = Point{50, 100};

    EXPECT_EQ(1, PathIntersection::countPathIntersections(e1, e2));
}

// =============================================================================
// EdgeScorer Tests
// =============================================================================

class EdgeScorerTest : public ::testing::Test {
protected:
    EdgeScorer scorer_;
    std::unordered_map<NodeId, NodeLayout> nodeLayouts_;
    std::unordered_map<EdgeId, EdgeLayout> edgeLayouts_;

    void SetUp() override {
        // Create two nodes
        nodeLayouts_[0] = NodeLayout{0, Point{0, 0}, Size{100, 50}};
        nodeLayouts_[1] = NodeLayout{1, Point{200, 0}, Size{100, 50}};
    }
};

TEST_F(EdgeScorerTest, CheckSnapPointProximity_FarApart_ReturnsZero) {
    EdgeLayout layout;
    layout.sourcePoint = Point{100, 25};
    layout.targetPoint = Point{200, 25};

    EXPECT_EQ(0, scorer_.checkSnapPointProximity(layout));
}

TEST_F(EdgeScorerTest, CheckSnapPointProximity_TooClose_ReturnsOne) {
    EdgeLayout layout;
    layout.sourcePoint = Point{100, 25};
    layout.targetPoint = Point{110, 25};  // Within default minSnapDistance (60)

    EXPECT_EQ(1, scorer_.checkSnapPointProximity(layout));
}

TEST_F(EdgeScorerTest, CheckSelfOverlap_NoOverlap_ReturnsZero) {
    EdgeLayout layout;
    layout.sourcePoint = Point{100, 25};
    layout.sourceEdge = NodeEdge::Right;
    layout.targetPoint = Point{200, 25};
    layout.targetEdge = NodeEdge::Left;

    EXPECT_EQ(0, scorer_.checkSelfOverlap(layout));
}

TEST_F(EdgeScorerTest, CheckSelfOverlap_WithOverlap_ReturnsOne) {
    EdgeLayout layout;
    layout.sourcePoint = Point{100, 25};
    layout.sourceEdge = NodeEdge::Left;  // Exit left: x1 = 100 - 20 = 80
    layout.targetPoint = Point{50, 25};
    layout.targetEdge = NodeEdge::Left;  // Exit left: x2 = 50 - 20 = 30
    // x1(80) > x2(30) → overlap detected

    EXPECT_EQ(1, scorer_.checkSelfOverlap(layout));
}

TEST_F(EdgeScorerTest, CountNodeCollisions_NoCollision) {
    EdgeLayout layout;
    layout.from = 0;
    layout.to = 1;
    layout.sourcePoint = Point{100, 25};
    layout.targetPoint = Point{200, 25};

    EXPECT_EQ(0, scorer_.countNodeCollisions(layout, nodeLayouts_));
}

TEST_F(EdgeScorerTest, CountNodeCollisions_WithCollision) {
    // Add a node in the middle
    nodeLayouts_[2] = NodeLayout{2, Point{140, 10}, Size{30, 30}};

    EdgeLayout layout;
    layout.from = 0;
    layout.to = 1;
    layout.sourcePoint = Point{100, 25};
    layout.targetPoint = Point{200, 25};

    EXPECT_EQ(1, scorer_.countNodeCollisions(layout, nodeLayouts_));
}

TEST_F(EdgeScorerTest, CalculateScore_CombinesAllComponents) {
    EdgeLayout layout;
    layout.from = 0;
    layout.to = 1;
    layout.sourcePoint = Point{100, 25};
    layout.sourceEdge = NodeEdge::Right;
    layout.targetPoint = Point{200, 25};
    layout.targetEdge = NodeEdge::Left;

    int score = scorer_.calculateScore(layout, edgeLayouts_, nodeLayouts_);

    // Score should be roughly the distance (100 pixels) since no penalties
    EXPECT_GT(score, 0);
    EXPECT_LT(score, 200);
}

TEST_F(EdgeScorerTest, CalculateScore_HighPenaltyForTooClose) {
    EdgeLayout layout;
    layout.from = 0;
    layout.to = 1;
    layout.sourcePoint = Point{100, 25};
    layout.sourceEdge = NodeEdge::Right;
    layout.targetPoint = Point{110, 25};  // Too close
    layout.targetEdge = NodeEdge::Left;

    int score = scorer_.calculateScore(layout, edgeLayouts_, nodeLayouts_);

    EXPECT_GE(score, 100000);
}

TEST_F(EdgeScorerTest, CustomWeights_AffectScore) {
    ScoringWeights customWeights;
    customWeights.tooCloseSnap = 50000;

    EdgeScorer customScorer(customWeights);

    EdgeLayout layout;
    layout.from = 0;
    layout.to = 1;
    // Both Top edges, same y → tooCloseSnap triggers but selfOverlap doesn't
    // (V-H-V path: y1 = y2 = 25-20=5, y1 > y2 is false)
    layout.sourcePoint = Point{100, 25};
    layout.sourceEdge = NodeEdge::Top;
    layout.targetPoint = Point{110, 25};
    layout.targetEdge = NodeEdge::Top;

    int score = customScorer.calculateScore(layout, edgeLayouts_, nodeLayouts_);

    // Score = 50000 (tooCloseSnap) + ~10 (distance)
    EXPECT_GE(score, 50000);
    EXPECT_LT(score, 60000);
}

// =============================================================================
// AStarEdgeOptimizer Tests
// =============================================================================

class AStarEdgeOptimizerTest : public ::testing::Test {
protected:
    std::unordered_map<NodeId, NodeLayout> nodeLayouts_;
    std::unordered_map<EdgeId, EdgeLayout> edgeLayouts_;

    void SetUp() override {
        nodeLayouts_[0] = NodeLayout{0, Point{0, 0}, Size{100, 50}};
        nodeLayouts_[1] = NodeLayout{1, Point{200, 100}, Size{100, 50}};
    }
};

TEST_F(AStarEdgeOptimizerTest, Optimize_SingleEdge_ReturnsOptimizedLayout) {
    AStarEdgeOptimizer optimizer;

    EdgeLayout initial;
    initial.from = 0;
    initial.to = 1;
    initial.sourcePoint = Point{100, 25};
    initial.sourceEdge = NodeEdge::Right;
    initial.targetPoint = Point{200, 125};
    initial.targetEdge = NodeEdge::Left;

    edgeLayouts_[0] = initial;

    auto result = optimizer.optimize({0}, edgeLayouts_, nodeLayouts_);

    ASSERT_EQ(1u, result.size());
    ASSERT_TRUE(result.find(0) != result.end());

    const auto& optimized = result[0];
    EXPECT_TRUE(optimized.sourceEdge == NodeEdge::Top ||
                optimized.sourceEdge == NodeEdge::Bottom ||
                optimized.sourceEdge == NodeEdge::Left ||
                optimized.sourceEdge == NodeEdge::Right);
}

TEST_F(AStarEdgeOptimizerTest, Optimize_SelfLoop_PreservesOriginal) {
    AStarEdgeOptimizer optimizer;

    EdgeLayout selfLoop;
    selfLoop.from = 0;
    selfLoop.to = 0;
    selfLoop.sourcePoint = Point{100, 0};
    selfLoop.sourceEdge = NodeEdge::Top;
    selfLoop.targetPoint = Point{100, 50};
    selfLoop.targetEdge = NodeEdge::Bottom;

    edgeLayouts_[0] = selfLoop;

    auto result = optimizer.optimize({0}, edgeLayouts_, nodeLayouts_);

    ASSERT_EQ(1u, result.size());
    EXPECT_EQ(selfLoop.sourceEdge, result[0].sourceEdge);
    EXPECT_EQ(selfLoop.targetEdge, result[0].targetEdge);
}

TEST_F(AStarEdgeOptimizerTest, Optimize_MultipleEdges_ConsidersIntersections) {
    AStarEdgeOptimizer optimizer;

    nodeLayouts_[2] = NodeLayout{2, Point{200, -50}, Size{100, 50}};

    EdgeLayout e1;
    e1.from = 0;
    e1.to = 1;
    e1.sourcePoint = Point{100, 25};
    e1.sourceEdge = NodeEdge::Right;
    e1.targetPoint = Point{200, 125};
    e1.targetEdge = NodeEdge::Left;

    EdgeLayout e2;
    e2.from = 0;
    e2.to = 2;
    e2.sourcePoint = Point{100, 25};
    e2.sourceEdge = NodeEdge::Right;
    e2.targetPoint = Point{200, -25};
    e2.targetEdge = NodeEdge::Left;

    edgeLayouts_[0] = e1;
    edgeLayouts_[1] = e2;

    auto result = optimizer.optimize({0, 1}, edgeLayouts_, nodeLayouts_);

    EXPECT_EQ(2u, result.size());
}

TEST_F(AStarEdgeOptimizerTest, AlgorithmName_ReturnsAStar) {
    AStarEdgeOptimizer optimizer;
    EXPECT_STREQ("AStar", optimizer.algorithmName());
}

TEST_F(AStarEdgeOptimizerTest, SetWeights_UpdatesScoring) {
    AStarEdgeOptimizer optimizer;

    ScoringWeights newWeights;
    newWeights.tooCloseSnap = 200000;

    optimizer.setWeights(newWeights);

    EXPECT_EQ(200000, optimizer.weights().tooCloseSnap);
}
