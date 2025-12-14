#include <gtest/gtest.h>

#include "arborvia/core/Types.h"
#include "arborvia/layout/config/LayoutResult.h"
#include "arborvia/layout/config/LayoutOptions.h"

// Internal headers
#include "../../../src/layout/optimization/astar/AStarEdgeOptimizer.h"
#include "../../../src/layout/sugiyama/routing/PathIntersection.h"
#include "arborvia/layout/api/EdgePenaltySystem.h"
#include "../../../src/layout/optimization/BuiltinPenalties.h"

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
// EdgePenaltySystem Tests
// =============================================================================

class EdgePenaltySystemTest : public ::testing::Test {
protected:
    std::shared_ptr<EdgePenaltySystem> penaltySystem_;
    std::unordered_map<NodeId, NodeLayout> nodeLayouts_;
    std::unordered_map<EdgeId, EdgeLayout> edgeLayouts_;
    std::vector<ForbiddenZone> emptyZones_;

    void SetUp() override {
        penaltySystem_ = EdgePenaltySystem::createDefault();
        // Create two nodes
        nodeLayouts_[0] = NodeLayout{0, Point{0, 0}, Size{100, 50}};
        nodeLayouts_[1] = NodeLayout{1, Point{200, 0}, Size{100, 50}};
    }

    PenaltyContext createContext(const EdgeLayout& layout) {
        PenaltyContext ctx{edgeLayouts_, nodeLayouts_, emptyZones_, 20.0f};
        ctx.sourceNodeId = layout.from;
        ctx.targetNodeId = layout.to;
        return ctx;
    }
};

TEST_F(EdgePenaltySystemTest, TooCloseSnapPenalty_FarApart_ReturnsZero) {
    TooCloseSnapPenalty penalty;

    EdgeLayout layout;
    layout.from = 0;
    layout.to = 1;
    layout.sourcePoint = Point{100, 25};
    layout.targetPoint = Point{200, 25};
    layout.sourceEdge = NodeEdge::Right;
    layout.targetEdge = NodeEdge::Left;

    auto ctx = createContext(layout);
    EXPECT_EQ(0, penalty.calculatePenalty(layout, ctx));
}

TEST_F(EdgePenaltySystemTest, TooCloseSnapPenalty_TooClose_ReturnsPenalty) {
    TooCloseSnapPenalty penalty;

    EdgeLayout layout;
    layout.from = 0;
    layout.to = 1;
    layout.sourcePoint = Point{100, 25};
    layout.targetPoint = Point{110, 25};  // Within default minSnapDistance (60)
    layout.sourceEdge = NodeEdge::Left;
    layout.targetEdge = NodeEdge::Left;

    auto ctx = createContext(layout);
    EXPECT_GT(penalty.calculatePenalty(layout, ctx), 0);
}

TEST_F(EdgePenaltySystemTest, SelfOverlapPenalty_NoOverlap_ReturnsZero) {
    SelfOverlapPenalty penalty;

    EdgeLayout layout;
    layout.from = 0;
    layout.to = 1;
    layout.sourcePoint = Point{100, 25};
    layout.sourceEdge = NodeEdge::Right;
    layout.targetPoint = Point{200, 25};
    layout.targetEdge = NodeEdge::Left;

    auto ctx = createContext(layout);
    EXPECT_EQ(0, penalty.calculatePenalty(layout, ctx));
}

TEST_F(EdgePenaltySystemTest, SelfOverlapPenalty_WithOverlap_ReturnsPenalty) {
    SelfOverlapPenalty penalty;

    EdgeLayout layout;
    layout.from = 0;
    layout.to = 1;
    layout.sourcePoint = Point{100, 25};
    layout.sourceEdge = NodeEdge::Left;  // Exit left: x1 = 100 - 20 = 80
    layout.targetPoint = Point{50, 25};
    layout.targetEdge = NodeEdge::Left;  // Exit left: x2 = 50 - 20 = 30
    // x1(80) > x2(30) → overlap detected

    auto ctx = createContext(layout);
    EXPECT_GT(penalty.calculatePenalty(layout, ctx), 0);
}

TEST_F(EdgePenaltySystemTest, NodeCollisionPenalty_NoCollision) {
    NodeCollisionPenalty penalty;

    EdgeLayout layout;
    layout.from = 0;
    layout.to = 1;
    layout.sourcePoint = Point{100, 25};
    layout.targetPoint = Point{200, 25};

    auto ctx = createContext(layout);
    EXPECT_EQ(0, penalty.calculatePenalty(layout, ctx));
}

TEST_F(EdgePenaltySystemTest, NodeCollisionPenalty_WithCollision) {
    NodeCollisionPenalty penalty;

    // Add a node in the middle
    nodeLayouts_[2] = NodeLayout{2, Point{140, 10}, Size{30, 30}};

    EdgeLayout layout;
    layout.from = 0;
    layout.to = 1;
    layout.sourcePoint = Point{100, 25};
    layout.targetPoint = Point{200, 25};

    auto ctx = createContext(layout);
    EXPECT_GT(penalty.calculatePenalty(layout, ctx), 0);
}

TEST_F(EdgePenaltySystemTest, CalculateTotalPenalty_NoViolations) {
    EdgeLayout layout;
    layout.from = 0;
    layout.to = 1;
    layout.sourcePoint = Point{100, 25};
    layout.sourceEdge = NodeEdge::Right;
    layout.targetPoint = Point{200, 25};
    layout.targetEdge = NodeEdge::Left;

    auto ctx = createContext(layout);
    int score = penaltySystem_->calculateTotalPenalty(layout, ctx);

    // Should be 0 since no violations
    EXPECT_EQ(0, score);
}

TEST_F(EdgePenaltySystemTest, PassesHardConstraints_ValidLayout) {
    EdgeLayout layout;
    layout.from = 0;
    layout.to = 1;
    layout.sourcePoint = Point{100, 25};
    layout.sourceEdge = NodeEdge::Right;
    layout.targetPoint = Point{200, 25};
    layout.targetEdge = NodeEdge::Left;

    auto ctx = createContext(layout);
    EXPECT_TRUE(penaltySystem_->passesHardConstraints(layout, ctx));
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

    auto result = optimizer.optimize({0}, edgeLayouts_, nodeLayouts_, 20.0f);

    ASSERT_EQ(1u, result.size());
    ASSERT_TRUE(result.find(0) != result.end());

    const auto& optimized = result[0];
    EXPECT_TRUE(optimized.sourceEdge == NodeEdge::Top ||
                optimized.sourceEdge == NodeEdge::Bottom ||
                optimized.sourceEdge == NodeEdge::Left ||
                optimized.sourceEdge == NodeEdge::Right);
}

TEST_F(AStarEdgeOptimizerTest, Optimize_SelfLoop_ProducesValidAdjacentEdges) {
    AStarEdgeOptimizer optimizer;

    EdgeLayout selfLoop;
    selfLoop.id = 0;
    selfLoop.from = 0;
    selfLoop.to = 0;
    // Self-loops use SelfLoopRouter which determines the best direction
    // via penalty scoring. We just verify the result uses valid adjacent edges.
    selfLoop.sourcePoint = Point{150, 0};
    selfLoop.sourceEdge = NodeEdge::Right;
    selfLoop.targetPoint = Point{100, 0};
    selfLoop.targetEdge = NodeEdge::Top;

    edgeLayouts_[0] = selfLoop;

    auto result = optimizer.optimize({0}, edgeLayouts_, nodeLayouts_, 20.0f);

    ASSERT_EQ(1u, result.size());

    // Verify the result uses valid adjacent edges (self-loop constraint)
    // Adjacent means: not same edge and not opposite edge
    NodeEdge srcEdge = result[0].sourceEdge;
    NodeEdge tgtEdge = result[0].targetEdge;

    // Self-loop constraint: source and target must be adjacent (not same, not opposite)
    bool isSameEdge = (srcEdge == tgtEdge);
    bool isOpposite = (srcEdge == NodeEdge::Top && tgtEdge == NodeEdge::Bottom) ||
                      (srcEdge == NodeEdge::Bottom && tgtEdge == NodeEdge::Top) ||
                      (srcEdge == NodeEdge::Left && tgtEdge == NodeEdge::Right) ||
                      (srcEdge == NodeEdge::Right && tgtEdge == NodeEdge::Left);

    EXPECT_FALSE(isSameEdge) << "Self-loop should not use same edge for source and target";
    EXPECT_FALSE(isOpposite) << "Self-loop should not use opposite edges";
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

    auto result = optimizer.optimize({0, 1}, edgeLayouts_, nodeLayouts_, 20.0f);

    EXPECT_EQ(2u, result.size());
}

TEST_F(AStarEdgeOptimizerTest, AlgorithmName_ReturnsAStar) {
    AStarEdgeOptimizer optimizer;
    EXPECT_STREQ("AStar", optimizer.algorithmName());
}

TEST_F(AStarEdgeOptimizerTest, SetPenaltySystem_ConfiguresPenalties) {
    AStarEdgeOptimizer optimizer;

    // Default penalty system should be set
    EXPECT_NE(nullptr, optimizer.penaltySystem());

    // Can override with custom penalty system
    auto customSystem = EdgePenaltySystem::createMinimal();
    optimizer.setPenaltySystem(customSystem);

    EXPECT_EQ(customSystem, optimizer.penaltySystem());
}

// =============================================================================
// SelfLoopRouter Snap Index Tests
// =============================================================================

#include "../../../src/layout/sugiyama/routing/SelfLoopRouter.h"
#include "../../../src/layout/snap/GridSnapCalculator.h"

TEST(SelfLoopRouterTest, Route_SetsCorrectSnapIndices) {
    // SelfLoopRouter::route() must set sourceSnapIndex and targetSnapIndex
    // that correctly correspond to the calculated positions
    
    NodeLayout node;
    node.id = 4;
    node.position = {300.0f, 600.0f};
    node.size = {200.0f, 100.0f};
    
    LayoutOptions options;
    options.gridConfig.cellSize = 20.0f;
    
    EdgeLayout layout = SelfLoopRouter::route(7, 4, node, 0, options);
    
    // Verify snap indices match positions
    int expectedSourceIdx = GridSnapCalculator::getCandidateIndexFromPosition(
        node, layout.sourceEdge, layout.sourcePoint, options.gridConfig.cellSize);
    int expectedTargetIdx = GridSnapCalculator::getCandidateIndexFromPosition(
        node, layout.targetEdge, layout.targetPoint, options.gridConfig.cellSize);
    
    EXPECT_EQ(layout.sourceSnapIndex, expectedSourceIdx) 
        << "sourceSnapIndex should match position (" << layout.sourcePoint.x << "," << layout.sourcePoint.y << ")";
    EXPECT_EQ(layout.targetSnapIndex, expectedTargetIdx)
        << "targetSnapIndex should match position (" << layout.targetPoint.x << "," << layout.targetPoint.y << ")";
}

TEST(SelfLoopRouterTest, MultipleSelfLoops_HaveUniqueSnapIndices) {
    // Multiple self-loops on the same node should have different snap indices
    
    NodeLayout node;
    node.id = 4;
    node.position = {300.0f, 600.0f};
    node.size = {200.0f, 100.0f};
    
    LayoutOptions options;
    options.gridConfig.cellSize = 20.0f;
    
    // Create two self-loops with different loop indices
    EdgeLayout layout1 = SelfLoopRouter::route(7, 4, node, 0, options);
    EdgeLayout layout2 = SelfLoopRouter::route(8, 4, node, 1, options);
    
    // They should have different source positions (and thus different indices) if on same edge
    if (layout1.sourceEdge == layout2.sourceEdge) {
        EXPECT_NE(layout1.sourceSnapIndex, layout2.sourceSnapIndex)
            << "Different self-loops on same edge should have different source snap indices";
    }
    
    // Verify each layout's indices match its positions
    int idx1 = GridSnapCalculator::getCandidateIndexFromPosition(
        node, layout1.sourceEdge, layout1.sourcePoint, options.gridConfig.cellSize);
    int idx2 = GridSnapCalculator::getCandidateIndexFromPosition(
        node, layout2.sourceEdge, layout2.sourcePoint, options.gridConfig.cellSize);
    
    EXPECT_EQ(layout1.sourceSnapIndex, idx1);
    EXPECT_EQ(layout2.sourceSnapIndex, idx2);
}
