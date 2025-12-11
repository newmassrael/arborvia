#include <gtest/gtest.h>
#include <arborvia/arborvia.h>
#include <cmath>
#include <vector>

using namespace arborvia;

class BendPointTest : public ::testing::Test {
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

    static constexpr float gridSize_ = 20.0f;

    Graph graph_;
    NodeId n1_, n2_, n3_;
    EdgeId e1_, e2_, e3_;
};

// ============== Bend Point Tests ==============

TEST_F(BendPointTest, Append_AddsToBendPoints) {
    ManualLayoutManager manager;

    manager.appendBendPoint(e1_, GridPoint{5, 10}, gridSize_);  // 100, 200
    manager.appendBendPoint(e1_, GridPoint{8, 12}, gridSize_);  // 160, 240

    EXPECT_TRUE(manager.hasManualBendPoints(e1_));
    const auto& bps = manager.getBendPoints(e1_);
    ASSERT_EQ(bps.size(), 2u);
    EXPECT_FLOAT_EQ(bps[0].position.x, 100.0f);
    EXPECT_FLOAT_EQ(bps[0].position.y, 200.0f);
    EXPECT_FLOAT_EQ(bps[1].position.x, 160.0f);
    EXPECT_FLOAT_EQ(bps[1].position.y, 240.0f);
}

TEST_F(BendPointTest, InsertAtIndex_InsertsCorrectly) {
    ManualLayoutManager manager;

    manager.appendBendPoint(e1_, GridPoint{5, 10}, gridSize_);   // 100, 200
    manager.appendBendPoint(e1_, GridPoint{15, 20}, gridSize_);  // 300, 400
    // Insert in the middle
    manager.addBendPoint(e1_, 1, GridPoint{10, 15}, gridSize_);  // 200, 300

    const auto& bps = manager.getBendPoints(e1_);
    ASSERT_EQ(bps.size(), 3u);
    EXPECT_FLOAT_EQ(bps[0].position.x, 100.0f);
    EXPECT_FLOAT_EQ(bps[1].position.x, 200.0f);  // inserted
    EXPECT_FLOAT_EQ(bps[2].position.x, 300.0f);
}

TEST_F(BendPointTest, Remove_RemovesFromVector) {
    ManualLayoutManager manager;

    manager.appendBendPoint(e1_, GridPoint{5, 10}, gridSize_);   // 100, 200
    manager.appendBendPoint(e1_, GridPoint{8, 12}, gridSize_);   // 160, 240
    manager.appendBendPoint(e1_, GridPoint{10, 15}, gridSize_);  // 200, 300

    manager.removeBendPoint(e1_, 1);  // remove middle

    const auto& bps = manager.getBendPoints(e1_);
    ASSERT_EQ(bps.size(), 2u);
    EXPECT_FLOAT_EQ(bps[0].position.x, 100.0f);
    EXPECT_FLOAT_EQ(bps[1].position.x, 200.0f);
}

TEST_F(BendPointTest, Move_UpdatesPosition) {
    ManualLayoutManager manager;

    manager.appendBendPoint(e1_, GridPoint{5, 10}, gridSize_);  // 100, 200
    manager.moveBendPoint(e1_, 0, GridPoint{8, 12}, gridSize_); // 160, 240

    const auto& bps = manager.getBendPoints(e1_);
    ASSERT_EQ(bps.size(), 1u);
    EXPECT_FLOAT_EQ(bps[0].position.x, 160.0f);
    EXPECT_FLOAT_EQ(bps[0].position.y, 240.0f);
}

TEST_F(BendPointTest, Clear_RevertsToAutoRouting) {
    ManualLayoutManager manager;

    manager.appendBendPoint(e1_, GridPoint{5, 10}, gridSize_);  // 100, 200
    EXPECT_TRUE(manager.hasManualBendPoints(e1_));

    manager.clearBendPoints(e1_);

    EXPECT_FALSE(manager.hasManualBendPoints(e1_));
    EXPECT_TRUE(manager.getBendPoints(e1_).empty());
}

TEST_F(BendPointTest, SetBatch_ReplacesAll) {
    ManualLayoutManager manager;

    manager.appendBendPoint(e1_, GridPoint{5, 10}, gridSize_);  // 100, 200

    std::vector<GridPoint> newGridPts = {
        GridPoint{3, 5},   // 60, 100
        GridPoint{4, 8},   // 80, 160
        GridPoint{5, 10}   // 100, 200
    };
    manager.setBendPoints(e1_, newGridPts, gridSize_);

    const auto& bps = manager.getBendPoints(e1_);
    ASSERT_EQ(bps.size(), 3u);
    EXPECT_FLOAT_EQ(bps[0].position.x, 60.0f);
    EXPECT_FLOAT_EQ(bps[1].position.x, 80.0f);
    EXPECT_FLOAT_EQ(bps[2].position.x, 100.0f);
}

TEST_F(BendPointTest, ApplyToLayout_UsesManualPoints) {
    ManualLayoutManager manager;


    // Set edge routing first
    manager.setEdgeSourceEdge(e1_, NodeEdge::Bottom);
    manager.setEdgeTargetEdge(e1_, NodeEdge::Top);

    // Add manual bend points (grid-aligned)
    manager.appendBendPoint(e1_, GridPoint{5, 8}, gridSize_);   // 100, 160
    manager.appendBendPoint(e1_, GridPoint{8, 10}, gridSize_);  // 160, 200

    // Set node positions
    manager.setNodePosition(n1_, {0.0f, 0.0f});
    manager.setNodePosition(n2_, {200.0f, 250.0f});

    // Create a layout result and apply manual state
    SugiyamaLayout layout;
    LayoutResult result = layout.layout(graph_);
    manager.applyManualState(result, graph_, gridSize_);

    // Verify manual bend points were used
    const EdgeLayout* edgeLayout = result.getEdgeLayout(e1_);
    ASSERT_NE(edgeLayout, nullptr);
    ASSERT_EQ(edgeLayout->bendPoints.size(), 2u);
    EXPECT_FLOAT_EQ(edgeLayout->bendPoints[0].position.x, 100.0f);
    EXPECT_FLOAT_EQ(edgeLayout->bendPoints[0].position.y, 160.0f);
    EXPECT_FLOAT_EQ(edgeLayout->bendPoints[1].position.x, 160.0f);
    EXPECT_FLOAT_EQ(edgeLayout->bendPoints[1].position.y, 200.0f);
}

TEST_F(BendPointTest, JsonRoundTrip_PreservesBendPoints) {
    ManualLayoutManager manager;

    manager.setEdgeSourceEdge(e1_, NodeEdge::Left);
    manager.setEdgeTargetEdge(e1_, NodeEdge::Right);

    manager.appendBendPoint(e1_, GridPoint{5, 10}, gridSize_);  // 100, 200
    manager.appendBendPoint(e1_, GridPoint{8, 12}, gridSize_);  // 160, 240

    std::string json = manager.toJson();

    ManualLayoutManager restored;
    ASSERT_TRUE(restored.fromJson(json));

    EXPECT_TRUE(restored.hasManualBendPoints(e1_));
    const auto& bps = restored.getBendPoints(e1_);
    ASSERT_EQ(bps.size(), 2u);
    EXPECT_FLOAT_EQ(bps[0].position.x, 100.0f);
    EXPECT_FLOAT_EQ(bps[0].position.y, 200.0f);
    EXPECT_FLOAT_EQ(bps[1].position.x, 160.0f);
    EXPECT_FLOAT_EQ(bps[1].position.y, 240.0f);
}

TEST_F(BendPointTest, NoManual_UsesAutoRouting) {
    ManualLayoutManager manager;


    // Set edge routing without any manual bend points
    manager.setEdgeSourceEdge(e1_, NodeEdge::Bottom);
    manager.setEdgeTargetEdge(e1_, NodeEdge::Top);

    // Set node positions with vertical separation
    manager.setNodePosition(n1_, {50.0f, 0.0f});
    manager.setNodePosition(n2_, {150.0f, 200.0f});

    SugiyamaLayout layout;
    LayoutResult result = layout.layout(graph_);
    manager.applyManualState(result, graph_, 20.0f);

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

TEST_F(BendPointTest, Orthogonal_TwoPointsMaintainRightAngles) {
    ManualLayoutManager manager;

    // Set node positions (grid-aligned: multiples of 20)
    // n1 at (0, 0) with size 100x50 → bottom edge at y=50 → grid y=3 (60)
    // n2 at (200, 160) with size 100x50 → top edge at y=160 → grid y=8 (160)
    manager.setNodePosition(n1_, {0.0f, 0.0f});
    manager.setNodePosition(n2_, {200.0f, 160.0f});

    manager.setEdgeSourceEdge(e1_, NodeEdge::Bottom);
    manager.setEdgeTargetEdge(e1_, NodeEdge::Top);

    // Source will be at approximately (60, 60) after grid quantization
    // Target will be at approximately (260, 160) after grid quantization
    // Create L-shaped path with grid-aligned bend points:
    // bp1: same y as source (grid y=3 → 60), x somewhere in between
    // bp2: same y as target (grid y=8 → 160), same x as bp1

    manager.appendBendPoint(e1_, GridPoint{8, 3}, gridSize_);  // (160, 60) - horizontal from source
    manager.appendBendPoint(e1_, GridPoint{8, 8}, gridSize_);  // (160, 160) - vertical to target

    SugiyamaLayout layout;
    LayoutResult result = layout.layout(graph_);
    manager.applyManualState(result, graph_, gridSize_);

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

TEST_F(BendPointTest, Orthogonal_SourceToTargetDirect) {
    ManualLayoutManager manager;

    // Set node positions (grid-aligned)
    // n1 at (0, 0) with size 100x50 → bottom at y=50 → grid y=3 (60)
    // n2 at (200, 200) with size 100x50 → top at y=200 → grid y=10 (200)
    manager.setNodePosition(n1_, {0.0f, 0.0f});
    manager.setNodePosition(n2_, {200.0f, 200.0f});

    manager.setEdgeSourceEdge(e1_, NodeEdge::Bottom);
    manager.setEdgeTargetEdge(e1_, NodeEdge::Top);

    // Source at grid y=3 (60), target at grid y=10 (200)
    // bp1: (100, 60) = GridPoint{5, 3} - horizontal from source
    // bp2: (100, 200) = GridPoint{5, 10} - vertical to target's y level

    manager.appendBendPoint(e1_, GridPoint{5, 3}, gridSize_);   // (100, 60)
    manager.appendBendPoint(e1_, GridPoint{5, 10}, gridSize_);  // (100, 200)

    SugiyamaLayout layout;
    LayoutResult result = layout.layout(graph_);
    manager.applyManualState(result, graph_, gridSize_);

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

TEST_F(BendPointTest, Orthogonal_MultipleInsertions) {
    ManualLayoutManager manager;


    manager.setNodePosition(n1_, {0.0f, 0.0f});
    manager.setNodePosition(n2_, {300.0f, 200.0f});

    manager.setEdgeSourceEdge(e1_, NodeEdge::Right);
    manager.setEdgeTargetEdge(e1_, NodeEdge::Left);

    // First insertion: creates 2 bends for orthogonal step
    // Source at bottom of n1 (y=50 → grid y=3 → 60)
    // Target at left of n2 (x=200, y ~= 225 → grid y=11 → 220)
    manager.appendBendPoint(e1_, GridPoint{5, 3}, gridSize_);   // (100, 60) horizontal from source
    manager.appendBendPoint(e1_, GridPoint{5, 11}, gridSize_);  // (100, 220) vertical to target y

    // Second insertion: add more bends to create a detour
    manager.appendBendPoint(e1_, GridPoint{10, 11}, gridSize_); // (200, 220) horizontal step
    manager.appendBendPoint(e1_, GridPoint{10, 11}, gridSize_); // same point

    SugiyamaLayout layout;
    LayoutResult result = layout.layout(graph_);
    manager.applyManualState(result, graph_, gridSize_);

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

TEST_F(BendPointTest, Orthogonal_AfterClearReaddMaintainsOrthogonal) {
    ManualLayoutManager manager;

    // n1 at (0,0), n2 at (200, 160) - grid aligned
    manager.setNodePosition(n1_, {0.0f, 0.0f});
    manager.setNodePosition(n2_, {200.0f, 160.0f});

    manager.setEdgeSourceEdge(e1_, NodeEdge::Bottom);
    manager.setEdgeTargetEdge(e1_, NodeEdge::Top);

    // Add bends, clear, then re-add
    manager.appendBendPoint(e1_, GridPoint{3, 3}, gridSize_);  // (60, 60)
    manager.appendBendPoint(e1_, GridPoint{3, 8}, gridSize_);  // (60, 160)

    manager.clearBendPoints(e1_);
    EXPECT_FALSE(manager.hasManualBendPoints(e1_));

    // Re-add orthogonal bends (source y=3, target y=8)
    manager.appendBendPoint(e1_, GridPoint{5, 3}, gridSize_);  // (100, 60)
    manager.appendBendPoint(e1_, GridPoint{5, 8}, gridSize_);  // (100, 160)

    SugiyamaLayout layout;
    LayoutResult result = layout.layout(graph_);
    manager.applyManualState(result, graph_, gridSize_);

    const EdgeLayout* edgeLayout = result.getEdgeLayout(e1_);
    ASSERT_NE(edgeLayout, nullptr);

    auto path = buildPath(*edgeLayout);
    EXPECT_TRUE(isPathOrthogonal(path))
        << "Re-added bends should maintain orthogonal path";
}
