#include <gtest/gtest.h>
#include "../../../src/layout/pathfinding/ObstacleMap.h"
#include "arborvia/layout/api/IObstacleProvider.h"

using namespace arborvia;

// =============================================================================
// Test Fixture
// =============================================================================

class ObstacleMapTest : public ::testing::Test {
protected:
    void SetUp() override {
        gridSize_ = 10.0f;

        // Create test nodes
        // Node 1: 100x80 at position (50, 50)
        node1_.id = 1;
        node1_.position = {50.0f, 50.0f};
        node1_.size = {100.0f, 80.0f};  // right=150, bottom=130

        // Node 2: 80x60 at position (250, 50)
        node2_.id = 2;
        node2_.position = {250.0f, 50.0f};
        node2_.size = {80.0f, 60.0f};  // right=330, bottom=110

        nodeLayouts_[1] = node1_;
        nodeLayouts_[2] = node2_;
    }

    NodeLayout node1_;
    NodeLayout node2_;
    std::unordered_map<NodeId, NodeLayout> nodeLayouts_;
    float gridSize_;
    ObstacleMap obstacles_;
};

// =============================================================================
// Basic Grid Construction Tests
// =============================================================================

TEST_F(ObstacleMapTest, BuildFromNodes_CreatesNonEmptyGrid) {
    obstacles_.buildFromNodes(nodeLayouts_, gridSize_, 0);

    EXPECT_GT(obstacles_.width(), 0);
    EXPECT_GT(obstacles_.height(), 0);
    EXPECT_GT(obstacles_.gridSize(), 0.0f);
}

TEST_F(ObstacleMapTest, BuildFromNodes_EmptyInput_CreatesEmptyGrid) {
    std::unordered_map<NodeId, NodeLayout> empty;
    obstacles_.buildFromNodes(empty, gridSize_, 0);

    EXPECT_EQ(obstacles_.width(), 0);
    EXPECT_EQ(obstacles_.height(), 0);
}

TEST_F(ObstacleMapTest, BuildFromNodes_ZeroGridSize_CreatesEmptyGrid) {
    obstacles_.buildFromNodes(nodeLayouts_, 0.0f, 0);

    EXPECT_EQ(obstacles_.width(), 0);
    EXPECT_EQ(obstacles_.height(), 0);
}

TEST_F(ObstacleMapTest, BuildFromNodes_NegativeGridSize_CreatesEmptyGrid) {
    obstacles_.buildFromNodes(nodeLayouts_, -10.0f, 0);

    EXPECT_EQ(obstacles_.width(), 0);
    EXPECT_EQ(obstacles_.height(), 0);
}

// =============================================================================
// Grid Bounds Tests
// =============================================================================

TEST_F(ObstacleMapTest, BuildFromNodes_GridBoundsContainAllNodes) {
    obstacles_.buildFromNodes(nodeLayouts_, gridSize_, 0);

    // Check that all node corners are within grid bounds
    for (const auto& [nodeId, node] : nodeLayouts_) {
        GridPoint topLeft = obstacles_.pixelToGrid({node.position.x, node.position.y});
        GridPoint bottomRight = obstacles_.pixelToGrid({
            node.position.x + node.size.width,
            node.position.y + node.size.height
        });

        EXPECT_TRUE(obstacles_.inBounds(topLeft.x, topLeft.y))
            << "Node " << nodeId << " top-left should be in bounds";
        EXPECT_TRUE(obstacles_.inBounds(bottomRight.x, bottomRight.y))
            << "Node " << nodeId << " bottom-right should be in bounds";
    }
}

TEST_F(ObstacleMapTest, BuildFromNodes_WithMargin_ExtendsBlockedArea) {
    obstacles_.buildFromNodes(nodeLayouts_, gridSize_, 1);  // margin=1

    // Cell at margin distance from node should still be blocked
    GridPoint nodeCenter = obstacles_.pixelToGrid({
        node1_.position.x + node1_.size.width / 2,
        node1_.position.y + node1_.size.height / 2
    });

    EXPECT_TRUE(obstacles_.isBlocked(nodeCenter.x, nodeCenter.y))
        << "Node center should be blocked";
}

// =============================================================================
// isBlocked Tests
// =============================================================================

TEST_F(ObstacleMapTest, IsBlocked_InsideNode_ReturnsTrue) {
    obstacles_.buildFromNodes(nodeLayouts_, gridSize_, 0);

    // Center of node1 should be blocked
    GridPoint center = obstacles_.pixelToGrid({
        node1_.position.x + node1_.size.width / 2,
        node1_.position.y + node1_.size.height / 2
    });

    EXPECT_TRUE(obstacles_.isBlocked(center.x, center.y));
}

TEST_F(ObstacleMapTest, IsBlocked_OutsideNodes_ReturnsFalse) {
    obstacles_.buildFromNodes(nodeLayouts_, gridSize_, 0);

    // Point between node1 and node2 should not be blocked
    GridPoint between = obstacles_.pixelToGrid({200.0f, 80.0f});

    EXPECT_FALSE(obstacles_.isBlocked(between.x, between.y));
}

TEST_F(ObstacleMapTest, IsBlocked_OutOfBounds_ReturnsFalse) {
    obstacles_.buildFromNodes(nodeLayouts_, gridSize_, 0);

    // Far outside grid bounds
    EXPECT_FALSE(obstacles_.isBlocked(-1000, -1000));
    EXPECT_FALSE(obstacles_.isBlocked(1000, 1000));
}

TEST_F(ObstacleMapTest, IsBlocked_WithExclude_ExcludesSpecifiedNodes) {
    obstacles_.buildFromNodes(nodeLayouts_, gridSize_, 0);

    // Center of node1
    GridPoint center = obstacles_.pixelToGrid({
        node1_.position.x + node1_.size.width / 2,
        node1_.position.y + node1_.size.height / 2
    });

    // Without exclusion - blocked
    EXPECT_TRUE(obstacles_.isBlocked(center.x, center.y));

    // With node1 excluded - not blocked
    std::unordered_set<NodeId> exclude = {1};
    EXPECT_FALSE(obstacles_.isBlocked(center.x, center.y, exclude));
}

// =============================================================================
// Coordinate Conversion Tests
// =============================================================================

TEST_F(ObstacleMapTest, PixelToGrid_ConvertsCorrectly) {
    obstacles_.buildFromNodes(nodeLayouts_, gridSize_, 0);

    Point pixel = {100.0f, 50.0f};
    GridPoint grid = obstacles_.pixelToGrid(pixel);

    EXPECT_EQ(grid.x, 10);  // 100 / 10 = 10
    EXPECT_EQ(grid.y, 5);   // 50 / 10 = 5
}

TEST_F(ObstacleMapTest, GridToPixel_ConvertsCorrectly) {
    obstacles_.buildFromNodes(nodeLayouts_, gridSize_, 0);

    Point pixel = obstacles_.gridToPixel(10, 5);

    EXPECT_FLOAT_EQ(pixel.x, 100.0f);
    EXPECT_FLOAT_EQ(pixel.y, 50.0f);
}

TEST_F(ObstacleMapTest, PixelToGrid_RoundsCorrectly) {
    obstacles_.buildFromNodes(nodeLayouts_, gridSize_, 0);

    // Test rounding: 104.9 should round to 10, 105.1 should round to 11
    GridPoint grid1 = obstacles_.pixelToGrid({104.9f, 54.9f});
    GridPoint grid2 = obstacles_.pixelToGrid({105.1f, 55.1f});

    EXPECT_EQ(grid1.x, 10);
    EXPECT_EQ(grid1.y, 5);
    EXPECT_EQ(grid2.x, 11);
    EXPECT_EQ(grid2.y, 6);
}

// =============================================================================
// inBounds Tests
// =============================================================================

TEST_F(ObstacleMapTest, InBounds_ValidCoordinates_ReturnsTrue) {
    obstacles_.buildFromNodes(nodeLayouts_, gridSize_, 0);

    GridPoint center = obstacles_.pixelToGrid({
        node1_.position.x + node1_.size.width / 2,
        node1_.position.y + node1_.size.height / 2
    });

    EXPECT_TRUE(obstacles_.inBounds(center.x, center.y));
}

TEST_F(ObstacleMapTest, InBounds_OutOfRange_ReturnsFalse) {
    obstacles_.buildFromNodes(nodeLayouts_, gridSize_, 0);

    // Way outside grid
    EXPECT_FALSE(obstacles_.inBounds(-1000, -1000));
    EXPECT_FALSE(obstacles_.inBounds(1000, 1000));
}

// =============================================================================
// Edge Bounds Expansion Tests (Bug Fix Verification)
// =============================================================================

TEST_F(ObstacleMapTest, BuildFromNodes_WithEdgeLayouts_ExpandsGridBounds) {
    // Create an edge that extends beyond node bounds
    EdgeLayout edge;
    edge.id = 0;
    edge.from = 1;
    edge.to = 2;
    edge.sourcePoint = {150.0f, 90.0f};  // Right side of node1
    edge.targetPoint = {250.0f, 80.0f};  // Left side of node2

    // Edge path goes far below the nodes (outside node bounds)
    edge.bendPoints.push_back({{150.0f, 200.0f}});  // y=200 is below node bounds
    edge.bendPoints.push_back({{250.0f, 200.0f}});

    std::unordered_map<EdgeId, EdgeLayout> edgeLayouts;
    edgeLayouts[0] = edge;

    // Build WITHOUT edge layouts
    ObstacleMap obstaclesWithoutEdges;
    obstaclesWithoutEdges.buildFromNodes(nodeLayouts_, gridSize_, 0, nullptr);

    // Build WITH edge layouts
    ObstacleMap obstaclesWithEdges;
    obstaclesWithEdges.buildFromNodes(nodeLayouts_, gridSize_, 0, &edgeLayouts);

    // Grid with edges should be larger (to include y=200 area)
    EXPECT_GT(obstaclesWithEdges.height(), obstaclesWithoutEdges.height())
        << "Grid with edge layouts should have greater height to include edge path below nodes";
}

TEST_F(ObstacleMapTest, BuildFromNodes_WithEdgeLayouts_EdgeSegmentsInBounds) {
    // Create an edge that extends beyond node bounds
    EdgeLayout edge;
    edge.id = 0;
    edge.from = 1;
    edge.to = 2;
    edge.sourcePoint = {150.0f, 90.0f};
    edge.targetPoint = {250.0f, 80.0f};

    // Edge path goes far below the nodes
    edge.bendPoints.push_back({{150.0f, 250.0f}});  // Far below nodes
    edge.bendPoints.push_back({{250.0f, 250.0f}});

    std::unordered_map<EdgeId, EdgeLayout> edgeLayouts;
    edgeLayouts[0] = edge;

    // Build with edge layouts
    obstacles_.buildFromNodes(nodeLayouts_, gridSize_, 0, &edgeLayouts);

    // Check that the edge segment area is within bounds
    GridPoint edgeBottom = obstacles_.pixelToGrid({200.0f, 250.0f});
    EXPECT_TRUE(obstacles_.inBounds(edgeBottom.x, edgeBottom.y))
        << "Edge segment at y=250 should be within grid bounds";
}

TEST_F(ObstacleMapTest, BuildFromNodes_EdgeLayoutsNullptr_SameAsWithout) {
    // Build with nullptr (default behavior)
    ObstacleMap obstacles1;
    obstacles1.buildFromNodes(nodeLayouts_, gridSize_, 0, nullptr);

    // Build without parameter (uses default nullptr)
    ObstacleMap obstacles2;
    obstacles2.buildFromNodes(nodeLayouts_, gridSize_, 0);

    // Both should have same dimensions
    EXPECT_EQ(obstacles1.width(), obstacles2.width());
    EXPECT_EQ(obstacles1.height(), obstacles2.height());
    EXPECT_EQ(obstacles1.offsetX(), obstacles2.offsetX());
    EXPECT_EQ(obstacles1.offsetY(), obstacles2.offsetY());
}

// =============================================================================
// addEdgeSegments Tests
// =============================================================================

TEST_F(ObstacleMapTest, AddEdgeSegments_RegistersHorizontalSegments) {
    // Create edge with horizontal segment
    EdgeLayout edge;
    edge.id = 0;
    edge.from = 1;
    edge.to = 2;
    edge.sourcePoint = {150.0f, 90.0f};
    edge.targetPoint = {250.0f, 90.0f};  // Same Y = horizontal

    std::unordered_map<EdgeId, EdgeLayout> edgeLayouts;
    edgeLayouts[0] = edge;

    // Build grid with edge layouts included in bounds
    obstacles_.buildFromNodes(nodeLayouts_, gridSize_, 0, &edgeLayouts);
    obstacles_.addEdgeSegments(edgeLayouts, INVALID_EDGE);  // Don't exclude any

    // Middle point of edge segment should be blocked for horizontal movement
    GridPoint midPoint = obstacles_.pixelToGrid({200.0f, 90.0f});
    std::unordered_set<NodeId> noExclude;

    // Horizontal movement should be blocked in BOTH directions
    EXPECT_TRUE(obstacles_.isBlockedForDirection(midPoint.x, midPoint.y,
                                                  MoveDirection::Left, noExclude))
        << "Horizontal segment should block Left movement";
    EXPECT_TRUE(obstacles_.isBlockedForDirection(midPoint.x, midPoint.y,
                                                  MoveDirection::Right, noExclude))
        << "Horizontal segment should block Right movement";

    // Vertical movement should NOT be blocked by horizontal segment
    EXPECT_FALSE(obstacles_.isBlockedForDirection(midPoint.x, midPoint.y,
                                                   MoveDirection::Up, noExclude))
        << "Horizontal segment should NOT block Up movement";
    EXPECT_FALSE(obstacles_.isBlockedForDirection(midPoint.x, midPoint.y,
                                                   MoveDirection::Down, noExclude))
        << "Horizontal segment should NOT block Down movement";
}

TEST_F(ObstacleMapTest, AddEdgeSegments_RegistersVerticalSegments) {
    // Create edge with vertical segment
    EdgeLayout edge;
    edge.id = 0;
    edge.from = 1;
    edge.to = 2;
    edge.sourcePoint = {200.0f, 50.0f};
    edge.targetPoint = {200.0f, 150.0f};  // Same X = vertical

    std::unordered_map<EdgeId, EdgeLayout> edgeLayouts;
    edgeLayouts[0] = edge;

    // Build grid with edge layouts included in bounds
    obstacles_.buildFromNodes(nodeLayouts_, gridSize_, 0, &edgeLayouts);
    obstacles_.addEdgeSegments(edgeLayouts, INVALID_EDGE);

    // Middle point of edge segment
    GridPoint midPoint = obstacles_.pixelToGrid({200.0f, 100.0f});
    std::unordered_set<NodeId> noExclude;

    // Vertical movement should be blocked in BOTH directions
    EXPECT_TRUE(obstacles_.isBlockedForDirection(midPoint.x, midPoint.y,
                                                  MoveDirection::Up, noExclude))
        << "Vertical segment should block Up movement";
    EXPECT_TRUE(obstacles_.isBlockedForDirection(midPoint.x, midPoint.y,
                                                  MoveDirection::Down, noExclude))
        << "Vertical segment should block Down movement";

    // Horizontal movement should NOT be blocked by vertical segment
    EXPECT_FALSE(obstacles_.isBlockedForDirection(midPoint.x, midPoint.y,
                                                   MoveDirection::Left, noExclude))
        << "Vertical segment should NOT block Left movement";
    EXPECT_FALSE(obstacles_.isBlockedForDirection(midPoint.x, midPoint.y,
                                                   MoveDirection::Right, noExclude))
        << "Vertical segment should NOT block Right movement";
}

TEST_F(ObstacleMapTest, AddEdgeSegments_ExcludesSpecifiedEdge) {
    // Create two edges at different Y positions
    EdgeLayout edge0, edge1;
    edge0.id = 0;
    edge0.from = 1;
    edge0.to = 2;
    edge0.sourcePoint = {150.0f, 170.0f};  // y=170 (below nodes)
    edge0.targetPoint = {250.0f, 170.0f};

    edge1.id = 1;
    edge1.from = 1;
    edge1.to = 2;
    edge1.sourcePoint = {150.0f, 190.0f};  // y=190 (below nodes)
    edge1.targetPoint = {250.0f, 190.0f};

    std::unordered_map<EdgeId, EdgeLayout> edgeLayouts;
    edgeLayouts[0] = edge0;
    edgeLayouts[1] = edge1;

    // Build and add segments, excluding edge 0
    obstacles_.buildFromNodes(nodeLayouts_, gridSize_, 0, &edgeLayouts);
    obstacles_.addEdgeSegments(edgeLayouts, 0);  // Exclude edge 0

    std::unordered_set<NodeId> noExclude;

    // Edge 0's segment (y=170) should NOT be registered - horizontal movement NOT blocked
    GridPoint edge0Mid = obstacles_.pixelToGrid({200.0f, 170.0f});
    EXPECT_FALSE(obstacles_.isBlockedForDirection(edge0Mid.x, edge0Mid.y,
                                                   MoveDirection::Left, noExclude))
        << "Excluded edge 0 should NOT block movement";

    // Edge 1's segment (y=190) SHOULD be registered - horizontal movement IS blocked
    GridPoint edge1Mid = obstacles_.pixelToGrid({200.0f, 190.0f});
    EXPECT_TRUE(obstacles_.isBlockedForDirection(edge1Mid.x, edge1Mid.y,
                                                  MoveDirection::Left, noExclude))
        << "Non-excluded edge 1 should block horizontal movement";
}

// =============================================================================
// isBlockedForDirection Tests
// =============================================================================

TEST_F(ObstacleMapTest, IsBlockedForDirection_NodeBlocking_BlocksAllDirections) {
    obstacles_.buildFromNodes(nodeLayouts_, gridSize_, 0);

    // Center of node1
    GridPoint center = obstacles_.pixelToGrid({
        node1_.position.x + node1_.size.width / 2,
        node1_.position.y + node1_.size.height / 2
    });

    std::unordered_set<NodeId> noExclude;

    // Node blocking should block all directions
    EXPECT_TRUE(obstacles_.isBlockedForDirection(center.x, center.y, MoveDirection::Up, noExclude));
    EXPECT_TRUE(obstacles_.isBlockedForDirection(center.x, center.y, MoveDirection::Down, noExclude));
    EXPECT_TRUE(obstacles_.isBlockedForDirection(center.x, center.y, MoveDirection::Left, noExclude));
    EXPECT_TRUE(obstacles_.isBlockedForDirection(center.x, center.y, MoveDirection::Right, noExclude));
}

TEST_F(ObstacleMapTest, IsBlockedForDirection_WithExclude_DoesNotBlockExcludedNode) {
    obstacles_.buildFromNodes(nodeLayouts_, gridSize_, 0);

    // Center of node1
    GridPoint center = obstacles_.pixelToGrid({
        node1_.position.x + node1_.size.width / 2,
        node1_.position.y + node1_.size.height / 2
    });

    std::unordered_set<NodeId> excludeNode1 = {1};

    // With node1 excluded, should not be blocked
    EXPECT_FALSE(obstacles_.isBlockedForDirection(center.x, center.y, MoveDirection::Up, excludeNode1));
    EXPECT_FALSE(obstacles_.isBlockedForDirection(center.x, center.y, MoveDirection::Down, excludeNode1));
    EXPECT_FALSE(obstacles_.isBlockedForDirection(center.x, center.y, MoveDirection::Left, excludeNode1));
    EXPECT_FALSE(obstacles_.isBlockedForDirection(center.x, center.y, MoveDirection::Right, excludeNode1));
}

TEST_F(ObstacleMapTest, IsBlockedForDirection_OutOfBounds_ReturnsFalse) {
    obstacles_.buildFromNodes(nodeLayouts_, gridSize_, 0);

    std::unordered_set<NodeId> noExclude;

    // Out of bounds should return false (infinite free space)
    EXPECT_FALSE(obstacles_.isBlockedForDirection(-1000, -1000, MoveDirection::Up, noExclude));
    EXPECT_FALSE(obstacles_.isBlockedForDirection(-1000, -1000, MoveDirection::Down, noExclude));
    EXPECT_FALSE(obstacles_.isBlockedForDirection(-1000, -1000, MoveDirection::Left, noExclude));
    EXPECT_FALSE(obstacles_.isBlockedForDirection(-1000, -1000, MoveDirection::Right, noExclude));
}

// =============================================================================
// segmentBlocked Tests
// =============================================================================

TEST_F(ObstacleMapTest, SegmentBlocked_ThroughNode_ReturnsTrue) {
    obstacles_.buildFromNodes(nodeLayouts_, gridSize_, 0);

    // Segment that passes through node1
    GridPoint start = obstacles_.pixelToGrid({40.0f, 90.0f});
    GridPoint end = obstacles_.pixelToGrid({160.0f, 90.0f});

    std::unordered_set<NodeId> noExclude;

    EXPECT_TRUE(obstacles_.segmentBlocked(start.x, start.y, end.x, end.y, noExclude));
}

TEST_F(ObstacleMapTest, SegmentBlocked_AroundNode_ReturnsFalse) {
    obstacles_.buildFromNodes(nodeLayouts_, gridSize_, 0);

    // Segment that goes between nodes (above both)
    GridPoint start = obstacles_.pixelToGrid({50.0f, 20.0f});
    GridPoint end = obstacles_.pixelToGrid({300.0f, 20.0f});

    std::unordered_set<NodeId> noExclude;

    EXPECT_FALSE(obstacles_.segmentBlocked(start.x, start.y, end.x, end.y, noExclude));
}

TEST_F(ObstacleMapTest, SegmentBlocked_WithExclude_IgnoresExcludedNodes) {
    obstacles_.buildFromNodes(nodeLayouts_, gridSize_, 0);

    // Segment through node1
    GridPoint start = obstacles_.pixelToGrid({40.0f, 90.0f});
    GridPoint end = obstacles_.pixelToGrid({160.0f, 90.0f});

    std::unordered_set<NodeId> excludeNode1 = {1};

    // With node1 excluded, segment through node1 should not be blocked
    EXPECT_FALSE(obstacles_.segmentBlocked(start.x, start.y, end.x, end.y, excludeNode1));
}

// =============================================================================
// getBlockingNode Tests
// =============================================================================

TEST_F(ObstacleMapTest, GetBlockingNode_InsideNode_ReturnsNodeId) {
    obstacles_.buildFromNodes(nodeLayouts_, gridSize_, 0);

    GridPoint center = obstacles_.pixelToGrid({
        node1_.position.x + node1_.size.width / 2,
        node1_.position.y + node1_.size.height / 2
    });

    NodeId blocking = obstacles_.getBlockingNode(center.x, center.y);
    EXPECT_EQ(blocking, node1_.id);
}

TEST_F(ObstacleMapTest, GetBlockingNode_OutsideNodes_ReturnsInvalidNode) {
    obstacles_.buildFromNodes(nodeLayouts_, gridSize_, 0);

    // Point between nodes
    GridPoint between = obstacles_.pixelToGrid({200.0f, 80.0f});

    NodeId blocking = obstacles_.getBlockingNode(between.x, between.y);
    EXPECT_EQ(blocking, INVALID_NODE);
}

TEST_F(ObstacleMapTest, GetBlockingNode_OutOfBounds_ReturnsInvalidNode) {
    obstacles_.buildFromNodes(nodeLayouts_, gridSize_, 0);

    NodeId blocking = obstacles_.getBlockingNode(-1000, -1000);
    EXPECT_EQ(blocking, INVALID_NODE);
}

// =============================================================================
// Safe Zones Tests
// =============================================================================

TEST_F(ObstacleMapTest, SafeZones_AreOutsideAllNodes) {
    obstacles_.buildFromNodes(nodeLayouts_, gridSize_, 0);

    auto safeZones = obstacles_.safeZones();

    // Safe zone Y values should be outside all node Y ranges
    for (const auto& [nodeId, node] : nodeLayouts_) {
        float nodeTop = node.position.y;
        float nodeBottom = node.position.y + node.size.height;
        int nodeTopGrid = static_cast<int>(nodeTop / gridSize_);
        int nodeBottomGrid = static_cast<int>(nodeBottom / gridSize_);

        EXPECT_LT(safeZones.yAbove, nodeTopGrid)
            << "yAbove should be above all nodes";
        EXPECT_GT(safeZones.yBelow, nodeBottomGrid)
            << "yBelow should be below all nodes";
    }

    // Safe zone X values should be outside all node X ranges
    for (const auto& [nodeId, node] : nodeLayouts_) {
        float nodeLeft = node.position.x;
        float nodeRight = node.position.x + node.size.width;
        int nodeLeftGrid = static_cast<int>(nodeLeft / gridSize_);
        int nodeRightGrid = static_cast<int>(nodeRight / gridSize_);

        EXPECT_LT(safeZones.xLeft, nodeLeftGrid)
            << "xLeft should be left of all nodes";
        EXPECT_GT(safeZones.xRight, nodeRightGrid)
            << "xRight should be right of all nodes";
    }
}

// =============================================================================
// Edge Path Management Tests
// =============================================================================

TEST_F(ObstacleMapTest, MarkEdgePath_RecordsPath) {
    obstacles_.buildFromNodes(nodeLayouts_, gridSize_, 0);

    std::vector<GridPoint> path = {{5, 5}, {6, 5}, {7, 5}};
    obstacles_.markEdgePath(0, path);

    EXPECT_TRUE(obstacles_.hasEdgePath(0));
}

TEST_F(ObstacleMapTest, ClearEdgePath_RemovesPath) {
    obstacles_.buildFromNodes(nodeLayouts_, gridSize_, 0);

    std::vector<GridPoint> path = {{5, 5}, {6, 5}, {7, 5}};
    obstacles_.markEdgePath(0, path);
    EXPECT_TRUE(obstacles_.hasEdgePath(0));

    obstacles_.clearEdgePath(0);
    EXPECT_FALSE(obstacles_.hasEdgePath(0));
}

TEST_F(ObstacleMapTest, ClearAllEdgePaths_RemovesAllPaths) {
    obstacles_.buildFromNodes(nodeLayouts_, gridSize_, 0);

    obstacles_.markEdgePath(0, {{5, 5}, {6, 5}});
    obstacles_.markEdgePath(1, {{7, 7}, {8, 7}});

    EXPECT_TRUE(obstacles_.hasEdgePath(0));
    EXPECT_TRUE(obstacles_.hasEdgePath(1));

    obstacles_.clearAllEdgePaths();

    EXPECT_FALSE(obstacles_.hasEdgePath(0));
    EXPECT_FALSE(obstacles_.hasEdgePath(1));
}

TEST_F(ObstacleMapTest, GetEdgePaths_ReturnsRecordedPaths) {
    obstacles_.buildFromNodes(nodeLayouts_, gridSize_, 0);

    std::vector<GridPoint> path0 = {{5, 5}, {6, 5}};
    std::vector<GridPoint> path1 = {{7, 7}, {8, 7}};

    obstacles_.markEdgePath(0, path0);
    obstacles_.markEdgePath(1, path1);

    const auto& paths = obstacles_.getEdgePaths();

    EXPECT_EQ(paths.size(), 2u);
    EXPECT_EQ(paths.at(0).size(), 2u);
    EXPECT_EQ(paths.at(1).size(), 2u);
}

// =============================================================================
// Clear Edge Segments Tests
// =============================================================================

TEST_F(ObstacleMapTest, ClearEdgeSegments_RemovesAllSegmentBlocking) {
    // Create edge with horizontal segment below nodes
    EdgeLayout edge;
    edge.id = 0;
    edge.from = 1;
    edge.to = 2;
    edge.sourcePoint = {150.0f, 170.0f};  // Below nodes
    edge.targetPoint = {250.0f, 170.0f};

    std::unordered_map<EdgeId, EdgeLayout> edgeLayouts;
    edgeLayouts[0] = edge;

    obstacles_.buildFromNodes(nodeLayouts_, gridSize_, 0, &edgeLayouts);
    obstacles_.addEdgeSegments(edgeLayouts, INVALID_EDGE);

    std::unordered_set<NodeId> noExclude;
    GridPoint midPoint = obstacles_.pixelToGrid({200.0f, 170.0f});

    // Before clear: horizontal movement should be blocked
    EXPECT_TRUE(obstacles_.isBlockedForDirection(midPoint.x, midPoint.y,
                                                  MoveDirection::Left, noExclude))
        << "Before clear: segment should block movement";

    // Clear all edge segments
    obstacles_.clearEdgeSegments();

    // After clear: horizontal movement should NOT be blocked
    EXPECT_FALSE(obstacles_.isBlockedForDirection(midPoint.x, midPoint.y,
                                                   MoveDirection::Left, noExclude))
        << "After clear: segment should no longer block movement";
}

// =============================================================================
// Cost Calculation Tests
// =============================================================================

TEST_F(ObstacleMapTest, GetCost_BlockedCell_ReturnsCostBlocked) {
    obstacles_.buildFromNodes(nodeLayouts_, gridSize_, 0);

    GridPoint center = obstacles_.pixelToGrid({
        node1_.position.x + node1_.size.width / 2,
        node1_.position.y + node1_.size.height / 2
    });

    int cost = obstacles_.getCost(center.x, center.y);
    EXPECT_EQ(cost, IObstacleProvider::COST_BLOCKED);
}

TEST_F(ObstacleMapTest, GetCost_FreeCell_ReturnsBaseCost) {
    obstacles_.buildFromNodes(nodeLayouts_, gridSize_, 0);

    // Point between nodes
    GridPoint between = obstacles_.pixelToGrid({200.0f, 0.0f});  // Far above

    int cost = obstacles_.getCost(between.x, between.y);
    EXPECT_EQ(cost, IObstacleProvider::COST_FREE);
}

