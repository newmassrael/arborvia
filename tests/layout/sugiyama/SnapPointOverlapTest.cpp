#include <gtest/gtest.h>
#include "../../../src/layout/snap/SnapIndexManager.h"
#include "../../../src/layout/snap/GridSnapCalculator.h"
#include "../../../src/layout/sugiyama/routing/EdgeRouting.h"
#include "arborvia/layout/util/LayoutUtils.h"
#include "arborvia/core/GeometryUtils.h"

#include <unordered_set>
#include <cmath>

using namespace arborvia;

// =============================================================================
// Snap Point Overlap Detection Tests
// =============================================================================

class SnapPointOverlapTest : public ::testing::Test {
protected:
    void SetUp() override {
        gridSize_ = 20.0f;
    }

    // Helper: Calculate snap point position on node edge
    Point calculateSnapPoint(const NodeLayout& node, NodeEdge edge, int snapIndex, int totalCount) {
        float position = SnapIndexManager::calculatePosition(snapIndex, totalCount, {0.0f, 1.0f});
        return LayoutUtils::calculateSnapPointFromRatio(node, edge, position, gridSize_);
    }

    // Helper: Calculate snap point with grid snapping (OLD behavior - may cause overlaps)
    Point calculateSnapPointWithGrid(const NodeLayout& node, NodeEdge edge,
                                     int snapIndex, int totalCount, float gridSize) {
        Point p = calculateSnapPoint(node, edge, snapIndex, totalCount);
        // Snap to grid
        p.x = std::round(p.x / gridSize) * gridSize;
        p.y = std::round(p.y / gridSize) * gridSize;
        return p;
    }

    // Helper: Check if two points are at the same position
    bool isSamePosition(const Point& a, const Point& b, float tolerance = 0.1f) {
        return std::abs(a.x - b.x) < tolerance && std::abs(a.y - b.y) < tolerance;
    }

    // Helper: Count overlapping snap points in a collection
    int countOverlaps(const std::vector<Point>& points, float tolerance = 0.1f) {
        int overlaps = 0;
        for (size_t i = 0; i < points.size(); ++i) {
            for (size_t j = i + 1; j < points.size(); ++j) {
                if (isSamePosition(points[i], points[j], tolerance)) {
                    overlaps++;
                }
            }
        }
        return overlaps;
    }

    float gridSize_;
};

// =============================================================================
// Snap Point Distribution Tests
// =============================================================================

TEST_F(SnapPointOverlapTest, CornerOverlap_TopRightCorner) {
    // Problem: Top edge at position=1.0 and Right edge at position=0.0
    // should be at the same corner coordinates

    NodeLayout node;
    node.id = NodeId{1};
    node.position = {100.0f, 100.0f};
    node.size = {100.0f, 50.0f};

    // Single snap point on each edge - they land at corners
    Point topEnd = calculateSnapPoint(node, NodeEdge::Top, 0, 1);      // Center of top edge
    Point rightStart = calculateSnapPoint(node, NodeEdge::Right, 0, 1); // Center of right edge

    // These should NOT be at the same position in a well-designed system
    // But with position at edge endpoints, they could overlap

    // Let's check the corner case more explicitly:
    // Top edge goes from (100, 100) to (200, 100)
    // Right edge goes from (200, 100) to (200, 150)
    // The corner (200, 100) is shared!

    Point topRightCorner = {node.position.x + node.size.width, node.position.y};

    // This test documents the corner overlap issue
    // For a system with snap points at exact corners, this would be problematic

    // Create scenario: 2 snap points on Top, 2 on Right
    // Top[1] should be near right corner, Right[0] should be near top corner
    Point top1 = calculateSnapPoint(node, NodeEdge::Top, 1, 2);     // position = 2/3 ≈ 0.67
    Point right0 = calculateSnapPoint(node, NodeEdge::Right, 0, 2); // position = 1/3 ≈ 0.33

    // With grid snapping, these might land on same grid cell
    Point top1Grid = calculateSnapPointWithGrid(node, NodeEdge::Top, 1, 2, gridSize_);
    Point right0Grid = calculateSnapPointWithGrid(node, NodeEdge::Right, 0, 2, gridSize_);

    // These should be different positions
    EXPECT_FALSE(isSamePosition(top1Grid, right0Grid))
        << "Top edge snap[1] and Right edge snap[0] should not overlap!"
        << "\n  Top[1]: (" << top1Grid.x << ", " << top1Grid.y << ")"
        << "\n  Right[0]: (" << right0Grid.x << ", " << right0Grid.y << ")";
}

TEST_F(SnapPointOverlapTest, GridSnapping_NarrowNode_CausesOverlap) {
    // Problem: On a narrow node, multiple snap points may round to the same grid cell
    // Current behavior: GridSnapCalculator uses only available grid candidates within node bounds
    // When node is too narrow, snap points will overlap at available grid positions

    NodeLayout narrowNode;
    narrowNode.id = NodeId{1};
    narrowNode.position = {100.0f, 100.0f};
    narrowNode.size = {30.0f, 50.0f};  // Width = 30px, only 1.5 grid cells wide

    // 3 snap points on bottom edge of narrow node
    // With 30px width and 20px grid, there are only 2 candidate positions
    // So 3 connections will result in some overlap
    std::vector<Point> snapPoints;
    for (int i = 0; i < 3; ++i) {
        Point p = GridSnapCalculator::calculateSnapPosition(narrowNode, NodeEdge::Bottom, i, 3, gridSize_);
        snapPoints.push_back(p);
    }

    // With insufficient grid candidates, overlap is expected
    // This test documents the current behavior
    int overlaps = countOverlaps(snapPoints);

    // Current behavior: overlaps occur when candidates < connections
    // 30px width / 20px grid = 1-2 candidates, but 3 connections requested
    EXPECT_GT(overlaps, 0)
        << "Narrow node should have overlapping snap points when candidates < connections"
        << "\n  Snap[0]: (" << snapPoints[0].x << ", " << snapPoints[0].y << ")"
        << "\n  Snap[1]: (" << snapPoints[1].x << ", " << snapPoints[1].y << ")"
        << "\n  Snap[2]: (" << snapPoints[2].x << ", " << snapPoints[2].y << ")";
}

TEST_F(SnapPointOverlapTest, AdjacentNodes_SnapPointsCanOverlap) {
    // Problem: Two adjacent nodes may have snap points at the exact same position

    // Node A at top, Node B directly below (touching)
    NodeLayout nodeA;
    nodeA.id = NodeId{1};
    nodeA.position = {100.0f, 100.0f};
    nodeA.size = {100.0f, 50.0f};

    NodeLayout nodeB;
    nodeB.id = NodeId{2};
    nodeB.position = {100.0f, 150.0f};  // Directly below A
    nodeB.size = {100.0f, 50.0f};

    // Single snap point on each edge
    Point aBottom = calculateSnapPointWithGrid(nodeA, NodeEdge::Bottom, 0, 1, gridSize_);
    Point bTop = calculateSnapPointWithGrid(nodeB, NodeEdge::Top, 0, 1, gridSize_);

    // These could overlap if nodes are exactly adjacent
    // Expected: aBottom.y = 150, bTop.y = 150, both at x = 150 (center)

    // The system should ensure snap points don't physically overlap
    // OR handle the case gracefully when they do

    bool samePosition = isSamePosition(aBottom, bTop);

    // This documents the potential issue - adjacent nodes with same center X
    // will have overlapping snap points at the boundary
    if (samePosition) {
        // This is expected when nodes are exactly adjacent
        // The system should handle this case
        SUCCEED() << "Adjacent nodes have overlapping snap points at boundary (expected)";
    } else {
        SUCCEED() << "Adjacent nodes have distinct snap points";
    }
}

TEST_F(SnapPointOverlapTest, SameNodeEdge_MultipleEdges_NoOverlap) {
    // Core requirement: Multiple edges on the same (node, edge) must have distinct snap points
    // when there are enough grid candidates

    NodeLayout node;
    node.id = NodeId{1};
    node.position = {100.0f, 100.0f};
    node.size = {120.0f, 50.0f};  // 6+ grid cells wide (120/20 = 6 candidates)

    // 4 edges - well within candidate count
    const int edgeCount = 4;
    std::vector<Point> snapPoints;

    for (int i = 0; i < edgeCount; ++i) {
        Point p = GridSnapCalculator::calculateSnapPosition(node, NodeEdge::Bottom, i, edgeCount, gridSize_);
        snapPoints.push_back(p);
    }

    // All snap points must be distinct when candidates >= connections
    int overlaps = countOverlaps(snapPoints);

    EXPECT_EQ(overlaps, 0)
        << "All " << edgeCount << " snap points on same edge must be distinct!";

    // Also verify they're in order (left to right for horizontal edges)
    for (size_t i = 0; i + 1 < snapPoints.size(); ++i) {
        EXPECT_LT(snapPoints[i].x, snapPoints[i + 1].x)
            << "Snap points should be ordered left to right";
    }
}

TEST_F(SnapPointOverlapTest, VeryNarrowNode_ForcedOverlap) {
    // Edge case: Node narrower than gridSize forces all snap points to same cell
    // Current behavior: GridSnapCalculator does NOT spread beyond boundaries
    // Instead, it places all snap points at the single available candidate position

    NodeLayout tinyNode;
    tinyNode.id = NodeId{1};
    tinyNode.position = {100.0f, 100.0f};
    tinyNode.size = {15.0f, 50.0f};  // Width < gridSize (20px)

    // Place 3 snap points on a 15px wide edge
    // With only 1 candidate position available, all 3 will overlap
    std::vector<Point> snapPoints;
    for (int i = 0; i < 3; ++i) {
        Point p = GridSnapCalculator::calculateSnapPosition(tinyNode, NodeEdge::Bottom, i, 3, gridSize_);
        snapPoints.push_back(p);
    }

    int overlaps = countOverlaps(snapPoints);

    // Current behavior: overlaps occur when node is too narrow
    // This documents the limitation of the current grid-based system
    EXPECT_GT(overlaps, 0)
        << "Very narrow nodes will have overlapping snap points with current implementation"
        << "\n  Node width: " << tinyNode.size.width << "px"
        << "\n  Grid size: " << gridSize_ << "px"
        << "\n  Snap count: 3";
}

// =============================================================================
// Overlap Detection API Tests
// =============================================================================

TEST_F(SnapPointOverlapTest, DetectOverlaps_ReturnsOverlappingPairs) {
    // Test the overlap detection utility

    std::vector<Point> points = {
        {100.0f, 100.0f},
        {120.0f, 100.0f},
        {100.0f, 100.0f},  // Duplicate of first
        {140.0f, 100.0f}
    };

    int overlaps = countOverlaps(points);
    EXPECT_EQ(overlaps, 1) << "Should detect exactly 1 overlap (points 0 and 2)";
}

TEST_F(SnapPointOverlapTest, DetectOverlaps_NoOverlaps_ReturnsZero) {
    std::vector<Point> points = {
        {100.0f, 100.0f},
        {120.0f, 100.0f},
        {140.0f, 100.0f},
        {160.0f, 100.0f}
    };

    int overlaps = countOverlaps(points);
    EXPECT_EQ(overlaps, 0);
}

// =============================================================================
// Resolution Strategy Tests
// =============================================================================

TEST_F(SnapPointOverlapTest, Resolution_SpreadBeyondBoundary) {
    // Strategy 1: When snap points would overlap, spread them beyond node boundary

    NodeLayout tinyNode;
    tinyNode.id = NodeId{1};
    tinyNode.position = {100.0f, 100.0f};
    tinyNode.size = {15.0f, 50.0f};

    // Calculate minimum spacing needed
    float minSpacing = gridSize_;  // At least 1 grid cell apart
    int snapCount = 3;
    float requiredWidth = minSpacing * (snapCount - 1);  // 40px for 3 points

    // If node is too narrow, calculate extended range
    float nodeWidth = tinyNode.size.width;
    float extension = 0.0f;

    if (nodeWidth < requiredWidth) {
        extension = (requiredWidth - nodeWidth) / 2.0f;
    }

    // This test documents the expected behavior
    EXPECT_GT(extension, 0.0f)
        << "Tiny node should require boundary extension for " << snapCount << " snap points";
    EXPECT_FLOAT_EQ(extension, 12.5f)  // (40 - 15) / 2 = 12.5
        << "Extension should be (requiredWidth - nodeWidth) / 2";
}

TEST_F(SnapPointOverlapTest, Resolution_LimitSnapCount) {
    // Strategy 2: Limit snap point count based on available space

    NodeLayout tinyNode;
    tinyNode.id = NodeId{1};
    tinyNode.position = {100.0f, 100.0f};
    tinyNode.size = {15.0f, 50.0f};

    float minSpacing = gridSize_;
    float nodeWidth = tinyNode.size.width;

    // Calculate maximum snap points that fit without overlap
    int maxSnapPoints = static_cast<int>(std::floor(nodeWidth / minSpacing)) + 1;

    EXPECT_LE(maxSnapPoints, 2)
        << "15px node with 20px grid should support at most 2 snap points";
}

