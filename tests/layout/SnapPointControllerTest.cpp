#include <gtest/gtest.h>
#include <arborvia/arborvia.h>
#include <arborvia/layout/interactive/SnapPointController.h>
#include "../../src/layout/snap/GridSnapCalculator.h"

using namespace arborvia;

class SnapPointControllerTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Create test graph with 3 nodes in vertical arrangement
        nodeA_ = graph_.addNode(Size{100, 50}, "NodeA");
        nodeB_ = graph_.addNode(Size{100, 50}, "NodeB");
        nodeC_ = graph_.addNode(Size{100, 50}, "NodeC");

        edgeAB_ = graph_.addEdge(nodeA_, nodeB_, "A->B");
        edgeAC_ = graph_.addEdge(nodeA_, nodeC_, "A->C");

        // Set up node layouts
        nodeLayouts_[nodeA_] = NodeLayout{nodeA_, {0, 0}, {100, 50}};
        nodeLayouts_[nodeB_] = NodeLayout{nodeB_, {0, 100}, {100, 50}};
        nodeLayouts_[nodeC_] = NodeLayout{nodeC_, {150, 100}, {100, 50}};

        // Set up edge layouts
        EdgeLayout layoutAB;
        layoutAB.id = edgeAB_;
        layoutAB.from = nodeA_;
        layoutAB.to = nodeB_;
        layoutAB.sourcePoint = {50, 50};
        layoutAB.targetPoint = {50, 100};
        layoutAB.sourceEdge = NodeEdge::Bottom;
        layoutAB.targetEdge = NodeEdge::Top;
        layoutAB.sourceSnapIndex = 4;
        layoutAB.targetSnapIndex = 4;
        edgeLayouts_[edgeAB_] = layoutAB;

        EdgeLayout layoutAC;
        layoutAC.id = edgeAC_;
        layoutAC.from = nodeA_;
        layoutAC.to = nodeC_;
        layoutAC.sourcePoint = {80, 50};
        layoutAC.targetPoint = {180, 100};
        layoutAC.sourceEdge = NodeEdge::Bottom;
        layoutAC.targetEdge = NodeEdge::Top;
        layoutAC.sourceSnapIndex = 7;
        layoutAC.targetSnapIndex = 2;
        edgeLayouts_[edgeAC_] = layoutAC;

        gridSize_ = 10.0f;
    }

    Graph graph_;
    NodeId nodeA_, nodeB_, nodeC_;
    EdgeId edgeAB_, edgeAC_;
    std::unordered_map<NodeId, NodeLayout> nodeLayouts_;
    std::unordered_map<EdgeId, EdgeLayout> edgeLayouts_;
    float gridSize_;
    SnapPointController controller_;
};

// ============== startDrag Tests ==============

TEST_F(SnapPointControllerTest, StartDrag_Success) {
    auto result = controller_.startDrag(
        edgeAB_, true, nodeLayouts_, edgeLayouts_, gridSize_);

    EXPECT_TRUE(result.success);
    EXPECT_TRUE(result.reason.empty());
    EXPECT_FALSE(result.candidates.empty());
    EXPECT_EQ(result.originalEdge, NodeEdge::Bottom);
    EXPECT_EQ(result.originalSnapIndex, 4);
    EXPECT_TRUE(controller_.isDragging());
    EXPECT_EQ(controller_.getDraggedEdgeId(), edgeAB_);
    EXPECT_TRUE(controller_.isDraggingSource());
}

TEST_F(SnapPointControllerTest, StartDrag_TargetSnapPoint) {
    auto result = controller_.startDrag(
        edgeAB_, false, nodeLayouts_, edgeLayouts_, gridSize_);

    EXPECT_TRUE(result.success);
    EXPECT_EQ(result.originalEdge, NodeEdge::Top);
    EXPECT_EQ(result.originalSnapIndex, 4);
    EXPECT_FALSE(controller_.isDraggingSource());
}

TEST_F(SnapPointControllerTest, StartDrag_InvalidEdge) {
    auto result = controller_.startDrag(
        999, true, nodeLayouts_, edgeLayouts_, gridSize_);

    EXPECT_FALSE(result.success);
    EXPECT_EQ(result.reason, "Edge not found");
    EXPECT_FALSE(controller_.isDragging());
}

TEST_F(SnapPointControllerTest, StartDrag_CandidatesIncludeAllEdges) {
    auto result = controller_.startDrag(
        edgeAB_, true, nodeLayouts_, edgeLayouts_, gridSize_);

    // Should have candidates for all 4 edges of nodeA
    bool hasTop = false, hasBottom = false, hasLeft = false, hasRight = false;
    for (const auto& candidate : result.candidates) {
        if (candidate.edge == NodeEdge::Top) hasTop = true;
        if (candidate.edge == NodeEdge::Bottom) hasBottom = true;
        if (candidate.edge == NodeEdge::Left) hasLeft = true;
        if (candidate.edge == NodeEdge::Right) hasRight = true;
    }

    EXPECT_TRUE(hasTop);
    EXPECT_TRUE(hasBottom);
    EXPECT_TRUE(hasLeft);
    EXPECT_TRUE(hasRight);
}

// ============== updateDrag Tests ==============

TEST_F(SnapPointControllerTest, UpdateDrag_FindsNearestCandidate) {
    controller_.startDrag(edgeAB_, true, nodeLayouts_, edgeLayouts_, gridSize_);

    // Position near bottom edge center of nodeA
    Point mousePos = {50, 50};
    auto result = controller_.updateDrag(mousePos, nodeLayouts_, gridSize_);

    EXPECT_GE(result.snappedCandidateIndex, 0);
    // Should snap to a candidate on Bottom edge
    const auto& candidates = controller_.getCandidates();
    EXPECT_EQ(candidates[result.snappedCandidateIndex].edge, NodeEdge::Bottom);
}

TEST_F(SnapPointControllerTest, UpdateDrag_CachesPreview) {
    controller_.startDrag(edgeAB_, true, nodeLayouts_, edgeLayouts_, gridSize_);

    Point mousePos = {50, 50};

    // First update - calculates path
    auto result1 = controller_.updateDrag(mousePos, nodeLayouts_, gridSize_);

    // Second update at same position - uses cached
    auto result2 = controller_.updateDrag(mousePos, nodeLayouts_, gridSize_);

    EXPECT_EQ(result1.snappedCandidateIndex, result2.snappedCandidateIndex);
}

TEST_F(SnapPointControllerTest, UpdateDrag_NotDragging_ReturnsInvalid) {
    // Don't call startDrag
    auto result = controller_.updateDrag({50, 50}, nodeLayouts_, gridSize_);

    EXPECT_FALSE(result.hasValidPreview);
    EXPECT_EQ(result.snappedCandidateIndex, -1);
}

// ============== completeDrag Tests ==============

TEST_F(SnapPointControllerTest, CompleteDrag_Success) {
    controller_.startDrag(edgeAB_, true, nodeLayouts_, edgeLayouts_, gridSize_);

    // Update to find a candidate
    auto updateResult = controller_.updateDrag({60, 50}, nodeLayouts_, gridSize_);

    LayoutOptions options;
    options.gridConfig.cellSize = gridSize_;

    auto dropResult = controller_.completeDrag(
        updateResult.snappedCandidateIndex,
        {60, 50},
        nodeLayouts_,
        edgeLayouts_,
        options);

    EXPECT_TRUE(dropResult.success);
    EXPECT_FALSE(controller_.isDragging());  // Should be cleared
    EXPECT_FALSE(dropResult.affectedEdges.empty());
}

TEST_F(SnapPointControllerTest, CompleteDrag_NotDragging_Fails) {
    LayoutOptions options;
    auto result = controller_.completeDrag(
        0, {50, 50}, nodeLayouts_, edgeLayouts_, options);

    EXPECT_FALSE(result.success);
    EXPECT_EQ(result.reason, "No drag in progress");
}

TEST_F(SnapPointControllerTest, CompleteDrag_SwapDetection) {
    // Set up two edges with snap points that can be swapped
    // edgeAB source at snapIndex=4 on Bottom
    // edgeAC source at snapIndex=7 on Bottom

    controller_.startDrag(edgeAB_, true, nodeLayouts_, edgeLayouts_, gridSize_);

    // Find the candidate that matches edgeAC's position (snapIndex=7)
    const auto& candidates = controller_.getCandidates();
    int targetCandidateIdx = -1;
    for (size_t i = 0; i < candidates.size(); ++i) {
        if (candidates[i].edge == NodeEdge::Bottom && candidates[i].candidateIndex == 7) {
            targetCandidateIdx = static_cast<int>(i);
            break;
        }
    }

    ASSERT_GE(targetCandidateIdx, 0) << "Should find candidate at snapIndex=7";

    LayoutOptions options;
    options.gridConfig.cellSize = gridSize_;

    auto dropResult = controller_.completeDrag(
        targetCandidateIdx,
        candidates[targetCandidateIdx].position,
        nodeLayouts_,
        edgeLayouts_,
        options);

    EXPECT_TRUE(dropResult.success);
    EXPECT_EQ(dropResult.swapEdgeId, edgeAC_);  // Should detect swap with edgeAC
    EXPECT_TRUE(dropResult.swapIsSource);  // edgeAC's source was swapped

    // Verify the swap happened
    EXPECT_EQ(edgeLayouts_[edgeAB_].sourceSnapIndex, 7);  // edgeAB moved to 7
    EXPECT_EQ(edgeLayouts_[edgeAC_].sourceSnapIndex, 4);  // edgeAC moved to 4
}

// ============== cancelDrag Tests ==============

TEST_F(SnapPointControllerTest, CancelDrag_ClearsState) {
    controller_.startDrag(edgeAB_, true, nodeLayouts_, edgeLayouts_, gridSize_);
    EXPECT_TRUE(controller_.isDragging());

    controller_.cancelDrag();

    EXPECT_FALSE(controller_.isDragging());
    EXPECT_TRUE(controller_.getCandidates().empty());
}

// ============== State Query Tests ==============

TEST_F(SnapPointControllerTest, GetCandidates_EmptyBeforeDrag) {
    EXPECT_TRUE(controller_.getCandidates().empty());
}

TEST_F(SnapPointControllerTest, GetPreviewLayout_ValidAfterUpdate) {
    controller_.startDrag(edgeAB_, true, nodeLayouts_, edgeLayouts_, gridSize_);
    controller_.updateDrag({60, 50}, nodeLayouts_, gridSize_);

    const auto& preview = controller_.getPreviewLayout();
    EXPECT_EQ(preview.from, nodeA_);
    EXPECT_EQ(preview.to, nodeB_);
}

// ============== Edge Case Tests ==============

TEST_F(SnapPointControllerTest, CompleteDrag_InvalidCandidateIndex_UsesFallback) {
    controller_.startDrag(edgeAB_, true, nodeLayouts_, edgeLayouts_, gridSize_);

    LayoutOptions options;
    options.gridConfig.cellSize = gridSize_;

    // Use invalid index but valid mouse position
    auto result = controller_.completeDrag(
        -1,  // Invalid index
        {50, 50},  // Valid position near Bottom edge
        nodeLayouts_,
        edgeLayouts_,
        options);

    EXPECT_TRUE(result.success);  // Should still succeed using fallback
}

TEST_F(SnapPointControllerTest, CompleteDrag_WithRegenerator) {
    controller_.startDrag(edgeAB_, true, nodeLayouts_, edgeLayouts_, gridSize_);
    controller_.updateDrag({60, 50}, nodeLayouts_, gridSize_);

    LayoutOptions options;
    options.gridConfig.cellSize = gridSize_;

    bool regeneratorCalled = false;
    auto regenerator = [&regeneratorCalled](
        std::unordered_map<EdgeId, EdgeLayout>&,
        const std::unordered_map<NodeId, NodeLayout>&,
        const std::vector<EdgeId>&,
        const LayoutOptions&) {
        regeneratorCalled = true;
    };

    controller_.completeDrag(0, {60, 50}, nodeLayouts_, edgeLayouts_, options, regenerator);

    EXPECT_TRUE(regeneratorCalled);
}

// =============================================================================
// TDD RED: Snap Point Change Must Trigger Full Pathfinding Flow
// =============================================================================
// When a snap point is moved, the ENTIRE pathfinding flow must execute:
// 1. Update snap point position/edge/index
// 2. Build ObstacleMap (nodes + other edge segments as obstacles)
// 3. Run A* pathfinding (avoiding obstacles)
// 4. Update bendPoints with orthogonal path
//
// Current bug: completeDrag() without regenerator skips steps 2-4
//
// To verify ObstacleMap is properly built with edge segments:
// - Create scenario where another edge blocks the direct path
// - If ObstacleMap includes edge segments, A* must route around it
// - If ObstacleMap is NOT built, path may cross the blocking edge

// Helper: Check if all segments in an edge path are orthogonal
bool isEdgeOrthogonal(const EdgeLayout& layout, float epsilon = 0.1f) {
    std::vector<Point> points;
    points.push_back(layout.sourcePoint);
    for (const auto& bp : layout.bendPoints) {
        points.push_back(bp.position);
    }
    points.push_back(layout.targetPoint);
    
    for (size_t i = 0; i + 1 < points.size(); ++i) {
        float dx = std::abs(points[i + 1].x - points[i].x);
        float dy = std::abs(points[i + 1].y - points[i].y);
        bool isHorizontal = dy < epsilon;
        bool isVertical = dx < epsilon;
        if (!isHorizontal && !isVertical) {
            return false;  // Diagonal segment found
        }
    }
    return true;
}

// Helper: Get diagonal segment info for debugging
std::string getDiagonalInfo(const EdgeLayout& layout) {
    std::ostringstream oss;
    std::vector<Point> points;
    points.push_back(layout.sourcePoint);
    for (const auto& bp : layout.bendPoints) {
        points.push_back(bp.position);
    }
    points.push_back(layout.targetPoint);
    
    oss << "Edge " << layout.id << " path:\n";
    for (size_t i = 0; i < points.size(); ++i) {
        oss << "  [" << i << "] (" << points[i].x << ", " << points[i].y << ")";
        if (i + 1 < points.size()) {
            float dx = std::abs(points[i + 1].x - points[i].x);
            float dy = std::abs(points[i + 1].y - points[i].y);
            if (dx > 0.1f && dy > 0.1f) {
                oss << " -> DIAGONAL (dx=" << dx << ", dy=" << dy << ")";
            }
        }
        oss << "\n";
    }
    return oss.str();
}

// TDD RED: completeDrag without regenerator must still produce orthogonal paths
// This test SHOULD FAIL with current implementation
TEST_F(SnapPointControllerTest, CompleteDrag_WithoutRegenerator_MustProduceOrthogonalPath) {
    // Setup: Create edge with initial orthogonal path
    EdgeLayout& edgeAB = edgeLayouts_[edgeAB_];
    edgeAB.sourcePoint = {50, 50};   // Bottom of NodeA
    edgeAB.targetPoint = {50, 100};  // Top of NodeB
    edgeAB.sourceEdge = NodeEdge::Bottom;
    edgeAB.targetEdge = NodeEdge::Top;
    edgeAB.bendPoints.clear();  // Direct vertical line - orthogonal
    
    // Verify initial state is orthogonal
    ASSERT_TRUE(isEdgeOrthogonal(edgeAB)) << "Initial edge must be orthogonal";
    
    // Start drag on source snap point
    auto startResult = controller_.startDrag(edgeAB_, true, nodeLayouts_, edgeLayouts_, gridSize_);
    ASSERT_TRUE(startResult.success);
    
    // Find candidate on RIGHT edge of NodeA (different from current Bottom edge)
    int rightEdgeCandidateIdx = -1;
    for (size_t i = 0; i < startResult.candidates.size(); ++i) {
        if (startResult.candidates[i].edge == NodeEdge::Right) {
            rightEdgeCandidateIdx = static_cast<int>(i);
            break;
        }
    }
    ASSERT_GE(rightEdgeCandidateIdx, 0) << "Should find candidate on Right edge";
    
    Point newPosition = startResult.candidates[rightEdgeCandidateIdx].position;
    
    // Complete drag WITHOUT regenerator (simulates current demo behavior)
    LayoutOptions options;
    options.gridConfig.cellSize = gridSize_;
    
    auto dropResult = controller_.completeDrag(
        rightEdgeCandidateIdx,
        newPosition,
        nodeLayouts_,
        edgeLayouts_,
        options);  // NO regenerator!
    
    ASSERT_TRUE(dropResult.success) << "Drop should succeed";
    
    // CRITICAL CHECK: After snap point change, edge must STILL be orthogonal
    // This requires:
    // 1. ObstacleMap built from nodes
    // 2. A* pathfinding executed
    // 3. bendPoints updated with orthogonal path
    
    EdgeLayout& afterEdge = edgeLayouts_[edgeAB_];
    
    // Verify snap point was moved
    EXPECT_EQ(afterEdge.sourceEdge, NodeEdge::Right) 
        << "Source edge should be Right after drag";
    EXPECT_FLOAT_EQ(afterEdge.sourcePoint.x, newPosition.x);
    EXPECT_FLOAT_EQ(afterEdge.sourcePoint.y, newPosition.y);
    
    // THE MAIN ASSERTION: Path must be orthogonal
    // Without proper flow, this will FAIL because:
    // - sourcePoint changed to Right edge
    // - targetPoint still on Top edge of NodeB
    // - bendPoints NOT updated -> diagonal segment!
    EXPECT_TRUE(isEdgeOrthogonal(afterEdge))
        << "Edge must be orthogonal after snap point change!\n"
        << "Without regenerator, bendPoints are not updated.\n"
        << getDiagonalInfo(afterEdge);
}

// TDD RED: Moving snap point to different edge requires bend points
TEST_F(SnapPointControllerTest, CompleteDrag_EdgeChange_MustHaveBendPoints) {
    // Setup: source on Bottom, target on Top (direct vertical line possible)
    EdgeLayout& edgeAB = edgeLayouts_[edgeAB_];
    edgeAB.sourcePoint = {50, 50};
    edgeAB.targetPoint = {50, 100};
    edgeAB.sourceEdge = NodeEdge::Bottom;
    edgeAB.targetEdge = NodeEdge::Top;
    edgeAB.bendPoints.clear();
    
    // Start drag
    auto startResult = controller_.startDrag(edgeAB_, true, nodeLayouts_, edgeLayouts_, gridSize_);
    ASSERT_TRUE(startResult.success);
    
    // Move source to LEFT edge of NodeA
    int leftEdgeCandidateIdx = -1;
    for (size_t i = 0; i < startResult.candidates.size(); ++i) {
        if (startResult.candidates[i].edge == NodeEdge::Left) {
            leftEdgeCandidateIdx = static_cast<int>(i);
            break;
        }
    }
    ASSERT_GE(leftEdgeCandidateIdx, 0);
    
    Point newPosition = startResult.candidates[leftEdgeCandidateIdx].position;
    
    LayoutOptions options;
    options.gridConfig.cellSize = gridSize_;
    
    auto dropResult = controller_.completeDrag(
        leftEdgeCandidateIdx, newPosition, nodeLayouts_, edgeLayouts_, options);
    
    ASSERT_TRUE(dropResult.success);
    
    EdgeLayout& afterEdge = edgeLayouts_[edgeAB_];
    
    // Source is now on LEFT edge, target on TOP edge
    // These are not aligned - MUST have bend points for orthogonal path
    // Left edge X = 0, Top edge Y = 100
    // Source: (0, ~25), Target: (50, 100)
    // Without bend points: diagonal line!
    
    // Check that source moved to Left edge
    EXPECT_EQ(afterEdge.sourceEdge, NodeEdge::Left);
    
    // CRITICAL: Must have bend points for non-aligned source/target
    // e.g., (0, 25) -> (0, 100) -> (50, 100) requires 1 bend
    // or (0, 25) -> (50, 25) -> (50, 100) requires 1 bend
    
    bool needsBends = true;  // Left->Top requires bends
    if (needsBends) {
        EXPECT_FALSE(afterEdge.bendPoints.empty())
            << "Moving source to Left edge while target is on Top edge "
            << "requires bend points for orthogonal path!\n"
            << getDiagonalInfo(afterEdge);
    }
    
    // Regardless, path MUST be orthogonal
    EXPECT_TRUE(isEdgeOrthogonal(afterEdge))
        << "Edge must be orthogonal!\n"
        << getDiagonalInfo(afterEdge);
}

// TDD RED: ObstacleMap must include other edge segments
// Verify that when moving a snap point, the pathfinder avoids other edges
TEST_F(SnapPointControllerTest, CompleteDrag_MustAvoidOtherEdgeSegments) {
    // Setup: Create a blocking scenario
    // NodeA at (0,0), NodeB at (0,100), NodeC at (150,100)
    //
    // Initial layout:
    //   NodeA ----edgeAC----> NodeC
    //     |
    //   edgeAB (blocking horizontal path)
    //     |
    //     v
    //   NodeB
    //
    // After moving edgeAB source to Right edge of NodeA:
    //   If ObstacleMap includes edgeAC, path must go AROUND edgeAC
    //   If ObstacleMap is NOT built, path may cross edgeAC
    
    // Create horizontal edge from A to C that will act as obstacle
    EdgeLayout& edgeAC = edgeLayouts_[edgeAC_];
    edgeAC.sourcePoint = {100, 25};  // Right edge of NodeA
    edgeAC.targetPoint = {150, 100}; // Left edge of NodeC  
    edgeAC.sourceEdge = NodeEdge::Right;
    edgeAC.targetEdge = NodeEdge::Left;
    edgeAC.bendPoints.clear();
    edgeAC.bendPoints.push_back({{100, 75}});  // Horizontal then vertical
    edgeAC.bendPoints.push_back({{150, 75}});
    
    // edgeAB: vertical from A to B
    EdgeLayout& edgeAB = edgeLayouts_[edgeAB_];
    edgeAB.sourcePoint = {50, 50};   // Bottom of NodeA
    edgeAB.targetPoint = {50, 100};  // Top of NodeB
    edgeAB.sourceEdge = NodeEdge::Bottom;
    edgeAB.targetEdge = NodeEdge::Top;
    edgeAB.bendPoints.clear();
    
    // Start drag on edgeAB source
    auto startResult = controller_.startDrag(edgeAB_, true, nodeLayouts_, edgeLayouts_, gridSize_);
    ASSERT_TRUE(startResult.success);
    
    // Move source to Right edge - this creates a path that could cross edgeAC
    int rightEdgeCandidateIdx = -1;
    for (size_t i = 0; i < startResult.candidates.size(); ++i) {
        if (startResult.candidates[i].edge == NodeEdge::Right) {
            rightEdgeCandidateIdx = static_cast<int>(i);
            break;
        }
    }
    ASSERT_GE(rightEdgeCandidateIdx, 0);
    
    Point newSourcePos = startResult.candidates[rightEdgeCandidateIdx].position;
    
    LayoutOptions options;
    options.gridConfig.cellSize = gridSize_;
    
    auto dropResult = controller_.completeDrag(
        rightEdgeCandidateIdx, newSourcePos, nodeLayouts_, edgeLayouts_, options);
    
    ASSERT_TRUE(dropResult.success);
    
    EdgeLayout& afterEdgeAB = edgeLayouts_[edgeAB_];
    
    // VERIFY: Path must be orthogonal
    EXPECT_TRUE(isEdgeOrthogonal(afterEdgeAB))
        << "Edge must be orthogonal!\n"
        << getDiagonalInfo(afterEdgeAB);
    
    // VERIFY: If ObstacleMap was built correctly with edgeAC segments,
    // the new path should NOT cross edgeAC's segments
    // This is hard to verify directly, but we can check:
    // 1. Path has bend points (required for Right->Top routing around obstacles)
    // 2. Path is orthogonal (required for valid routing)
    
    // For Right edge source to Top edge target, with edgeAC blocking,
    // the path MUST have bend points to route around
    EXPECT_FALSE(afterEdgeAB.bendPoints.empty())
        << "OBSTACLE AVOIDANCE FAIL: Path from Right edge to Top edge "
        << "should have bend points to avoid other edges\n"
        << "If bendPoints is empty, ObstacleMap was not built with edge segments!\n"
        << getDiagonalInfo(afterEdgeAB);
    
    std::cout << "\n=== ObstacleMap Verification ===" << std::endl;
    std::cout << "edgeAC (obstacle): src=(" << edgeAC.sourcePoint.x << "," << edgeAC.sourcePoint.y
              << ") tgt=(" << edgeAC.targetPoint.x << "," << edgeAC.targetPoint.y 
              << ") bends=" << edgeAC.bendPoints.size() << std::endl;
    std::cout << "edgeAB (moved): src=(" << afterEdgeAB.sourcePoint.x << "," << afterEdgeAB.sourcePoint.y
              << ") tgt=(" << afterEdgeAB.targetPoint.x << "," << afterEdgeAB.targetPoint.y
              << ") bends=" << afterEdgeAB.bendPoints.size() << std::endl;
    std::cout << getDiagonalInfo(afterEdgeAB);
}

// TDD RED: Full flow verification with explicit checks
TEST_F(SnapPointControllerTest, CompleteDrag_FullFlowVerification) {
    // This test verifies the complete flow is executed:
    // 1. Snap point update
    // 2. ObstacleMap build (we can't directly check, but result proves it)
    // 3. A* pathfinding (we can't directly check, but result proves it)
    // 4. bendPoints update (we CAN check)
    // 5. Orthogonal result (we CAN check)
    
    EdgeLayout& edgeAB = edgeLayouts_[edgeAB_];
    edgeAB.sourcePoint = {50, 50};
    edgeAB.targetPoint = {50, 100};
    edgeAB.sourceEdge = NodeEdge::Bottom;
    edgeAB.targetEdge = NodeEdge::Top;
    edgeAB.bendPoints.clear();
    
    // Record initial state
    Point initialSource = edgeAB.sourcePoint;
    NodeEdge initialEdge = edgeAB.sourceEdge;
    size_t initialBendCount = edgeAB.bendPoints.size();
    
    // Start drag
    auto startResult = controller_.startDrag(edgeAB_, true, nodeLayouts_, edgeLayouts_, gridSize_);
    ASSERT_TRUE(startResult.success);
    
    // Find candidate on Right edge
    int candidateIdx = -1;
    Point candidatePos;
    for (size_t i = 0; i < startResult.candidates.size(); ++i) {
        if (startResult.candidates[i].edge == NodeEdge::Right) {
            candidateIdx = static_cast<int>(i);
            candidatePos = startResult.candidates[i].position;
            break;
        }
    }
    ASSERT_GE(candidateIdx, 0);
    
    LayoutOptions options;
    options.gridConfig.cellSize = gridSize_;
    
    // Complete drag
    auto dropResult = controller_.completeDrag(
        candidateIdx, candidatePos, nodeLayouts_, edgeLayouts_, options);
    
    ASSERT_TRUE(dropResult.success);
    
    EdgeLayout& afterEdge = edgeLayouts_[edgeAB_];
    
    // STEP 1 CHECK: Snap point updated
    EXPECT_NE(afterEdge.sourceEdge, initialEdge)
        << "STEP 1 FAIL: Source edge should have changed";
    EXPECT_EQ(afterEdge.sourceEdge, NodeEdge::Right)
        << "STEP 1 FAIL: Source edge should be Right";
    
    // STEP 4 CHECK: bendPoints updated (should be non-empty for Right->Top routing)
    // Right edge of NodeA: x=100, y varies
    // Top edge of NodeB: y=100, x varies
    // These require at least one bend point
    EXPECT_FALSE(afterEdge.bendPoints.empty())
        << "STEP 4 FAIL: bendPoints should be updated for Right->Top routing\n"
        << "Right edge source cannot directly connect to Top edge target\n"
        << getDiagonalInfo(afterEdge);
    
    // STEP 5 CHECK: Result is orthogonal
    EXPECT_TRUE(isEdgeOrthogonal(afterEdge))
        << "STEP 5 FAIL: Final path must be orthogonal\n"
        << getDiagonalInfo(afterEdge);
    
    // Additional: Print path for debugging
    std::cout << "\n=== Full Flow Verification ===\n";
    std::cout << "Initial: source=(" << initialSource.x << "," << initialSource.y 
              << ") edge=" << static_cast<int>(initialEdge) 
              << " bends=" << initialBendCount << "\n";
    std::cout << "After: source=(" << afterEdge.sourcePoint.x << "," << afterEdge.sourcePoint.y
              << ") edge=" << static_cast<int>(afterEdge.sourceEdge)
              << " bends=" << afterEdge.bendPoints.size() << "\n";
    std::cout << getDiagonalInfo(afterEdge);
}

// =============================================================================
// TDD RED: Segment Overlap Detection
// =============================================================================
// When regenerateBendPoints() is called, it must consider OTHER edges
// (not in affectedEdges) as obstacles. Otherwise, segments can overlap.

// Helper: Get all segments from an edge layout
std::vector<std::pair<Point, Point>> getEdgeSegments(const EdgeLayout& layout) {
    std::vector<std::pair<Point, Point>> segments;
    std::vector<Point> points;
    
    points.push_back(layout.sourcePoint);
    for (const auto& bp : layout.bendPoints) {
        points.push_back(bp.position);
    }
    points.push_back(layout.targetPoint);
    
    for (size_t i = 0; i + 1 < points.size(); ++i) {
        segments.push_back({points[i], points[i + 1]});
    }
    return segments;
}

// Helper: Check if two segments overlap (collinear and sharing length)
struct OverlapInfo {
    bool overlaps = false;
    std::string type;  // "horizontal" or "vertical"
    float coordinate;  // shared x (vertical) or y (horizontal)
    float rangeStart;
    float rangeEnd;
};

OverlapInfo checkSegmentOverlap(const std::pair<Point, Point>& seg1,
                                 const std::pair<Point, Point>& seg2,
                                 float tolerance = 1.0f) {
    OverlapInfo result;
    
    auto [p1a, p1b] = seg1;
    auto [p2a, p2b] = seg2;
    
    // Check horizontal overlap (both segments at same Y)
    bool seg1Horizontal = std::abs(p1a.y - p1b.y) < tolerance;
    bool seg2Horizontal = std::abs(p2a.y - p2b.y) < tolerance;
    
    if (seg1Horizontal && seg2Horizontal && std::abs(p1a.y - p2a.y) < tolerance) {
        float x1Min = std::min(p1a.x, p1b.x);
        float x1Max = std::max(p1a.x, p1b.x);
        float x2Min = std::min(p2a.x, p2b.x);
        float x2Max = std::max(p2a.x, p2b.x);
        
        if (x1Min < x2Max && x2Min < x1Max) {
            result.overlaps = true;
            result.type = "horizontal";
            result.coordinate = p1a.y;
            result.rangeStart = std::max(x1Min, x2Min);
            result.rangeEnd = std::min(x1Max, x2Max);
        }
    }
    
    // Check vertical overlap (both segments at same X)
    bool seg1Vertical = std::abs(p1a.x - p1b.x) < tolerance;
    bool seg2Vertical = std::abs(p2a.x - p2b.x) < tolerance;
    
    if (seg1Vertical && seg2Vertical && std::abs(p1a.x - p2a.x) < tolerance) {
        float y1Min = std::min(p1a.y, p1b.y);
        float y1Max = std::max(p1a.y, p1b.y);
        float y2Min = std::min(p2a.y, p2b.y);
        float y2Max = std::max(p2a.y, p2b.y);
        
        if (y1Min < y2Max && y2Min < y1Max) {
            result.overlaps = true;
            result.type = "vertical";
            result.coordinate = p1a.x;
            result.rangeStart = std::max(y1Min, y2Min);
            result.rangeEnd = std::min(y1Max, y2Max);
        }
    }
    
    return result;
}

// Helper: Check all edge pairs for segment overlap
std::string findAllOverlaps(const std::unordered_map<EdgeId, EdgeLayout>& edgeLayouts) {
    std::ostringstream oss;
    std::vector<std::pair<EdgeId, EdgeLayout>> edges(edgeLayouts.begin(), edgeLayouts.end());
    
    for (size_t i = 0; i < edges.size(); ++i) {
        for (size_t j = i + 1; j < edges.size(); ++j) {
            auto segs1 = getEdgeSegments(edges[i].second);
            auto segs2 = getEdgeSegments(edges[j].second);
            
            for (size_t si = 0; si < segs1.size(); ++si) {
                for (size_t sj = 0; sj < segs2.size(); ++sj) {
                    auto overlap = checkSegmentOverlap(segs1[si], segs2[sj]);
                    if (overlap.overlaps) {
                        oss << "OVERLAP: e" << edges[i].first << "[seg" << si << "] & "
                            << "e" << edges[j].first << "[seg" << sj << "]: "
                            << overlap.type << " at " 
                            << (overlap.type == "vertical" ? "x" : "y") << "="
                            << overlap.coordinate << " range=[" 
                            << overlap.rangeStart << "," << overlap.rangeEnd << "]\n";
                    }
                }
            }
        }
    }
    return oss.str();
}

// TDD RED: regenerateBendPoints must avoid OTHER edges' segments
// This test creates a scenario where:
// - NodeA and NodeC are vertically aligned (center x=75)
// - edgeCD has a vertical segment at x=70 (grid-aligned)
// - edgeAC's snap point is moved to Bottom edge, creating path through x=75
// - The new path for edgeAC must NOT overlap with edgeCD's segment
TEST_F(SnapPointControllerTest, CompleteDrag_MustNotOverlapWithOtherEdgeSegments) {
    // Reposition nodes for vertical alignment at center x=75
    // NodeA at (25, 0), size (100, 50) → center x=75, bottom y=50
    nodeLayouts_[nodeA_] = NodeLayout{nodeA_, {25, 0}, {100, 50}};
    
    // NodeC at (25, 150), size (100, 50) → center x=75, top y=150
    nodeLayouts_[nodeC_] = NodeLayout{nodeC_, {25, 150}, {100, 50}};
    
    // NodeB moved to not interfere
    nodeLayouts_[nodeB_] = NodeLayout{nodeB_, {200, 0}, {100, 50}};
    
    // Add NodeD for edgeCD target
    NodeId nodeD = graph_.addNode(Size{100, 50}, "NodeD");
    nodeLayouts_[nodeD] = NodeLayout{nodeD, {200, 150}, {100, 50}};
    
    // edgeCD: create path with vertical segment at x=75, y=[75, 175]
    // Route: Left of C (25, 175) -> (75, 175) -> (75, 75) -> (200, 75) -> Right of D
    EdgeId edgeCD = graph_.addEdge(nodeC_, nodeD, "C->D");
    EdgeLayout layoutCD;
    layoutCD.id = edgeCD;
    layoutCD.from = nodeC_;
    layoutCD.to = nodeD;
    layoutCD.sourcePoint = {25, 175};   // Left edge of NodeC
    layoutCD.targetPoint = {200, 175};  // Left edge of NodeD
    layoutCD.sourceEdge = NodeEdge::Left;
    layoutCD.targetEdge = NodeEdge::Left;
    layoutCD.bendPoints.clear();
    layoutCD.bendPoints.push_back({{70, 175}});  // Creates horizontal segment
    layoutCD.bendPoints.push_back({{70, 60}});   // Creates VERTICAL segment at x=70, y=[60,175] - overlaps with y=50~150
    layoutCD.bendPoints.push_back({{200, 75}});  // Creates horizontal segment
    layoutCD.sourceSnapIndex = 5;
    layoutCD.targetSnapIndex = 5;
    edgeLayouts_[edgeCD] = layoutCD;
    
    // edgeAB: simple edge (not important for this test)
    EdgeLayout& edgeAB = edgeLayouts_[edgeAB_];
    edgeAB.sourcePoint = {125, 25};   // Right of NodeA
    edgeAB.targetPoint = {200, 25};   // Left of NodeB
    edgeAB.sourceEdge = NodeEdge::Right;
    edgeAB.targetEdge = NodeEdge::Left;
    edgeAB.bendPoints.clear();
    
    // edgeAC: initially from Right edge of A to Top of C
    // Will be dragged to Bottom edge, creating vertical path at x=75
    EdgeLayout& edgeAC = edgeLayouts_[edgeAC_];
    edgeAC.sourcePoint = {125, 25};   // Right edge of NodeA
    edgeAC.targetPoint = {75, 150};   // Top center of NodeC
    edgeAC.sourceEdge = NodeEdge::Right;
    edgeAC.targetEdge = NodeEdge::Top;
    edgeAC.sourceSnapIndex = 5;
    edgeAC.targetSnapIndex = 5;
    edgeAC.bendPoints.clear();
    edgeAC.bendPoints.push_back({{125, 150}});  // Initial orthogonal path
    
    std::cout << "\n=== BEFORE drag ===" << std::endl;
    std::cout << "edgeCD vertical segment at x=70, y=[75, 175]" << std::endl;
    std::cout << "edgeAC source on Right edge: (" << edgeAC.sourcePoint.x 
              << ", " << edgeAC.sourcePoint.y << ")" << std::endl;
    
    // Start drag on edgeAC source
    auto startResult = controller_.startDrag(edgeAC_, true, nodeLayouts_, edgeLayouts_, gridSize_);
    ASSERT_TRUE(startResult.success);
    
    // Move source to BOTTOM edge of NodeA
    // Some candidates near x=70 will be blocked (edgeCD's vertical segment is there)
    // We should find a NON-BLOCKED candidate on the Bottom edge
    
    // First, verify that blocked candidates exist (the blocking logic works)
    int blockedCount = 0;
    int unblockedBottomCount = 0;
    for (const auto& c : startResult.candidates) {
        if (c.blocked) blockedCount++;
        if (c.edge == NodeEdge::Bottom && !c.blocked) unblockedBottomCount++;
    }
    std::cout << "Blocked candidates: " << blockedCount << std::endl;
    std::cout << "Unblocked Bottom candidates: " << unblockedBottomCount << std::endl;
    
    // Find a NON-BLOCKED candidate on Bottom edge (furthest from x=70)
    int bottomEdgeCandidateIdx = -1;
    float bestDistance = -1.0f;
    for (size_t i = 0; i < startResult.candidates.size(); ++i) {
        const auto& c = startResult.candidates[i];
        if (c.edge == NodeEdge::Bottom && !c.blocked) {
            // Prefer candidates far from x=70 (the blocked zone)
            float distFrom70 = std::abs(c.position.x - 70.0f);
            if (distFrom70 > bestDistance) {
                bestDistance = distFrom70;
                bottomEdgeCandidateIdx = static_cast<int>(i);
            }
        }
    }
    ASSERT_GE(bottomEdgeCandidateIdx, 0) << "Should find non-blocked candidate on Bottom edge";
    
    Point newSourcePos = startResult.candidates[bottomEdgeCandidateIdx].position;
    std::cout << "Moving source to Bottom edge: (" << newSourcePos.x 
              << ", " << newSourcePos.y << ")" << std::endl;
    
    LayoutOptions options;
    options.gridConfig.cellSize = gridSize_;
    
    // Complete drag - this should route edgeAC avoiding edgeCD's segments
    auto dropResult = controller_.completeDrag(
        bottomEdgeCandidateIdx, newSourcePos, nodeLayouts_, edgeLayouts_, options);
    
    ASSERT_TRUE(dropResult.success);
    
    // Debug output
    std::cout << "\n=== AFTER drag ===" << std::endl;
    
    EdgeLayout& afterEdgeAC = edgeLayouts_[edgeAC_];
    std::cout << "edgeAC path: src=(" << afterEdgeAC.sourcePoint.x 
              << ", " << afterEdgeAC.sourcePoint.y << ")";
    for (const auto& bp : afterEdgeAC.bendPoints) {
        std::cout << " -> (" << bp.position.x << ", " << bp.position.y << ")";
    }
    std::cout << " -> tgt=(" << afterEdgeAC.targetPoint.x 
              << ", " << afterEdgeAC.targetPoint.y << ")" << std::endl;
    
    // Show edgeCD for reference
    std::cout << "edgeCD path: src=(" << layoutCD.sourcePoint.x 
              << ", " << layoutCD.sourcePoint.y << ")";
    for (const auto& bp : layoutCD.bendPoints) {
        std::cout << " -> (" << bp.position.x << ", " << bp.position.y << ")";
    }
    std::cout << " -> tgt=(" << layoutCD.targetPoint.x 
              << ", " << layoutCD.targetPoint.y << ")" << std::endl;
    
    // CRITICAL CHECK: edgeAC must not overlap with edgeCD
    std::string overlaps = findAllOverlaps(edgeLayouts_);
    
    std::cout << "\n=== Overlap Analysis ===" << std::endl;
    if (overlaps.empty()) {
        std::cout << "No overlaps found." << std::endl;
    } else {
        std::cout << overlaps;
    }
    
    // THE ASSERTION: No segment overlaps allowed!
    // BUG: Phase 1 of regenerateBendPoints() doesn't add edgeCD as obstacle,
    // so edgeAC's path uses x=75 vertical segment, overlapping with edgeCD.
    // Expected overlap: x=70, y=[75, 140] (if bug exists)
    EXPECT_TRUE(overlaps.empty())
        << "Segments must not overlap after snap point drag!\n"
        << "Phase 1 of regenerateBendPoints() must consider OTHER edges as obstacles.\n"
        << overlaps;
}
