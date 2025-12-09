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
