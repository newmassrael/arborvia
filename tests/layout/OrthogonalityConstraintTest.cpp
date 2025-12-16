/**
 * @file OrthogonalityConstraintTest.cpp
 * @brief TDD tests for orthogonality constraint violations
 *
 * Two issues discovered during constraint violation analysis:
 * 1. Negative coordinates cause A* failure → diagonal segments preserved
 * 2. Snap collision detected but not resolved → duplicate snap points
 */

#include <gtest/gtest.h>
#include "../../src/layout/optimization/astar/AStarEdgeOptimizer.h"
#include "../../src/layout/pathfinding/AStarPathFinder.h"
#include "../../src/layout/optimization/OptimizerRegistry.h"
#include "arborvia/core/Types.h"

namespace arborvia {
namespace test {

class OrthogonalityConstraintTest : public ::testing::Test {
protected:
    void SetUp() override {
        pathFinder_ = std::make_shared<AStarPathFinder>();
        optimizer_ = std::make_unique<AStarEdgeOptimizer>(pathFinder_);
    }

    // Helper to check if path is orthogonal
    bool isPathOrthogonal(const EdgeLayout& layout) {
        std::vector<Point> points;
        points.push_back(layout.sourcePoint);
        for (const auto& bp : layout.bendPoints) {
            points.push_back(bp.position);
        }
        points.push_back(layout.targetPoint);

        for (size_t i = 0; i + 1 < points.size(); ++i) {
            float dx = std::abs(points[i + 1].x - points[i].x);
            float dy = std::abs(points[i + 1].y - points[i].y);
            // Must be horizontal (dy < 1) or vertical (dx < 1)
            if (dx > 1.0f && dy > 1.0f) {
                return false;  // Diagonal!
            }
        }
        return true;
    }

    // Helper to check for duplicate snap points on same NodeEdge
    bool hasSnapPointDuplicates(
        const std::unordered_map<EdgeId, EdgeLayout>& edgeLayouts) {

        // Group by (nodeId, nodeEdge) -> list of snap positions
        std::map<std::tuple<NodeId, int>, std::vector<std::pair<EdgeId, Point>>> snapPoints;

        for (const auto& [edgeId, layout] : edgeLayouts) {
            auto srcKey = std::make_tuple(layout.from, static_cast<int>(layout.sourceEdge));
            snapPoints[srcKey].push_back({edgeId, layout.sourcePoint});

            auto tgtKey = std::make_tuple(layout.to, static_cast<int>(layout.targetEdge));
            snapPoints[tgtKey].push_back({edgeId, layout.targetPoint});
        }

        // Check for duplicates
        for (const auto& [key, points] : snapPoints) {
            for (size_t i = 0; i < points.size(); ++i) {
                for (size_t j = i + 1; j < points.size(); ++j) {
                    float dx = std::abs(points[i].second.x - points[j].second.x);
                    float dy = std::abs(points[i].second.y - points[j].second.y);
                    if (dx < 1.0f && dy < 1.0f) {
                        return true;  // Duplicate found!
                    }
                }
            }
        }
        return false;
    }

    std::shared_ptr<AStarPathFinder> pathFinder_;
    std::unique_ptr<AStarEdgeOptimizer> optimizer_;
};

// =============================================================================
// Issue 1: Negative coordinates cause A* failure → diagonal preserved
// =============================================================================

TEST_F(OrthogonalityConstraintTest, NegativeCoordinates_ShouldNotCreateDiagonal) {
    // Setup: Node 1 at negative Y coordinate (dragged outside screen)
    std::unordered_map<NodeId, NodeLayout> nodeLayouts;
    nodeLayouts[0] = {0, {0, 0}, {200, 100}, 0, 0};      // Node 0 at origin
    nodeLayouts[1] = {1, {440, -60}, {200, 100}, 0, 1};  // Node 1 at NEGATIVE Y!

    std::unordered_map<EdgeId, EdgeLayout> edgeLayouts;
    EdgeLayout edge0;
    edge0.id = 0;
    edge0.from = 0;
    edge0.to = 1;
    edge0.sourceEdge = NodeEdge::Right;
    edge0.targetEdge = NodeEdge::Left;
    edge0.sourcePoint = {200, 50};   // Right edge of Node 0
    edge0.targetPoint = {440, -10};  // Left edge of Node 1 (negative Y!)
    // Existing bendPoints that create diagonal with current sourcePoint
    edge0.bendPoints = {
        {{100, 120}},  // This creates diagonal from (200,50)!
        {{540, 120}}
    };
    edgeLayouts[0] = edge0;

    // Act: Regenerate bend points
    std::vector<EdgeId> edges = {0};
    optimizer_->regenerateBendPoints(edges, edgeLayouts, nodeLayouts, 20.0f);

    // Assert: Path MUST be orthogonal even with negative coordinates
    EXPECT_TRUE(isPathOrthogonal(edgeLayouts[0]))
        << "Edge 0 has DIAGONAL segment after A* with negative coordinates!\n"
        << "src=(" << edgeLayouts[0].sourcePoint.x << "," << edgeLayouts[0].sourcePoint.y << ")\n"
        << "bend[0]=(" << edgeLayouts[0].bendPoints[0].position.x << ","
                       << edgeLayouts[0].bendPoints[0].position.y << ")";
}

TEST_F(OrthogonalityConstraintTest, AStarFailure_ShouldNotPreserveDiagonalBendPoints) {
    // Setup: Create a situation where A* will fail
    std::unordered_map<NodeId, NodeLayout> nodeLayouts;
    nodeLayouts[0] = {0, {0, 0}, {200, 100}, 0, 0};
    nodeLayouts[1] = {1, {0, 200}, {200, 100}, 1, 0};
    nodeLayouts[2] = {2, {0, 400}, {200, 100}, 2, 0};

    std::unordered_map<EdgeId, EdgeLayout> edgeLayouts;

    // Edge 0: will be blocked by Edge 1
    EdgeLayout edge0;
    edge0.id = 0;
    edge0.from = 0;
    edge0.to = 2;
    edge0.sourceEdge = NodeEdge::Bottom;
    edge0.targetEdge = NodeEdge::Top;
    edge0.sourcePoint = {100, 100};
    edge0.targetPoint = {100, 400};
    // Diagonal bendPoints (from previous state when sourcePoint was different)
    edge0.bendPoints = {
        {{120, 150}},  // Diagonal from (100,100)!
        {{120, 380}}
    };
    edgeLayouts[0] = edge0;

    // Edge 1: Blocks Edge 0's path completely
    EdgeLayout edge1;
    edge1.id = 1;
    edge1.from = 0;
    edge1.to = 1;
    edge1.sourceEdge = NodeEdge::Bottom;
    edge1.targetEdge = NodeEdge::Top;
    edge1.sourcePoint = {100, 100};  // Same as Edge 0!
    edge1.targetPoint = {100, 200};
    edge1.bendPoints = {};  // Direct path blocks (100, 100->200)
    edgeLayouts[1] = edge1;

    // Act
    std::vector<EdgeId> edges = {0};
    optimizer_->regenerateBendPoints(edges, edgeLayouts, nodeLayouts, 20.0f);

    // Assert: Even if A* fails, path MUST be orthogonal
    EXPECT_TRUE(isPathOrthogonal(edgeLayouts[0]))
        << "Edge 0 has DIAGONAL after A* failure - diagonal bendPoints were preserved!";
}

// =============================================================================
// Issue 2: Snap collision detected but not resolved
// =============================================================================

TEST_F(OrthogonalityConstraintTest, SnapCollision_ShouldBeResolved) {
    // Setup: Two edges from same node, same edge, should get different snap points
    std::unordered_map<NodeId, NodeLayout> nodeLayouts;
    nodeLayouts[0] = {0, {0, 0}, {200, 100}, 0, 0};
    nodeLayouts[1] = {1, {0, 200}, {200, 100}, 1, 0};
    nodeLayouts[2] = {2, {200, 200}, {200, 100}, 1, 1};

    std::unordered_map<EdgeId, EdgeLayout> edgeLayouts;

    // Edge 0: Node 0 bottom -> Node 1
    EdgeLayout edge0;
    edge0.id = 0;
    edge0.from = 0;
    edge0.to = 1;
    edge0.sourceEdge = NodeEdge::Bottom;
    edge0.targetEdge = NodeEdge::Top;
    edge0.sourcePoint = {100, 100};  // Same snap point!
    edge0.targetPoint = {100, 200};
    edgeLayouts[0] = edge0;

    // Edge 1: Node 0 bottom -> Node 2 (SAME source snap point!)
    EdgeLayout edge1;
    edge1.id = 1;
    edge1.from = 0;
    edge1.to = 2;
    edge1.sourceEdge = NodeEdge::Bottom;
    edge1.targetEdge = NodeEdge::Top;
    edge1.sourcePoint = {100, 100};  // DUPLICATE! Same as Edge 0
    edge1.targetPoint = {300, 200};
    edgeLayouts[1] = edge1;

    // Precondition: Verify duplicate exists
    ASSERT_TRUE(hasSnapPointDuplicates(edgeLayouts))
        << "Test setup error: Expected duplicate snap points";

    // Act: Optimize with movedNodes to trigger snap recalculation
    std::vector<EdgeId> allEdges = {0, 1};
    std::unordered_set<NodeId> movedNodes = {0};

    OptimizerConfig config = OptimizerConfig::aggressive();
    config.preserveDirections = false;
    auto optimizer = OptimizerRegistry::instance().create("AStar", config);
    ASSERT_NE(optimizer, nullptr) << "Failed to create AStar optimizer";

    auto optimizedLayouts = optimizer->optimize(
        allEdges, edgeLayouts, nodeLayouts, 20.0f, movedNodes);

    // Apply optimized layouts
    for (const auto& [id, layout] : optimizedLayouts) {
        edgeLayouts[id] = layout;
    }

    // Assert: No duplicate snap points after redistribution
    EXPECT_FALSE(hasSnapPointDuplicates(edgeLayouts))
        << "Snap collision was detected but NOT resolved!\n"
        << "Edge 0 src=(" << edgeLayouts[0].sourcePoint.x << ","
                          << edgeLayouts[0].sourcePoint.y << ")\n"
        << "Edge 1 src=(" << edgeLayouts[1].sourcePoint.x << ","
                          << edgeLayouts[1].sourcePoint.y << ")";
}

TEST_F(OrthogonalityConstraintTest, SnapCollision_AfterNodeMove_ShouldRedistribute) {
    // Setup: Multiple edges from Node 1's bottom edge
    std::unordered_map<NodeId, NodeLayout> nodeLayouts;
    nodeLayouts[0] = {0, {0, 0}, {200, 100}, 0, 0};
    nodeLayouts[1] = {1, {0, 200}, {200, 100}, 1, 0};
    nodeLayouts[2] = {2, {0, 400}, {200, 100}, 2, 0};
    nodeLayouts[3] = {3, {200, 400}, {200, 100}, 2, 1};

    std::unordered_map<EdgeId, EdgeLayout> edgeLayouts;

    // Edge 1: Node 1 -> Node 2
    EdgeLayout edge1;
    edge1.id = 1;
    edge1.from = 1;
    edge1.to = 2;
    edge1.sourceEdge = NodeEdge::Bottom;
    edge1.targetEdge = NodeEdge::Top;
    edge1.sourcePoint = {100, 300};  // Will collide after move
    edge1.targetPoint = {100, 400};
    edgeLayouts[1] = edge1;

    // Edge 3: Node 1 -> Node 3 (same source NodeEdge)
    EdgeLayout edge3;
    edge3.id = 3;
    edge3.from = 1;
    edge3.to = 3;
    edge3.sourceEdge = NodeEdge::Bottom;
    edge3.targetEdge = NodeEdge::Top;
    edge3.sourcePoint = {100, 300};  // SAME as Edge 1 - collision!
    edge3.targetPoint = {300, 400};
    edgeLayouts[3] = edge3;

    // Act: Optimize with movedNodes to trigger snap recalculation
    std::vector<EdgeId> allEdges = {1, 3};
    std::unordered_set<NodeId> movedNodes = {1};

    OptimizerConfig config = OptimizerConfig::aggressive();
    config.preserveDirections = false;
    auto optimizer = OptimizerRegistry::instance().create("AStar", config);
    ASSERT_NE(optimizer, nullptr) << "Failed to create AStar optimizer";

    auto optimizedLayouts = optimizer->optimize(
        allEdges, edgeLayouts, nodeLayouts, 20.0f, movedNodes);

    // Apply optimized layouts
    for (const auto& [id, layout] : optimizedLayouts) {
        edgeLayouts[id] = layout;
    }

    // Assert: Different snap points after redistribution
    float dx = std::abs(edgeLayouts[1].sourcePoint.x - edgeLayouts[3].sourcePoint.x);
    float dy = std::abs(edgeLayouts[1].sourcePoint.y - edgeLayouts[3].sourcePoint.y);

    EXPECT_TRUE(dx > 1.0f || dy > 1.0f)
        << "Snap collision not resolved after node move!\n"
        << "Edge 1 src=(" << edgeLayouts[1].sourcePoint.x << ","
                          << edgeLayouts[1].sourcePoint.y << ")\n"
        << "Edge 3 src=(" << edgeLayouts[3].sourcePoint.x << ","
                          << edgeLayouts[3].sourcePoint.y << ")";
}

// =============================================================================
// Issue 3: Only target moved - sibling edge not redistributed
// =============================================================================

TEST_F(OrthogonalityConstraintTest, TargetMoved_SiblingEdge_ShouldBeRedistributed) {
    // Setup: Two edges from Node 1, both from Bottom edge
    // Only Node 2 (target of Edge A) is moved, Node 3 (target of Edge B) is not moved
    // BUG: Edge B is not in updates, so it keeps old snap position
    // This can cause snap collision or A* blocking
    
    std::unordered_map<NodeId, NodeLayout> nodeLayouts;
    nodeLayouts[1] = {1, {0, 0}, {200, 100}, 0, 0};       // Node 1: source
    nodeLayouts[2] = {2, {0, 200}, {200, 100}, 1, 0};     // Node 2: target of Edge A
    nodeLayouts[3] = {3, {200, 200}, {200, 100}, 1, 1};   // Node 3: target of Edge B

    std::unordered_map<EdgeId, EdgeLayout> edgeLayouts;

    // Edge A: Node 1 Bottom -> Node 2 Top (target will be moved)
    EdgeLayout edgeA;
    edgeA.id = 0;
    edgeA.from = 1;
    edgeA.to = 2;
    edgeA.sourceEdge = NodeEdge::Bottom;
    edgeA.targetEdge = NodeEdge::Top;
    edgeA.sourcePoint = {100, 100};  // Initially at center
    edgeA.targetPoint = {100, 200};
    edgeLayouts[0] = edgeA;

    // Edge B: Node 1 Bottom -> Node 3 Top (target will NOT be moved)
    // SAME source edge as Edge A!
    EdgeLayout edgeB;
    edgeB.id = 1;
    edgeB.from = 1;
    edgeB.to = 3;
    edgeB.sourceEdge = NodeEdge::Bottom;
    edgeB.targetEdge = NodeEdge::Top;
    edgeB.sourcePoint = {100, 100};  // SAME as Edge A - collision setup
    edgeB.targetPoint = {300, 200};
    edgeLayouts[1] = edgeB;

    // Precondition: Verify duplicate exists
    ASSERT_TRUE(hasSnapPointDuplicates(edgeLayouts))
        << "Test setup error: Expected duplicate snap points";

    // Act: Move Node 2 ONLY (not Node 1 or Node 3)
    // Edge A is in updates (its target moved)
    // Edge B is NOT in updates (neither source nor target moved)
    std::vector<EdgeId> allEdges = {0, 1};
    std::unordered_set<NodeId> movedNodes = {2};  // Only Node 2 moved!

    OptimizerConfig config = OptimizerConfig::aggressive();
    config.preserveDirections = false;
    auto optimizer = OptimizerRegistry::instance().create("AStar", config);
    ASSERT_NE(optimizer, nullptr) << "Failed to create AStar optimizer";

    auto optimizedLayouts = optimizer->optimize(
        allEdges, edgeLayouts, nodeLayouts, 20.0f, movedNodes);

    // Apply optimized layouts
    for (const auto& [id, layout] : optimizedLayouts) {
        edgeLayouts[id] = layout;
    }

    // Assert: Both edges should have different snap points after redistribution
    // Even though only Edge A's target was moved, Edge B should also be redistributed
    // because they share the same source (Node 1 Bottom)
    float dx = std::abs(edgeLayouts[0].sourcePoint.x - edgeLayouts[1].sourcePoint.x);
    float dy = std::abs(edgeLayouts[0].sourcePoint.y - edgeLayouts[1].sourcePoint.y);

    EXPECT_TRUE(dx > 1.0f || dy > 1.0f)
        << "Snap collision not resolved! Sibling edges not redistributed when only target moved.\\n"
        << "Edge A src=(" << edgeLayouts[0].sourcePoint.x << ","
                          << edgeLayouts[0].sourcePoint.y << ")\\n"
        << "Edge B src=(" << edgeLayouts[1].sourcePoint.x << ","
                          << edgeLayouts[1].sourcePoint.y << ")";
}

// =============================================================================
// Issue 4: Snap collision + existing diagonal bendPoints = A* failure preserves diagonal
// =============================================================================

TEST_F(OrthogonalityConstraintTest, ParallelPath_SnapCollision_MustUseRedistributedLayouts) {
    // BUG: Parallel optimization path uses currentLayouts (original) instead of
    // assignedLayouts (after snap redistribution). This causes snap collision
    // even after recalculateSourceEdgesForMovedNodes() runs.
    //
    // This test uses 4 edges to trigger useParallel (needs >= 3 edges).
    
    std::unordered_map<NodeId, NodeLayout> nodeLayouts;
    nodeLayouts[1] = {1, {0, 0}, {200, 100}, 0, 0};       // Node 1: source
    nodeLayouts[2] = {2, {0, 200}, {200, 100}, 1, 0};     // Node 2
    nodeLayouts[3] = {3, {200, 200}, {200, 100}, 1, 1};   // Node 3
    nodeLayouts[4] = {4, {400, 200}, {200, 100}, 1, 2};   // Node 4
    nodeLayouts[5] = {5, {600, 200}, {200, 100}, 1, 3};   // Node 5

    std::unordered_map<EdgeId, EdgeLayout> edgeLayouts;

    // 4 edges from Node 1 Bottom - will trigger parallel optimization
    // All share same snap point initially (collision)
    for (int i = 0; i < 4; ++i) {
        EdgeLayout edge;
        edge.id = i;
        edge.from = 1;
        edge.to = 2 + i;  // To nodes 2, 3, 4, 5
        edge.sourceEdge = NodeEdge::Bottom;
        edge.targetEdge = NodeEdge::Top;
        edge.sourcePoint = {100, 100};  // ALL SAME - collision!
        edge.targetPoint = {static_cast<float>(100 + i * 200), 200};
        edgeLayouts[i] = edge;
    }

    // Precondition: Verify duplicates exist
    ASSERT_TRUE(hasSnapPointDuplicates(edgeLayouts))
        << "Test setup error: Expected duplicate snap points";

    // Act: Optimize with movedNodes (triggers parallel path for 4 edges)
    std::vector<EdgeId> allEdges = {0, 1, 2, 3};
    std::unordered_set<NodeId> movedNodes = {1};  // Source node moved

    OptimizerConfig config = OptimizerConfig::aggressive();
    config.preserveDirections = false;
    auto optimizer = OptimizerRegistry::instance().create("AStar", config);
    ASSERT_NE(optimizer, nullptr) << "Failed to create AStar optimizer";

    auto optimizedLayouts = optimizer->optimize(
        allEdges, edgeLayouts, nodeLayouts, 20.0f, movedNodes);

    // Apply optimized layouts
    for (const auto& [id, layout] : optimizedLayouts) {
        edgeLayouts[id] = layout;
    }

    // Assert: No snap point duplicates after parallel optimization
    // This will FAIL if parallel path uses currentLayouts instead of assignedLayouts
    EXPECT_FALSE(hasSnapPointDuplicates(edgeLayouts))
        << "Snap collision exists after parallel optimization!\\n"
        << "BUG: Parallel path uses currentLayouts (original) instead of assignedLayouts\\n"
        << "Edge 0 src=(" << edgeLayouts[0].sourcePoint.x << ","
                          << edgeLayouts[0].sourcePoint.y << ")\\n"
        << "Edge 1 src=(" << edgeLayouts[1].sourcePoint.x << ","
                          << edgeLayouts[1].sourcePoint.y << ")\\n"
        << "Edge 2 src=(" << edgeLayouts[2].sourcePoint.x << ","
                          << edgeLayouts[2].sourcePoint.y << ")\\n"
        << "Edge 3 src=(" << edgeLayouts[3].sourcePoint.x << ","
                          << edgeLayouts[3].sourcePoint.y << ")";
}

TEST_F(OrthogonalityConstraintTest, ExactUserScenario_SnapCollision_BlocksFirstMove) {
    // EXACT reproduction from user's debug logs:
    // - Edge 2: src=(80,400), bend[0]=(-20,460) (DIAGONAL!)
    // - A* fails: start=(4,20) neighbor=(4,19) BLOCKED (moveDir=Up)
    // - gridSize=20, so (4,20)*20 = (80,400)
    //
    // Root cause: Edge segments from another edge at same snap point block A* first move
    
    std::unordered_map<NodeId, NodeLayout> nodeLayouts;
    // Node positions to create src=(80,400) scenario
    nodeLayouts[1] = {1, {0, 300}, {200, 100}, 0, 0};     // Node 1: source at y=300-400
    nodeLayouts[2] = {2, {-100, 500}, {200, 100}, 1, 0};  // Node 2: target of Edge 2
    nodeLayouts[3] = {3, {200, 500}, {200, 100}, 1, 1};   // Node 3: target of Edge 3

    std::unordered_map<EdgeId, EdgeLayout> edgeLayouts;

    // Edge 2: Has DIAGONAL bendPoints (exact reproduction from user log)
    EdgeLayout edge2;
    edge2.id = 2;
    edge2.from = 1;
    edge2.to = 2;
    edge2.sourceEdge = NodeEdge::Bottom;  // Bottom of Node 1
    edge2.targetEdge = NodeEdge::Top;
    edge2.sourcePoint = {80, 400};        // EXACT from user log
    edge2.targetPoint = {0, 500};
    // DIAGONAL: src=(80,400) -> bend=(-20,460) is diagonal!
    edge2.bendPoints = {
        {{-20, 460}},  // DIAGONAL from (80,400)!
        {{-20, 480}}
    };
    edgeLayouts[2] = edge2;

    // Edge 3: SAME snap point as Edge 2 (snap collision)
    EdgeLayout edge3;
    edge3.id = 3;
    edge3.from = 1;
    edge3.to = 3;
    edge3.sourceEdge = NodeEdge::Bottom;  // Same sourceEdge as Edge 2
    edge3.targetEdge = NodeEdge::Top;
    edge3.sourcePoint = {80, 400};        // SAME as Edge 2 - collision!
    edge3.targetPoint = {300, 500};
    // Edge 3 has a vertical segment from (80,400) going down
    // This segment blocks Edge 2's first move (trying to go UP from sourceEdge=Bottom)
    edge3.bendPoints = {
        {{80, 450}},   // Vertical segment blocks Edge 2's upward movement!
        {{300, 450}}
    };
    edgeLayouts[3] = edge3;

    // Precondition 1: Edge 2 has diagonal path
    ASSERT_FALSE(isPathOrthogonal(edgeLayouts[2]))
        << "Test setup error: Edge 2 should have diagonal bendPoints";

    // Precondition 2: Snap collision exists
    float dx = std::abs(edgeLayouts[2].sourcePoint.x - edgeLayouts[3].sourcePoint.x);
    float dy = std::abs(edgeLayouts[2].sourcePoint.y - edgeLayouts[3].sourcePoint.y);
    ASSERT_TRUE(dx < 1.0f && dy < 1.0f)
        << "Test setup error: Expected snap collision between Edge 2 and Edge 3";

    // Act: Call optimize (what happens in demo during drag)
    std::vector<EdgeId> edges = {2, 3};
    std::unordered_set<NodeId> movedNodes = {1};  // Source node moved
    
    OptimizerConfig config = OptimizerConfig::aggressive();
    config.preserveDirections = false;
    auto optimizer = OptimizerRegistry::instance().create("AStar", config);
    ASSERT_NE(optimizer, nullptr) << "Failed to create AStar optimizer";

    auto optimizedLayouts = optimizer->optimize(
        edges, edgeLayouts, nodeLayouts, 20.0f, movedNodes);

    // Apply optimized layouts
    for (const auto& [id, layout] : optimizedLayouts) {
        edgeLayouts[id] = layout;
    }

    // Assert 1: Edge 2's diagonal MUST be fixed
    EXPECT_TRUE(isPathOrthogonal(edgeLayouts[2]))
        << "Edge 2 still has DIAGONAL after regenerateBendPoints!\\n"
        << "This is the EXACT user-reported bug.\\n"
        << "src=(" << edgeLayouts[2].sourcePoint.x << "," << edgeLayouts[2].sourcePoint.y << ")\\n"
        << "bend[0]=(" << (edgeLayouts[2].bendPoints.empty() ? 0 : edgeLayouts[2].bendPoints[0].position.x)
        << "," << (edgeLayouts[2].bendPoints.empty() ? 0 : edgeLayouts[2].bendPoints[0].position.y) << ")";

    // Assert 2: Snap collision should be resolved
    dx = std::abs(edgeLayouts[2].sourcePoint.x - edgeLayouts[3].sourcePoint.x);
    dy = std::abs(edgeLayouts[2].sourcePoint.y - edgeLayouts[3].sourcePoint.y);
    EXPECT_TRUE(dx > 1.0f || dy > 1.0f)
        << "Snap collision still exists!\\n"
        << "Edge 2 src=(" << edgeLayouts[2].sourcePoint.x << ","
                          << edgeLayouts[2].sourcePoint.y << ")\\n"
        << "Edge 3 src=(" << edgeLayouts[3].sourcePoint.x << ","
                          << edgeLayouts[3].sourcePoint.y << ")";
}

// =============================================================================
// Issue 5: Parallel path uses currentLayouts instead of assignedLayouts
// This is the ROOT CAUSE of snap collision persisting during A* optimization
// =============================================================================

TEST_F(OrthogonalityConstraintTest, ParallelPath_MustUseAssignedLayouts_NotCurrentLayouts) {
    // BUG DESCRIPTION:
    // In AStarEdgeOptimizer::optimize(), the parallel path code does:
    //   1. assignedLayouts = currentLayouts (copy)
    //   2. recalculateSourceEdgesForMovedNodes(assignedLayouts, ...) - MODIFIES assignedLayouts
    //   3. BUT: parallel loop uses currentLayouts.find(edgeId) - WRONG! Uses OLD positions
    //   4. createCandidateLayout extracts candidateIdx from OLD positions
    //   5. All edges get SAME candidateIdx = SAME snap positions = COLLISION
    //
    // FIX: Use assignedLayouts.find(edgeId) instead of currentLayouts.find(edgeId)
    //
    // This test creates a scenario where:
    // - 4 edges share same snap position (triggers parallel path)
    // - Each edge has bendPoints that block other edges' first move
    // - If A* uses currentLayouts: edges block each other → A* fails → diagonal preserved
    // - If A* uses assignedLayouts: snap positions redistributed → no blocking → orthogonal

    std::unordered_map<NodeId, NodeLayout> nodeLayouts;
    // Source node at top
    nodeLayouts[1] = {1, {0, 0}, {200, 100}, 0, 0};
    // Target nodes arranged horizontally below
    nodeLayouts[2] = {2, {0, 200}, {100, 50}, 1, 0};
    nodeLayouts[3] = {3, {120, 200}, {100, 50}, 1, 1};
    nodeLayouts[4] = {4, {240, 200}, {100, 50}, 1, 2};
    nodeLayouts[5] = {5, {360, 200}, {100, 50}, 1, 3};

    std::unordered_map<EdgeId, EdgeLayout> edgeLayouts;

    // All 4 edges from Node 1 Bottom edge at SAME snap position
    // Each has a vertical segment that blocks others' downward movement
    float collisionX = 100.0f;  // All edges at same X
    float sourceY = 100.0f;     // Node 1 bottom

    for (int i = 0; i < 4; ++i) {
        EdgeLayout edge;
        edge.id = i;
        edge.from = 1;
        edge.to = 2 + i;  // To nodes 2, 3, 4, 5
        edge.sourceEdge = NodeEdge::Bottom;
        edge.targetEdge = NodeEdge::Top;
        edge.sourcePoint = {collisionX, sourceY};  // ALL SAME - collision!
        edge.targetPoint = {static_cast<float>(50 + i * 120), 200};
        
        // CRITICAL: Each edge has existing bendPoints going DOWN from source
        // This creates a vertical segment from (100, 100) to (100, 150)
        // When A* tries to route another edge from (100, 100), this segment BLOCKS it
        edge.bendPoints = {
            {{collisionX, 150}},  // Vertical segment blocks other edges!
            {{static_cast<float>(50 + i * 120), 150}}
        };
        edgeLayouts[i] = edge;
    }

    // Precondition 1: All edges start at same point (collision)
    for (int i = 1; i < 4; ++i) {
        float dx = std::abs(edgeLayouts[0].sourcePoint.x - edgeLayouts[i].sourcePoint.x);
        float dy = std::abs(edgeLayouts[0].sourcePoint.y - edgeLayouts[i].sourcePoint.y);
        ASSERT_TRUE(dx < 1.0f && dy < 1.0f)
            << "Test setup error: Edges 0 and " << i << " should have same sourcePoint";
    }

    // Precondition 2: All edges have bendPoints (will block A*)
    for (int i = 0; i < 4; ++i) {
        ASSERT_FALSE(edgeLayouts[i].bendPoints.empty())
            << "Test setup error: Edge " << i << " should have bendPoints";
    }

    // Act: Optimize with movedNodes (triggers recalculateSourceEdgesForMovedNodes)
    std::vector<EdgeId> allEdges = {0, 1, 2, 3};
    std::unordered_set<NodeId> movedNodes = {1};  // Source node moved

    OptimizerConfig config = OptimizerConfig::aggressive();
    config.preserveDirections = false;
    auto optimizer = OptimizerRegistry::instance().create("AStar", config);
    ASSERT_NE(optimizer, nullptr) << "Failed to create AStar optimizer";

    auto optimizedLayouts = optimizer->optimize(
        allEdges, edgeLayouts, nodeLayouts, 20.0f, movedNodes);

    // Apply optimized layouts
    for (const auto& [id, layout] : optimizedLayouts) {
        edgeLayouts[id] = layout;
    }

    // Assert 1: All edges must be orthogonal (no diagonal segments)
    // If bug exists: A* blocks → diagonal preserved
    // If fixed: snap redistribution → no blocking → orthogonal
    for (int i = 0; i < 4; ++i) {
        EXPECT_TRUE(isPathOrthogonal(edgeLayouts[i]))
            << "Edge " << i << " has DIAGONAL segments!\\n"
            << "src=(" << edgeLayouts[i].sourcePoint.x << ","
                       << edgeLayouts[i].sourcePoint.y << ")\\n"
            << "tgt=(" << edgeLayouts[i].targetPoint.x << ","
                       << edgeLayouts[i].targetPoint.y << ")\\n"
            << "bendPoints count: " << edgeLayouts[i].bendPoints.size();
    }

    // Assert 2: Snap positions must be redistributed (no collision)
    // Collect all sourcePoints
    std::vector<Point> sourcePoints;
    for (int i = 0; i < 4; ++i) {
        sourcePoints.push_back(edgeLayouts[i].sourcePoint);
    }

    // Check for duplicates
    bool hasCollision = false;
    for (size_t i = 0; i < sourcePoints.size(); ++i) {
        for (size_t j = i + 1; j < sourcePoints.size(); ++j) {
            float dx = std::abs(sourcePoints[i].x - sourcePoints[j].x);
            float dy = std::abs(sourcePoints[i].y - sourcePoints[j].y);
            if (dx < 1.0f && dy < 1.0f) {
                hasCollision = true;
                ADD_FAILURE() << "Snap collision between Edge " << i << " and Edge " << j << "!\\n"
                    << "Edge " << i << " src=(" << sourcePoints[i].x << "," << sourcePoints[i].y << ")\\n"
                    << "Edge " << j << " src=(" << sourcePoints[j].x << "," << sourcePoints[j].y << ")";
            }
        }
    }

    EXPECT_FALSE(hasCollision)
        << "Snap collision persists after optimization!\\n"
        << "BUG: Parallel path uses currentLayouts (original) instead of assignedLayouts (redistributed)";
}

}  // namespace test
}  // namespace arborvia
