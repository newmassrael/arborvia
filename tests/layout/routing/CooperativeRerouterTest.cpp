#include <gtest/gtest.h>
#include "../../../src/layout/routing/CooperativeRerouter.h"
#include "../../../src/layout/pathfinding/ObstacleMap.h"
#include "arborvia/core/Graph.h"
#include <set>

using namespace arborvia;

class CooperativeRerouterTest : public ::testing::Test {
protected:
    static constexpr float GRID_SIZE = 20.0f;

    void SetUp() override {
        // Horizontal layout:
        // [Node 0]         [Node 1]
        // pos(40,100)      pos(240,100)
        // size(80,40)      size(80,40)
        // grid X: 2-6      grid X: 12-16
        // grid Y: 5-7      grid Y: 5-7

        NodeLayout node0;
        node0.id = 0;
        node0.position = {40, 100};
        node0.size = {80, 40};
        nodeLayouts_[0] = node0;

        NodeLayout node1;
        node1.id = 1;
        node1.position = {240, 100};
        node1.size = {80, 40};
        nodeLayouts_[1] = node1;
    }

    // Helper: Get all grid cells on an edge's path (orthogonal segments only)
    std::set<std::pair<int, int>> getEdgePathCells(
        const EdgeLayout& edge, ObstacleMap& obstacles) {
        
        std::set<std::pair<int, int>> cells;
        
        std::vector<Point> points;
        points.push_back(edge.sourcePoint);
        for (const auto& bp : edge.bendPoints) {
            points.push_back(bp.position);
        }
        points.push_back(edge.targetPoint);

        for (size_t i = 0; i + 1 < points.size(); ++i) {
            GridPoint from = obstacles.pixelToGrid(points[i]);
            GridPoint to = obstacles.pixelToGrid(points[i + 1]);

            // Handle horizontal segment
            if (from.y == to.y) {
                int minX = std::min(from.x, to.x);
                int maxX = std::max(from.x, to.x);
                for (int x = minX; x <= maxX; ++x) {
                    cells.insert({x, from.y});
                }
            }
            // Handle vertical segment
            else if (from.x == to.x) {
                int minY = std::min(from.y, to.y);
                int maxY = std::max(from.y, to.y);
                for (int y = minY; y <= maxY; ++y) {
                    cells.insert({from.x, y});
                }
            }
            // Diagonal: just add endpoints (shouldn't happen in orthogonal paths)
            else {
                cells.insert({from.x, from.y});
                cells.insert({to.x, to.y});
            }
        }

        return cells;
    }

    // Helper: Check if two cell sets overlap
    bool cellSetsOverlap(
        const std::set<std::pair<int, int>>& a,
        const std::set<std::pair<int, int>>& b) {
        
        for (const auto& cell : a) {
            if (b.count(cell)) {
                return true;
            }
        }
        return false;
    }

    std::unordered_map<NodeId, NodeLayout> nodeLayouts_;
};

// =============================================================================
// Step 1: B의 예상 경로 계산 검증
// =============================================================================
TEST_F(CooperativeRerouterTest, Step1_ExpectedPathCalculation) {
    ObstacleMap obstacles;
    obstacles.buildFromNodes(nodeLayouts_, GRID_SIZE, 0);

    // B: sourcePoint(120, 120) → targetPoint(240, 120)
    Point sourcePoint = {120, 120};
    Point targetPoint = {240, 120};

    GridPoint startGrid = obstacles.pixelToGrid(sourcePoint);
    GridPoint endGrid = obstacles.pixelToGrid(targetPoint);

    // 검증: Grid 변환 정확성
    ASSERT_EQ(startGrid.x, 6) << "Source X must be grid 6";
    ASSERT_EQ(startGrid.y, 6) << "Source Y must be grid 6";
    ASSERT_EQ(endGrid.x, 12) << "Target X must be grid 12";
    ASSERT_EQ(endGrid.y, 6) << "Target Y must be grid 6";

    // 검증: 경로 길이 (horizontal line)
    int pathLength = endGrid.x - startGrid.x + 1;
    ASSERT_EQ(pathLength, 7) << "Expected path must have 7 grid points (6 to 12 inclusive)";
}

// =============================================================================
// Step 2: 장애물맵에 경로 예약 검증
// =============================================================================
TEST_F(CooperativeRerouterTest, Step2_PathReservedInObstacleMap) {
    ObstacleMap obstacles;
    obstacles.buildFromNodes(nodeLayouts_, GRID_SIZE, 0);

    // 노드 외부 영역 사용: Y=9 (노드는 Y=5-7)
    std::vector<GridPoint> testPath;
    for (int x = 7; x <= 11; ++x) {
        testPath.push_back({x, 9});
    }

    // 검증: 예약 전 비용
    for (int x = 7; x <= 11; ++x) {
        int cost = obstacles.getCost(x, 9);
        ASSERT_EQ(cost, 1) << "Cell (" << x << ",9) must have cost 1 before reservation";
    }

    // 예약 실행
    obstacles.markEdgePath(0, testPath);

    // 검증: 예약 후 비용 (COST_FREE=1 + COST_EDGE_PATH=50 = 51)
    for (int x = 7; x <= 11; ++x) {
        int cost = obstacles.getCost(x, 9);
        ASSERT_EQ(cost, 51) << "Cell (" << x << ",9) must have cost 51 after reservation";
    }

    // 검증: 경로 외부는 영향 없음
    ASSERT_EQ(obstacles.getCost(6, 9), 1) << "Cell outside path must remain cost 1";
    ASSERT_EQ(obstacles.getCost(12, 9), 1) << "Cell outside path must remain cost 1";
}

// =============================================================================
// Step 3: Blocking edge 감지 검증
// =============================================================================
TEST_F(CooperativeRerouterTest, Step3_BlockingEdgeDetection) {
    // Node2 추가 (A의 source)
    NodeLayout node2;
    node2.id = 2;
    node2.position = {160, 200};  // 아래에 위치
    node2.size = {80, 40};
    nodeLayouts_[2] = node2;

    ObstacleMap obstacles;
    obstacles.buildFromNodes(nodeLayouts_, GRID_SIZE, 0);

    // B: horizontal path at Y=120 (grid Y=6)
    // B's expected path cells: (6,6), (7,6), (8,6), (9,6), (10,6), (11,6), (12,6)
    EdgeLayout edgeB;
    edgeB.id = 0;
    edgeB.from = 0;
    edgeB.to = 1;
    edgeB.sourceEdge = NodeEdge::Right;
    edgeB.targetEdge = NodeEdge::Left;
    edgeB.sourcePoint = {120, 120};
    edgeB.targetPoint = {240, 120};

    std::set<std::pair<int, int>> bExpectedCells;
    for (int x = 6; x <= 12; ++x) {
        bExpectedCells.insert({x, 6});
    }

    // A: orthogonal path that crosses B's expected path
    // A goes: (9, 10) → (9, 6) → (4, 6) → (4, 7)
    // This crosses B's path at Y=6
    EdgeLayout edgeA;
    edgeA.id = 1;
    edgeA.from = 2;
    edgeA.to = 0;
    edgeA.sourceEdge = NodeEdge::Top;
    edgeA.targetEdge = NodeEdge::Bottom;
    edgeA.sourcePoint = {180, 200};  // grid (9, 10)
    edgeA.targetPoint = {80, 140};   // grid (4, 7)
    // Orthogonal bendpoints: down to Y=120, then left to X=80, then down to target
    edgeA.bendPoints = {
        {Point{180, 120}},  // grid (9, 6) - ON B's path!
        {Point{80, 120}}    // grid (4, 6)
    };

    // 검증: A의 현재 경로가 B의 예상 경로와 겹침
    std::set<std::pair<int, int>> aCells = getEdgePathCells(edgeA, obstacles);
    
    bool hasOverlap = cellSetsOverlap(aCells, bExpectedCells);
    ASSERT_TRUE(hasOverlap) << "A's path MUST overlap with B's expected path for this test";

    // 구체적으로 어떤 셀이 겹치는지 확인
    bool cell_9_6_in_A = aCells.count({9, 6}) > 0;
    bool cell_9_6_in_B = bExpectedCells.count({9, 6}) > 0;
    ASSERT_TRUE(cell_9_6_in_A) << "A's path must include cell (9,6)";
    ASSERT_TRUE(cell_9_6_in_B) << "B's expected path must include cell (9,6)";
}

// =============================================================================
// Step 4: A가 reroute 후 B의 예약 영역을 피하는지 검증
// =============================================================================
TEST_F(CooperativeRerouterTest, Step4_ReroutedEdgeAvoidsBPath) {
    // Node2 추가
    NodeLayout node2;
    node2.id = 2;
    node2.position = {160, 200};
    node2.size = {80, 40};
    nodeLayouts_[2] = node2;

    CooperativeRerouter rerouter(nullptr, GRID_SIZE);

    // B: horizontal at Y=120
    EdgeLayout edgeB;
    edgeB.id = 0;
    edgeB.from = 0;
    edgeB.to = 1;
    edgeB.sourceEdge = NodeEdge::Right;
    edgeB.targetEdge = NodeEdge::Left;
    edgeB.sourcePoint = {120, 120};
    edgeB.targetPoint = {240, 120};

    // B's expected path cells
    std::set<std::pair<int, int>> bExpectedCells;
    for (int x = 6; x <= 12; ++x) {
        bExpectedCells.insert({x, 6});
    }

    // A: orthogonal path that crosses B's path at Y=6
    EdgeLayout edgeA;
    edgeA.id = 1;
    edgeA.from = 2;
    edgeA.to = 0;
    edgeA.sourceEdge = NodeEdge::Top;
    edgeA.targetEdge = NodeEdge::Bottom;
    edgeA.sourcePoint = {180, 200};
    edgeA.targetPoint = {80, 140};
    edgeA.bendPoints = {
        {Point{180, 120}},  // grid (9, 6) - ON B's path!
        {Point{80, 120}}    // grid (4, 6)
    };

    std::unordered_map<EdgeId, EdgeLayout> otherLayouts;
    otherLayouts[1] = edgeA;

    // Cooperative rerouting 실행
    auto result = rerouter.rerouteWithCooperation(0, edgeB, otherLayouts, nodeLayouts_);

    // 검증 1: 성공해야 함
    ASSERT_TRUE(result.success) << "Rerouting must succeed. Failure: " << result.failureReason;

    // 검증 2: A가 reroute 되어야 함
    ASSERT_FALSE(result.reroutedEdges.empty()) << "A must be rerouted";
    ASSERT_EQ(result.reroutedEdges[0].id, 1) << "Rerouted edge must be A (id=1)";

    // 검증 3: otherLayouts가 실제로 수정됐는지 확인
    const EdgeLayout& modifiedA = otherLayouts[1];
    ASSERT_EQ(modifiedA.id, 1) << "otherLayouts[1] must still be edge A";

    // 검증 4: result.reroutedEdges와 otherLayouts가 일치하는지 확인
    ASSERT_EQ(result.reroutedEdges[0].bendPoints.size(), modifiedA.bendPoints.size())
        << "reroutedEdges and otherLayouts must have same bendPoints count";

    // 검증 5: A의 새 경로가 B의 예상 경로와 겹치지 않아야 함
    ObstacleMap obstacles;
    obstacles.buildFromNodes(nodeLayouts_, GRID_SIZE, 0);

    std::set<std::pair<int, int>> newACells = getEdgePathCells(modifiedA, obstacles);

    // B의 내부 경로 셀 (끝점 제외)
    std::set<std::pair<int, int>> bInternalCells;
    for (int x = 7; x <= 11; ++x) {
        bInternalCells.insert({x, 6});
    }

    bool newAOverlapsB = cellSetsOverlap(newACells, bInternalCells);
    ASSERT_FALSE(newAOverlapsB) << "A's new path must NOT overlap with B's internal path cells";

    // 검증 6: A의 경로가 실제로 변경됐는지 확인
    bool pathChanged = (modifiedA.bendPoints.size() != edgeA.bendPoints.size());
    if (!pathChanged && modifiedA.bendPoints.size() > 0) {
        for (size_t i = 0; i < modifiedA.bendPoints.size(); ++i) {
            if (std::abs(modifiedA.bendPoints[i].position.x - edgeA.bendPoints[i].position.x) > 1.0f ||
                std::abs(modifiedA.bendPoints[i].position.y - edgeA.bendPoints[i].position.y) > 1.0f) {
                pathChanged = true;
                break;
            }
        }
    }
    ASSERT_TRUE(pathChanged) << "A's path must have changed after rerouting";

    // 검증 7: A의 snap points 보존됐는지 확인
    ASSERT_FLOAT_EQ(modifiedA.sourcePoint.x, edgeA.sourcePoint.x) 
        << "A's sourcePoint.x must be preserved";
    ASSERT_FLOAT_EQ(modifiedA.sourcePoint.y, edgeA.sourcePoint.y)
        << "A's sourcePoint.y must be preserved";
    ASSERT_FLOAT_EQ(modifiedA.targetPoint.x, edgeA.targetPoint.x)
        << "A's targetPoint.x must be preserved";
    ASSERT_FLOAT_EQ(modifiedA.targetPoint.y, edgeA.targetPoint.y)
        << "A's targetPoint.y must be preserved";
    ASSERT_EQ(modifiedA.sourceEdge, edgeA.sourceEdge)
        << "A's sourceEdge must be preserved";
    ASSERT_EQ(modifiedA.targetEdge, edgeA.targetEdge)
        << "A's targetEdge must be preserved";

    // 검증 8: A의 새 경로가 orthogonal인지 확인
    std::vector<Point> aPathPoints;
    aPathPoints.push_back(modifiedA.sourcePoint);
    for (const auto& bp : modifiedA.bendPoints) {
        aPathPoints.push_back(bp.position);
    }
    aPathPoints.push_back(modifiedA.targetPoint);

    for (size_t i = 0; i + 1 < aPathPoints.size(); ++i) {
        float dx = std::abs(aPathPoints[i + 1].x - aPathPoints[i].x);
        float dy = std::abs(aPathPoints[i + 1].y - aPathPoints[i].y);
        bool isHorizontal = dy < 1.0f;
        bool isVertical = dx < 1.0f;
        ASSERT_TRUE(isHorizontal || isVertical)
            << "A's segment " << i << " is diagonal: ("
            << aPathPoints[i].x << "," << aPathPoints[i].y << ") -> ("
            << aPathPoints[i + 1].x << "," << aPathPoints[i + 1].y << ")";
    }
}

// =============================================================================
// Step 5: B의 최종 경로 검증
// =============================================================================
TEST_F(CooperativeRerouterTest, Step5_BFinalPathValid) {
    CooperativeRerouter rerouter(nullptr, GRID_SIZE);

    EdgeLayout edgeB;
    edgeB.id = 0;
    edgeB.from = 0;
    edgeB.to = 1;
    edgeB.sourceEdge = NodeEdge::Right;
    edgeB.targetEdge = NodeEdge::Left;
    edgeB.sourcePoint = {120, 120};
    edgeB.targetPoint = {240, 120};

    std::unordered_map<EdgeId, EdgeLayout> otherLayouts;

    auto result = rerouter.rerouteWithCooperation(0, edgeB, otherLayouts, nodeLayouts_);

    // 검증 1: 성공
    ASSERT_TRUE(result.success) << "B must succeed. Failure: " << result.failureReason;

    // 검증 2: Snap points 보존
    ASSERT_FLOAT_EQ(result.layout.sourcePoint.x, edgeB.sourcePoint.x);
    ASSERT_FLOAT_EQ(result.layout.sourcePoint.y, edgeB.sourcePoint.y);
    ASSERT_FLOAT_EQ(result.layout.targetPoint.x, edgeB.targetPoint.x);
    ASSERT_FLOAT_EQ(result.layout.targetPoint.y, edgeB.targetPoint.y);
    ASSERT_EQ(result.layout.sourceEdge, edgeB.sourceEdge);
    ASSERT_EQ(result.layout.targetEdge, edgeB.targetEdge);

    // 검증 3: 경로가 orthogonal
    std::vector<Point> allPoints;
    allPoints.push_back(result.layout.sourcePoint);
    for (const auto& bp : result.layout.bendPoints) {
        allPoints.push_back(bp.position);
    }
    allPoints.push_back(result.layout.targetPoint);

    ASSERT_GE(allPoints.size(), 2u) << "Path must have at least 2 points";

    for (size_t i = 0; i + 1 < allPoints.size(); ++i) {
        float dx = std::abs(allPoints[i + 1].x - allPoints[i].x);
        float dy = std::abs(allPoints[i + 1].y - allPoints[i].y);
        bool isHorizontal = dy < 1.0f;
        bool isVertical = dx < 1.0f;
        ASSERT_TRUE(isHorizontal || isVertical)
            << "Segment " << i << " is diagonal: ("
            << allPoints[i].x << "," << allPoints[i].y << ") -> ("
            << allPoints[i + 1].x << "," << allPoints[i + 1].y << ")";
    }

    // 검증 4: 경로가 source에서 target으로 연결됨
    ASSERT_FLOAT_EQ(allPoints.front().x, edgeB.sourcePoint.x);
    ASSERT_FLOAT_EQ(allPoints.front().y, edgeB.sourcePoint.y);
    ASSERT_FLOAT_EQ(allPoints.back().x, edgeB.targetPoint.x);
    ASSERT_FLOAT_EQ(allPoints.back().y, edgeB.targetPoint.y);
}

// =============================================================================
// Integration: A가 B를 막고 있을 때 전체 흐름
// =============================================================================
TEST_F(CooperativeRerouterTest, Integration_ABlockingB_FullFlow) {
    // Setup: A가 B의 경로를 막고 있음
    NodeLayout node2;
    node2.id = 2;
    node2.position = {160, 200};
    node2.size = {80, 40};
    nodeLayouts_[2] = node2;

    ObstacleMap obstacles;
    obstacles.buildFromNodes(nodeLayouts_, GRID_SIZE, 0);

    CooperativeRerouter rerouter(nullptr, GRID_SIZE);

    // B: horizontal
    EdgeLayout edgeB;
    edgeB.id = 0;
    edgeB.from = 0;
    edgeB.to = 1;
    edgeB.sourceEdge = NodeEdge::Right;
    edgeB.targetEdge = NodeEdge::Left;
    edgeB.sourcePoint = {120, 120};
    edgeB.targetPoint = {240, 120};

    // A: orthogonal path that crosses B's path at Y=6
    EdgeLayout edgeA_original;
    edgeA_original.id = 1;
    edgeA_original.from = 2;
    edgeA_original.to = 0;
    edgeA_original.sourceEdge = NodeEdge::Top;
    edgeA_original.targetEdge = NodeEdge::Bottom;
    edgeA_original.sourcePoint = {180, 200};
    edgeA_original.targetPoint = {80, 140};
    edgeA_original.bendPoints = {
        {Point{180, 120}},  // grid (9, 6) - ON B's path!
        {Point{80, 120}}    // grid (4, 6)
    };

    // B의 예상 경로 셀
    std::set<std::pair<int, int>> bExpectedCells;
    for (int x = 7; x <= 11; ++x) {
        bExpectedCells.insert({x, 6});
    }

    // 검증 1: Before - A가 B의 경로와 겹침
    std::set<std::pair<int, int>> aOriginalCells = getEdgePathCells(edgeA_original, obstacles);
    ASSERT_TRUE(cellSetsOverlap(aOriginalCells, bExpectedCells))
        << "PRECONDITION: A must overlap with B's path before rerouting";

    std::unordered_map<EdgeId, EdgeLayout> otherLayouts;
    otherLayouts[1] = edgeA_original;

    // Execute
    auto result = rerouter.rerouteWithCooperation(0, edgeB, otherLayouts, nodeLayouts_);

    // 검증 2: 성공
    ASSERT_TRUE(result.success) << "Must succeed. Failure: " << result.failureReason;

    // 검증 3: A가 reroute 됨
    ASSERT_EQ(result.reroutedEdges.size(), 1u) << "Exactly one edge (A) must be rerouted";
    ASSERT_EQ(result.reroutedEdges[0].id, 1) << "Rerouted edge must be A";

    // 검증 4: A의 새 경로가 B의 경로와 겹치지 않음
    const EdgeLayout& aNew = otherLayouts[1];  // otherLayouts가 수정됨
    std::set<std::pair<int, int>> aNewCells = getEdgePathCells(aNew, obstacles);
    ASSERT_FALSE(cellSetsOverlap(aNewCells, bExpectedCells))
        << "POSTCONDITION: A's new path must NOT overlap with B's path";

    // 검증 5: A의 경로가 실제로 변경됨
    bool pathChanged = (aNew.bendPoints.size() != edgeA_original.bendPoints.size());
    if (!pathChanged && !aNew.bendPoints.empty() && !edgeA_original.bendPoints.empty()) {
        pathChanged = (std::abs(aNew.bendPoints[0].position.x - edgeA_original.bendPoints[0].position.x) > 1.0f ||
                      std::abs(aNew.bendPoints[0].position.y - edgeA_original.bendPoints[0].position.y) > 1.0f);
    }
    ASSERT_TRUE(pathChanged) << "A's path must have changed";

    // 검증 6: B의 snap points 보존
    ASSERT_FLOAT_EQ(result.layout.sourcePoint.x, edgeB.sourcePoint.x);
    ASSERT_FLOAT_EQ(result.layout.sourcePoint.y, edgeB.sourcePoint.y);
    ASSERT_FLOAT_EQ(result.layout.targetPoint.x, edgeB.targetPoint.x);
    ASSERT_FLOAT_EQ(result.layout.targetPoint.y, edgeB.targetPoint.y);

    // 검증 7: B의 경로가 orthogonal인지 확인
    std::vector<Point> bPathPoints;
    bPathPoints.push_back(result.layout.sourcePoint);
    for (const auto& bp : result.layout.bendPoints) {
        bPathPoints.push_back(bp.position);
    }
    bPathPoints.push_back(result.layout.targetPoint);

    for (size_t i = 0; i + 1 < bPathPoints.size(); ++i) {
        float dx = std::abs(bPathPoints[i + 1].x - bPathPoints[i].x);
        float dy = std::abs(bPathPoints[i + 1].y - bPathPoints[i].y);
        bool isHorizontal = dy < 1.0f;
        bool isVertical = dx < 1.0f;
        ASSERT_TRUE(isHorizontal || isVertical)
            << "B's segment " << i << " is diagonal: ("
            << bPathPoints[i].x << "," << bPathPoints[i].y << ") -> ("
            << bPathPoints[i + 1].x << "," << bPathPoints[i + 1].y << ")";
    }

    // 검증 8: A의 snap points 보존
    const EdgeLayout& aFinal = otherLayouts[1];
    ASSERT_FLOAT_EQ(aFinal.sourcePoint.x, edgeA_original.sourcePoint.x);
    ASSERT_FLOAT_EQ(aFinal.sourcePoint.y, edgeA_original.sourcePoint.y);
    ASSERT_FLOAT_EQ(aFinal.targetPoint.x, edgeA_original.targetPoint.x);
    ASSERT_FLOAT_EQ(aFinal.targetPoint.y, edgeA_original.targetPoint.y);

    // 검증 9: A의 경로가 orthogonal인지 확인
    std::vector<Point> aPathPoints;
    aPathPoints.push_back(aFinal.sourcePoint);
    for (const auto& bp : aFinal.bendPoints) {
        aPathPoints.push_back(bp.position);
    }
    aPathPoints.push_back(aFinal.targetPoint);

    for (size_t i = 0; i + 1 < aPathPoints.size(); ++i) {
        float dx = std::abs(aPathPoints[i + 1].x - aPathPoints[i].x);
        float dy = std::abs(aPathPoints[i + 1].y - aPathPoints[i].y);
        bool isHorizontal = dy < 1.0f;
        bool isVertical = dx < 1.0f;
        ASSERT_TRUE(isHorizontal || isVertical)
            << "A's segment " << i << " is diagonal: ("
            << aPathPoints[i].x << "," << aPathPoints[i].y << ") -> ("
            << aPathPoints[i + 1].x << "," << aPathPoints[i + 1].y << ")";
    }
}

// =============================================================================
// Edge case: 겹침 없으면 reroute 안 함
// =============================================================================
TEST_F(CooperativeRerouterTest, EdgeCase_NoOverlap_NoReroute) {
    CooperativeRerouter rerouter(nullptr, GRID_SIZE);

    // B: horizontal at Y=120
    EdgeLayout edgeB;
    edgeB.id = 0;
    edgeB.from = 0;
    edgeB.to = 1;
    edgeB.sourceEdge = NodeEdge::Right;
    edgeB.targetEdge = NodeEdge::Left;
    edgeB.sourcePoint = {120, 120};
    edgeB.targetPoint = {240, 120};

    // No other edges
    std::unordered_map<EdgeId, EdgeLayout> otherLayouts;

    auto result = rerouter.rerouteWithCooperation(0, edgeB, otherLayouts, nodeLayouts_);

    ASSERT_TRUE(result.success) << "Must succeed when no blocking edges";
    ASSERT_TRUE(result.reroutedEdges.empty()) << "No edges should be rerouted when no overlap";
}

// =============================================================================
// Edge case: Node not found
// =============================================================================
TEST_F(CooperativeRerouterTest, EdgeCase_NodeNotFound) {
    CooperativeRerouter rerouter(nullptr, GRID_SIZE);

    EdgeLayout badEdge;
    badEdge.id = 99;
    badEdge.from = 999;  // 존재하지 않음
    badEdge.to = 1;
    badEdge.sourcePoint = {0, 0};
    badEdge.targetPoint = {100, 100};

    std::unordered_map<EdgeId, EdgeLayout> otherLayouts;
    auto result = rerouter.rerouteWithCooperation(99, badEdge, otherLayouts, nodeLayouts_);

    ASSERT_FALSE(result.success);
    ASSERT_TRUE(result.failureReason.find("not found") != std::string::npos);
}

// =============================================================================
// hasSegmentOverlap 검증
// =============================================================================
TEST_F(CooperativeRerouterTest, HasSegmentOverlap_ParallelOverlap) {
    EdgeLayout edgeA;
    edgeA.id = 0;
    edgeA.sourcePoint = {100, 0};
    edgeA.targetPoint = {100, 200};

    EdgeLayout edgeB;
    edgeB.id = 1;
    edgeB.sourcePoint = {100, 100};
    edgeB.targetPoint = {100, 300};

    ASSERT_TRUE(CooperativeRerouter::hasSegmentOverlap(edgeA, edgeB))
        << "Parallel overlapping segments must be detected";
}

TEST_F(CooperativeRerouterTest, HasSegmentOverlap_CrossNoOverlap) {
    EdgeLayout edgeA;
    edgeA.id = 0;
    edgeA.sourcePoint = {0, 100};
    edgeA.targetPoint = {200, 100};

    EdgeLayout edgeB;
    edgeB.id = 1;
    edgeB.sourcePoint = {100, 0};
    edgeB.targetPoint = {100, 200};

    ASSERT_FALSE(CooperativeRerouter::hasSegmentOverlap(edgeA, edgeB))
        << "Cross intersection (X shape) must NOT be detected as overlap";
}
