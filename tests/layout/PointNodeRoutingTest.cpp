#include <gtest/gtest.h>
#include <arborvia/arborvia.h>
#include <arborvia/layout/api/LayoutController.h>
#include "../infrastructure/scxml/SCXMLTestLoader.h"

#include <filesystem>

using namespace arborvia;
using namespace arborvia::test::scxml;

class PointNodeRoutingTest : public ::testing::Test {
protected:
    Graph graph_;
    std::filesystem::path resourcePath_;

    void SetUp() override {
        std::vector<std::string> searchPaths = {
            "resources/scxml",
            "../resources/scxml",
            "../../resources/scxml",
            "/home/coin/arborvia/resources/scxml"
        };

        for (const auto& path : searchPaths) {
            if (std::filesystem::exists(path)) {
                resourcePath_ = path;
                break;
            }
        }
    }

    // Helper: Point 노드 중심과 일치하는지 확인
    bool isPointAtNodeCenter(const Point& point, const NodeLayout& node, float tolerance = 0.5f) {
        Point center = node.center();
        return std::abs(point.x - center.x) < tolerance &&
               std::abs(point.y - center.y) < tolerance;
    }

    // Helper: 세그먼트가 직교(수평 또는 수직)인지 확인
    bool isOrthogonalSegment(const Point& p1, const Point& p2, float tolerance = 0.5f) {
        bool isHorizontal = std::abs(p1.y - p2.y) < tolerance;
        bool isVertical = std::abs(p1.x - p2.x) < tolerance;
        return isHorizontal || isVertical;
    }

    // Helper: 엣지의 모든 세그먼트가 직교인지 확인
    bool hasOrthogonalPath(const EdgeLayout& edge) {
        std::vector<Point> path;
        path.push_back(edge.sourcePoint);
        for (const auto& bend : edge.bendPoints) {
            path.push_back(bend.position);  // BendPoint.position
        }
        path.push_back(edge.targetPoint);

        for (size_t i = 0; i + 1 < path.size(); ++i) {
            if (!isOrthogonalSegment(path[i], path[i + 1])) {
                return false;
            }
        }
        return true;
    }

    // Helper: 스파이크 패턴 감지 (방향 반전)
    // 스파이크: 첫 번째 이동 방향과 반대 방향으로 돌아온 후 다시 원래 방향으로 이동
    // 예: (0,0) → (0,20) → (0,-20) - Y축에서 아래로 갔다가 위로 올라옴
    bool hasSpikePattern(const EdgeLayout& edge, float tolerance = 0.5f) {
        std::vector<Point> path;
        path.push_back(edge.sourcePoint);
        for (const auto& bend : edge.bendPoints) {
            path.push_back(bend.position);
        }
        path.push_back(edge.targetPoint);

        if (path.size() < 3) return false;

        // 각 축별로 방향 반전 감지
        for (size_t i = 0; i + 2 < path.size(); ++i) {
            // Y축 스파이크 감지
            float dy1 = path[i + 1].y - path[i].y;
            float dy2 = path[i + 2].y - path[i + 1].y;

            // dy1과 dy2가 반대 방향이고, 둘 다 유의미한 이동인 경우
            if (std::abs(dy1) > tolerance && std::abs(dy2) > tolerance) {
                if ((dy1 > 0 && dy2 < 0) || (dy1 < 0 && dy2 > 0)) {
                    return true;  // Y축 스파이크
                }
            }

            // X축 스파이크 감지
            float dx1 = path[i + 1].x - path[i].x;
            float dx2 = path[i + 2].x - path[i + 1].x;

            if (std::abs(dx1) > tolerance && std::abs(dx2) > tolerance) {
                if ((dx1 > 0 && dx2 < 0) || (dx1 < 0 && dx2 > 0)) {
                    return true;  // X축 스파이크
                }
            }
        }
        return false;
    }

    // Helper: 스파이크 패턴 정보 반환 (디버깅용)
    std::string getSpikePatternInfo(const EdgeLayout& edge) {
        std::vector<Point> path;
        path.push_back(edge.sourcePoint);
        for (const auto& bend : edge.bendPoints) {
            path.push_back(bend.position);
        }
        path.push_back(edge.targetPoint);

        std::string result = "Path: ";
        for (size_t i = 0; i < path.size(); ++i) {
            if (i > 0) result += " -> ";
            result += "(" + std::to_string(static_cast<int>(path[i].x)) + "," +
                     std::to_string(static_cast<int>(path[i].y)) + ")";
        }
        return result;
    }

    // Helper: 대각선 세그먼트 정보 반환 (디버깅용)
    std::string getDiagonalSegmentInfo(const EdgeLayout& edge) {
        std::vector<Point> path;
        path.push_back(edge.sourcePoint);
        for (const auto& bend : edge.bendPoints) {
            path.push_back(bend.position);  // BendPoint.position
        }
        path.push_back(edge.targetPoint);

        std::string result;
        for (size_t i = 0; i + 1 < path.size(); ++i) {
            if (!isOrthogonalSegment(path[i], path[i + 1])) {
                result += "Segment " + std::to_string(i) + ": (" +
                          std::to_string(path[i].x) + "," + std::to_string(path[i].y) + ")->(" +
                          std::to_string(path[i + 1].x) + "," + std::to_string(path[i + 1].y) + ") ";
            }
        }
        return result.empty() ? "No diagonal segments" : result;
    }
};

// RED: test144.scxml을 직접 로드하여 데모와 동일한 조건 재현
// Bug: sourceEdge=right이고 target이 Point 노드일 때,
// targetPoint.x = sourcePoint.x가 되어 노드 중심에서 벗어남
TEST_F(PointNodeRoutingTest, Test144_SideEdgeToPointNode_TargetPointAtCenter) {
    if (resourcePath_.empty()) {
        GTEST_SKIP() << "SCXML resources directory not found";
    }

    SCXMLTestLoader loader(resourcePath_);
    ASSERT_TRUE(loader.loadIndex()) << loader.getLastError();

    auto graph = loader.loadGraphById("144");
    ASSERT_NE(graph, nullptr) << loader.getLastError();

    // fail 노드 확인
    auto failNodeOpt = graph->findByScxmlId("fail");
    ASSERT_TRUE(failNodeOpt.has_value()) << "fail node not found in test144";
    NodeId failId = *failNodeOpt;

    // s1 노드 확인
    auto s1NodeOpt = graph->findByScxmlId("s1");
    ASSERT_TRUE(s1NodeOpt.has_value()) << "s1 node not found in test144";
    NodeId s1Id = *s1NodeOpt;

    // s1 -> fail 엣지 찾기
    auto targetEdgeOpt = graph->findEdge(s1Id, failId);
    ASSERT_TRUE(targetEdgeOpt.has_value()) << "s1->fail edge not found";
    EdgeId targetEdge = *targetEdgeOpt;

    // 레이아웃 실행 (기본 gridSize=20)
    SugiyamaLayout layout;
    LayoutResult result = layout.layout(*graph);

    const NodeLayout* failLayout = result.getNodeLayout(failId);
    const EdgeLayout* edgeLayout = result.getEdgeLayout(targetEdge);

    ASSERT_NE(failLayout, nullptr);
    ASSERT_NE(edgeLayout, nullptr);
    ASSERT_LT(failLayout->size.width, 1.0f) << "fail should be a Point node";

    Point failCenter = failLayout->center();

    // 디버그 출력
    std::cout << "[TEST144] s1Id=" << s1Id << " failId=" << failId << " targetEdge(edgeId)=" << targetEdge << std::endl;
    std::cout << "[TEST144] fail (Point): pos=(" << failLayout->position.x << ", " << failLayout->position.y
              << ") center=(" << failCenter.x << ", " << failCenter.y << ")" << std::endl;
    std::cout << "[TEST144] Edge s1->fail sourcePoint=(" << edgeLayout->sourcePoint.x << ", " << edgeLayout->sourcePoint.y
              << ") targetPoint=(" << edgeLayout->targetPoint.x << ", " << edgeLayout->targetPoint.y << ")" << std::endl;
    std::cout << "[TEST144] sourceEdge=" << static_cast<int>(edgeLayout->sourceEdge)
              << " targetEdge=" << static_cast<int>(edgeLayout->targetEdge) << std::endl;

    // 모든 fail 노드로 가는 엣지 출력
    std::cout << "[TEST144] All edges to fail node:" << std::endl;
    for (const auto& [eid, el] : result.edgeLayouts()) {
        if (el.to == failId) {
            std::cout << "  Edge " << eid << ": from=" << el.from << " srcEdge=" << static_cast<int>(el.sourceEdge)
                      << " tgtEdge=" << static_cast<int>(el.targetEdge)
                      << " srcPt=(" << el.sourcePoint.x << "," << el.sourcePoint.y << ")"
                      << " tgtPt=(" << el.targetPoint.x << "," << el.targetPoint.y << ")" << std::endl;
        }
    }

    // 핵심 검증: Point 노드의 targetPoint는 항상 노드 중심이어야 함
    // Bug: sourceEdge=right일 때 targetPoint.x가 sourcePoint.x가 됨 (예: 120 vs 100)
    EXPECT_TRUE(isPointAtNodeCenter(edgeLayout->targetPoint, *failLayout, 0.5f))
        << "Edge s1->fail targetPoint (" << edgeLayout->targetPoint.x << ", " << edgeLayout->targetPoint.y
        << ") should be at Point node center (" << failCenter.x << ", " << failCenter.y << ")"
        << " [Bug: targetPoint.x may incorrectly match sourcePoint.x when sourceEdge is right]";
}

// RED: Point 노드로 수렴하는 여러 엣지의 직교성 검증
// Bug: sourceEdge=Right에서 출발하여 왼쪽에 있는 Point 노드로 갈 때,
// 다른 엣지의 vertical segment가 blocking하여 A* 실패 → 대각선 fallback 생성
// Root cause: Edge segment blocking이 같은 Point 타겟을 공유하는 엣지들 사이에서
// 마지막 세그먼트를 제외하지 않아 서로를 차단함
TEST_F(PointNodeRoutingTest, Test144_AllEdgesToPointNode_MustBeOrthogonal) {
    if (resourcePath_.empty()) {
        GTEST_SKIP() << "SCXML resources directory not found";
    }

    SCXMLTestLoader loader(resourcePath_);
    ASSERT_TRUE(loader.loadIndex()) << loader.getLastError();

    auto graph = loader.loadGraphById("144");
    ASSERT_NE(graph, nullptr) << loader.getLastError();

    // fail 노드 (Point 노드) 확인
    auto failNodeOpt = graph->findByScxmlId("fail");
    ASSERT_TRUE(failNodeOpt.has_value()) << "fail node not found in test144";
    NodeId failId = *failNodeOpt;

    // 레이아웃 실행
    SugiyamaLayout layout;
    LayoutResult result = layout.layout(*graph);

    const NodeLayout* failLayout = result.getNodeLayout(failId);
    ASSERT_NE(failLayout, nullptr);
    ASSERT_LT(failLayout->size.width, 1.0f) << "fail should be a Point node";

    // Point 노드로 향하는 모든 엣지 수집
    std::vector<std::pair<EdgeId, const EdgeLayout*>> edgesToPointNode;
    for (const auto& [eid, el] : result.edgeLayouts()) {
        if (el.to == failId) {
            edgesToPointNode.emplace_back(eid, &el);
        }
    }

    ASSERT_GE(edgesToPointNode.size(), 2u) 
        << "Test144 should have at least 2 edges to Point node 'fail'";

    // 각 엣지의 직교성 검증
    int diagonalCount = 0;
    for (const auto& [eid, edgePtr] : edgesToPointNode) {
        const EdgeLayout& edge = *edgePtr;
        
        std::cout << "[TEST144-ORTHOGONAL] Edge " << eid 
                  << ": from=" << edge.from 
                  << " srcEdge=" << static_cast<int>(edge.sourceEdge)
                  << " srcPt=(" << edge.sourcePoint.x << "," << edge.sourcePoint.y << ")"
                  << " tgtPt=(" << edge.targetPoint.x << "," << edge.targetPoint.y << ")"
                  << " bends=" << edge.bendPoints.size() << std::endl;

        if (!hasOrthogonalPath(edge)) {
            diagonalCount++;
            std::cout << "[TEST144-ORTHOGONAL] DIAGONAL DETECTED: Edge " << eid 
                      << " - " << getDiagonalSegmentInfo(edge) << std::endl;
        }
    }

    // 핵심 검증: 모든 엣지가 직교 경로를 가져야 함
    EXPECT_EQ(diagonalCount, 0)
        << "All edges to Point node must have orthogonal paths, but found " 
        << diagonalCount << " edge(s) with diagonal segments. "
        << "Root cause: Edge segment blocking conflict when multiple edges converge on Point node. "
        << "Specifically, sourceEdge=Right edges fail A* when target is to the left, "
        << "because other edges' vertical segments block the approach path.";
}

// RED: sourceEdge=Right + target이 왼쪽에 있는 특정 케이스 검증
// 이 조합에서 A* 경로 탐색이 실패하고 대각선 fallback이 생성됨
TEST_F(PointNodeRoutingTest, Test144_RightEdgeToLeftTarget_MustBeOrthogonal) {
    if (resourcePath_.empty()) {
        GTEST_SKIP() << "SCXML resources directory not found";
    }

    SCXMLTestLoader loader(resourcePath_);
    ASSERT_TRUE(loader.loadIndex()) << loader.getLastError();

    auto graph = loader.loadGraphById("144");
    ASSERT_NE(graph, nullptr) << loader.getLastError();

    auto failNodeOpt = graph->findByScxmlId("fail");
    ASSERT_TRUE(failNodeOpt.has_value());
    NodeId failId = *failNodeOpt;

    auto s1NodeOpt = graph->findByScxmlId("s1");
    ASSERT_TRUE(s1NodeOpt.has_value());
    NodeId s1Id = *s1NodeOpt;

    SugiyamaLayout layout;
    LayoutResult result = layout.layout(*graph);

    // s1 -> fail 엣지 찾기
    const EdgeLayout* targetEdge = nullptr;
    EdgeId targetEdgeId = 0;
    for (const auto& [eid, el] : result.edgeLayouts()) {
        if (el.from == s1Id && el.to == failId) {
            targetEdge = &el;
            targetEdgeId = eid;
            break;
        }
    }
    ASSERT_NE(targetEdge, nullptr) << "s1->fail edge not found";

    std::cout << "[TEST144-RIGHT-EDGE] Edge " << targetEdgeId
              << " sourceEdge=" << static_cast<int>(targetEdge->sourceEdge)
              << " srcPt=(" << targetEdge->sourcePoint.x << "," << targetEdge->sourcePoint.y << ")"
              << " tgtPt=(" << targetEdge->targetPoint.x << "," << targetEdge->targetPoint.y << ")"
              << std::endl;

    // sourceEdge=Right(3)이고 sourcePoint.x > targetPoint.x인 경우가 문제의 핵심
    bool isRightEdge = (targetEdge->sourceEdge == NodeEdge::Right);
    bool targetIsLeft = (targetEdge->sourcePoint.x > targetEdge->targetPoint.x);

    if (isRightEdge && targetIsLeft) {
        std::cout << "[TEST144-RIGHT-EDGE] This is the problematic case: "
                  << "sourceEdge=Right, target is to the left" << std::endl;
    }

    // 직교성 검증
    bool isOrthogonal = hasOrthogonalPath(*targetEdge);
    
    EXPECT_TRUE(isOrthogonal)
        << "Edge s1->fail must have orthogonal path. "
        << "Diagonal info: " << getDiagonalSegmentInfo(*targetEdge) << ". "
        << "This edge has sourceEdge=" << static_cast<int>(targetEdge->sourceEdge) 
        << " (Right=3) and target is " << (targetIsLeft ? "to the left" : "to the right")
        << ". When sourceEdge=Right and target is left, A* fails due to edge segment blocking "
        << "from other edges sharing the same Point target.";
}

// RED: Point 노드가 source인 경우도 테스트
TEST_F(PointNodeRoutingTest, EdgeFromPointNode_SourcePointAtCenter) {
    NodeId initial = graph_.addNode(Size{0, 0}, "initial");  // Point node
    NodeId s0 = graph_.addNode(Size{120, 60}, "s0");

    EdgeId edge = graph_.addEdge(initial, s0, "");

    SugiyamaLayout layout;
    LayoutResult result = layout.layout(graph_);

    const NodeLayout* initialLayout = result.getNodeLayout(initial);
    const EdgeLayout* edgeLayout = result.getEdgeLayout(edge);

    ASSERT_NE(initialLayout, nullptr);
    ASSERT_NE(edgeLayout, nullptr);
    ASSERT_LT(initialLayout->size.width, 1.0f) << "initial should be a Point node";

    EXPECT_TRUE(isPointAtNodeCenter(edgeLayout->sourcePoint, *initialLayout, 0.5f))
        << "Edge sourcePoint ("
        << edgeLayout->sourcePoint.x << ", " << edgeLayout->sourcePoint.y
        << ") should be at Point node center ("
        << initialLayout->center().x << ", " << initialLayout->center().y << ")";
}

// RED: Point 노드(initial)에서 위쪽 노드(s0)로 가는 엣지의 스파이크 패턴 감지
// Bug: sourceEdge=Bottom이 유지되어 A*가 아래쪽으로 먼저 이동 후 위로 되돌아옴
// 예: (0,0) → (0,20) → (0,-20) → (200,-20) - 스파이크 패턴
// Root cause: sourceEdge가 상대적 위치에 맞게 업데이트되지 않음
TEST_F(PointNodeRoutingTest, EdgeFromPointNode_NoSpikePattern_WhenTargetAbove) {
    // 시나리오: Point 노드(initial)가 아래쪽, 일반 노드(s0)가 위쪽
    // initial: (0, 0) - Point 노드
    // s0: (200, -60) - 일반 노드 (initial보다 위에 위치)
    NodeId initial = graph_.addNode(Size{0, 0}, "initial");  // Point node
    NodeId s0 = graph_.addNode(Size{120, 60}, "s0");

    EdgeId edge = graph_.addEdge(initial, s0, "");

    SugiyamaLayout layout;
    LayoutResult result = layout.layout(graph_);

    const NodeLayout* initialLayout = result.getNodeLayout(initial);
    const NodeLayout* s0Layout = result.getNodeLayout(s0);
    const EdgeLayout* edgeLayout = result.getEdgeLayout(edge);

    ASSERT_NE(initialLayout, nullptr);
    ASSERT_NE(s0Layout, nullptr);
    ASSERT_NE(edgeLayout, nullptr);
    ASSERT_LT(initialLayout->size.width, 1.0f) << "initial should be a Point node";

    // 디버그 출력
    std::cout << "[SPIKE-TEST] initial (Point): pos=(" << initialLayout->position.x
              << "," << initialLayout->position.y << ")" << std::endl;
    std::cout << "[SPIKE-TEST] s0: pos=(" << s0Layout->position.x
              << "," << s0Layout->position.y << ")" << std::endl;
    std::cout << "[SPIKE-TEST] Edge: sourceEdge=" << static_cast<int>(edgeLayout->sourceEdge)
              << " targetEdge=" << static_cast<int>(edgeLayout->targetEdge) << std::endl;
    std::cout << "[SPIKE-TEST] " << getSpikePatternInfo(*edgeLayout) << std::endl;

    // 핵심 검증: 스파이크 패턴이 없어야 함
    // Bug: sourceEdge=Bottom(1)이면 아래로 먼저 가서 스파이크 생성
    EXPECT_FALSE(hasSpikePattern(*edgeLayout))
        << "Edge initial->s0 should NOT have spike pattern. "
        << getSpikePatternInfo(*edgeLayout) << ". "
        << "Bug: sourceEdge=" << static_cast<int>(edgeLayout->sourceEdge)
        << " may force initial movement opposite to target direction, "
        << "creating inefficient detour.";

    // 추가 검증: 타겟이 위쪽이면 sourceEdge는 Top이어야 함
    bool targetIsAbove = (s0Layout->center().y < initialLayout->center().y);
    if (targetIsAbove) {
        EXPECT_EQ(edgeLayout->sourceEdge, NodeEdge::Top)
            << "When target is above source, sourceEdge should be Top(0), not "
            << static_cast<int>(edgeLayout->sourceEdge)
            << ". Current positions: initial.y=" << initialLayout->center().y
            << ", s0.y=" << s0Layout->center().y;
    }
}

// RED: 드래그 후 스파이크 패턴 검증
// 시나리오: Point 노드에서 출발한 엣지가, 드래그 후 타겟이 위로 이동했을 때
// sourceEdge=Bottom이 유지되면 스파이크 패턴 발생
// Bug: sourceEdge가 상대 위치에 맞게 업데이트되지 않음
TEST_F(PointNodeRoutingTest, AfterDrag_NoSpikePattern_WhenTargetMovesAbove) {
    // 시나리오: Point 노드(initial)에서 일반 노드(s0)로 연결
    // 초기: initial 위에, s0 아래
    // 드래그 후: s0가 initial 위로 이동
    NodeId initial = graph_.addNode(Size{0, 0}, "initial");  // Point node
    NodeId s0 = graph_.addNode(Size{120, 60}, "s0");

    EdgeId edge = graph_.addEdge(initial, s0, "");

    // 1. 초기 레이아웃
    SugiyamaLayout layout;
    LayoutResult result = layout.layout(graph_);

    // 2. LayoutController를 사용하여 노드 드래그
    LayoutController controller(graph_);
    controller.initializeFrom(result);

    // s0를 initial 위로 이동 (200, -60) - initial(0,0)보다 y가 작음 = 위쪽
    auto moveResult = controller.moveNode(s0, {200, -60});
    ASSERT_TRUE(moveResult.success) << "Node move should succeed";

    // 3. 레이아웃 결과 확인 (LayoutController에서 읽기)
    const NodeLayout* initialLayout = controller.getNode(initial);
    const NodeLayout* s0Layout = controller.getNode(s0);
    const EdgeLayout* edgeLayout = controller.getEdge(edge);

    ASSERT_NE(initialLayout, nullptr);
    ASSERT_NE(s0Layout, nullptr);
    ASSERT_NE(edgeLayout, nullptr);

    // 디버그 출력
    std::cout << "[DRAG-SPIKE] initial (Point): pos=(" << initialLayout->position.x
              << "," << initialLayout->position.y << ")" << std::endl;
    std::cout << "[DRAG-SPIKE] s0 (after drag): pos=(" << s0Layout->position.x
              << "," << s0Layout->position.y << ")" << std::endl;
    std::cout << "[DRAG-SPIKE] Edge: sourceEdge=" << static_cast<int>(edgeLayout->sourceEdge)
              << " targetEdge=" << static_cast<int>(edgeLayout->targetEdge) << std::endl;
    std::cout << "[DRAG-SPIKE] " << getSpikePatternInfo(*edgeLayout) << std::endl;

    // 핵심 검증: 타겟이 위로 이동했으면 스파이크가 없어야 함
    bool targetIsAbove = (s0Layout->center().y < initialLayout->center().y);
    std::cout << "[DRAG-SPIKE] targetIsAbove=" << (targetIsAbove ? "true" : "false")
              << " (initial.y=" << initialLayout->center().y
              << ", s0.y=" << s0Layout->center().y << ")" << std::endl;

    // 스파이크 패턴 검사
    EXPECT_FALSE(hasSpikePattern(*edgeLayout))
        << "After drag, edge should NOT have spike pattern when target moved above. "
        << getSpikePatternInfo(*edgeLayout) << ". "
        << "Bug: sourceEdge=" << static_cast<int>(edgeLayout->sourceEdge)
        << " (0=Top, 1=Bottom) may not match new target direction after drag.";

    // 추가 검증: 타겟이 위쪽이면 sourceEdge는 Top이어야 함 (Point 노드는 중심에서 출발)
    if (targetIsAbove) {
        // Point 노드는 어느 방향이든 중심에서 출발하지만,
        // A*의 첫 이동 방향을 결정하는 sourceEdge는 타겟 방향과 일치해야 함
        EXPECT_EQ(edgeLayout->sourceEdge, NodeEdge::Top)
            << "When target is above source, sourceEdge should be Top(0) to avoid spike, not "
            << static_cast<int>(edgeLayout->sourceEdge)
            << " (Bottom=1 would cause downward first move creating spike)";
    }
}
