#include <gtest/gtest.h>
#include <arborvia/arborvia.h>
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
