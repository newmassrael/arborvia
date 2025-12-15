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
