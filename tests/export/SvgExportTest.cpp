#include <gtest/gtest.h>
#include <arborvia/arborvia.h>
#include <arborvia/export/SvgExport.h>

using namespace arborvia;

// ============================================================================
// SvgExportTest - SVG 내보내기 기능 테스트
// ============================================================================

TEST(SvgExportTest, SimpleGraph_ProducesValidSvg) {
    Graph graph;
    NodeId n1 = graph.addNode("Source");
    NodeId n2 = graph.addNode("Target");
    graph.addEdge(n1, n2, "edge");

    SugiyamaLayout layout;
    LayoutResult result = layout.layout(graph);

    SvgExport svg;
    std::string output = svg.exportToString(graph, result);

    EXPECT_FALSE(output.empty());
    EXPECT_NE(output.find("<svg"), std::string::npos);
    EXPECT_NE(output.find("</svg>"), std::string::npos);
    EXPECT_NE(output.find("<rect"), std::string::npos);

    // Verify all nodes are rendered (should have rect for each node + background)
    size_t rectCount = 0;
    size_t pos = 0;
    while ((pos = output.find("<rect", pos)) != std::string::npos) {
        ++rectCount;
        ++pos;
    }
    EXPECT_EQ(rectCount, 3);  // 1 background + 2 nodes

    // Verify edge is rendered (should have path or line element)
    EXPECT_TRUE(output.find("<path") != std::string::npos ||
                output.find("<line") != std::string::npos ||
                output.find("<polyline") != std::string::npos);
}

TEST(SvgExportTest, CompoundGraph_IncludesCompoundMarker) {
    CompoundGraph graph;
    NodeId parent = graph.addCompoundNode("Parent", CompoundType::Compound);
    NodeId child = graph.addNode("Child");
    graph.setParent(child, parent);
    
    SugiyamaLayout layout;
    LayoutResult result = layout.layout(graph);
    
    SvgExport svg;
    std::string output = svg.exportToString(graph, result);
    
    EXPECT_FALSE(output.empty());
    EXPECT_NE(output.find("compound"), std::string::npos);
}
