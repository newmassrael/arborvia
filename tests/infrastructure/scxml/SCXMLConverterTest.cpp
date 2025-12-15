#include "SCXMLConverter.h"
#include "SCXMLConverter.h"
#include "SCXMLGraph.h"
#include "SCXMLTestLoader.h"

#include <gtest/gtest.h>
#include <filesystem>

namespace arborvia::test::scxml {

class SCXMLConverterTest : public ::testing::Test {
protected:
    SCXMLConverter converter;
    ConvertOptions opts;
};

TEST_F(SCXMLConverterTest, ParseSimpleStateMachine) {
    const char* xml = R"(
        <?xml version="1.0"?>
        <scxml initial="s0" version="1.0" xmlns="http://www.w3.org/2005/07/scxml">
            <state id="s0">
                <transition event="go" target="s1"/>
            </state>
            <state id="s1">
                <transition event="back" target="s0"/>
            </state>
            <final id="pass"/>
        </scxml>
    )";
    
    auto graph = converter.fromString(xml, opts);
    ASSERT_NE(graph, nullptr);
    
    // 3 states + 1 initial pseudo-state
    EXPECT_EQ(graph->nodeCount(), 4);
    
    // 2 transitions + 1 initial edge
    EXPECT_EQ(graph->edgeCount(), 3);
    
    // Verify state types
    auto s0 = graph->findByScxmlId("s0");
    ASSERT_TRUE(s0.has_value());
    EXPECT_EQ(graph->getNodeType(*s0), SCXMLNodeType::State);
    
    auto pass = graph->findByScxmlId("pass");
    ASSERT_TRUE(pass.has_value());
    EXPECT_EQ(graph->getNodeType(*pass), SCXMLNodeType::Final);
}

TEST_F(SCXMLConverterTest, ParseCompoundStates) {
    const char* xml = R"(
        <?xml version="1.0"?>
        <scxml initial="s1" version="1.0" xmlns="http://www.w3.org/2005/07/scxml">
            <state id="s1" initial="s11">
                <state id="s11">
                    <transition event="next" target="s12"/>
                </state>
                <state id="s12"/>
            </state>
            <final id="done"/>
        </scxml>
    )";
    
    auto graph = converter.fromString(xml, opts);
    ASSERT_NE(graph, nullptr);
    
    // Check hierarchy
    auto s1 = graph->findByScxmlId("s1");
    auto s11 = graph->findByScxmlId("s11");
    auto s12 = graph->findByScxmlId("s12");
    
    ASSERT_TRUE(s1.has_value());
    ASSERT_TRUE(s11.has_value());
    ASSERT_TRUE(s12.has_value());
    
    // s11 and s12 should have s1 as parent
    EXPECT_EQ(graph->getParent(*s11), s1);
    EXPECT_EQ(graph->getParent(*s12), s1);
    
    // s1 should have children
    EXPECT_TRUE(graph->hasChildren(*s1));
    auto children = graph->getChildren(*s1);
    EXPECT_GE(children.size(), 2);  // s11, s12, and possibly initial pseudo
}

TEST_F(SCXMLConverterTest, ParseParallelStates) {
    const char* xml = R"(
        <?xml version="1.0"?>
        <scxml initial="p1" version="1.0" xmlns="http://www.w3.org/2005/07/scxml">
            <parallel id="p1">
                <state id="region1">
                    <state id="r1s1"/>
                </state>
                <state id="region2">
                    <state id="r2s1"/>
                </state>
            </parallel>
        </scxml>
    )";
    
    auto graph = converter.fromString(xml, opts);
    ASSERT_NE(graph, nullptr);
    
    auto p1 = graph->findByScxmlId("p1");
    ASSERT_TRUE(p1.has_value());
    EXPECT_EQ(graph->getNodeType(*p1), SCXMLNodeType::Parallel);
    EXPECT_TRUE(graph->isParallel(*p1));
}

TEST_F(SCXMLConverterTest, ParseHistoryState) {
    const char* xml = R"(
        <?xml version="1.0"?>
        <scxml initial="s1" version="1.0" xmlns="http://www.w3.org/2005/07/scxml">
            <state id="s1">
                <history id="h1" type="shallow"/>
                <history id="h2" type="deep"/>
                <state id="s11"/>
            </state>
        </scxml>
    )";
    
    auto graph = converter.fromString(xml, opts);
    ASSERT_NE(graph, nullptr);
    
    auto h1 = graph->findByScxmlId("h1");
    auto h2 = graph->findByScxmlId("h2");
    
    ASSERT_TRUE(h1.has_value());
    ASSERT_TRUE(h2.has_value());
    
    EXPECT_EQ(graph->getNodeType(*h1), SCXMLNodeType::History);
    EXPECT_EQ(graph->getNodeType(*h2), SCXMLNodeType::HistoryDeep);
}

TEST_F(SCXMLConverterTest, ParseTransitionWithCondition) {
    const char* xml = R"(
        <?xml version="1.0"?>
        <scxml initial="s0" version="1.0" xmlns="http://www.w3.org/2005/07/scxml">
            <state id="s0">
                <transition event="check" cond="x > 0" target="s1"/>
            </state>
            <state id="s1"/>
        </scxml>
    )";
    
    auto graph = converter.fromString(xml, opts);
    ASSERT_NE(graph, nullptr);
    
    // Find the transition edge
    for (EdgeId eid : graph->edges()) {
        auto* info = graph->getEdgeInfo(eid);
        if (info && info->type == SCXMLEdgeType::Transition) {
            if (info->event == "check") {
                EXPECT_EQ(info->cond, "x > 0");
                return;
            }
        }
    }
    FAIL() << "Transition with event 'check' not found";
}

TEST_F(SCXMLConverterTest, ParseW3CTest144) {
    // W3C test 144: Basic event ordering
    const char* xml = R"(
        <?xml version="1.0"?>
        <scxml initial="s0" version="1.0" datamodel="ecmascript" xmlns="http://www.w3.org/2005/07/scxml">
            <state id="s0">
                <onentry>
                    <raise event="foo"/>
                    <raise event="bar"/>
                </onentry>
                <transition event="foo" target="s1"/>
                <transition event="*" target="fail"/>
            </state>
            <state id="s1">
                <transition event="bar" target="pass"/>
                <transition event="*" target="fail"/> 
            </state>
            <final id="pass"/>
            <final id="fail"/>
        </scxml>
    )";
    
    auto graph = converter.fromString(xml, opts);
    ASSERT_NE(graph, nullptr);
    
    // Should have: s0, s1, pass, fail, plus initial pseudo
    EXPECT_GE(graph->nodeCount(), 4);
    
    auto s0 = graph->findByScxmlId("s0");
    auto s1 = graph->findByScxmlId("s1");
    auto pass = graph->findByScxmlId("pass");
    auto fail = graph->findByScxmlId("fail");
    
    ASSERT_TRUE(s0.has_value());
    ASSERT_TRUE(s1.has_value());
    ASSERT_TRUE(pass.has_value());
    ASSERT_TRUE(fail.has_value());
    
    EXPECT_EQ(graph->getNodeType(*pass), SCXMLNodeType::Final);
    EXPECT_EQ(graph->getNodeType(*fail), SCXMLNodeType::Final);
    
    // Verify edge connections are correct
    int s0_to_s1 = 0, s0_to_fail = 0, s1_to_pass = 0, s1_to_fail = 0;
    for (EdgeId eid : graph->edges()) {
        auto& edge = graph->getEdge(eid);
        auto* info = graph->getEdgeInfo(eid);
        if (!info || info->type != SCXMLEdgeType::Transition) continue;
        
        std::string fromId = graph->getScxmlId(edge.from);
        std::string toId = graph->getScxmlId(edge.to);
        
        if (fromId == "s0" && toId == "s1") s0_to_s1++;
        else if (fromId == "s0" && toId == "fail") s0_to_fail++;
        else if (fromId == "s1" && toId == "pass") s1_to_pass++;
        else if (fromId == "s1" && toId == "fail") s1_to_fail++;
    }
    
    EXPECT_EQ(s0_to_s1, 1) << "Expected 1 edge from s0 to s1 (event=foo)";
    EXPECT_EQ(s0_to_fail, 1) << "Expected 1 edge from s0 to fail (event=*)";
    EXPECT_EQ(s1_to_pass, 1) << "Expected 1 edge from s1 to pass (event=bar)";
    EXPECT_EQ(s1_to_fail, 1) << "Expected 1 edge from s1 to fail (event=*)";
}

TEST_F(SCXMLConverterTest, NoInitialPseudoStates) {
    opts.createInitialPseudoStates = false;
    
    const char* xml = R"(
        <?xml version="1.0"?>
        <scxml initial="s0" version="1.0" xmlns="http://www.w3.org/2005/07/scxml">
            <state id="s0"/>
            <state id="s1"/>
        </scxml>
    )";
    
    auto graph = converter.fromString(xml, opts);
    ASSERT_NE(graph, nullptr);
    
    // Only 2 states, no initial pseudo-state
    EXPECT_EQ(graph->nodeCount(), 2);
}

TEST_F(SCXMLConverterTest, InvalidXML) {
    const char* xml = "<not valid xml";
    
    auto graph = converter.fromString(xml, opts);
    EXPECT_EQ(graph, nullptr);
    
    auto error = converter.getLastError();
    ASSERT_TRUE(error.has_value());
    EXPECT_FALSE(error->message.empty());
}

TEST_F(SCXMLConverterTest, NoScxmlRoot) {
    const char* xml = R"(
        <?xml version="1.0"?>
        <notscxml>
            <state id="s0"/>
        </notscxml>
    )";
    
    auto graph = converter.fromString(xml, opts);
    EXPECT_EQ(graph, nullptr);
    
    auto error = converter.getLastError();
    ASSERT_TRUE(error.has_value());
    EXPECT_TRUE(error->message.find("scxml") != std::string::npos);
}

TEST_F(SCXMLConverterTest, DumpGraph) {
    const char* xml = R"(
        <?xml version="1.0"?>
        <scxml initial="s0" version="1.0" xmlns="http://www.w3.org/2005/07/scxml">
            <state id="s0">
                <transition event="go" target="s1"/>
            </state>
            <state id="s1"/>
        </scxml>
    )";
    
    auto graph = converter.fromString(xml, opts);
    ASSERT_NE(graph, nullptr);
    
    // Should not crash
    testing::internal::CaptureStdout();
    graph->dump();
    std::string output = testing::internal::GetCapturedStdout();
    
    EXPECT_TRUE(output.find("s0") != std::string::npos);
    EXPECT_TRUE(output.find("s1") != std::string::npos);
}

// Test loading from W3C test file (if available)
TEST_F(SCXMLConverterTest, LoadFromFile) {
    std::string testPath = "/home/coin/scxml-core-engine/resources/144/test144.scxml";
    
    if (!std::filesystem::exists(testPath)) {
        GTEST_SKIP() << "W3C test file not found: " << testPath;
    }
    
    auto graph = converter.fromFile(testPath, opts);
    ASSERT_NE(graph, nullptr);
    
    // test144 has: s0, s1, pass, fail
    auto pass = graph->findByScxmlId("pass");
    auto fail = graph->findByScxmlId("fail");
    
    ASSERT_TRUE(pass.has_value());
    ASSERT_TRUE(fail.has_value());
    
    EXPECT_EQ(graph->getNodeType(*pass), SCXMLNodeType::Final);
    EXPECT_EQ(graph->getNodeType(*fail), SCXMLNodeType::Final);
}

// ============================================================================
// SCXMLTestLoader tests
// ============================================================================

class SCXMLTestLoaderTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Try to find resources directory
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
    
    std::filesystem::path resourcePath_;
};

TEST_F(SCXMLTestLoaderTest, LoadIndex) {
    if (resourcePath_.empty()) {
        GTEST_SKIP() << "SCXML resources directory not found";
    }
    
    SCXMLTestLoader loader(resourcePath_);
    ASSERT_TRUE(loader.loadIndex()) << loader.getLastError();
    
    EXPECT_GT(loader.getTestCount(), 0);
    std::cout << "Loaded " << loader.getTestCount() << " tests" << std::endl;
}

TEST_F(SCXMLTestLoaderTest, GetTestById) {
    if (resourcePath_.empty()) {
        GTEST_SKIP() << "SCXML resources directory not found";
    }
    
    SCXMLTestLoader loader(resourcePath_);
    ASSERT_TRUE(loader.loadIndex());
    
    auto* test144 = loader.getTestById("144");
    ASSERT_NE(test144, nullptr);
    EXPECT_EQ(test144->id, "144");
    EXPECT_EQ(test144->file, "test144.scxml");
    EXPECT_FALSE(test144->description.empty());
}

TEST_F(SCXMLTestLoaderTest, LoadGraphById) {
    if (resourcePath_.empty()) {
        GTEST_SKIP() << "SCXML resources directory not found";
    }
    
    SCXMLTestLoader loader(resourcePath_);
    ASSERT_TRUE(loader.loadIndex());
    
    auto graph = loader.loadGraphById("144");
    ASSERT_NE(graph, nullptr) << loader.getLastError();
    
    // test144 should have s0, s1, pass, fail
    EXPECT_TRUE(graph->findByScxmlId("s0").has_value());
    EXPECT_TRUE(graph->findByScxmlId("pass").has_value());
}

TEST_F(SCXMLTestLoaderTest, LoadGraphByIndex) {
    if (resourcePath_.empty()) {
        GTEST_SKIP() << "SCXML resources directory not found";
    }
    
    SCXMLTestLoader loader(resourcePath_);
    ASSERT_TRUE(loader.loadIndex());
    
    // Load first test
    auto graph = loader.loadGraph(0);
    ASSERT_NE(graph, nullptr) << loader.getLastError();
    EXPECT_GT(graph->nodeCount(), 0);
}

TEST_F(SCXMLTestLoaderTest, IterateAllTests) {
    if (resourcePath_.empty()) {
        GTEST_SKIP() << "SCXML resources directory not found";
    }
    
    SCXMLTestLoader loader(resourcePath_);
    ASSERT_TRUE(loader.loadIndex());
    
    size_t successCount = 0;
    size_t failCount = 0;
    
    for (size_t i = 0; i < loader.getTestCount(); ++i) {
        auto* test = loader.getTest(i);
        ASSERT_NE(test, nullptr);
        
        auto graph = loader.loadGraph(i);
        if (graph) {
            successCount++;
        } else {
            failCount++;
            // Uncomment to see failures:
            // std::cerr << "Failed to load test " << test->id << ": " 
            //           << loader.getLastError() << std::endl;
        }
    }
    
    std::cout << "Loaded " << successCount << "/" << loader.getTestCount() 
              << " tests successfully" << std::endl;
    
    // At least 90% should load successfully
    EXPECT_GT(successCount, loader.getTestCount() * 0.9);
}

}  // namespace arborvia::test::scxml
