#include <gtest/gtest.h>
#include <arborvia/arborvia.h>
#include <arborvia/layout/ValidRegionCalculator.h>
#include <arborvia/layout/LayoutUtils.h>
#include <arborvia/layout/ConstraintConfig.h>

using namespace arborvia;

/**
 * ValidRegionCalculatorTest
 *
 * TDD tests to ensure ValidRegionCalculator predictions match
 * actual canMoveNodeTo() validation results.
 *
 * The key invariant:
 *   ValidRegionCalculator::isValid(pos) == true
 *   implies
 *   LayoutUtils::canMoveNodeTo(pos).valid == true
 *
 * In other words: if ValidRegionCalculator says a position is valid,
 * the actual constraint system must also say it's valid.
 * (False negatives are acceptable, false positives are not)
 */
class ValidRegionCalculatorTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Create state machine graph (same as interactive demo)
        idle_ = graph_.addNode(Size{100, 50}, "Idle");
        running_ = graph_.addNode(Size{100, 50}, "Running");
        paused_ = graph_.addNode(Size{100, 50}, "Paused");
        stopped_ = graph_.addNode(Size{100, 50}, "Stopped");
        error_ = graph_.addNode(Size{100, 50}, "Error");

        // Add edges with varying connectivity
        graph_.addEdge(idle_, running_, "start");      // idle: 1, running: 1
        graph_.addEdge(running_, paused_, "pause");    // running: 2, paused: 1
        graph_.addEdge(paused_, running_, "resume");   // running: 3, paused: 2
        graph_.addEdge(running_, stopped_, "stop");    // running: 4, stopped: 1
        graph_.addEdge(paused_, stopped_, "stop");     // paused: 3, stopped: 2
        graph_.addEdge(running_, error_, "fail");      // running: 5, error: 1
        graph_.addEdge(error_, idle_, "reset");        // error: 2, idle: 2

        // Perform initial layout
        LayoutOptions options;
        options.gridConfig.cellSize = gridSize_;
        SugiyamaLayout layout(options);
        layoutResult_ = layout.layout(graph_);

        nodeLayouts_ = layoutResult_.nodeLayouts();
        edgeLayouts_ = layoutResult_.edgeLayouts();
    }

    // Test helper: check if ValidRegionCalculator and canMoveNodeTo agree
    struct ConsistencyResult {
        int totalPositions = 0;
        int falsePositives = 0;  // Calculator says valid, but canMoveNodeTo says invalid
        int falseNegatives = 0;  // Calculator says invalid, but canMoveNodeTo says valid
        std::vector<Point> falsePositivePositions;
        std::vector<Point> falseNegativePositions;
    };

    ConsistencyResult checkConsistency(NodeId draggedNode, int gridRange = 20) {
        ConsistencyResult result;

        // Get dragged node info
        auto nodeIt = nodeLayouts_.find(draggedNode);
        if (nodeIt == nodeLayouts_.end()) return result;
        const Size& nodeSize = nodeIt->second.size;

        // Calculate forbidden zones
        auto zones = ValidRegionCalculator::calculate(
            draggedNode, nodeLayouts_, edgeLayouts_, gridSize_);

        // Test positions in a grid around the node's original position
        Point originalPos = nodeIt->second.position;
        float testRange = gridRange * gridSize_;

        for (float dx = -testRange; dx <= testRange; dx += gridSize_) {
            for (float dy = -testRange; dy <= testRange; dy += gridSize_) {
                Point testPos = {originalPos.x + dx, originalPos.y + dy};
                result.totalPositions++;

                // ValidRegionCalculator prediction
                bool calculatorSaysValid = ValidRegionCalculator::isValid(testPos, nodeSize, zones);

                // Actual canMoveNodeTo result
                auto validation = LayoutUtils::canMoveNodeTo(
                    draggedNode, testPos, nodeLayouts_, edgeLayouts_, gridSize_);
                bool actuallyValid = validation.valid;

                if (calculatorSaysValid && !actuallyValid) {
                    // FALSE POSITIVE: Calculator says OK but constraint fails
                    result.falsePositives++;
                    result.falsePositivePositions.push_back(testPos);
                } else if (!calculatorSaysValid && actuallyValid) {
                    // FALSE NEGATIVE: Calculator says NO but constraint passes
                    result.falseNegatives++;
                    result.falseNegativePositions.push_back(testPos);
                }
            }
        }

        return result;
    }

    Graph graph_;
    LayoutResult layoutResult_;
    std::unordered_map<NodeId, NodeLayout> nodeLayouts_;
    std::unordered_map<EdgeId, EdgeLayout> edgeLayouts_;

    NodeId idle_, running_, paused_, stopped_, error_;
    float gridSize_ = 20.0f;
};

// RED TEST: ValidRegionCalculator must not have false positives
// A false positive means showing a cell as valid (green) when it's actually invalid
TEST_F(ValidRegionCalculatorTest, NoFalsePositives_RunningNode) {
    // Running node has 5 edges - highest connectivity
    auto result = checkConsistency(running_);

    std::cout << "\n=== Running Node (5 edges) Consistency Test ===" << std::endl;
    std::cout << "Total positions tested: " << result.totalPositions << std::endl;
    std::cout << "False positives (CRITICAL): " << result.falsePositives << std::endl;
    std::cout << "False negatives (acceptable): " << result.falseNegatives << std::endl;

    if (result.falsePositives > 0) {
        std::cout << "\nFalse positive positions (calculator says valid, actually invalid):" << std::endl;
        for (size_t i = 0; i < std::min(result.falsePositivePositions.size(), size_t(5)); ++i) {
            auto& pos = result.falsePositivePositions[i];
            std::cout << "  (" << pos.x << ", " << pos.y << ")" << std::endl;
        }
    }

    // FALSE POSITIVES ARE NOT ALLOWED
    // If ValidRegionCalculator says a position is valid, it MUST actually be valid
    EXPECT_EQ(result.falsePositives, 0)
        << "ValidRegionCalculator has " << result.falsePositives << " false positives. "
        << "These positions are shown as valid but canMoveNodeTo() rejects them.";
}

TEST_F(ValidRegionCalculatorTest, NoFalsePositives_IdleNode) {
    // Idle node has 2 edges
    auto result = checkConsistency(idle_);

    std::cout << "\n=== Idle Node (2 edges) Consistency Test ===" << std::endl;
    std::cout << "Total positions tested: " << result.totalPositions << std::endl;
    std::cout << "False positives (CRITICAL): " << result.falsePositives << std::endl;
    std::cout << "False negatives (acceptable): " << result.falseNegatives << std::endl;

    EXPECT_EQ(result.falsePositives, 0)
        << "ValidRegionCalculator has " << result.falsePositives << " false positives.";
}

TEST_F(ValidRegionCalculatorTest, NoFalsePositives_StoppedNode) {
    // Stopped node has 2 edges
    auto result = checkConsistency(stopped_);

    std::cout << "\n=== Stopped Node (2 edges) Consistency Test ===" << std::endl;
    std::cout << "Total positions tested: " << result.totalPositions << std::endl;
    std::cout << "False positives (CRITICAL): " << result.falsePositives << std::endl;
    std::cout << "False negatives (acceptable): " << result.falseNegatives << std::endl;

    EXPECT_EQ(result.falsePositives, 0)
        << "ValidRegionCalculator has " << result.falsePositives << " false positives.";
}

// Test that false negatives are reasonable (not too many)
// False negatives mean the forbidden zone is larger than necessary
TEST_F(ValidRegionCalculatorTest, FalseNegativesAreReasonable) {
    auto result = checkConsistency(running_);

    // Allow some false negatives (conservative estimate is OK)
    // Inverse calculation is intentionally conservative to guarantee valid routing
    // Up to 25% false negatives is acceptable for this safety margin
    float falseNegativeRate = static_cast<float>(result.falseNegatives) / result.totalPositions;

    std::cout << "\n=== False Negative Rate Test ===" << std::endl;
    std::cout << "False negative rate: " << (falseNegativeRate * 100) << "%" << std::endl;

    EXPECT_LT(falseNegativeRate, 0.25f)
        << "ValidRegionCalculator is too conservative. "
        << "False negative rate: " << (falseNegativeRate * 100) << "%";
}
