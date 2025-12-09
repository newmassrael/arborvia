#include <gtest/gtest.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <signal.h>
#include <sys/wait.h>
#include <cstring>
#include <string>
#include <thread>
#include <chrono>
#include <sstream>
#include <vector>
#include <cmath>
#include <iostream>
#include <nlohmann/json.hpp>

using json = nlohmann::json;

// Edge path structure for testing
struct EdgePath {
    std::vector<std::pair<float, float>> points;  // All points: source, bends, target
    
    static EdgePath fromJson(const json& layout, int edgeId) {
        EdgePath path;
        
        for (const auto& edge : layout["edgeLayouts"]) {
            if (edge["id"].get<int>() == edgeId) {
                // Source point
                path.points.push_back({
                    edge["sourcePoint"]["x"].get<float>(),
                    edge["sourcePoint"]["y"].get<float>()
                });
                
                // Bend points
                for (const auto& bp : edge["bendPoints"]) {
                    path.points.push_back({
                        bp["position"]["x"].get<float>(),
                        bp["position"]["y"].get<float>()
                    });
                }
                
                // Target point
                path.points.push_back({
                    edge["targetPoint"]["x"].get<float>(),
                    edge["targetPoint"]["y"].get<float>()
                });
                break;
            }
        }
        return path;
    }
    
    void print(const std::string& label) const {
        std::cout << label << std::endl;
        for (size_t i = 0; i < points.size(); ++i) {
            std::string name = (i == 0) ? "source" : 
                               (i == points.size() - 1) ? "target" : 
                               "bend[" + std::to_string(i - 1) + "]";
            std::cout << "  " << name << ": (" << points[i].first << ", " << points[i].second << ")" << std::endl;
        }
    }
    
    bool isOrthogonal(float epsilon = 0.1f) const {
        for (size_t i = 0; i + 1 < points.size(); ++i) {
            float dx = std::abs(points[i + 1].first - points[i].first);
            float dy = std::abs(points[i + 1].second - points[i].second);
            bool isHorizontal = dy < epsilon;
            bool isVertical = dx < epsilon;
            if (!isHorizontal && !isVertical) {
                return false;
            }
        }
        return true;
    }
    
    std::string checkSegments(float epsilon = 0.1f) const {
        std::ostringstream oss;
        for (size_t i = 0; i + 1 < points.size(); ++i) {
            float dx = std::abs(points[i + 1].first - points[i].first);
            float dy = std::abs(points[i + 1].second - points[i].second);
            bool isHorizontal = dy < epsilon;
            bool isVertical = dx < epsilon;

            std::string fromName = (i == 0) ? "source" : "bend[" + std::to_string(i - 1) + "]";
            std::string toName = (i + 1 == points.size() - 1) ? "target" : "bend[" + std::to_string(i) + "]";

            std::cout << "  Segment " << fromName << " -> " << toName
                      << ": dx=" << dx << ", dy=" << dy;

            if (!isHorizontal && !isVertical) {
                std::cout << " [DIAGONAL!]" << std::endl;
                oss << "Segment " << fromName << " -> " << toName
                    << ": (" << points[i].first << "," << points[i].second << ") -> ("
                    << points[i + 1].first << "," << points[i + 1].second << ") is DIAGONAL!\n";
            } else {
                std::cout << " [OK]" << std::endl;
            }
        }
        return oss.str();
    }
};

// Check if two edge paths have overlapping segments
std::string checkSegmentOverlap(const EdgePath& path1, const EdgePath& path2, float tolerance = 1.0f) {
    std::ostringstream oss;

    for (size_t i = 0; i + 1 < path1.points.size(); ++i) {
        for (size_t j = 0; j + 1 < path2.points.size(); ++j) {
            auto [x1a, y1a] = path1.points[i];
            auto [x1b, y1b] = path1.points[i + 1];
            auto [x2a, y2a] = path2.points[j];
            auto [x2b, y2b] = path2.points[j + 1];

            // Both horizontal segments at same Y
            if (std::abs(y1a - y1b) < tolerance &&
                std::abs(y2a - y2b) < tolerance &&
                std::abs(y1a - y2a) < tolerance) {
                float x1_min = std::min(x1a, x1b), x1_max = std::max(x1a, x1b);
                float x2_min = std::min(x2a, x2b), x2_max = std::max(x2a, x2b);
                if (x1_min < x2_max && x2_min < x1_max) {
                    oss << "Horizontal overlap at y=" << y1a
                        << " x=[" << std::max(x1_min, x2_min) << "," << std::min(x1_max, x2_max) << "]\n";
                }
            }

            // Both vertical segments at same X
            if (std::abs(x1a - x1b) < tolerance &&
                std::abs(x2a - x2b) < tolerance &&
                std::abs(x1a - x2a) < tolerance) {
                float y1_min = std::min(y1a, y1b), y1_max = std::max(y1a, y1b);
                float y2_min = std::min(y2a, y2b), y2_max = std::max(y2a, y2b);
                if (y1_min < y2_max && y2_min < y1_max) {
                    oss << "Vertical overlap at x=" << x1a
                        << " y=[" << std::max(y1_min, y2_min) << "," << std::min(y1_max, y2_max) << "]\n";
                }
            }
        }
    }

    return oss.str();
}

class DemoApiTest : public ::testing::Test {
protected:
    pid_t demoPid_ = -1;
    int sock_ = -1;
    static constexpr int TEST_PORT = 9998;  // Use different port to avoid conflict

    void SetUp() override {
        // Start interactive_demo with test port
        demoPid_ = fork();
        if (demoPid_ == 0) {
            // Child process - run demo
            freopen("/dev/null", "w", stdout);
            freopen("/dev/null", "w", stderr);
            std::string portArg = "--port=" + std::to_string(TEST_PORT);
            execl("./build/examples/interactive_demo", "interactive_demo", portArg.c_str(), nullptr);
            _exit(1);
        }

        // Wait for server to start
        std::this_thread::sleep_for(std::chrono::milliseconds(2000));

        // Connect to command server
        sock_ = socket(AF_INET, SOCK_STREAM, 0);
        ASSERT_GT(sock_, 0) << "Failed to create socket";

        sockaddr_in addr{};
        addr.sin_family = AF_INET;
        addr.sin_port = htons(TEST_PORT);
        inet_pton(AF_INET, "127.0.0.1", &addr.sin_addr);

        int connected = connect(sock_, (sockaddr*)&addr, sizeof(addr));
        ASSERT_EQ(connected, 0) << "Failed to connect to demo server";
    }

    void TearDown() override {
        if (sock_ > 0) {
            sendCommand("quit");
            close(sock_);
        }

        if (demoPid_ > 0) {
            kill(demoPid_, SIGTERM);
            waitpid(demoPid_, nullptr, 0);
        }
    }

    std::string sendCommand(const std::string& cmd) {
        std::string cmdWithNewline = cmd + "\n";
        send(sock_, cmdWithNewline.c_str(), cmdWithNewline.length(), 0);

        char buffer[65536] = {0};
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        ssize_t received = recv(sock_, buffer, sizeof(buffer) - 1, 0);
        if (received > 0) {
            buffer[received] = '\0';
        }
        return std::string(buffer);
    }
};

// Helper to find self-loop edge path from JSON
EdgePath findSelfLoopEdge(const json& layout) {
    EdgePath path;
    for (const auto& edge : layout["edgeLayouts"]) {
        if (edge["from"].get<int>() == edge["to"].get<int>()) {
            // Found self-loop
            path.points.push_back({
                edge["sourcePoint"]["x"].get<float>(),
                edge["sourcePoint"]["y"].get<float>()
            });
            for (const auto& bp : edge["bendPoints"]) {
                path.points.push_back({
                    bp["position"]["x"].get<float>(),
                    bp["position"]["y"].get<float>()
                });
            }
            path.points.push_back({
                edge["targetPoint"]["x"].get<float>(),
                edge["targetPoint"]["y"].get<float>()
            });
            break;
        }
    }
    return path;
}

// TDD RED: Self-loop edge must be orthogonal
// Reproduces: error -> error self-transition has diagonal first segment
TEST_F(DemoApiTest, SelfLoopEdge_MustBeOrthogonal) {
    // Get layout
    std::string layoutStr = sendCommand("get_layout");
    json layout = json::parse(layoutStr);

    // Find self-loop edge
    EdgePath selfLoop = findSelfLoopEdge(layout);
    ASSERT_FALSE(selfLoop.points.empty()) << "Demo should have a self-loop edge";

    selfLoop.print("\n=== Self-loop Edge ===");

    // Check all segments are orthogonal
    std::string errorMsg = selfLoop.checkSegments();
    bool allOrthogonal = selfLoop.isOrthogonal();

    EXPECT_TRUE(allOrthogonal)
        << "Self-loop edge must be orthogonal!\n"
        << errorMsg;
}

// TDD RED: Edge must remain orthogonal after node drag
// Reproduces: idle dragged right by 20px causes diagonal first segment
TEST_F(DemoApiTest, DragIdleRight_EdgeMustRemainOrthogonal) {
    // Get initial state
    std::string state = sendCommand("get_state");
    std::cout << "Initial state: " << state << std::endl;
    ASSERT_TRUE(state.find("nodes=5") != std::string::npos) << "Demo should have 5 nodes";

    // Get initial edge 0 layout using proper JSON parsing
    std::string layoutBefore = sendCommand("get_layout");
    json jsonBefore = json::parse(layoutBefore);
    EdgePath pathBefore = EdgePath::fromJson(jsonBefore, 0);
    pathBefore.print("\n=== Edge 0 BEFORE drag ===");

    // Drag idle (node 0) right by 20 pixels
    std::string dragResult = sendCommand("drag 0 20 0");
    std::cout << "\nDrag result: " << dragResult << std::endl;

    // Get edge 0 layout after drag
    std::string layoutAfter = sendCommand("get_layout");
    json jsonAfter = json::parse(layoutAfter);
    EdgePath pathAfter = EdgePath::fromJson(jsonAfter, 0);
    pathAfter.print("\n=== Edge 0 AFTER drag ===");

    // Check all segments are orthogonal
    std::string errorMsg = pathAfter.checkSegments();
    bool allOrthogonal = pathAfter.isOrthogonal();

    EXPECT_TRUE(allOrthogonal)
        << "All edge segments must be orthogonal after drag!\n"
        << errorMsg;
}

// TDD RED: Edge segments must not overlap after drag
// Reproduces: Edge 0 and Edge 1 overlap at y=60 after dragging node 1 right
TEST_F(DemoApiTest, EdgeSegments_MustNotOverlapAfterDrag) {
    // Drag running (node 1) right by 380 pixels to trigger edge overlap
    // This reproduces the state where Edge 0 and Edge 1 overlap at y=60
    std::string dragResult = sendCommand("drag 1 380 -120");
    std::cout << "Drag result: " << dragResult << std::endl;

    // Get layout after drag
    std::string layoutStr = sendCommand("get_layout");
    json layout = json::parse(layoutStr);

    // Print all edge paths for debugging
    std::cout << "\n=== Edge paths after drag ===" << std::endl;
    for (const auto& edge : layout["edgeLayouts"]) {
        int id = edge["id"].get<int>();
        EdgePath path = EdgePath::fromJson(layout, id);
        path.print("Edge " + std::to_string(id) + " (" + 
                   std::to_string(edge["from"].get<int>()) + "→" +
                   std::to_string(edge["to"].get<int>()) + "):");
    }

    // Check all edge pairs for segment overlap
    auto edges = layout["edgeLayouts"];
    bool hasOverlap = false;
    std::ostringstream allOverlaps;

    for (size_t i = 0; i < edges.size(); ++i) {
        for (size_t j = i + 1; j < edges.size(); ++j) {
            int id1 = edges[i]["id"].get<int>();
            int id2 = edges[j]["id"].get<int>();

            EdgePath path1 = EdgePath::fromJson(layout, id1);
            EdgePath path2 = EdgePath::fromJson(layout, id2);

            std::string overlap = checkSegmentOverlap(path1, path2);
            if (!overlap.empty()) {
                hasOverlap = true;
                allOverlaps << "Edge " << id1 << " and Edge " << id2
                            << " overlap:\n" << overlap;
            }
        }
    }

    EXPECT_FALSE(hasOverlap)
        << "Edge segments must not overlap after drag!\n"
        << allOverlaps.str();
}

// Helper: Check if segment is orthogonal (horizontal or vertical)
bool isOrthogonalSegment(const std::pair<float, float>& p1,
                         const std::pair<float, float>& p2,
                         float epsilon = 0.1f) {
    float dx = std::abs(p2.first - p1.first);
    float dy = std::abs(p2.second - p1.second);
    return (dx < epsilon) || (dy < epsilon);
}

// Helper: Find intersection point between two segments
// Returns true if segments intersect, with intersection point in 'out'
bool segmentIntersection(const std::pair<float, float>& p1,
                         const std::pair<float, float>& p2,
                         const std::pair<float, float>& p3,
                         const std::pair<float, float>& p4,
                         std::pair<float, float>& out) {
    float x1 = p1.first, y1 = p1.second;
    float x2 = p2.first, y2 = p2.second;
    float x3 = p3.first, y3 = p3.second;
    float x4 = p4.first, y4 = p4.second;

    float denom = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4);
    if (std::abs(denom) < 0.0001f) {
        return false;  // Parallel
    }

    float t = ((x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4)) / denom;
    float u = -((x1 - x2) * (y1 - y3) - (y1 - y2) * (x1 - x3)) / denom;

    if (t >= 0 && t <= 1 && u >= 0 && u <= 1) {
        out.first = x1 + t * (x2 - x1);
        out.second = y1 + t * (y2 - y1);
        return true;
    }
    return false;
}

// TDD RED: Edge crossings must be orthogonal (horizontal vs vertical only)
// Non-orthogonal crossings (involving diagonal segments) are not allowed
// Reproduces: Edge 3 diagonal segment crossing Edge 6 vertical segment
TEST_F(DemoApiTest, EdgeCrossings_MustBeOrthogonalOnly) {
    // Drag node 1 (running) to reproduce diagonal segment issue
    std::string dragResult = sendCommand("drag 1 420 -140");
    std::cout << "Drag result: " << dragResult << std::endl;

    // Get layout after drag
    std::string layoutStr = sendCommand("get_layout");
    json layout = json::parse(layoutStr);

    // Collect all edge paths
    struct EdgeInfo {
        int id;
        int from;
        int to;
        std::vector<std::pair<float, float>> path;
    };
    std::vector<EdgeInfo> edges;

    for (const auto& edge : layout["edgeLayouts"]) {
        EdgeInfo info;
        info.id = edge["id"].get<int>();
        info.from = edge["from"].get<int>();
        info.to = edge["to"].get<int>();

        info.path.push_back({
            edge["sourcePoint"]["x"].get<float>(),
            edge["sourcePoint"]["y"].get<float>()
        });
        for (const auto& bp : edge["bendPoints"]) {
            info.path.push_back({
                bp["position"]["x"].get<float>(),
                bp["position"]["y"].get<float>()
            });
        }
        info.path.push_back({
            edge["targetPoint"]["x"].get<float>(),
            edge["targetPoint"]["y"].get<float>()
        });

        edges.push_back(info);
    }

    // Check all edge pairs for crossings
    bool hasInvalidCrossing = false;
    std::ostringstream errors;

    for (size_t i = 0; i < edges.size(); ++i) {
        for (size_t j = i + 1; j < edges.size(); ++j) {
            const auto& e1 = edges[i];
            const auto& e2 = edges[j];

            // Check each segment pair
            for (size_t s1 = 0; s1 + 1 < e1.path.size(); ++s1) {
                for (size_t s2 = 0; s2 + 1 < e2.path.size(); ++s2) {
                    auto seg1_p1 = e1.path[s1];
                    auto seg1_p2 = e1.path[s1 + 1];
                    auto seg2_p1 = e2.path[s2];
                    auto seg2_p2 = e2.path[s2 + 1];

                    std::pair<float, float> intersection;
                    if (segmentIntersection(seg1_p1, seg1_p2, seg2_p1, seg2_p2, intersection)) {
                        bool seg1_ortho = isOrthogonalSegment(seg1_p1, seg1_p2);
                        bool seg2_ortho = isOrthogonalSegment(seg2_p1, seg2_p2);

                        if (!seg1_ortho || !seg2_ortho) {
                            hasInvalidCrossing = true;
                            errors << "Non-orthogonal crossing at ("
                                   << intersection.first << ", " << intersection.second << "):\n";
                            errors << "  Edge " << e1.id << " (" << e1.from << "->" << e1.to << ") seg["
                                   << s1 << "]: (" << seg1_p1.first << "," << seg1_p1.second
                                   << ") -> (" << seg1_p2.first << "," << seg1_p2.second << ")";
                            if (!seg1_ortho) errors << " [DIAGONAL]";
                            errors << "\n";
                            errors << "  Edge " << e2.id << " (" << e2.from << "->" << e2.to << ") seg["
                                   << s2 << "]: (" << seg2_p1.first << "," << seg2_p1.second
                                   << ") -> (" << seg2_p2.first << "," << seg2_p2.second << ")";
                            if (!seg2_ortho) errors << " [DIAGONAL]";
                            errors << "\n\n";
                        }
                    }
                }
            }
        }
    }

    EXPECT_FALSE(hasInvalidCrossing)
        << "All edge crossings must be orthogonal (horizontal vs vertical)!\n"
        << errors.str();
}

// TDD RED: All edges must be orthogonal at startup (before any drag)
// Check for edges with empty bendPoints that form diagonal
TEST_F(DemoApiTest, InitialLayout_AllEdgesMustBeOrthogonal) {
    // Get initial layout WITHOUT any drags
    std::string layoutStr = sendCommand("get_layout");
    json layout = json::parse(layoutStr);

    bool allOrthogonal = true;
    std::ostringstream errors;

    for (const auto& edge : layout["edgeLayouts"]) {
        int edgeId = edge["id"].get<int>();
        EdgePath path = EdgePath::fromJson(layout, edgeId);

        if (!path.isOrthogonal()) {
            allOrthogonal = false;
            errors << "Edge " << edgeId << " is NOT orthogonal:\n";
            errors << path.checkSegments();

            if (path.points.size() == 2) {
                errors << "  -> Edge has EMPTY bendPoints but forms diagonal!\n";
            }
        }
    }

    EXPECT_TRUE(allOrthogonal)
        << "All edges must be orthogonal at startup!\n"
        << errors.str();
}

// TDD RED: Empty bendPoints must trigger retry when source/target form diagonal
// Reproduce by complex drag sequence that causes A* exhaustion
TEST_F(DemoApiTest, EmptyBendPoints_MustTriggerRetryOnDiagonal) {
    // Complex drag sequence that may cause A* exhaustion
    // Move nodes to create congested routing situation
    sendCommand("drag 0 200 0");   // Move idle right
    sendCommand("drag 2 -100 0");  // Move paused left
    sendCommand("drag 1 150 -50"); // Move running
    sendCommand("drag 3 0 -100");  // Move stopped up
    sendCommand("drag 4 -50 0");   // Move error left

    // Get layout and check ALL edges for orthogonality
    std::string layoutStr = sendCommand("get_layout");
    json layout = json::parse(layoutStr);

    bool allOrthogonal = true;
    std::ostringstream errors;

    for (const auto& edge : layout["edgeLayouts"]) {
        int edgeId = edge["id"].get<int>();
        EdgePath path = EdgePath::fromJson(layout, edgeId);

        if (!path.isOrthogonal()) {
            allOrthogonal = false;
            errors << "Edge " << edgeId << " is NOT orthogonal:\n";
            errors << path.checkSegments();

            // Special check: empty bendPoints = A* total failure
            if (path.points.size() == 2) {
                errors << "  -> Edge has EMPTY bendPoints but forms diagonal!\n";
                errors << "  -> A* retry system failed to trigger.\n";
            }
        }
    }

    EXPECT_TRUE(allOrthogonal)
        << "All edges must be orthogonal after drag!\n"
        << "When A* fails and clears bendPoints, retry must trigger.\n\n"
        << errors.str();
}

// TDD RED: Edge 2 (paused→running) must remain orthogonal after drag
TEST_F(DemoApiTest, Edge2_MustRemainOrthogonalAfterDrag) {
    // Drag running (node 1) far right and up
    std::string dragResult = sendCommand("drag 1 300 -100");
    std::cout << "Drag result: " << dragResult << std::endl;

    // Get layout after drag
    std::string layoutStr = sendCommand("get_layout");
    json layout = json::parse(layoutStr);

    // Find Edge 2 specifically
    EdgePath edge2 = EdgePath::fromJson(layout, 2);
    edge2.print("\n=== Edge 2 (paused->running) AFTER drag ===");

    // Edge 2 must be orthogonal
    std::string errorMsg = edge2.checkSegments();
    bool isOrthogonal = edge2.isOrthogonal();

    EXPECT_TRUE(isOrthogonal)
        << "Edge 2 must be orthogonal after drag!\n"
        << errorMsg;

    // Additional check: if only 2 points and not orthogonal = A* failed + no retry
    if (edge2.points.size() == 2 && !isOrthogonal) {
        ADD_FAILURE() << "Edge 2 has NO bends and is diagonal!\n"
                      << "This indicates A* failed but retry was NOT triggered.\n"
                      << "Bug: Diagonal detection only checks non-empty bendPoints.";
    }
}

// Helper: Get edge info from JSON
struct EdgeLayoutInfo {
    int id;
    int from;
    int to;
    float sourceX, sourceY;
    float targetX, targetY;
    int sourceSnapIndex, targetSnapIndex;
    int sourceEdge, targetEdge;  // 0=Top, 1=Bottom, 2=Left, 3=Right
    int bendCount;

    static EdgeLayoutInfo fromJson(const json& layout, int edgeId) {
        EdgeLayoutInfo info{};
        for (const auto& edge : layout["edgeLayouts"]) {
            int thisId = edge["id"].is_string() ? std::stoi(edge["id"].get<std::string>()) : edge["id"].get<int>();
            if (thisId == edgeId) {
                info.id = edgeId;
                info.from = edge["from"].is_string() ? std::stoi(edge["from"].get<std::string>()) : edge["from"].get<int>();
                info.to = edge["to"].is_string() ? std::stoi(edge["to"].get<std::string>()) : edge["to"].get<int>();
                info.sourceX = edge["sourcePoint"]["x"].get<float>();
                info.sourceY = edge["sourcePoint"]["y"].get<float>();
                info.targetX = edge["targetPoint"]["x"].get<float>();
                info.targetY = edge["targetPoint"]["y"].get<float>();
                info.sourceSnapIndex = edge.contains("sourceSnapIndex") ?
                    (edge["sourceSnapIndex"].is_string() ? std::stoi(edge["sourceSnapIndex"].get<std::string>()) : edge["sourceSnapIndex"].get<int>()) : -1;
                info.targetSnapIndex = edge.contains("targetSnapIndex") ?
                    (edge["targetSnapIndex"].is_string() ? std::stoi(edge["targetSnapIndex"].get<std::string>()) : edge["targetSnapIndex"].get<int>()) : -1;
                info.sourceEdge = edge.contains("sourceEdge") ?
                    (edge["sourceEdge"].is_string() ? std::stoi(edge["sourceEdge"].get<std::string>()) : edge["sourceEdge"].get<int>()) : -1;
                info.targetEdge = edge.contains("targetEdge") ?
                    (edge["targetEdge"].is_string() ? std::stoi(edge["targetEdge"].get<std::string>()) : edge["targetEdge"].get<int>()) : -1;
                info.bendCount = edge["bendPoints"].size();
                break;
            }
        }
        return info;
    }

    void print(const std::string& label) const {
        std::cout << label << std::endl;
        std::cout << "  from=" << from << " to=" << to << std::endl;
        std::cout << "  source: (" << sourceX << "," << sourceY << ") snapIdx=" << sourceSnapIndex << " edge=" << sourceEdge << std::endl;
        std::cout << "  target: (" << targetX << "," << targetY << ") snapIdx=" << targetSnapIndex << " edge=" << targetEdge << std::endl;
        std::cout << "  bendCount=" << bendCount << std::endl;
    }
};

// TDD RED: Snap point drag must preserve user's intended edge and position
// When user drags a snap point along the same edge, the edge type should NOT change
TEST_F(DemoApiTest, SnapPointDrag_MustPreserveEdgeAndPosition) {
    // Get initial layout
    std::string layoutBefore = sendCommand("get_layout");
    json jsonBefore = json::parse(layoutBefore);

    // Get Edge 0 (idle -> running) initial state
    EdgeLayoutInfo edgeBefore = EdgeLayoutInfo::fromJson(jsonBefore, 0);
    edgeBefore.print("\n=== Edge 0 BEFORE snap drag ===");

    // Use test_snap_drag to move target snap point by 40px right
    // test_snap_drag <edge_id> <is_source:0|1> <dx> <dy>
    std::string dragResult = sendCommand("test_snap_drag 0 0 40 0");  // 0=target, dx=40, dy=0
    std::cout << "\ntest_snap_drag result: " << dragResult << std::endl;

    // Get layout after snap drag
    std::string layoutAfter = sendCommand("get_layout");
    json jsonAfter = json::parse(layoutAfter);

    EdgeLayoutInfo edgeAfter = EdgeLayoutInfo::fromJson(jsonAfter, 0);
    edgeAfter.print("\n=== Edge 0 AFTER snap drag ===");

    EdgePath pathAfter = EdgePath::fromJson(jsonAfter, 0);

    int failures = 0;
    std::ostringstream errors;

    // CONSTRAINT 1: Target edge should be PRESERVED
    if (edgeAfter.targetEdge != edgeBefore.targetEdge) {
        ++failures;
        errors << "[FAIL] Target edge changed from " << edgeBefore.targetEdge
               << " to " << edgeAfter.targetEdge
               << " - user's intended edge should be preserved!\n";
    }

    // CONSTRAINT 2: Target position should move in the requested direction
    float expectedX = edgeBefore.targetX + 40.0f;
    float positionError = std::abs(edgeAfter.targetX - expectedX);
    if (positionError > 30.0f) {  // Allow some grid snapping tolerance
        ++failures;
        errors << "[FAIL] Target X position error too large. "
               << "Expected ~" << expectedX << ", Got " << edgeAfter.targetX
               << ", Error=" << positionError << "\n";
    }

    // CONSTRAINT 3: All segments must be ORTHOGONAL
    std::string orthoErrors = pathAfter.checkSegments();
    if (!pathAfter.isOrthogonal()) {
        ++failures;
        errors << "[FAIL] Edge has diagonal segments!\n" << orthoErrors;
    }

    // CONSTRAINT 4: Edge must have bend points for non-trivial routing
    if (pathAfter.points.size() <= 2) {
        // Check if source and target form a diagonal
        float dx = std::abs(pathAfter.points.back().first - pathAfter.points.front().first);
        float dy = std::abs(pathAfter.points.back().second - pathAfter.points.front().second);
        if (dx > 0.1f && dy > 0.1f) {
            ++failures;
            errors << "[FAIL] Edge has no bends but forms diagonal - A* failed to find path!\n";
        }
    }

    std::cout << "\n========== SUMMARY ==========" << std::endl;
    std::cout << "Total failures: " << failures << std::endl;

    EXPECT_EQ(failures, 0)
        << "Snap point drag must satisfy all constraints!\n"
        << errors.str();
}

// TDD RED: All edges must remain orthogonal after snap point drag
TEST_F(DemoApiTest, SnapPointDrag_AllEdgesMustRemainOrthogonal) {
    // Drag snap point of edge 0
    sendCommand("test_snap_drag 0 0 40 0");  // Move target snap right

    // Get layout after snap drag
    std::string layoutStr = sendCommand("get_layout");
    json layout = json::parse(layoutStr);

    bool allOrthogonal = true;
    std::ostringstream errors;

    for (const auto& edge : layout["edgeLayouts"]) {
        int edgeId = edge["id"].get<int>();
        EdgePath path = EdgePath::fromJson(layout, edgeId);

        if (!path.isOrthogonal()) {
            allOrthogonal = false;
            errors << "Edge " << edgeId << " is NOT orthogonal:\n";
            errors << path.checkSegments();
        }
    }

    EXPECT_TRUE(allOrthogonal)
        << "All edges must be orthogonal after snap point drag!\n"
        << errors.str();
}
