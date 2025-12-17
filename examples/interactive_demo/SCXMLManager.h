#pragma once

#include "DemoState.h"
#include "SCXMLTestLoader.h"
#include "SCXMLGraph.h"
#include "SCXMLTypes.h"
#include <functional>
#include <memory>
#include <string>

namespace arborvia {

/// Manages SCXML test visualization for the interactive demo
///
/// Extracts SCXML-related logic from InteractiveDemo for better separation of concerns.
/// Handles loading, navigation, and node type queries for SCXML tests.
class SCXMLManager {
public:
    /// Callback types
    using OnTestLoadedCallback = std::function<void(Graph& graph)>;
    using OnModeExitCallback = std::function<void()>;

    /// Constructor
    /// @param scxmlState Reference to SCXML state in DemoState (for direct updates)
    explicit SCXMLManager(DemoState::SCXMLState& scxmlState);

    /// Initialize SCXML test loader from a base path
    /// @param basePath Path to SCXML test directory containing index.json
    /// @return true if initialization succeeded
    bool init(const std::string& basePath);

    /// Load a specific test by index
    /// @param index Test index (0-based)
    /// @return true if loading succeeded
    bool loadTest(size_t index);

    /// Navigate to next test
    void nextTest();

    /// Navigate to previous test
    void prevTest();

    /// Exit SCXML mode and reset state
    void exitMode();

    /// Check if SCXML mode is currently active
    bool isModeActive() const;

    /// Get total number of available tests
    size_t getTestCount() const;

    /// Get info about the currently loaded test
    const test::scxml::TestInfo* getCurrentTestInfo() const;

    /// Get the test loader (for UI panel)
    test::scxml::SCXMLTestLoader* getLoader();

    /// Get the current SCXML graph (for rendering)
    test::scxml::SCXMLGraph* getCurrentGraph();

    /// Check if a node can be converted to Point type
    /// @param nodeId Node to check
    /// @return true if conversion is allowed (only Final nodes in SCXML mode)
    bool canConvertToPoint(NodeId nodeId) const;

    /// Check if a node can be converted to Regular type
    /// @param nodeId Node to check
    /// @return true if conversion is allowed (only Final nodes in SCXML mode)
    bool canConvertToRegular(NodeId nodeId) const;

    /// Get SCXML ID for a node (for label display)
    /// @param nodeId Node to query
    /// @return SCXML ID string, or empty if not in SCXML mode
    std::string getScxmlId(NodeId nodeId) const;

    /// Set callback for when a test is loaded
    void setOnTestLoadedCallback(OnTestLoadedCallback cb) { onTestLoadedCallback_ = std::move(cb); }

    /// Set callback for when SCXML mode is exited
    void setOnModeExitCallback(OnModeExitCallback cb) { onModeExitCallback_ = std::move(cb); }

private:
    DemoState::SCXMLState& scxmlState_;
    std::unique_ptr<test::scxml::SCXMLTestLoader> loader_;
    std::unique_ptr<test::scxml::SCXMLGraph> graph_;

    OnTestLoadedCallback onTestLoadedCallback_;
    OnModeExitCallback onModeExitCallback_;
};

}  // namespace arborvia
