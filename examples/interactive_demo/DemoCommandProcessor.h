#pragma once

#include "DemoState.h"
#include "CommandServer.h"
#include <arborvia/layout/interactive/ManualLayoutManager.h>
#include <arborvia/layout/interactive/PathRoutingCoordinator.h>
#include <functional>
#include <string>
#include <vector>

namespace arborvia {

/// Handles TCP command processing for the interactive demo
///
/// Extracts command processing logic from InteractiveDemo for better separation of concerns.
/// Uses callbacks for operations that require demo context.
class DemoCommandProcessor {
public:
    /// Callback types for operations requiring demo context
    using DoLayoutCallback = std::function<void()>;
    using RerouteEdgesCallback = std::function<void()>;
    using StartAsyncOptimizationCallback = std::function<void(const std::vector<EdgeId>&)>;
    using GetLogCallback = std::function<std::string(const std::string&, int)>;
    using ClearLogCallback = std::function<void()>;
    using WaitIdleCallback = std::function<bool(int)>;
    using InitSCXMLCallback = std::function<bool(const std::string&)>;
    using LoadSCXMLTestCallback = std::function<bool(int)>;
    using NextTestCallback = std::function<void()>;
    using PrevTestCallback = std::function<void()>;
    using GetSCXMLTestCountCallback = std::function<int()>;
    using GetCurrentTestInfoCallback = std::function<std::string()>;

    /// Constructor
    DemoCommandProcessor(
        CommandServer& commandServer,
        std::shared_ptr<ManualLayoutManager> manualManager,
        std::shared_ptr<PathRoutingCoordinator> routingCoordinator);

    /// Set callbacks
    void setDoLayoutCallback(DoLayoutCallback cb) { doLayoutCallback_ = std::move(cb); }
    void setRerouteEdgesCallback(RerouteEdgesCallback cb) { rerouteEdgesCallback_ = std::move(cb); }
    void setStartAsyncOptimizationCallback(StartAsyncOptimizationCallback cb) { startAsyncOptCallback_ = std::move(cb); }
    void setGetLogCallback(GetLogCallback cb) { getLogCallback_ = std::move(cb); }
    void setClearLogCallback(ClearLogCallback cb) { clearLogCallback_ = std::move(cb); }
    void setWaitIdleCallback(WaitIdleCallback cb) { waitIdleCallback_ = std::move(cb); }
    void setInitSCXMLCallback(InitSCXMLCallback cb) { initSCXMLCallback_ = std::move(cb); }
    void setLoadSCXMLTestCallback(LoadSCXMLTestCallback cb) { loadSCXMLTestCallback_ = std::move(cb); }
    void setNextTestCallback(NextTestCallback cb) { nextTestCallback_ = std::move(cb); }
    void setPrevTestCallback(PrevTestCallback cb) { prevTestCallback_ = std::move(cb); }
    void setGetSCXMLTestCountCallback(GetSCXMLTestCountCallback cb) { getSCXMLTestCountCallback_ = std::move(cb); }
    void setGetCurrentTestInfoCallback(GetCurrentTestInfoCallback cb) { getCurrentTestInfoCallback_ = std::move(cb); }

    /// Process a single command
    /// @param cmd The command to process
    /// @param state Demo state (will be modified for some commands)
    /// @return true if command was handled
    bool processCommand(const Command& cmd, DemoState& state);

    /// Get/set pause state
    bool isPaused() const { return paused_; }
    const std::string& pauseMessage() const { return pauseMessage_; }
    bool shouldQuit() const { return shouldQuit_; }

private:
    // Command handlers
    void handleDrag(const Command& cmd, DemoState& state);
    void handleSetPos(const Command& cmd, DemoState& state);
    void handleTestSnapDrag(const Command& cmd, DemoState& state);
    void handleTestSnapPreview(const Command& cmd, DemoState& state);
    void handleTestCoopReroute(const Command& cmd, DemoState& state);
    void handleTestCoopPair(const Command& cmd, DemoState& state);
    void handleTestDrag(const Command& cmd, DemoState& state);
    void handlePause(const Command& cmd);
    void handleResume();
    void handleGetState(DemoState& state);
    void handleGetLayout(DemoState& state);
    void handleCheckPenetration(DemoState& state);
    void handleGetLog(const Command& cmd);
    void handleClearLog();
    void handleWaitIdle(const Command& cmd);
    void handleGetStateFull(DemoState& state);
    void handleAStarViz(const Command& cmd, DemoState& state);
    void handleAStarVizOff(DemoState& state);
    void handleSCXMLLoad(const Command& cmd);
    void handleSCXMLNext();
    void handleSCXMLPrev();
    void handleSCXMLList();
    void handleSCXMLInfo();
    void handleQuit();

    // Helper methods
    std::string formatLayoutAsJson(DemoState& state);
    bool checkEdgePenetration(const EdgeLayout& edge, const std::unordered_map<NodeId, NodeLayout>& nodeLayouts);

    // References
    CommandServer& commandServer_;
    std::shared_ptr<ManualLayoutManager> manualManager_;
    std::shared_ptr<PathRoutingCoordinator> routingCoordinator_;

    // State
    bool paused_ = false;
    std::string pauseMessage_;
    bool shouldQuit_ = false;

    // Callbacks
    DoLayoutCallback doLayoutCallback_;
    RerouteEdgesCallback rerouteEdgesCallback_;
    StartAsyncOptimizationCallback startAsyncOptCallback_;
    GetLogCallback getLogCallback_;
    ClearLogCallback clearLogCallback_;
    WaitIdleCallback waitIdleCallback_;
    InitSCXMLCallback initSCXMLCallback_;
    LoadSCXMLTestCallback loadSCXMLTestCallback_;
    NextTestCallback nextTestCallback_;
    PrevTestCallback prevTestCallback_;
    GetSCXMLTestCountCallback getSCXMLTestCountCallback_;
    GetCurrentTestInfoCallback getCurrentTestInfoCallback_;
};

}  // namespace arborvia
