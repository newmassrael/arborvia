#pragma once

#include "DemoState.h"
#include <arborvia/layout/interactive/UserLayoutController.h>
#include <functional>
#include <string>

namespace arborvia {

// Forward declarations
namespace test::scxml {
    class SCXMLTestLoader;
    struct TestInfo;
}

/// Actions requested by UI panel
enum class UIAction {
    None,
    DoLayout,
    ReRouteEdges,
    SaveLayout,
    LoadLayout
};

/// Result of UI rendering
struct UIResult {
    UIAction action = UIAction::None;
    bool stateChanged = false;
};

/// Renders the ImGui control panel for the interactive demo
///
/// Extracts UI rendering logic from InteractiveDemo for better separation of concerns.
class DemoUIPanel {
public:
    /// Callback types
    using DoLayoutCallback = std::function<void()>;
    using ReRouteEdgesCallback = std::function<void()>;
    using SaveLayoutCallback = std::function<void(const std::string&)>;
    using LoadLayoutCallback = std::function<void(const std::string&)>;
    using LoadSCXMLTestCallback = std::function<void(size_t)>;
    using NextTestCallback = std::function<void()>;
    using PrevTestCallback = std::function<void()>;
    using ExitSCXMLModeCallback = std::function<void()>;
    using SetNodeTypeCallback = std::function<bool(NodeId, NodeType)>;
    using CanConvertToPointCallback = std::function<bool(NodeId)>;
    using CanConvertToRegularCallback = std::function<bool(NodeId)>;

    /// Constructor
    explicit DemoUIPanel(std::shared_ptr<ManualLayoutManager> manualManager);

    /// Set callbacks
    void setDoLayoutCallback(DoLayoutCallback cb) { doLayoutCallback_ = std::move(cb); }
    void setReRouteEdgesCallback(ReRouteEdgesCallback cb) { reRouteEdgesCallback_ = std::move(cb); }
    void setSaveLayoutCallback(SaveLayoutCallback cb) { saveLayoutCallback_ = std::move(cb); }
    void setLoadLayoutCallback(LoadLayoutCallback cb) { loadLayoutCallback_ = std::move(cb); }
    void setLoadSCXMLTestCallback(LoadSCXMLTestCallback cb) { loadSCXMLTestCallback_ = std::move(cb); }
    void setNextTestCallback(NextTestCallback cb) { nextTestCallback_ = std::move(cb); }
    void setPrevTestCallback(PrevTestCallback cb) { prevTestCallback_ = std::move(cb); }
    void setExitSCXMLModeCallback(ExitSCXMLModeCallback cb) { exitSCXMLModeCallback_ = std::move(cb); }
    void setSetNodeTypeCallback(SetNodeTypeCallback cb) { setNodeTypeCallback_ = std::move(cb); }
    void setCanConvertToPointCallback(CanConvertToPointCallback cb) { canConvertToPointCallback_ = std::move(cb); }
    void setCanConvertToRegularCallback(CanConvertToRegularCallback cb) { canConvertToRegularCallback_ = std::move(cb); }

    /// Set SCXML loader reference for test list display
    void setSCXMLLoader(test::scxml::SCXMLTestLoader* loader) { scxmlLoader_ = loader; }

    /// Render the UI panel
    /// @param state Current demo state (will be modified for UI controls)
    /// @return Result indicating what actions were requested
    UIResult render(DemoState& state);

private:
    // UI sections
    void renderEdgeRoutingOptions(DemoState& state, bool& changed);
    void renderNodeInfo(DemoState& state);
    void renderSelectedNodeInfo(DemoState& state);
    void renderDisplayOptions(DemoState& state);
    void renderViewControls(DemoState& state);
    void renderSCXMLSection(DemoState& state);
    void renderActionButtons(DemoState& state);

    // State
    std::shared_ptr<ManualLayoutManager> manualManager_;
    test::scxml::SCXMLTestLoader* scxmlLoader_ = nullptr;

    // Callbacks
    DoLayoutCallback doLayoutCallback_;
    ReRouteEdgesCallback reRouteEdgesCallback_;
    SaveLayoutCallback saveLayoutCallback_;
    LoadLayoutCallback loadLayoutCallback_;
    LoadSCXMLTestCallback loadSCXMLTestCallback_;
    NextTestCallback nextTestCallback_;
    PrevTestCallback prevTestCallback_;
    ExitSCXMLModeCallback exitSCXMLModeCallback_;
    SetNodeTypeCallback setNodeTypeCallback_;
    CanConvertToPointCallback canConvertToPointCallback_;
    CanConvertToRegularCallback canConvertToRegularCallback_;
};

}  // namespace arborvia
