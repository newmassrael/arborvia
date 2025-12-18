#pragma once

#include "DemoState.h"
#include <arborvia/layout/interactive/UserLayoutController.h>
#include <imgui.h>
#include <functional>

namespace arborvia {

/// Actions that input handler requests from the demo
enum class InputAction {
    None,
    DoLayout,           ///< Full layout recalculation needed
    RerouteEdges,       ///< Edge rerouting needed
    StartAsyncOptimization  ///< Async optimization should be triggered
};

/// Result of input processing
struct InputResult {
    InputAction action = InputAction::None;
    bool stateChanged = false;  ///< True if any state was modified
};

/// Handles user input for the interactive demo
///
/// Extracts input handling logic from InteractiveDemo for better separation of concerns.
/// Uses DemoState for state access and modification.
class DemoInputHandler {
public:
    /// Callback types for operations that require demo context
    using DoLayoutCallback = std::function<void()>;
    using RerouteEdgesCallback = std::function<void()>;
    using StartDragCallback = std::function<void(NodeId)>;
    using EndDragCallback = std::function<void(NodeId)>;  // Called when drag ends (mouse release)
    using ValidateDragCallback = std::function<bool(NodeId, Point)>;  // Returns true if valid position

    /// Constructor
    /// @param manualManager Manual layout manager for bend point operations
    explicit DemoInputHandler(std::shared_ptr<ManualLayoutManager> manualManager);

    /// Set callbacks for operations
    void setDoLayoutCallback(DoLayoutCallback cb) { doLayoutCallback_ = std::move(cb); }
    void setRerouteEdgesCallback(RerouteEdgesCallback cb) { rerouteEdgesCallback_ = std::move(cb); }
    void setStartDragCallback(StartDragCallback cb) { startDragCallback_ = std::move(cb); }
    void setEndDragCallback(EndDragCallback cb) { endDragCallback_ = std::move(cb); }
    void setValidateDragCallback(ValidateDragCallback cb) { validateDragCallback_ = std::move(cb); }

    /// Process input and update state
    /// @param state Current demo state (will be modified)
    /// @param io ImGui IO for input access
    /// @return Result indicating what actions the demo should take
    InputResult processInput(DemoState& state, const ImGuiIO& io);

private:
    // Input processing phases
    void handleZoom(DemoState& state, const ImGuiIO& io);
    void handlePan(DemoState& state, const ImGuiIO& io);
    void updateHoveredNode(DemoState& state, const Point& graphMouse);
    void updateHoveredSnapPoint(DemoState& state, const Point& graphMouse);
    void updateHoveredBendPoint(DemoState& state, const Point& graphMouse);
    void updateHoveredEdge(DemoState& state, const Point& graphMouse);
    void handleClicks(DemoState& state, const ImGuiIO& io, const Point& graphMouse);
    void handleDragThreshold(DemoState& state, const ImGuiIO& io);
    void handleEmptyAreaPan(DemoState& state, const ImGuiIO& io);
    void handleMouseRelease(DemoState& state, const ImGuiIO& io, const Point& graphMouse);
    void handleSnapPointDrag(DemoState& state, const ImGuiIO& io, const Point& graphMouse);
    void handleBendPointDrag(DemoState& state, const ImGuiIO& io, const Point& graphMouse);
    void handleNodeDrag(DemoState& state, const ImGuiIO& io, const Point& graphMouse);
    void handleKeyboard(DemoState& state, const ImGuiIO& io);

    // Helpers
    Point screenToWorld(const DemoState& state, const ImVec2& screen) const;

    // State
    std::shared_ptr<ManualLayoutManager> manualManager_;

    // Callbacks
    DoLayoutCallback doLayoutCallback_;
    RerouteEdgesCallback rerouteEdgesCallback_;
    StartDragCallback startDragCallback_;
    EndDragCallback endDragCallback_;
    ValidateDragCallback validateDragCallback_;

    // Constants
    static constexpr float SNAP_HIT_RADIUS = 8.0f;
    static constexpr float BEND_HIT_RADIUS = 6.0f;
    static constexpr float POINT_NODE_RADIUS = 14.0f;  // Larger than visual radius (10) for easier selection
};

}  // namespace arborvia
