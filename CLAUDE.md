# Arborvia Development Guide

## Interactive Demo Testing

### Build
```bash
cmake --build build --target interactive_demo -j4
```

### Run Demo
```bash
./build/examples/interactive_demo
```
Demo listens on TCP port 9999 (configurable with `--port=XXXX` or `-p XXXX`).

### TCP Commands

| Command | Description | Example |
|---------|-------------|---------|
| `drag <nodeId> <dx> <dy>` | Drag node by offset | `drag 1 420 -140` |
| `set_pos <id> <x> <y>` | Set node position (triggers A* optimization) | `set_pos 1 500 200` |
| `pause [msg]` | Pause demo with optional message | `pause "Testing"` |
| `resume` | Resume paused demo | `resume` |
| `get_state` | Get brief node/edge count and positions | `get_state` |
| `get_state_full` | Get state with routing status | `get_state_full` |
| `get_layout` | Get full layout as JSON | `get_layout` |
| `check_penetration` | Check if edges penetrate nodes | `check_penetration` |
| `get_log` | Get captured debug log (non-destructive) | `get_log` |
| `clear_log` | Clear log buffer | `clear_log` |
| `wait_idle [timeout_ms]` | Wait for optimization to complete | `wait_idle 10000` |
| `astar_viz [edgeId]` | Enable A* debug visualization | `astar_viz 0` |
| `astar_viz_off` | Disable A* debug visualization | `astar_viz_off` |
| `quit` | Exit demo | `quit` |

### Test Workflow

**User runs demo manually**, Claude sends commands via nc:
```bash
# User runs demo in their terminal
./build/examples/interactive_demo

# Claude sends commands via nc (do NOT start demo)
echo "drag 1 420 -140" | nc -q 1 localhost 9999
echo "get_log" | nc -q 1 localhost 9999 | grep -E "(DIAGONAL|RETRY)"
```

**Important**: Claude must NOT run the demo. Only send TCP commands after user confirms demo is running.

### Performance Testing Workflow

```bash
# 1. Clear logs
echo "clear_log" | nc -q 1 localhost 9999

# 2. Move node (triggers full A* optimization cycle)
echo "set_pos 4 400 400" | nc -q 1 localhost 9999

# 3. Wait for optimization to complete
echo "wait_idle 10000" | nc -q 1 localhost 9999

# 4. Get performance logs (save to file for parsing)
echo "get_log" | nc -q 1 localhost 9999 > /tmp/demo_log.txt

# 5. Parse and filter logs
cat /tmp/demo_log.txt | sed 's/\\n/\n/g' | grep -E "PERF"
```

### Performance Log Patterns

| Pattern | Description |
|---------|-------------|
| `[PERF] optimizer->optimize()` | Total A* optimization time |
| `[PERF-AStar] Pass N/3` | Multi-pass optimization progress |
| `[PERF-evalCombo] buildNodes` | ObstacleMap build time |
| `[PERF-evalCombo] pathFind` | A* pathfinding time (main bottleneck) |
| `[PERF-evalCombo] penalty` | Penalty calculation time |
| `[PERF] TOTAL` | Total edge routing time |

Example output:
```
[PERF-evalCombo] After 42 calls:
  buildNodes: 0 ms
  pathFind: 175 ms    <- Main bottleneck
  penalty: 0 ms
[PERF] optimizer->optimize(): 85 ms (8 edges)
```

### Handling Large Logs

```bash
# Filter by pattern
echo "get_log" | nc -q 1 localhost 9999 | grep -E "(DIAGONAL|RETRY|FAILED)"

# Filter by specific Edge
echo "get_log" | nc -q 1 localhost 9999 | grep "Edge 0"

# Save to file for analysis
echo "get_log" | nc -q 1 localhost 9999 > /tmp/log.txt
grep "DIAGONAL" /tmp/log.txt

# Count pattern occurrences
echo "get_log" | nc -q 1 localhost 9999 | grep -c "DIAGONAL"

# Log is non-destructive, can be retrieved multiple times
echo "get_log" | nc -q 1 localhost 9999 | grep "PERF"
echo "get_log" | nc -q 1 localhost 9999 | grep "evalCombo"
```

### Key Log Patterns
- `[Phase4] Edge X DIAGONAL detected!` - Diagonal detected, retry triggered
- `[A* RETRY]` - Retry system active
- `[A*] Edge X FAILED!` - A* pathfinding failed
- `[A*] Edge X SUCCESS` - A* pathfinding succeeded

## Drag Algorithm Options

Located in `include/arborvia/layout/LayoutOptions.h`:

| DragAlgorithm | Description |
|---------------|-------------|
| `None` | No optimization during drag (use existing routing) |
| `Geometric` | Fast geometric path prediction (no A* pathfinding) |
| `AStar` | A* pathfinding for optimal obstacle-avoiding paths |
| `HideUntilDrop` | Hide edges during drag, calculate with A* on drop (default) |

| PostDragAlgorithm | Description |
|-------------------|-------------|
| `None` | No post-drag optimization |
| `AStar` | A* pathfinding after drop (default) |

### HideUntilDrop Flow
1. Drag start: edges hidden
2. During drag: edges remain hidden, no recalculation
3. Drop: A* runs immediately (0ms debounce)
4. A* completes: `onOptimizationComplete()` clears `affectedEdges_`, edges become visible

## Development Rules

### Algorithm Separation Rule
**During drag, only use the selected drag algorithm. Do not bring in other algorithms.**

During drag operations:
- Use ONLY the selected drag algorithm (e.g., `DragAlgorithm::Geometric`)
- Do NOT bring in other algorithms (like A*) to create orthogonal paths
- A* pathfinding should ONLY run after drop (via debounce callback with `PostDragAlgorithm::AStar`)

This ensures:
1. Consistent behavior during drag (no algorithm mixing)
2. Fast drag response (Geometric is faster than A*)
3. Clean separation between drag-time prediction and drop-time optimization

### No Simpler Fix Rule
**Do not choose simpler fixes. Follow the correct architecture for long-term maintainability.**

When fixing issues:
- Do NOT duplicate algorithm logic (e.g., copying geometric path code to EdgeRouting)
- Do NOT add inline workarounds that bypass the optimizer
- ALWAYS extend the interface (`IEdgeOptimizer`) when new capabilities are needed
- Each optimizer implements its own path generation strategy

Example (correct):
```cpp
// IEdgeOptimizer interface
virtual void regenerateBendPoints(edges, edgeLayouts, nodeLayouts) = 0;

// GeometricEdgeOptimizer uses createPathWithObstacleAvoidance
// AStarEdgeOptimizer uses pathFinder_->findPath

// EdgeRouting calls optimizer
edgeOptimizer_->regenerateBendPoints(edges, edgeLayouts, nodeLayouts);
```

Example (wrong - do NOT do this):
```cpp
// EdgeRouting.cpp - inline geometric path (duplicates GeometricEdgeOptimizer logic)
float midY = (src.y + tgt.y) * 0.5f;
newBends.push_back({{src.x, midY}});
```

## Current Work: A* Retry System

### Problem
When node is dragged, edges may fail A* pathfinding and create DIAGONAL segments.

### Solution Architecture
3-layer retry system:
1. **Snap Index Retry** - Try alternative snap indices on same NodeEdge
2. **Neighbor Adjustment** - Move neighbor edges to create space
3. **NodeEdge Switch** - Try different NodeEdge combinations (16 total)
