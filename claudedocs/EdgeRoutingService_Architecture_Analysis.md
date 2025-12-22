# EdgeRoutingService Architecture Analysis

**Date:** 2025-12-22
**Component:** EdgeRoutingService
**Location:** `include/arborvia/layout/api/EdgeRoutingService.h`, `src/layout/api/EdgeRoutingService.cpp`

## Executive Summary

EdgeRoutingService is a **facade service** that provides a unified, constraint-validated entry point for all edge routing operations. It orchestrates EdgeRouting (the core routing engine) and ConstraintGateway (validation layer) to ensure all edge layouts pass validation before being returned to clients.

**Key Design Pattern:** Facade + Service Layer
**Primary Responsibility:** Constraint-validated edge routing coordination
**Coupling Level:** Medium (deliberate service layer coupling)

---

## 1. Dependency Graph

### 1.1 Direct Dependencies (What EdgeRoutingService Depends On)

```
EdgeRoutingService
├── EdgeRouting (owned, unique_ptr)
│   └── Core routing engine for channel allocation, snap distribution, optimization
├── ConstraintGateway (owned, unique_ptr)
│   └── Validation layer for constraint enforcement
├── IPathFinder (shared_ptr, injected)
│   └── A* pathfinding for obstacle avoidance
├── Graph (parameter)
│   └── Graph structure for initial routing
└── LayoutOptions (parameter)
    └── Configuration for routing behavior

Key Includes:
- arborvia/layout/config/LayoutResult.h
- arborvia/layout/config/LayoutOptions.h
- arborvia/layout/constraints/ValidatedEdgeLayout.h
- arborvia/layout/api/IPathFinder.h
- arborvia/core/Graph.h
- arborvia/core/GeometryUtils.h
- arborvia/common/Logger.h
- sugiyama/routing/EdgeRouting.h (internal)
- arborvia/layout/constraints/ConstraintGateway.h
- arborvia/layout/constraints/IEdgeConstraint.h
- arborvia/layout/util/LayoutUtils.h
```

### 1.2 Reverse Dependencies (Who Depends On EdgeRoutingService)

```
EdgeRoutingService
├── SugiyamaLayout::routeEdges()
│   └── Phase 5 of Sugiyama layout algorithm
│   └── Calls: routeInitial()
│   └── Usage: Initial edge routing with validation
│
└── LayoutController::updateEdgeRouting()
    └── Interactive node movement handler
    └── Calls: updateAfterNodeMove(), updateAfterSnapMove()
    └── Usage: Edge rerouting after user interaction

Key Clients (2 total):
1. SugiyamaLayout (src/layout/SugiyamaLayout.cpp:366-388)
2. LayoutController (src/layout/api/LayoutController.cpp:346-366)
```

### 1.3 Dependency Flow Diagram

```
┌──────────────────────────────────────────────────────────────┐
│                    CLIENT LAYER                              │
│  ┌───────────────────┐      ┌──────────────────────┐        │
│  │ SugiyamaLayout    │      │ LayoutController     │        │
│  │ (initial layout)  │      │ (interactive drag)   │        │
│  └────────┬──────────┘      └──────────┬───────────┘        │
│           │                            │                     │
└───────────┼────────────────────────────┼─────────────────────┘
            │                            │
            └────────────┬───────────────┘
                         │
            ┌────────────▼─────────────┐
            │  EdgeRoutingService      │  FACADE/SERVICE LAYER
            │  (orchestration + val)   │
            └────────────┬─────────────┘
                         │
        ┌────────────────┼────────────────┐
        │                │                │
┌───────▼────────┐ ┌────▼────────┐ ┌────▼──────────┐
│ EdgeRouting    │ │ Constraint  │ │ IPathFinder   │
│ (routing       │ │ Gateway     │ │ (A* search)   │
│  engine)       │ │ (validation)│ │               │
└───────┬────────┘ └────┬────────┘ └───────────────┘
        │               │
        │               │
┌───────▼───────────────▼─────────────────────────────┐
│            CORE LAYER                                │
│  Graph, NodeLayout, EdgeLayout, LayoutOptions       │
└──────────────────────────────────────────────────────┘
```

---

## 2. Responsibility Distribution Analysis

### 2.1 Single Responsibility Principle (SRP) Compliance

**EdgeRoutingService Responsibilities:**
1. Orchestrate edge routing workflow (channel → snap → optimize)
2. Validate all edge layouts before returning to clients
3. Convert raw EdgeLayout to ValidatedEdgeLayout
4. Ensure label positions are calculated
5. Map PostDragAlgorithm to DragAlgorithm for post-drop processing

**SRP Assessment:** ✅ **COMPLIANT**

EdgeRoutingService has a single high-level responsibility: **"Provide constraint-validated edge routing as a service."** All lower-level responsibilities (routing algorithms, validation logic, pathfinding) are delegated to owned/injected dependencies.

### 2.2 Responsibility Delegation Matrix

| Responsibility | EdgeRoutingService | EdgeRouting | ConstraintGateway |
|----------------|-------------------|-------------|-------------------|
| Channel allocation | Delegates | ✓ Implements | - |
| Snap distribution | Delegates | ✓ Implements | - |
| A* optimization | Delegates | ✓ Implements | - |
| Constraint validation | Delegates | - | ✓ Implements |
| Label position calc | Calls utility | - | - |
| Workflow orchestration | ✓ Implements | - | - |
| Result packaging | ✓ Implements | - | - |

### 2.3 Method-Level Responsibility Map

```cpp
EdgeRoutingService::routeInitial()
├── EdgeRouting::route()               // Step 1: Initial routing
├── EdgeRouting::distributeAutoSnapPoints()  // Step 2: Snap distribution
├── EdgeRouting::optimizeRouting()     // Step 3: Optimization
├── ensureLabelPositions()             // Step 4: Label calculation
└── validateAndWrap()                  // Step 5: Validation + wrapping

EdgeRoutingService::updateAfterNodeMove()
├── applyPostDragAlgorithm()           // Convert PostDrag → Drag
├── EdgeRouting::updateEdgeRoutingWithOptimization()  // First pass
├── LayoutUtils::edgePassesThroughNode()  // Detect pass-through edges
├── EdgeRouting::updateEdgeRoutingWithOptimization()  // Second pass
├── ensureLabelPositions()             // Label calculation
└── validateAndWrap()                  // Validation + wrapping

EdgeRoutingService::updateAfterSnapMove()
├── applyPostDragAlgorithm()           // Convert PostDrag → Drag
├── EdgeRouting::updateEdgeRoutingWithOptimization()  // Update edges
├── ensureLabelPositions()             // Label calculation
└── validateAndWrap()                  // Validation + wrapping
```

---

## 3. Coupling Analysis

### 3.1 Afferent Coupling (Ca) - Incoming Dependencies

**Afferent Coupling = 2 (LOW)**

Only 2 classes depend on EdgeRoutingService:
- SugiyamaLayout
- LayoutController

**Assessment:** ✅ **LOW COUPLING** - Service is used by limited, well-defined clients.

### 3.2 Efferent Coupling (Ce) - Outgoing Dependencies

**Efferent Coupling = 5 (MEDIUM)**

EdgeRoutingService depends on:
1. EdgeRouting (owned)
2. ConstraintGateway (owned)
3. IPathFinder (injected interface)
4. Graph (parameter)
5. LayoutOptions (parameter)

**Assessment:** ✅ **ACCEPTABLE** - Dependencies are through abstractions (interfaces/parameters) or owned components.

### 3.3 Instability Metric (I = Ce / (Ca + Ce))

```
I = 5 / (2 + 5) = 0.71
```

**Instability = 0.71 (HIGH)**

This indicates EdgeRoutingService is more dependent on other modules than other modules depend on it. This is **EXPECTED AND CORRECT** for a service layer:
- Service layers should depend on concrete implementations (EdgeRouting, ConstraintGateway)
- Clients should depend on stable interfaces (EdgeRoutingService is stable)
- High instability at service layer is acceptable because it isolates change

### 3.4 Coupling Classification

```
EdgeRoutingService Coupling Types:

TIGHT COUPLING (Deliberate):
├── EdgeRouting (concrete class, owned)
│   └── Reason: Service owns routing engine lifecycle
└── ConstraintGateway (concrete class, owned)
    └── Reason: Service owns validation lifecycle

LOOSE COUPLING (Interface):
├── IPathFinder (interface, injected)
│   └── Reason: Algorithm swappable via DI
└── Graph (parameter dependency)
    └── Reason: Read-only data structure

PARAMETER COUPLING (Acceptable):
├── LayoutOptions (value object)
└── NodeLayout/EdgeLayout (data structures)
```

### 3.5 Coupling Hotspots

**Potential Issue:** EdgeRoutingService knows about EdgeRouting implementation details (e.g., `route()`, `distributeAutoSnapPoints()`, `optimizeRouting()` methods).

**Mitigation:** This is acceptable for a facade service because:
1. EdgeRoutingService IS the public API for edge routing
2. EdgeRouting is an internal implementation (not in public API headers)
3. Clients don't need to know EdgeRouting exists
4. Service layer can change EdgeRouting implementation without affecting clients

---

## 4. Circular Dependency Analysis

### 4.1 Direct Circular Dependencies

**Result:** ✅ **NONE DETECTED**

```
Dependency Chain Analysis:

EdgeRoutingService → EdgeRouting → [no back reference]
EdgeRoutingService → ConstraintGateway → [no back reference]
EdgeRoutingService → IPathFinder → [interface, no back reference]

SugiyamaLayout → EdgeRoutingService → [no back reference]
LayoutController → EdgeRoutingService → [no back reference]
```

### 4.2 Indirect Circular Dependencies

**Potential Path:** EdgeRoutingService → LayoutUtils → EdgeRouting

Analysis:
```
EdgeRoutingService.cpp includes:
├── arborvia/layout/util/LayoutUtils.h  // For edgePassesThroughNode()
└── sugiyama/routing/EdgeRouting.h     // For owned EdgeRouting

LayoutUtils.cpp includes:
└── sugiyama/routing/EdgeRouting.h     // For updateEdgePositions()

Result: SHARED DEPENDENCY, not circular
Both EdgeRoutingService and LayoutUtils depend on EdgeRouting,
but neither EdgeRouting nor EdgeRoutingService depend on LayoutUtils
(EdgeRoutingService only calls LayoutUtils::edgePassesThroughNode,
which is a static utility function).
```

**Result:** ✅ **NO CIRCULAR DEPENDENCIES**

### 4.3 Dependency Graph (Acyclic Verification)

```
┌────────────────────────────────────────┐
│          APPLICATION LAYER             │
│  SugiyamaLayout, LayoutController      │
└────────────┬───────────────────────────┘
             │
             ▼
┌────────────────────────────────────────┐
│          SERVICE LAYER                 │
│  EdgeRoutingService                    │
└────────────┬───────────────────────────┘
             │
             ▼
┌────────────────────────────────────────┐
│          ENGINE LAYER                  │
│  EdgeRouting, ConstraintGateway        │
└────────────┬───────────────────────────┘
             │
             ▼
┌────────────────────────────────────────┐
│          CORE LAYER                    │
│  Graph, NodeLayout, EdgeLayout         │
└────────────────────────────────────────┘

Direction: Top-to-bottom only (ACYCLIC)
```

---

## 5. Architecture Patterns & Design Quality

### 5.1 Identified Patterns

1. **Facade Pattern**
   - EdgeRoutingService hides complexity of EdgeRouting + ConstraintGateway
   - Clients call simple methods: `routeInitial()`, `updateAfterNodeMove()`
   - Internal workflow orchestration is hidden

2. **Service Layer Pattern**
   - Provides high-level business operations (routing + validation)
   - Coordinates multiple subsystems (EdgeRouting, ConstraintGateway)
   - Returns validated results (RoutingResult with ValidatedEdgeLayout)

3. **Dependency Injection**
   - IPathFinder injected via constructor
   - Enables algorithm swapping (AStarPathFinder, GeometricPathFinder)

4. **Template Method (Implicit)**
   - All 3 public methods follow same template:
     1. Prepare options (e.g., applyPostDragAlgorithm)
     2. Route edges (via EdgeRouting)
     3. Ensure label positions
     4. Validate and wrap
   - Template ensures consistent constraint enforcement

### 5.2 SOLID Principles Compliance

| Principle | Status | Evidence |
|-----------|--------|----------|
| **S** Single Responsibility | ✅ PASS | Single high-level responsibility: validated edge routing service |
| **O** Open/Closed | ✅ PASS | Extensible via IPathFinder injection; closed to modification |
| **L** Liskov Substitution | N/A | No inheritance hierarchy (final class) |
| **I** Interface Segregation | ✅ PASS | No interface bloat; 3 focused methods for 3 use cases |
| **D** Dependency Inversion | ✅ PASS | Depends on IPathFinder abstraction, not concrete implementation |

### 5.3 Clean Architecture Alignment

```
EdgeRoutingService as Boundary:

USE CASES (Application Layer):
├── SugiyamaLayout (initial layout use case)
└── LayoutController (interactive drag use case)
    ↓
INTERFACE ADAPTER (Service Layer):
└── EdgeRoutingService  ← YOU ARE HERE
    ↓
ENGINE/DOMAIN (Business Logic):
├── EdgeRouting (routing algorithms)
└── ConstraintGateway (validation rules)
    ↓
ENTITIES (Core):
└── Graph, NodeLayout, EdgeLayout
```

**Assessment:** ✅ **PROPERLY LAYERED**

EdgeRoutingService acts as an **Interface Adapter** that:
- Translates high-level use case requests (route initial, update after move) into engine-level operations
- Enforces cross-cutting concerns (validation, label calculation)
- Returns use-case-specific DTOs (RoutingResult)

---

## 6. Constraint Validation Integration

### 6.1 Validation Flow

```cpp
EdgeRoutingService::validateAndWrap()
├── Creates EdgeConstraintContext { rawLayouts, nodeLayouts, gridSize }
├── For each edge:
│   ├── constraintGateway_->validateAndWrapRelaxed(layout, ctx)
│   ├── If valid: store ValidatedEdgeLayout
│   └── If invalid: store edgeId in rejectedEdges
└── Returns RoutingResult { validatedLayouts, rejectedEdges }
```

### 6.2 Type Safety via ValidatedEdgeLayout

**Key Design Decision:** EdgeRoutingService returns `ValidatedEdgeLayout`, not raw `EdgeLayout`.

Benefits:
1. **Compile-time enforcement:** LayoutResult::setEdgeLayout() requires ValidatedEdgeLayout
2. **Explicit validation:** Cannot bypass constraint checking
3. **Clear contract:** Clients receive only validated layouts
4. **Error transparency:** rejectedEdges list shows what failed

### 6.3 Constraint Gateway Abstraction

```
EdgeRoutingService does NOT:
├── Know what constraints exist (OrthogonalityConstraint, OverlapConstraint, etc.)
├── Implement validation logic
└── Decide what "valid" means

EdgeRoutingService DOES:
├── Call ConstraintGateway.validateAndWrapRelaxed()
├── Classify results as valid/rejected
└── Package results for clients
```

**Benefit:** Constraint rules can change without modifying EdgeRoutingService.

---

## 7. PostDragAlgorithm Mapping

### 7.1 Algorithm Conversion Logic

```cpp
LayoutOptions applyPostDragAlgorithm(const LayoutOptions& options) {
    LayoutOptions result = options;
    switch (options.optimizationOptions.postDragAlgorithm) {
        case PostDragAlgorithm::AStar:
            result.optimizationOptions.dragAlgorithm = DragAlgorithm::AStar;
            break;
        case PostDragAlgorithm::Geometric:
        case PostDragAlgorithm::None:
            result.optimizationOptions.dragAlgorithm = DragAlgorithm::Geometric;
            break;
    }
    return result;
}
```

### 7.2 Rationale

**Problem:** LayoutController::moveNode() and LayoutController::setNodeType() are "drop" operations (user finished moving), but EdgeRouting expects `dragAlgorithm` (used during drag).

**Solution:** EdgeRoutingService translates `postDragAlgorithm` to `dragAlgorithm` so EdgeRouting can route edges as if "drag just ended, now route with post-drop algorithm."

**Why here?** Service layer is the right place for this semantic translation:
- Clients provide high-level intent (postDragAlgorithm)
- Service translates to low-level engine setting (dragAlgorithm)
- Engine doesn't need to know about "post-drag" concept

---

## 8. Pass-Through Edge Detection (updateAfterNodeMove)

### 8.1 Problem

When node moves, edges connected to it are recalculated (first pass). But edges NOT connected to moved node may still pass through its new position, causing collisions.

### 8.2 Solution (Two-Pass Routing)

```cpp
EdgeRoutingService::updateAfterNodeMove() {
    // First pass: update directly connected edges
    edgeRouting_->updateEdgeRoutingWithOptimization(
        workingLayouts, nodeLayouts, affectedEdges, postDropOptions, movedNodes);

    // Second pass: find pass-through edges
    std::vector<EdgeId> edgesToRecalculate;
    for (auto& [edgeId, edgeLayout] : workingLayouts) {
        if (affectedSet.count(edgeId)) continue;  // Already processed

        for (auto& movedNodeId : movedNodes) {
            if (LayoutUtils::edgePassesThroughNode(edgeLayout, nodeIt->second)) {
                edgesToRecalculate.push_back(edgeId);
                break;
            }
        }
    }

    // Recalculate pass-through edges
    edgeRouting_->updateEdgeRoutingWithOptimization(
        workingLayouts, nodeLayouts, edgesToRecalculate, postDropOptions, emptySet);
}
```

### 8.3 Performance Optimization

**Optimization:** Use `std::unordered_set` for O(1) lookup instead of O(n) `std::find`:

```cpp
// Convert affectedEdges vector to set for O(1) lookup
std::unordered_set<EdgeId> affectedSet(affectedEdges.begin(), affectedEdges.end());
```

---

## 9. Strengths & Weaknesses

### 9.1 Strengths

1. **Clear Separation of Concerns**
   - Routing logic in EdgeRouting
   - Validation logic in ConstraintGateway
   - Orchestration in EdgeRoutingService

2. **Type Safety**
   - Returns ValidatedEdgeLayout (compile-time enforcement)
   - Cannot bypass validation by accident

3. **Testability**
   - IPathFinder injectable (can mock for tests)
   - Clear input/output contract
   - No global state

4. **Extensibility**
   - New constraints: modify ConstraintGateway only
   - New routing algorithms: inject different IPathFinder
   - Service layer unchanged

5. **Error Transparency**
   - rejectedEdges list shows what failed
   - LOG_ERROR messages explain why

### 9.2 Weaknesses

1. **Implicit Dependency on EdgeRouting Methods**
   - Service knows about EdgeRouting's public methods (route, distributeAutoSnapPoints, optimizeRouting)
   - Not behind an interface
   - Mitigation: EdgeRouting is internal, not part of public API

2. **Two-Pass Routing Complexity**
   - updateAfterNodeMove() has complex pass-through edge detection logic
   - Could be extracted to separate helper class
   - Mitigation: Well-documented, unit-testable

3. **No IEdgeRoutingService Interface**
   - EdgeRoutingService is concrete class, not behind interface
   - Hard to mock for unit tests of SugiyamaLayout/LayoutController
   - Mitigation: Service is stable, rarely changes

4. **Label Position Responsibility**
   - ensureLabelPositions() calls LayoutUtils::calculateEdgeLabelPosition()
   - Somewhat unrelated to routing/validation
   - Mitigation: Necessary for complete layout; logical place in workflow

### 9.3 Technical Debt

**Minor:** No interface for EdgeRoutingService
- Impact: Hard to unit test clients (SugiyamaLayout, LayoutController)
- Recommendation: Extract IEdgeRoutingService interface if testing becomes issue

**Minor:** Pass-through edge detection in service layer
- Impact: Complex logic in orchestration layer
- Recommendation: Extract to PassThroughEdgeDetector helper class

---

## 10. Interaction Scenarios

### 10.1 Scenario 1: Initial Layout (SugiyamaLayout)

```
SugiyamaLayout::routeEdges()
    ↓
EdgeRoutingService::routeInitial(graph, nodeLayouts, reversedEdges, options)
    ↓
1. EdgeRouting::route(graph, nodeLayouts, reversedEdges, options, skipOptimization=true)
   → Creates initial layouts WITHOUT optimization (snap positions not final yet)
    ↓
2. EdgeRouting::distributeAutoSnapPoints(result, nodeLayouts, gridSize)
   → Evenly distributes snap points on node edges
    ↓
3. EdgeRouting::optimizeRouting(result, nodeLayouts, options)
   → Runs A* optimizer to avoid obstacles/overlaps
    ↓
4. ensureLabelPositions(result.edgeLayouts)
   → Calculates label position for each edge
    ↓
5. validateAndWrap(result.edgeLayouts, nodeLayouts, gridSize)
   → ConstraintGateway validates each edge
   → Returns RoutingResult { validatedLayouts, rejectedEdges }
    ↓
SugiyamaLayout::routeEdges()
   → Stores validatedLayouts in LayoutResult
   → Logs rejectedEdges as errors
```

### 10.2 Scenario 2: Interactive Drag (LayoutController)

```
LayoutController::moveNode(nodeId, newPosition)
    ↓
1. Validate constraints (ConstraintManager)
    ↓
2. Apply position change
    ↓
3. getExpandedAffectedEdges(nodeId)
   → Returns connected edges + pass-through edges
    ↓
EdgeRoutingService::updateAfterNodeMove(
    currentLayouts, nodeLayouts, affectedEdges, options, movedNodes)
    ↓
1. applyPostDragAlgorithm(options)
   → Converts PostDragAlgorithm::AStar to DragAlgorithm::AStar
    ↓
2. FIRST PASS: EdgeRouting::updateEdgeRoutingWithOptimization(
       workingLayouts, nodeLayouts, affectedEdges, postDropOptions, movedNodes)
   → Updates directly connected edges with A* optimization
    ↓
3. SECOND PASS DETECTION:
   For each edge NOT in affectedEdges:
       if LayoutUtils::edgePassesThroughNode(edge, movedNode):
           add to edgesToRecalculate
    ↓
4. SECOND PASS ROUTING: EdgeRouting::updateEdgeRoutingWithOptimization(
       workingLayouts, nodeLayouts, edgesToRecalculate, postDropOptions, emptySet)
   → Reroutes pass-through edges to avoid moved node
    ↓
5. ensureLabelPositions(workingLayouts)
    ↓
6. validateAndWrap(workingLayouts, nodeLayouts, gridSize)
   → Returns RoutingResult
    ↓
LayoutController::moveNode()
   → Updates edgeLayouts_ with validated results
   → Returns NodeMoveResult::ok(position, affectedEdges)
```

### 10.3 Scenario 3: Snap Point Move

```
LayoutController (via LayoutUtils::moveSnapPoint)
    ↓
User manually drags edge snap point to new position
    ↓
EdgeRoutingService::updateAfterSnapMove(
    currentLayouts, nodeLayouts, affectedEdges, options)
    ↓
1. applyPostDragAlgorithm(options)
    ↓
2. EdgeRouting::updateEdgeRoutingWithOptimization(
       workingLayouts, nodeLayouts, affectedEdges, snapOptions, emptyMovedNodes)
   → Recalculates bend points for edges with moved snap points
    ↓
3. ensureLabelPositions(workingLayouts)
    ↓
4. validateAndWrap(workingLayouts, nodeLayouts, gridSize)
    ↓
Returns RoutingResult
```

---

## 11. Recommendations

### 11.1 Keep As-Is (No Changes Needed)

1. **Facade pattern** - Appropriate for service layer
2. **Validation integration** - Well-designed, type-safe
3. **Dependency injection** - Good use of IPathFinder interface
4. **Two-pass routing** - Correct solution for pass-through edges

### 11.2 Optional Improvements (Low Priority)

1. **Extract IEdgeRoutingService interface**
   - Benefit: Easier unit testing of SugiyamaLayout/LayoutController
   - Cost: Interface maintenance overhead
   - Priority: LOW (only if testing becomes problematic)

2. **Extract PassThroughEdgeDetector helper**
   - Benefit: Cleaner updateAfterNodeMove() method
   - Cost: Additional class to maintain
   - Priority: LOW (current code is readable)

3. **Extract LabelPositionCalculator service**
   - Benefit: Separate concern from routing service
   - Cost: More indirection, unclear ownership
   - Priority: VERY LOW (label calc is tightly coupled to routing)

### 11.3 Monitoring Points

Watch for these potential issues:

1. **Performance:** Two-pass routing in updateAfterNodeMove() may be slow for large graphs
   - Mitigation: Benchmark with >100 edges
   - Optimization: Spatial index for pass-through detection

2. **Validation overhead:** Validating every edge on every update
   - Mitigation: Cache validation results if layout unchanged
   - Optimization: Incremental validation (only changed edges)

3. **Memory:** Copying edgeLayouts in updateAfterNodeMove()
   - Mitigation: Acceptable for current use cases
   - Optimization: In-place modification with rollback mechanism

---

## 12. Conclusion

**Overall Architecture Grade: A-**

EdgeRoutingService is a well-designed service layer component that:
- Follows SOLID principles
- Uses appropriate design patterns (Facade, Service Layer, DI)
- Has clear responsibility boundaries
- Provides type-safe, validated results
- Has low coupling to clients (Ca=2)
- Has acceptable coupling to implementations (Ce=5)
- Contains no circular dependencies
- Is properly layered in Clean Architecture

**Primary Strength:** Clean separation of routing (EdgeRouting), validation (ConstraintGateway), and orchestration (EdgeRoutingService).

**Primary Weakness:** Some complexity in two-pass routing logic for pass-through edges, but this is necessary for correctness.

**Recommendation:** **No changes needed.** Current design is maintainable, extensible, and follows architectural best practices.

---

## Appendix: File Locations

| Component | Header | Implementation |
|-----------|--------|----------------|
| EdgeRoutingService | `include/arborvia/layout/api/EdgeRoutingService.h` | `src/layout/api/EdgeRoutingService.cpp` |
| EdgeRouting | `src/layout/sugiyama/routing/EdgeRouting.h` | `src/layout/sugiyama/routing/EdgeRouting.cpp` |
| ConstraintGateway | `include/arborvia/layout/constraints/ConstraintGateway.h` | `src/layout/constraints/ConstraintGateway.cpp` |
| SugiyamaLayout | `include/arborvia/layout/SugiyamaLayout.h` | `src/layout/SugiyamaLayout.cpp` |
| LayoutController | `include/arborvia/layout/api/LayoutController.h` | `src/layout/api/LayoutController.cpp` |
| LayoutUtils | `include/arborvia/layout/util/LayoutUtils.h` | `src/layout/LayoutUtils.cpp` |

## Appendix: Metrics Summary

```
Afferent Coupling (Ca): 2
Efferent Coupling (Ce): 5
Instability (I): 0.71
Abstractness (A): 0.0 (concrete class)
Distance from Main Sequence (D): 0.29 (acceptable, service layer)

Lines of Code: ~150 (service implementation)
Public Methods: 3 (focused API)
Dependencies: 5 (managed)
Dependents: 2 (stable)

Circular Dependencies: 0
SOLID Violations: 0
Design Pattern Usage: 3 (Facade, Service Layer, DI)
```
