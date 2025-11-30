# ArborVia - 아키텍처 문서

> **계층적 그래프 레이아웃을 위한 경량 C++ 라이브러리**

---

## 📊 구현 상태

| 상태 | 의미 |
|------|------|
| ✅ 구현 완료 | 프로덕션 사용 가능, 테스트 완료 |
| 🚧 구현 중 | 개발 진행 중 |
| 📋 설계 완료 | 설계는 완료, 구현 예정 |
| 📝 계획됨 | 향후 구현 계획 |

**현재 상태 요약:**
- **core/** ✅ 구현 완료 - Graph, CompoundGraph, Types
- **layout/** ✅ 구현 완료 - SugiyamaLayout
- **export/** ✅ 구현 완료 - SvgExport
- **routing/** 📋 설계 완료 - IEdgeRouter, OrthogonalRouter (구현 예정)

---

## 🎯 핵심 철학

```
"외부 의존성 없이 순수 C++로 구현된 그래프 레이아웃 엔진"
```

**핵심 원칙:**
1. **제로 의존성** - C++ 표준 라이브러리만 사용
2. **단일 책임** - 그래프 레이아웃만 담당, 렌더링은 사용자 몫
3. **MIT 라이센스** - 상업용 게임에서 자유롭게 사용 가능
4. **플랫폼 독립적** - SDL, ImGui, Qt 등 어떤 UI 프레임워크와도 통합 가능

---

## 🔧 설계 목표

**ArborVia는 다음을 위해 설계되었습니다:**

1. **상태머신 에디터** - SCXML/FSM 시각화
2. **워크플로우 다이어그램** - 프로세스 흐름 표현
3. **계층적 그래프** - 조직도, 트리 구조 등
4. **Compound/Parallel 노드** - 중첩 상태 및 병렬 상태 지원

**왜 ArborVia인가?**
- ELK.js는 WASM으로 사용하면 45-55% 성능 저하
- OGDF는 GPL 라이센스로 상업용 게임에서 사용 불가
- 네이티브 C++로 최적의 성능과 자유로운 라이센스 제공

---

## 🏗️ 아키텍처 레이어

```
┌─────────────────────────────────────────────────────┐
│  레이어 1: 사용자 애플리케이션 (ArborVia 외부)       │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐ │
│  │ SDL3+ImGui  │  │    Qt       │  │   Custom    │ │
│  │  (게임 UI)  │  │  (데스크탑) │  │   (웹 등)   │ │
│  └─────────────┘  └─────────────┘  └─────────────┘ │
├─────────────────────────────────────────────────────┤
│  레이어 2: 출력 모듈 (export/) ✅                   │
│  - IExporter 인터페이스                             │
│  - SvgExport (SVG 파일 내보내기)                    │
├─────────────────────────────────────────────────────┤
│  레이어 3: 라우팅 모듈 (routing/) 📋                │
│  - IEdgeRouter 인터페이스 (설계 완료)               │
│  - OrthogonalRouter (구현 예정)                     │
├─────────────────────────────────────────────────────┤
│  레이어 4: 레이아웃 엔진 (layout/) ✅               │
│  - ILayout 인터페이스                               │
│  - SugiyamaLayout (Sugiyama 알고리즘)               │
├─────────────────────────────────────────────────────┤
│  레이어 5: 핵심 자료구조 (core/) ✅                 │
│  - Graph (기본 유향 그래프)                         │
│  - CompoundGraph (계층적 그래프)                    │
│  - LayoutOptions / LayoutResult                     │
├─────────────────────────────────────────────────────┤
│  레이어 6: 기반 타입 ✅                             │
│  - Types (Point, Size, Rect)                        │
│  - NodeId, EdgeId                                   │
└─────────────────────────────────────────────────────┘

의존성 흐름: 위 → 아래 (단방향만)
```

---

## 📦 모듈 구조

### 현재 구현된 구조 ✅

```
arborvia/
├── include/arborvia/           # 공개 헤더
│   ├── arborvia.h              # 메인 헤더 (모든 모듈 포함)
│   ├── core/                   # ✅ 핵심 자료구조
│   │   ├── Types.h             # Point, Size, Rect, NodeId, EdgeId
│   │   ├── Graph.h             # 기본 그래프 클래스
│   │   └── CompoundGraph.h     # 계층적 그래프 클래스
│   ├── layout/                 # ✅ 레이아웃 엔진
│   │   ├── ILayout.h           # 레이아웃 인터페이스
│   │   ├── LayoutOptions.h     # 레이아웃 설정
│   │   ├── LayoutResult.h      # 레이아웃 결과
│   │   ├── LayoutTypes.h       # LayoutMode, NodeEdge, SnapPointConfig 등
│   │   ├── LayoutUtils.h       # 레이아웃 유틸리티 함수
│   │   ├── ManualLayoutManager.h # 수동 레이아웃 관리자
│   │   └── SugiyamaLayout.h    # Sugiyama 알고리즘 구현
│   └── export/                 # ✅ 내보내기 모듈
│       ├── IExporter.h         # 내보내기 인터페이스
│       └── SvgExport.h         # SVG 내보내기 구현
├── src/                        # 구현 파일
│   ├── core/
│   │   ├── Graph.cpp
│   │   └── CompoundGraph.cpp
│   ├── layout/
│   │   ├── LayoutResult.cpp
│   │   ├── LayoutUtils.cpp
│   │   ├── ManualLayoutManager.cpp
│   │   ├── SugiyamaLayout.cpp
│   │   └── sugiyama/           # Sugiyama 내부 알고리즘
│   │       ├── CycleRemoval.cpp
│   │       ├── LayerAssignment.cpp
│   │       ├── CrossingMinimization.cpp
│   │       ├── CoordinateAssignment.cpp
│   │       └── EdgeRouting.cpp
│   └── export/
│       └── SvgExport.cpp
├── tests/                      # 테스트
│   ├── core/
│   │   ├── GraphTest.cpp
│   │   └── CompoundGraphTest.cpp
│   ├── layout/
│   │   ├── SugiyamaLayoutTest.cpp
│   │   ├── LayoutResultTest.cpp
│   │   └── ManualLayoutTest.cpp
│   ├── export/
│   │   └── SvgExportTest.cpp
│   └── interactive/
│       └── DragBehaviorTest.cpp
├── examples/                   # 예제 프로그램
│   ├── interactive_demo/       # 인터랙티브 데모
│   │   └── main.cpp
│   ├── drag_demo/              # 드래그 기능 데모
│   │   └── main.cpp
│   └── visual_demo/            # 시각적 테스트 데모
│       └── main.cpp
├── cmake/                      # 모듈별 CMake 설정
│   ├── ArborviaCore.cmake
│   ├── ArborviaLayout.cmake
│   └── ArborviaExport.cmake
├── CMakeLists.txt              # 메인 빌드 설정
└── docs/
    └── ko/
        └── ARCHITECTURE.md
```

### 계획된 구조 📋 (routing 모듈)

```
include/arborvia/
└── routing/                    # 📋 엣지 라우팅 모듈 (구현 예정)
    ├── IEdgeRouter.h           # 라우팅 인터페이스
    ├── EdgeRoutingConfig.h     # 엣지별 라우팅 설정
    └── OrthogonalRouter.h      # 직교 라우팅 구현

src/routing/
├── OrthogonalRouter.cpp
└── SnapPointAllocator.cpp

tests/routing/
└── EdgeRoutingTest.cpp
```

**모듈 의존성 규칙:**
```
                    ┌─────────────┐
                    │   export    │
                    │ (IExporter) │
                    └──────┬──────┘
                           │
    ┌──────────────────────┼──────────────────────┐
    │                      │                      │
    ▼                      ▼                      ▼
┌─────────┐         ┌─────────────┐        ┌─────────────┐
│ routing │ ──────► │   layout    │ ◄───── │   export    │
│(IRouter)│         │  (ILayout)  │        │ (IExporter) │
└────┬────┘         └──────┬──────┘        └──────┬──────┘
     │                     │                      │
     └─────────────────────┼──────────────────────┘
                           │
                           ▼
                    ┌─────────────┐
                    │    core     │
                    │ (Graph 등)  │
                    └─────────────┘
```

- `core/`는 의존성 없음 (기반 모듈)
- `layout/`은 `core/`에 의존
- `routing/`은 `core/`, `layout/`에 의존
- `export/`는 `core/`, `layout/`에 의존

---

## 📊 레이어 5: 기반 타입

**순수 데이터 타입 - 의존성 없음**

### Types.h

```cpp
namespace arborvia {

using NodeId = uint32_t;
using EdgeId = uint32_t;

constexpr NodeId INVALID_NODE = UINT32_MAX;
constexpr EdgeId INVALID_EDGE = UINT32_MAX;

struct Point {
    float x = 0.0f;
    float y = 0.0f;
    
    Point operator+(const Point& o) const;
    Point operator-(const Point& o) const;
    float length() const;
    float distanceTo(const Point& o) const;
};

struct Size {
    float width = 0.0f;
    float height = 0.0f;
};

struct Rect {
    float x, y, width, height;
    
    Point center() const;
    bool contains(const Point& p) const;
    Rect united(const Rect& other) const;
    Rect expanded(float padding) const;
};

}
```

**핵심:** 렌더링 프레임워크 의존성 없음. 순수 수학적 타입.

---

## 🔌 인터페이스 설계

**확장성을 위한 추상 인터페이스**

### ILayout (layout/ILayout.h)

```cpp
namespace arborvia {

/// 레이아웃 알고리즘의 추상 인터페이스
class ILayout {
public:
    virtual ~ILayout() = default;
    
    /// 레이아웃 옵션 설정
    virtual void setOptions(const LayoutOptions& options) = 0;
    virtual const LayoutOptions& options() const = 0;
    
    /// 기본 그래프 레이아웃
    virtual LayoutResult layout(const Graph& graph) = 0;
    
    /// 계층적 그래프 레이아웃
    virtual LayoutResult layout(const CompoundGraph& graph) = 0;
};

}
```

### IExporter (export/IExporter.h)

```cpp
namespace arborvia {

/// 내보내기 포맷의 추상 인터페이스
class IExporter {
public:
    virtual ~IExporter() = default;
    
    /// 문자열로 내보내기
    virtual std::string exportToString(const Graph& graph, 
                                       const LayoutResult& layout) = 0;
    virtual std::string exportToString(const CompoundGraph& graph,
                                       const LayoutResult& layout) = 0;
    
    /// 스트림으로 내보내기
    virtual void exportToStream(const Graph& graph, const LayoutResult& layout,
                               std::ostream& out) = 0;
    
    /// 파일로 내보내기
    virtual bool exportToFile(const Graph& graph, const LayoutResult& layout,
                             const std::string& filename) = 0;
    
    /// 메타데이터
    virtual std::string fileExtension() const = 0;
    virtual std::string mimeType() const = 0;
};

}
```

### IEdgeRouter (routing/IEdgeRouter.h) 📋 설계 완료

> **상태:** 설계 완료, 구현 예정. 아래는 설계 명세입니다.

```cpp
namespace arborvia {

/// 엣지 라우팅 알고리즘의 추상 인터페이스
/// 설계 원칙: Primitive API만 제공, 편의 기능은 애플리케이션에서 조합
class IEdgeRouter {
public:
    virtual ~IEdgeRouter() = default;
    
    /// 라우팅 옵션 설정
    virtual void setOptions(const EdgeRoutingOptions& options) = 0;
    virtual const EdgeRoutingOptions& options() const = 0;
    
    /// [Primitive] 단일 엣지 라우팅
    virtual EdgeLayout route(
        const EdgeData& edge,
        const NodeLayout& from,
        const NodeLayout& to,
        const EdgeRoutingConfig* config = nullptr) = 0;
    
    /// [Batch] 선택된 엣지들 라우팅 (최적화된 배치 처리)
    virtual std::unordered_map<EdgeId, EdgeLayout> routeEdges(
        const std::vector<EdgeId>& edgeIds,
        const Graph& graph,
        const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
        const std::unordered_map<EdgeId, EdgeRoutingConfig>* configs = nullptr) = 0;
    
    /// [Batch] 전체 엣지 라우팅
    virtual std::unordered_map<EdgeId, EdgeLayout> routeAll(
        const Graph& graph,
        const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
        const std::unordered_map<EdgeId, EdgeRoutingConfig>* configs = nullptr) = 0;
};

}
```

**핵심:** 인터페이스 기반 설계로 새로운 알고리즘 추가 용이

---

## 📈 레이어 5: 그래프 자료구조

**그래프 데이터 관리 - 레이아웃 알고리즘과 분리**

### Graph (기본 그래프)

```cpp
class Graph {
public:
    // 노드 관리
    NodeId addNode();
    NodeId addNode(const std::string& label);
    NodeId addNode(Size size, const std::string& label);
    void removeNode(NodeId id);
    
    // 엣지 관리
    EdgeId addEdge(NodeId from, NodeId to);
    EdgeId addEdge(NodeId from, NodeId to, const std::string& label);
    void removeEdge(EdgeId id);
    
    // 노드 연결 쿼리
    std::vector<NodeId> successors(NodeId id) const;
    std::vector<NodeId> predecessors(NodeId id) const;
    std::optional<EdgeId> findEdge(NodeId from, NodeId to) const;
    
    // 엣지 쿼리 (라우팅 모듈 연동용)
    std::vector<EdgeId> getOutEdges(NodeId id) const;       // 나가는 엣지
    std::vector<EdgeId> getInEdges(NodeId id) const;        // 들어오는 엣지
    std::vector<EdgeId> getConnectedEdges(NodeId id) const; // 모든 연결된 엣지
    
protected:
    std::vector<NodeData> nodes_;
    std::vector<EdgeData> edges_;
    std::unordered_map<NodeId, std::vector<EdgeId>> outEdges_;
    std::unordered_map<NodeId, std::vector<EdgeId>> inEdges_;
};
```

### CompoundGraph (계층적 그래프)

```cpp
enum class CompoundType {
    Atomic,      // 자식 없음 (리프 노드)
    Compound,    // 순차적 자식 (세로 레이아웃)
    Parallel     // 병렬 자식 (가로 레이아웃)
};

class CompoundGraph : public Graph {
public:
    // 계층 관리
    void setParent(NodeId child, NodeId parent);
    void removeFromParent(NodeId child);
    
    // 계층 쿼리
    std::optional<NodeId> getParent(NodeId id) const;
    std::vector<NodeId> getChildren(NodeId id) const;
    std::vector<NodeId> getDescendants(NodeId id) const;
    std::vector<NodeId> getAncestors(NodeId id) const;
    
    // 타입 쿼리
    bool isCompound(NodeId id) const;
    bool isParallel(NodeId id) const;
    bool isAtomic(NodeId id) const;
    bool isRoot(NodeId id) const;
    
    // 접기/펼치기
    void setCollapsed(NodeId id, bool collapsed);
    bool isCollapsed(NodeId id) const;
    bool isVisible(NodeId id) const;
    
    // 유틸리티
    std::optional<NodeId> lowestCommonAncestor(NodeId a, NodeId b) const;
};
```

**핵심:** 그래프 구조만 관리. 위치 정보 없음.

---

## ⚙️ 레이어 3: 레이아웃 엔진

**Sugiyama 알고리즘 - 5단계 레이아웃 프로세스**

### Sugiyama 알고리즘 파이프라인

```
입력: Graph/CompoundGraph
          ↓
┌─────────────────────────────────┐
│ 1. Cycle Removal                │
│    - DFS로 백엣지 탐지          │
│    - 사이클 형성 엣지 역전 마킹  │
└─────────────────────────────────┘
          ↓
┌─────────────────────────────────┐
│ 2. Layer Assignment             │
│    - Longest Path 알고리즘      │
│    - 각 노드에 레이어 번호 할당  │
└─────────────────────────────────┘
          ↓
┌─────────────────────────────────┐
│ 3. Crossing Minimization        │
│    - Barycenter Heuristic       │
│    - 레이어 내 노드 순서 최적화  │
└─────────────────────────────────┘
          ↓
┌─────────────────────────────────┐
│ 4. Coordinate Assignment        │
│    - 실제 x, y 좌표 계산        │
│    - 노드 크기 및 간격 고려      │
└─────────────────────────────────┘
          ↓
┌─────────────────────────────────┐
│ 5. Edge Routing                 │
│    - Orthogonal (직교) 라우팅   │
│    - Polyline 라우팅            │
│    - 벤드 포인트 계산            │
└─────────────────────────────────┘
          ↓
출력: LayoutResult
```

### SugiyamaLayout 클래스

```cpp
class SugiyamaLayout {
public:
    SugiyamaLayout();
    explicit SugiyamaLayout(const LayoutOptions& options);
    
    // 레이아웃 실행
    LayoutResult layout(const Graph& graph);
    LayoutResult layout(const CompoundGraph& graph);
    
    // 통계
    struct LayoutStats {
        int layerCount;
        int maxLayerWidth;
        int edgeCrossings;
        int reversedEdges;
        float totalEdgeLength;
    };
    const LayoutStats& lastStats() const;
    
private:
    void removeCycles();
    void assignLayers();
    void minimizeCrossings();
    void assignCoordinates();
    void routeEdges();
    
    // Compound 그래프 전용
    void layoutCompoundNode(NodeId id, const CompoundGraph& graph);
    void layoutParallelRegions(NodeId id, const CompoundGraph& graph);
};
```

### LayoutOptions

```cpp
enum class Direction { TopToBottom, BottomToTop, LeftToRight, RightToLeft };
enum class EdgeRouting { Orthogonal, Polyline, Splines };
enum class CrossingMinimization { None, BarycenterHeuristic, MedianHeuristic };

struct LayoutOptions {
    Direction direction = Direction::TopToBottom;
    
    float nodeSpacingHorizontal = 50.0f;
    float nodeSpacingVertical = 75.0f;
    float compoundPadding = 20.0f;
    float parallelSpacing = 30.0f;
    
    EdgeRouting edgeRouting = EdgeRouting::Orthogonal;
    CrossingMinimization crossingMinimization = CrossingMinimization::BarycenterHeuristic;
    int crossingMinimizationPasses = 4;
    
    // Builder 패턴
    LayoutOptions& setDirection(Direction d);
    LayoutOptions& setNodeSpacing(float h, float v);
};
```

### LayoutResult

```cpp
struct NodeLayout {
    NodeId id;
    Point position;    // 좌상단 좌표
    Size size;
    int layer;         // 레이어 인덱스
    int order;         // 레이어 내 순서
    
    Point center() const;
    Rect bounds() const;
};

struct EdgeLayout {
    EdgeId id;
    NodeId from, to;
    Point sourcePoint;
    Point targetPoint;
    std::vector<BendPoint> bendPoints;
    
    std::vector<Point> allPoints() const;
};

class LayoutResult {
public:
    const NodeLayout* getNodeLayout(NodeId id) const;
    const EdgeLayout* getEdgeLayout(EdgeId id) const;
    
    Rect computeBounds() const;
    Rect computeBounds(float padding) const;
    
    void translate(float dx, float dy);
};
```

---

## 🔀 레이어 3: 라우팅 모듈 📋 설계 완료

> **상태:** 설계 완료, 구현 예정. 아래는 향후 구현될 API 명세입니다.

**엣지 라우팅, 스냅 포인트, 포트 제약 관리**

### 설계 목표

SCE(SCXML Core Engine)의 visualizer.html에서 제공하는 기능들을 C++ 라이브러리 레벨에서 지원:

1. **스냅 포인트 (Snap Points)** - 노드 경계의 연결점 자동 배치
2. **포트 제약 (Port Constraints)** - 엣지 연결 위치 제어
3. **꺾임 포인트 (Bend Points)** - 엣지 경로의 중간 지점
4. **직교 라우팅 (Orthogonal Routing)** - 수직/수평 엣지 경로

### PortSide 열거형

```cpp
namespace arborvia {

/// 노드의 어느 면에 포트가 위치하는지
enum class PortSide {
    Any,        // 자동 선택
    Top,        // 상단
    Bottom,     // 하단
    Left,       // 좌측
    Right,      // 우측
    Center      // 중앙 (특수 케이스)
};

}
```

### PortConstraint

```cpp
namespace arborvia {

/// 노드에 대한 포트/스냅 포인트 제약
struct PortConstraint {
    NodeId nodeId;
    PortSide side = PortSide::Any;           // 연결 면
    
    // 정밀 위치 지정 (0.0 ~ 1.0, 면의 상대적 위치)
    std::optional<float> position;            // 0.0=시작, 0.5=중앙, 1.0=끝
    
    // 고정 좌표 (position 대신 사용)
    std::optional<Point> fixedPoint;
    
    // 포트 식별자 (여러 포트 구분용)
    std::string portId;
    
    // 빌더 패턴
    PortConstraint& setSide(PortSide s);
    PortConstraint& setPosition(float pos);
    PortConstraint& setFixed(Point p);
};

}
```

### EdgeRoutingConfig

```cpp
namespace arborvia {

/// 개별 엣지에 대한 라우팅 설정
struct EdgeRoutingConfig {
    EdgeId edgeId;
    
    // 소스/타겟 포트 제약
    PortConstraint sourcePort;
    PortConstraint targetPort;
    
    // 라우팅 타입 (기본값은 글로벌 옵션 따름)
    std::optional<EdgeRouting> routingType;
    
    // 수동 벤드 포인트 (자동 라우팅 대신 사용)
    std::vector<Point> manualBendPoints;
    
    // 라우팅 우선순위 (충돌 시 사용)
    int priority = 0;
    
    // 빌더 패턴
    EdgeRoutingConfig& setSource(PortConstraint pc);
    EdgeRoutingConfig& setTarget(PortConstraint pc);
    EdgeRoutingConfig& addBendPoint(Point p);
};

}
```

### EdgeRoutingOptions

```cpp
namespace arborvia {

/// 글로벌 엣지 라우팅 옵션
struct EdgeRoutingOptions {
    // 기본 라우팅 타입
    EdgeRouting defaultRouting = EdgeRouting::Orthogonal;
    
    // 스냅 포인트 설정
    bool enableSnapPoints = true;
    float snapPointSpacing = 20.0f;          // 스냅 포인트 간 최소 간격
    int maxSnapPointsPerSide = 10;           // 면당 최대 스냅 포인트 수
    
    // 직교 라우팅 설정
    float orthogonalMargin = 10.0f;          // 노드로부터의 최소 거리
    float bendPointMinDistance = 15.0f;      // 벤드 포인트 간 최소 거리
    
    // 엣지 간격 설정
    float parallelEdgeSpacing = 8.0f;        // 평행 엣지 간 간격
    
    // 충돌 회피
    bool avoidNodeOverlap = true;            // 노드 통과 방지
    bool avoidEdgeCrossing = false;          // 엣지 교차 최소화 (비용 높음)
    
    // 빌더 패턴
    EdgeRoutingOptions& setSnapSpacing(float spacing);
    EdgeRoutingOptions& setOrthogonalMargin(float margin);
};

}
```

### SnapPointAllocator

```cpp
namespace arborvia {

/// 스냅 포인트 자동 배분 알고리즘
class SnapPointAllocator {
public:
    struct SnapPoint {
        NodeId nodeId;
        PortSide side;
        Point position;           // 절대 좌표
        float relativePosition;   // 면 내 상대 위치 (0.0~1.0)
        bool isOccupied = false;
        EdgeId occupyingEdge = INVALID_EDGE;
    };
    
    struct AllocationResult {
        std::unordered_map<NodeId, std::vector<SnapPoint>> snapPoints;
        std::unordered_map<EdgeId, std::pair<SnapPoint, SnapPoint>> edgeConnections;
    };
    
    /// 노드 레이아웃 기반 스냅 포인트 생성
    void generateSnapPoints(
        const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
        const EdgeRoutingOptions& options);
    
    /// 엣지에 스냅 포인트 할당
    AllocationResult allocate(
        const Graph& graph,
        const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
        const std::unordered_map<EdgeId, EdgeRoutingConfig>* configs = nullptr);
    
    /// 특정 노드의 스냅 포인트 조회
    std::vector<SnapPoint> getSnapPoints(NodeId nodeId, PortSide side) const;
    
    /// 가장 가까운 빈 스냅 포인트 찾기
    std::optional<SnapPoint> findNearestAvailable(
        NodeId nodeId, 
        PortSide preferredSide,
        const Point& targetPoint) const;

private:
    EdgeRoutingOptions options_;
    std::unordered_map<NodeId, std::vector<SnapPoint>> nodeSnapPoints_;
    
    void distributeSnapPointsOnSide(
        const NodeLayout& node,
        PortSide side,
        int count);
    
    float calculateOptimalPosition(
        const NodeLayout& node,
        PortSide side,
        const Point& targetPoint);
};

}
```

### OrthogonalRouter

```cpp
namespace arborvia {

/// 직교(수직/수평) 엣지 라우팅 구현
class OrthogonalRouter : public IEdgeRouter {
public:
    OrthogonalRouter();
    explicit OrthogonalRouter(const EdgeRoutingOptions& options);
    
    // IEdgeRouter 구현
    void setOptions(const EdgeRoutingOptions& options) override;
    const EdgeRoutingOptions& options() const override;
    
    // [Primitive] 단일 엣지 라우팅
    EdgeLayout route(
        const EdgeData& edge,
        const NodeLayout& from,
        const NodeLayout& to,
        const EdgeRoutingConfig* config = nullptr) override;
    
    // [Batch] 선택된 엣지들 라우팅
    std::unordered_map<EdgeId, EdgeLayout> routeEdges(
        const std::vector<EdgeId>& edgeIds,
        const Graph& graph,
        const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
        const std::unordered_map<EdgeId, EdgeRoutingConfig>* configs = nullptr) override;
    
    // [Batch] 전체 엣지 라우팅
    std::unordered_map<EdgeId, EdgeLayout> routeAll(
        const Graph& graph,
        const std::unordered_map<NodeId, NodeLayout>& nodeLayouts,
        const std::unordered_map<EdgeId, EdgeRoutingConfig>* configs = nullptr) override;

private:
    EdgeRoutingOptions options_;
    SnapPointAllocator snapAllocator_;
    
    // 직교 경로 계산
    std::vector<Point> computeOrthogonalPath(
        const Point& source,
        const Point& target,
        PortSide sourceSide,
        PortSide targetSide);
    
    // 벤드 포인트 최적화
    std::vector<Point> optimizeBendPoints(
        const std::vector<Point>& path);
    
    // 노드 회피 경로 계산
    std::vector<Point> avoidNodes(
        const std::vector<Point>& path,
        const std::unordered_map<NodeId, NodeLayout>& nodeLayouts);
};

}
```

### 라우팅 파이프라인

```
입력: Graph + NodeLayouts + EdgeRoutingConfigs
              ↓
┌──────────────────────────────────────┐
│ 1. 스냅 포인트 생성                    │
│    - 각 노드 면에 스냅 포인트 배치      │
│    - EdgeRoutingOptions 기반 간격 설정 │
└──────────────────────────────────────┘
              ↓
┌──────────────────────────────────────┐
│ 2. 포트 제약 처리                      │
│    - EdgeRoutingConfig의 제약 적용     │
│    - 고정 포트 우선 할당               │
└──────────────────────────────────────┘
              ↓
┌──────────────────────────────────────┐
│ 3. 스냅 포인트 할당                    │
│    - 엣지별 최적 스냅 포인트 선택      │
│    - 충돌 방지 및 분산 배치            │
└──────────────────────────────────────┘
              ↓
┌──────────────────────────────────────┐
│ 4. 경로 계산                          │
│    - 직교/폴리라인/스플라인 경로 생성  │
│    - 벤드 포인트 최적화               │
└──────────────────────────────────────┘
              ↓
┌──────────────────────────────────────┐
│ 5. 충돌 회피 (선택적)                  │
│    - 노드 통과 방지                   │
│    - 엣지 교차 최소화                 │
└──────────────────────────────────────┘
              ↓
출력: EdgeLayout (소스점, 타겟점, 벤드포인트)
```

### 사용 예시 📋

> **참고:** 아래 예시는 routing 모듈 구현 완료 후 사용 가능한 패턴입니다.
> 현재는 SugiyamaLayout의 내장 EdgeRouting만 사용 가능합니다.

```cpp
// ═══════════════════════════════════════════════════════════
// 현재 사용 가능한 패턴 ✅
// ═══════════════════════════════════════════════════════════
SugiyamaLayout layout;
LayoutResult result = layout.layout(graph);  // 엣지 라우팅 포함

// LayoutResult에서 엣지 레이아웃 조회
for (const auto& [edgeId, edgeLayout] : result.edgeLayouts()) {
    // sourcePoint, targetPoint, bendPoints 사용 가능
    auto points = edgeLayout.allPoints();
    // 렌더링...
}

// ═══════════════════════════════════════════════════════════
// 향후 지원 예정 패턴 📋 (routing 모듈 구현 후)
// ═══════════════════════════════════════════════════════════

// 1. 노드 드래그 시 연결된 엣지만 재라우팅
void onNodeDragged(NodeId nodeId, Point newPos) {
    // Step 1: 노드 위치 업데이트 (애플리케이션 책임)
    nodeLayouts_[nodeId].position = newPos;
    
    // Step 2: Graph에서 연결된 엣지 조회 ✅ (현재 사용 가능)
    auto connectedEdges = graph_.getConnectedEdges(nodeId);
    
    // Step 3: 해당 엣지만 재라우팅 📋 (routing 모듈 구현 후)
    // auto updatedEdges = router_.routeEdges(
    //     connectedEdges, graph_, nodeLayouts_);
    
    render();
}

// 2. 포트 제약 적용 📋 (routing 모듈 구현 후)
// EdgeRoutingConfig config;
// config.sourcePort.setSide(PortSide::Right).setPosition(0.5f);
// config.targetPort.setSide(PortSide::Left);
```

**설계 철학:**
```
┌─────────────────────────────────────────────────────────┐
│  "Graph가 연결 정보를, Router가 경로 계산을 담당"        │
│                                                         │
│  Graph.getConnectedEdges(nodeId)  →  어떤 엣지?         │
│  Router.routeEdges(edges, ...)    →  어떻게 그릴까?     │
│  Application                      →  언제, 왜?          │
└─────────────────────────────────────────────────────────┘
```

**핵심:** SCE visualizer.html의 스냅 포인트/벤드 포인트 기능을 C++ 라이브러리 레벨에서 지원

---

## 📤 레이어 2: 출력 모듈

**다양한 포맷으로 내보내기**

### SvgExport

```cpp
struct SvgExportOptions {
    float padding = 20.0f;
    std::string backgroundColor = "white";
    
    // 노드 스타일
    std::string nodeFill = "#e0e0e0";
    std::string nodeStroke = "#333333";
    float nodeCornerRadius = 5.0f;
    
    // Compound 노드 스타일
    std::string compoundFill = "#f5f5f5";
    std::string parallelFill = "#f0f8ff";
    
    // 엣지 스타일
    std::string edgeStroke = "#333333";
    
    bool showNodeLabels = true;
    bool showEdgeLabels = true;
};

class SvgExport {
public:
    std::string exportToString(const Graph& graph, const LayoutResult& layout);
    bool exportToFile(const Graph& graph, const LayoutResult& layout, 
                      const std::string& filename);
};
```

**핵심:** SVG는 테스트용. 실제 렌더링은 사용자 애플리케이션에서 LayoutResult를 활용.

---

## 🔄 데이터 흐름 예시

**시나리오: 상태머신을 레이아웃하고 ImGui로 렌더링**

```
1. 그래프 구성:
   CompoundGraph graph;
   NodeId idle = graph.addNode("Idle");
   NodeId running = graph.addNode("Running");
   graph.addEdge(idle, running, "start");
   
2. 레이아웃 실행:
   SugiyamaLayout layout;
   LayoutResult result = layout.layout(graph);
   
3. 레이아웃 결과 활용:
   for (auto& [id, nodeLayout] : result.nodeLayouts()) {
       // ImGui 렌더링
       ImVec2 pos(nodeLayout.position.x, nodeLayout.position.y);
       ImVec2 size(nodeLayout.size.width, nodeLayout.size.height);
       ImGui::GetWindowDrawList()->AddRect(pos, pos + size, color);
   }
   
   for (auto& [id, edgeLayout] : result.edgeLayouts()) {
       auto points = edgeLayout.allPoints();
       for (size_t i = 1; i < points.size(); ++i) {
           ImGui::GetWindowDrawList()->AddLine(
               {points[i-1].x, points[i-1].y},
               {points[i].x, points[i].y}, color);
       }
   }
```

**핵심:** ArborVia는 좌표만 계산. 렌더링은 사용자 애플리케이션의 책임.

---

## 🧪 테스트 전략

### 테스트 구조

```
tests/
├── core/                       # 핵심 자료구조 테스트
│   ├── GraphTest.cpp           # Graph 클래스 테스트
│   └── CompoundGraphTest.cpp   # CompoundGraph 클래스 테스트
├── layout/                     # 레이아웃 엔진 테스트
│   ├── SugiyamaLayoutTest.cpp  # Sugiyama 알고리즘 테스트
│   ├── LayoutResultTest.cpp    # LayoutResult 직렬화/역직렬화 테스트
│   └── ManualLayoutTest.cpp    # 수동 레이아웃 테스트
├── export/                     # 내보내기 모듈 테스트
│   └── SvgExportTest.cpp       # SVG 내보내기 테스트
└── interactive/                # 인터랙티브 기능 테스트
    └── DragBehaviorTest.cpp    # 드래그 동작 및 스냅 포인트 테스트
```

### 테스트 명명 규칙

`[Group_]Action_ExpectedResult` 패턴 사용:

```cpp
// 그룹 접두사를 통한 명확한 분류
TEST_F(DragBehaviorTest, Layout_InitialRouting_IsValid)
TEST_F(DragBehaviorTest, Drag_SingleNode_PreservesRouting)
TEST_F(DragBehaviorTest, Snap_DuringDrag_PointsDontMerge)
TEST_F(DragBehaviorTest, Mode_Unified_DistributesCorrectly)
```

### 단위 테스트 예시

```cpp
// Graph 테스트
TEST(GraphTest, AddNode_WithLabel_ReturnsValidId) {
    Graph graph;
    NodeId id = graph.addNode("test");
    EXPECT_TRUE(graph.hasNode(id));
}

// CompoundGraph 테스트
TEST(CompoundGraphTest, SetParent_ValidNodes_EstablishesHierarchy) {
    CompoundGraph graph;
    NodeId parent = graph.addCompoundNode(CompoundType::Compound);
    NodeId child = graph.addNode("child");
    graph.setParent(child, parent);
    EXPECT_EQ(graph.getParent(child).value(), parent);
}

// Layout 테스트
TEST(SugiyamaLayoutTest, ChainGraph_AssignsSequentialLayers) {
    Graph graph;
    NodeId n1 = graph.addNode("A");
    NodeId n2 = graph.addNode("B");
    graph.addEdge(n1, n2);
    
    SugiyamaLayout layout;
    LayoutResult result = layout.layout(graph);
    
    EXPECT_EQ(result.getNodeLayout(n1)->layer, 0);
    EXPECT_EQ(result.getNodeLayout(n2)->layer, 1);
}

// LayoutResult 테스트
TEST(LayoutResultTest, ToJson_ContainsRequiredFields) {
    // JSON 직렬화 검증
}

// SVG Export 테스트
TEST(SvgExportTest, SimpleGraph_ProducesValidSvg) {
    // SVG 출력 검증
}
```

**핵심 장점:** 렌더링 프레임워크 없이 모든 로직 테스트 가능 (95개 테스트)

---

## 🎨 통합 예시

### Terraform Automata (SDL3 + ImGui)

```cpp
// terraform-automata에서 ArborVia 사용
#include <arborvia/arborvia.h>

class StateMachineEditor {
    arborvia::CompoundGraph graph_;
    arborvia::LayoutResult layoutResult_;
    arborvia::SugiyamaLayout layout_;
    
public:
    void addState(const std::string& name) {
        graph_.addNode(name);
        relayout();
    }
    
    void addTransition(NodeId from, NodeId to) {
        graph_.addEdge(from, to);
        relayout();
    }
    
    void relayout() {
        layoutResult_ = layout_.layout(graph_);
    }
    
    void render() {
        // LayoutResult를 사용해 ImGui로 렌더링
        for (auto& [id, layout] : layoutResult_.nodeLayouts()) {
            renderNode(layout);
        }
        for (auto& [id, layout] : layoutResult_.edgeLayouts()) {
            renderEdge(layout);
        }
    }
};
```

---

## 📊 성능 고려사항

### 시간 복잡도

| 알고리즘 단계 | 복잡도 |
|--------------|--------|
| Cycle Removal | O(V + E) |
| Layer Assignment | O(V + E) |
| Crossing Minimization | O(iterations × layers × maxWidth²) |
| Coordinate Assignment | O(V) |
| Edge Routing | O(E) |

**전체:** O(V + E + iterations × layers × maxWidth²)

### 최적화 전략

1. **작은 그래프 (< 100 노드):** 모든 기능 활성화
2. **중간 그래프 (100-1000 노드):** crossing minimization passes 감소
3. **큰 그래프 (> 1000 노드):** CrossingMinimization::None 사용

---

## 📝 요약

**4개 핵심 모듈, 명확한 책임:**

| 모듈 | 책임 | 인터페이스 | 상태 |
|------|------|-----------|------|
| **core/** | 그래프 자료구조, 기본 타입 | - | ✅ 구현 완료 |
| **layout/** | 노드 좌표 계산 | ILayout | ✅ 구현 완료 |
| **export/** | 결과 내보내기 | IExporter | ✅ 구현 완료 |
| **routing/** | 엣지 경로 계산, 스냅 포인트 | IEdgeRouter | 📋 설계 완료 |

**핵심 이점:**

✅ **제로 의존성** - C++ 표준 라이브러리만 사용  
✅ **MIT 라이센스** - 상업용 게임에서 자유롭게 사용  
✅ **플랫폼 독립적** - 어떤 UI 프레임워크와도 통합 가능  
✅ **테스트 용이** - 렌더링 없이 모든 로직 테스트 (95개 테스트)
✅ **확장 가능** - 인터페이스 기반으로 새 알고리즘 추가 용이  
✅ **모듈화** - 필요한 모듈만 선택적 사용 가능  

**황금률:**

> "ArborVia는 좌표만 계산합니다. 렌더링은 사용자의 몫입니다."

---

## 🗺️ 로드맵

### v0.3.0 (다음 버전)
- 📋 **routing/** 모듈 구현
  - IEdgeRouter 인터페이스 구현
  - OrthogonalRouter 구현
  - SnapPointAllocator 구현

### v0.4.0 이후
- 📝 Network Simplex 레이어 할당
- 📝 Spline 라우터
- 📝 증분 레이아웃
- 📝 Force-Directed 레이아웃

---

**버전:** 0.2.0  
**최종 업데이트:** 2025-11-30  
**상태:**
- ✅ Core 모듈 구현 완료 (Graph, CompoundGraph)
- ✅ Layout 모듈 구현 완료 (SugiyamaLayout)
- ✅ Export 모듈 구현 완료 (SvgExport)
- 📋 Routing 모듈 설계 완료, 구현 예정
