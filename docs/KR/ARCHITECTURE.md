# ArborVia Edge Routing Architecture

> **목표: Overlap-Free Edge Routing**

---

## 1. 문제 정의

### 1.1 Overlap vs Intersection

| 용어 | 정의 | 허용 여부 |
|------|------|----------|
| **Intersection** | 두 엣지가 한 점에서 교차 | ✅ 허용 (soft penalty) |
| **Overlap** | 두 엣지가 선분을 공유 (길이 > 0) | ❌ 금지 (hard constraint) |

```
Intersection (허용):          Overlap (금지):
      │                           ══════
  ────┼────                       ══════
      │                       
```

### 1.2 경로 유형

| 유형 | 패턴 | NodeEdge | 중간 Segment |
|------|------|----------|--------------|
| **H-V-H** | 수평→수직→수평 | Left/Right | 수직 (x=channel) |
| **V-H-V** | 수직→수평→수직 | Top/Bottom | 수평 (y=channel) |

### 1.3 현재 문제

```
현재 A* 결과: 57 overlaps / 50 drags
원인: 각 엣지를 독립적으로 계산, 전역 최적화 없음
```

---

## 2. 목표 아키텍처: Dynamic Cost A* + Post-Nudging

### 2.1 핵심 아이디어

```
1. A* 비용 함수 개조 → "자연스럽게" 빈 곳 찾기
2. Rip-up and Reroute → 순서 의존성 해결
3. Post-Nudging → 불가피한 겹침 시각적 분리
```

### 2.2 아키텍처 다이어그램

```
┌─────────────────────────────────────────────────────────────────┐
│                        EdgeRouting                              │
│  ┌───────────────────────────────────────────────────────────┐  │
│  │                    ObstacleMap (확장)                      │  │
│  │  + getCost(GridPoint): int      ← 동적 비용 반환          │  │
│  │  + markEdgePath(EdgeId, path)   ← 엣지 경로 등록          │  │
│  │  + clearEdgePath(EdgeId)        ← 엣지 경로 해제          │  │
│  │                                                           │  │
│  │  비용 상수:                                                │  │
│  │  - COST_FREE = 1          (빈 공간)                       │  │
│  │  - COST_NODE_BUFFER = 10  (노드 주변)                     │  │
│  │  - COST_EDGE_PATH = 50    (다른 엣지 경로)                │  │
│  │  - COST_CROSSING = 100    (교차점)                        │  │
│  │  - COST_BLOCKED = 999999  (노드 내부)                     │  │
│  └───────────────────────────────────────────────────────────┘  │
│                              │                                  │
│                              ▼                                  │
│  ┌───────────────────────────────────────────────────────────┐  │
│  │                 AStarPathFinder (수정)                     │  │
│  │  + findPath(start, end): Path                             │  │
│  │                                                           │  │
│  │  비용 계산:                                                │  │
│  │  - baseCost = obstacleMap.getCost(nextCell)               │  │
│  │  - bendCost = (방향 전환 시 +5)                            │  │
│  │  - crossingCost = (교차 시 +100)                          │  │
│  │                                                           │  │
│  │  동작: "가능하면 피하되, 정 안 되면 겹쳐라"                  │  │
│  └───────────────────────────────────────────────────────────┘  │
│                              │                                  │
│                              ▼                                  │
│  ┌───────────────────────────────────────────────────────────┐  │
│  │               Rip-up and Reroute 로직                      │  │
│  │                                                           │  │
│  │  1. 새 경로 계산 후 ObstacleMap에 등록                     │  │
│  │  2. 겹치는 기존 엣지 식별 → "dirty" 표시                   │  │
│  │  3. dirty 엣지 재라우팅 (새 경로가 장애물로 작용)           │  │
│  │  4. 반복 (수렴할 때까지 또는 최대 반복 횟수)               │  │
│  └───────────────────────────────────────────────────────────┘  │
└─────────────────────────────────────────────────────────────────┘
```

---

## 3. 컴포넌트 상세 설계

### 3.1 ObstacleMap 확장

**현재**: `bool` (blocked/not blocked)
**변경**: `int` (비용 값)

```cpp
// include/arborvia/layout/sugiyama/ObstacleMap.h

class ObstacleMap {
public:
    // 비용 상수
    static constexpr int COST_FREE = 1;
    static constexpr int COST_NODE_BUFFER = 10;
    static constexpr int COST_EDGE_PATH = 50;
    static constexpr int COST_CROSSING = 100;
    static constexpr int COST_BLOCKED = 999999;
    
    // 비용 조회
    int getCost(GridPoint p) const;
    
    // 엣지 경로 관리
    void markEdgePath(EdgeId edgeId, const std::vector<GridPoint>& path);
    void clearEdgePath(EdgeId edgeId);
    void clearAllEdgePaths();
    
    // 어떤 엣지가 어떤 셀을 사용하는지 조회
    const std::unordered_map<EdgeId, std::vector<GridPoint>>& getEdgePaths() const;
    
private:
    std::unordered_map<GridPoint, int> costMap_;
    std::unordered_map<EdgeId, std::vector<GridPoint>> edgePaths_;
};
```

### 3.2 AStarPathFinder 수정

```cpp
// src/layout/sugiyama/AStarPathFinder.cpp

float AStarPathFinder::calculateMoveCost(GridPoint from, GridPoint to) const {
    // 1. 기본 비용 (동적)
    int baseCost = obstacleMap_->getCost(to);
    
    // 2. 방향 전환 비용 (bend 최소화)
    Direction newDir = getDirection(from, to);
    if (lastDirection_ != Direction::None && lastDirection_ != newDir) {
        baseCost += 5;
    }
    
    // 3. 교차 비용 (다른 엣지와 수직 교차 시)
    if (wouldCrossOtherEdge(from, to)) {
        baseCost += COST_CROSSING;
    }
    
    return static_cast<float>(baseCost);
}
```

### 3.3 Rip-up and Reroute

```cpp
// src/layout/sugiyama/EdgeRouting.cpp

void EdgeRouting::ripUpAndReroute(EdgeId newEdgeId, 
                                   const std::vector<GridPoint>& newPath) {
    // 1. 새 경로 등록
    obstacleMap_->markEdgePath(newEdgeId, newPath);
    
    // 2. 겹치는 기존 엣지 찾기
    std::vector<EdgeId> dirtyEdges;
    for (const auto& [edgeId, existingPath] : obstacleMap_->getEdgePaths()) {
        if (edgeId == newEdgeId) continue;
        if (hasSegmentOverlap(newPath, existingPath)) {
            dirtyEdges.push_back(edgeId);
        }
    }
    
    // 3. 겹치는 엣지들 재라우팅 (최대 3회 반복)
    for (int iteration = 0; iteration < 3 && !dirtyEdges.empty(); ++iteration) {
        std::vector<EdgeId> stillDirty;
        
        for (EdgeId dirtyId : dirtyEdges) {
            // 기존 경로 제거
            obstacleMap_->clearEdgePath(dirtyId);
            
            // 새로 A* 실행 (새 경로들이 장애물로 작용)
            auto& layout = edgeLayouts_[dirtyId];
            auto reroutedPath = pathFinder_->findPath(
                layout.sourcePoint, layout.targetPoint);
            
            // 결과 적용
            applyPath(dirtyId, reroutedPath);
            obstacleMap_->markEdgePath(dirtyId, toGridPath(reroutedPath));
            
            // 여전히 겹치면 다음 반복에서 처리
            if (hasAnyOverlap(dirtyId)) {
                stillDirty.push_back(dirtyId);
            }
        }
        
        dirtyEdges = std::move(stillDirty);
    }
}
```

---

## 4. 드래그/드롭 통합

### 4.1 전체 흐름

```
[초기 레이아웃]
     │
     ▼
┌────────────────────────────────────────┐
│ 모든 엣지 A* 라우팅                     │
│ + Rip-up and Reroute                  │
└────────────────────────────────────────┘
     │
     ▼
[사용자 드래그 시작]
     │
     ▼
┌────────────────────────────────────────┐
│ 드래그 중:                              │
│ - DragAlgorithm::Geometric (빠른 예측)  │
│ - 또는 HideUntilDrop                   │
└────────────────────────────────────────┘
     │
     ▼
[사용자 드래그 종료]
     │
     ▼ (debounce 300ms)
┌────────────────────────────────────────┐
│ 드래그 후:                              │
│ 1. 이동된 노드의 엣지들 A* 실행          │
│ 2. Rip-up and Reroute (겹친 엣지 재계산)│
└────────────────────────────────────────┘
     │
     ▼
[렌더링]
```

### 4.2 LayoutOptions 연동

```cpp
// LayoutOptions.h (기존)

enum class DragAlgorithm {
    None,           // 드래그 중 기존 경로 유지
    Geometric,      // 빠른 기하학적 예측
    AStar,          // 실시간 A* (느림)
    HideUntilDrop   // 숨김
};

enum class PostDragAlgorithm {
    None,           // 드래그 후 최적화 없음
    AStar           // A* + Rip-up and Reroute
};

struct OptimizationOptions {
    DragAlgorithm dragAlgorithm = DragAlgorithm::HideUntilDrop;
    PostDragAlgorithm postDragAlgorithm = PostDragAlgorithm::AStar;

    // 새로 추가
    bool enableRipUpAndReroute = true;
    int maxRipUpIterations = 3;
};
```

---

## 5. 예상 결과

| 항목 | 현재 | 구현 후 |
|------|------|---------|
| 논리적 Overlap | 57 / 50 drags | ~5-10 (대부분 회피) |
| 시각적 Overlap | 57 / 50 drags | **0** (Nudging) |
| 기존 코드 재사용 | - | **80%+** |
| 구현 복잡도 | - | **중간** |

---

## 6. 구현 계획

### Phase 1: ObstacleMap 확장 (1일)
- [ ] `int getCost(GridPoint)` 추가
- [ ] `markEdgePath()`, `clearEdgePath()` 추가
- [ ] 비용 상수 정의

### Phase 2: AStarPathFinder 수정 (1일)
- [ ] `calculateMoveCost()` 동적 비용 사용
- [ ] 방향 전환 비용 추가
- [ ] 교차 비용 추가

### Phase 3: Rip-up and Reroute (2일)
- [ ] `ripUpAndReroute()` 구현
- [ ] EdgeRouting에 통합
- [ ] 최대 반복 횟수 설정

### Phase 4: 테스트 및 튜닝 (1일)
- [ ] 기존 테스트 통과 확인
- [ ] 50 drags 테스트 → 0 시각적 overlap 확인
- [ ] 비용 상수 튜닝

---

## 7. 파일 구조

```
src/layout/sugiyama/
    ├── ObstacleMap.h         # 수정: 동적 비용 지원
    ├── ObstacleMap.cpp       # 수정
    ├── AStarPathFinder.cpp   # 수정: 비용 함수 개조
    └── EdgeRouting.cpp       # 수정: Rip-up and Reroute 추가
```

---

## 8. 참고: 수학적 배경

### 8.1 Channel Pre-allocation의 한계

수학적으로 증명된 사항:
- Same-Type Overlap: Left-Edge Algorithm으로 O(E log E)에 해결 가능
- Cross-Type Overlap: 짝수/홀수 Grid 분리로 해결 가능
- **그러나**: NodeEdge 선택, 노드 충돌 등 현실 제약으로 완벽한 보장 어려움

### 8.2 Dynamic Cost A*의 장점

```
Pre-allocation:
  "미리 계획" → 현실 제약에 취약

Dynamic Cost A*:
  "자연스럽게 피하기" → 현실 제약에 유연
  + Post-Nudging: "정 안 되면 시각적 분리"
```

### 8.3 Rip-up and Reroute의 역할

```
문제: A*는 순차적 → "먼저 계산된 엣지가 유리"
해결: 나중 엣지가 기존 엣지를 밀어내고 재계산 유도
결과: 순서 의존성 제거, 전역적 균형
```

---

## 9. 기존 시스템 상세

### 9.1 EdgeRouting 4단계 파이프라인 (현재)

```
updateSnapPositions() 호출 시:

┌─────────────────────────────────────────────────────────────────┐
│ Phase 1: 영향받는 연결 수집                                      │
│ - movedNodes에 연결된 엣지들 식별                                │
│ - 각 엣지의 source/target NodeEdge별로 그룹화                    │
│ - affectedConnections 맵 구축                                   │
└─────────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────────┐
│ Phase 2: Snap Position 계산                                     │
│ - 각 NodeEdge에서 snap index 재분배 필요 여부 확인               │
│ - SNAP_INDEX_UNASSIGNED(-1) 또는 중복 index 발견 시 재분배       │
│ - calculateSnapPositionQuantized()로 실제 좌표 계산              │
│ - skipBendPointRecalc=true면 index만 할당 (좌표는 유지)          │
└─────────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────────┐
│ Phase 3: 처리할 엣지 병합                                        │
│ - Phase 2에서 재분배된 엣지들                                    │
│ - 원래 영향받은 엣지들                                           │
│ - 중복 제거 후 processedEdges 집합 생성                          │
└─────────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────────┐
│ Phase 4: Bend Point 재계산                                       │
│ - skipBendPointRecalc=false인 경우에만 실행                      │
│ - IEdgeOptimizer.regenerateBendPoints() 호출                    │
│ - A* 또는 Geometric 알고리즘으로 경로 생성                       │
└─────────────────────────────────────────────────────────────────┘
```

### 9.2 IEdgeOptimizer 동작 흐름 (현재)

```
┌─────────────────────────────────────────────────────────────────┐
│                    IEdgeOptimizer                               │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│  optimize() 호출 시:                                            │
│  ┌───────────────────────────────────────────────────────────┐  │
│  │ 1. 각 엣지에 대해:                                         │  │
│  │    - 여러 후보 경로 생성 (NodeEdge 조합 시도)               │  │
│  │    - EdgePenaltySystem으로 각 후보 평가                    │  │
│  │    - 최저 penalty 후보 선택                                │  │
│  └───────────────────────────────────────────────────────────┘  │
│                                                                 │
│  regenerateBendPoints() 호출 시:                                │
│  ┌───────────────────────────────────────────────────────────┐  │
│  │ 1. ObstacleMap 구축 (노드들을 장애물로 등록)               │  │
│  │ 2. 각 엣지에 대해:                                         │  │
│  │    - source/target NodeEdge 유지                          │  │
│  │    - A* pathfinding으로 bend points 계산                  │  │
│  │    - 또는 Geometric으로 단순 경로 생성                     │  │
│  └───────────────────────────────────────────────────────────┘  │
│                                                                 │
├─────────────────────────────────────────────────────────────────┤
│  AStarEdgeOptimizer          │  GeometricEdgeOptimizer         │
│  - ObstacleMap 사용          │  - 단순 L-path, S-path          │
│  - A* pathfinding            │  - 빠름, 근사치                  │
│  - 장애물 회피               │  - 드래그 중 사용                │
└─────────────────────────────────────────────────────────────────┘
```

### 9.3 드래그/드롭 흐름 (현재)

```
[사용자 드래그 시작]
     │
     ▼
┌────────────────────────────────────────┐
│ LayoutUtils::handleDrag()              │
│ - DragAlgorithm에 따라 분기:           │
│   - Geometric: 빠른 예측 경로          │
│   - HideUntilDrop: 엣지 숨김           │
│   - AStar: 실시간 A* (느림)            │
└────────────────────────────────────────┘
     │
     ▼
[사용자 드래그 종료]
     │
     ▼ (debounce callback)
┌────────────────────────────────────────┐
│ LayoutUtils::updateEdgePositions()     │
│ - PostDragAlgorithm::AStar인 경우:     │
│   - EdgeRouting.updateSnapPositions()  │
│   - IEdgeOptimizer.optimize() 호출     │
└────────────────────────────────────────┘
```

### 9.4 현재 문제점

```
문제 1: 각 엣지 독립 계산
┌─────────────────────────────────────────────────────────────────┐
│ Edge 1 계산 → 완료                                              │
│ Edge 2 계산 → Edge 1 경로 모름 → 같은 채널 선택 → OVERLAP!      │
│ Edge 3 계산 → Edge 1,2 경로 모름 → 또 같은 채널 → OVERLAP!      │
└─────────────────────────────────────────────────────────────────┘

문제 2: ObstacleMap이 노드만 등록
┌─────────────────────────────────────────────────────────────────┐
│ 현재: 노드 = 장애물, 다른 엣지 = 무시                           │
│ 결과: A*가 다른 엣지 경로를 "빈 공간"으로 인식                   │
│       → 같은 공간으로 경로 생성 → OVERLAP!                       │
└─────────────────────────────────────────────────────────────────┘

문제 3: 순서 의존성
┌─────────────────────────────────────────────────────────────────┐
│ 먼저 계산된 엣지가 "좋은 채널" 선점                              │
│ 나중 엣지는 남은 채널 사용 → 품질 저하 또는 OVERLAP              │
└─────────────────────────────────────────────────────────────────┘
```

### 9.5 현재 구현된 컴포넌트 목록

```
┌─────────────────────────────────────────────────────────────────┐
│                     현재 구현된 시스템                           │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│  IEdgeOptimizer (Interface)                                    │
│  ├── AStarEdgeOptimizer      ← A* pathfinding 사용             │
│  └── GeometricEdgeOptimizer  ← 단순 기하학적 경로               │
│                                                                 │
│  ConstraintGateway (제약 검증 단일 진입점)                      │
│  └── ConstraintRegistry                                        │
│      ├── OrthogonalityConstraint     - 직교성 제약              │
│      ├── DirectionalSourcePenetrationConstraint - 소스 관통 제약│
│      └── DirectionalTargetPenetrationConstraint - 타겟 관통 제약│
│                                                                 │
│  EdgePenaltySystem                                             │
│  ├── SegmentOverlapPenalty   (200,000) - 세그먼트 겹침          │
│  ├── DirectionPenalty        (200,000) - 방향 제약 위반         │
│  ├── NodeCollisionPenalty    (200,000) - 노드 관통              │
│  ├── TooCloseSnapPenalty     (200,000) - Snap point 너무 가까움 │
│  ├── SelfOverlapPenalty      (200,000) - 자기 경로 백트래킹     │
│  ├── ForbiddenZonePenalty    (200,000) - 금지 영역              │
│  ├── SnapPointOverlapPenalty (200,000) - Snap point 겹침        │
│  ├── FixedEndpointPenalty    (200,000) - 고정 노드 엔드포인트   │
│  ├── ConstraintPenaltyAdapter(OrthogonalityConstraint)          │
│  ├── ConstraintPenaltyAdapter(DirectionalSourcePenetrationConstraint)│
│  ├── ConstraintPenaltyAdapter(DirectionalTargetPenetrationConstraint)│
│  └── PathIntersectionPenalty (1,000)   - 다른 엣지와 교차       │
│                                                                 │
│  ObstacleMap                                                   │
│  └── bool (blocked/not blocked) + cost API                     │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
```

### 9.6 3단계 평가 구조

```
┌─────────────────────────────────────────────────────────────────┐
│  Stage 1: 경로 탐색 (A* Pathfinding)                            │
│  ┌───────────────────────────────────────────────────────────┐  │
│  │ ObstacleMap Cost                                          │  │
│  │ - 역할: A*가 경로를 찾을 때 어디로 갈지 GUIDE              │  │
│  │ - 시점: 경로 탐색 중 (실시간)                              │  │
│  │ - 출력: 후보 경로들                                        │  │
│  └───────────────────────────────────────────────────────────┘  │
│                              │                                  │
│                              ▼                                  │
│  Stage 2: 제약 검증 (ConstraintGateway)                         │
│  ┌───────────────────────────────────────────────────────────┐  │
│  │ ConstraintGateway                                         │  │
│  │ - 역할: Hard Constraint 위반 여부 검증 (단일 진입점)       │  │
│  │ - 시점: 경로 탐색 직후 (유효성 검증)                       │  │
│  │ - 출력: 위반 목록 (ConstraintViolation[])                 │  │
│  │ - 특징: 어떤 알고리즘도 제약 우회 불가                     │  │
│  └───────────────────────────────────────────────────────────┘  │
│                              │                                  │
│                              ▼                                  │
│  Stage 3: 경로 평가 (EdgePenaltySystem)                         │
│  ┌───────────────────────────────────────────────────────────┐  │
│  │ EdgePenaltySystem                                         │  │
│  │ - 역할: 후보 경로들 중 최선을 EVALUATE/SELECT              │  │
│  │ - 시점: 제약 검증 후 (품질 평가)                           │  │
│  │ - 출력: 최종 선택된 경로                                   │  │
│  │ - 특징: ConstraintPenaltyAdapter로 제약 재사용             │  │
│  └───────────────────────────────────────────────────────────┘  │
└─────────────────────────────────────────────────────────────────┘

핵심 차이:
- ObstacleMap Cost: "어디로 가야 할까?" (탐색 유도)
- ConstraintGateway: "이 경로가 유효한가?" (제약 검증)
- EdgePenaltySystem: "이 경로가 좋은가?" (품질 평가)

제약 통합 (Single Source of Truth):
- IEdgeConstraint: 제약의 원본 정의 (ConstraintGateway에서 사용)
- ConstraintPenaltyAdapter: IEdgeConstraint를 IEdgePenalty로 래핑
- 결과: 동일한 제약 로직을 두 시스템에서 재사용, 중복 제거
```

### 9.7 새 아키텍처에서 변경되는 부분

| 컴포넌트 | 현재 | 변경 후 |
|----------|------|---------|
| **ObstacleMap** | bool (blocked/free) | int (비용 값) |
| **A* Cost** | 균일 비용 (1) | 동적 비용 (1~100) |
| **EdgePenaltySystem** | 유지 | 유지 (변경 없음) |
| **IEdgeOptimizer** | 유지 | 유지 (변경 없음) |
| **Rip-up and Reroute** | 없음 | 새로 추가 |

### 9.8 IEdgeOptimizer 인터페이스 (기존)

```cpp
// 기존 인터페이스 - 변경 없음
class IEdgeOptimizer {
public:
    virtual ~IEdgeOptimizer() = default;
    
    // Penalty 시스템 설정
    virtual void setPenaltySystem(std::shared_ptr<EdgePenaltySystem> system) = 0;
    virtual std::shared_ptr<EdgePenaltySystem> penaltySystem() const = 0;
    
    // 방향 보존 설정
    virtual void setPreserveDirections(bool preserve) = 0;
    virtual bool preserveDirections() const = 0;
    
    // 최적화 실행
    virtual std::unordered_map<EdgeId, EdgeLayout> optimize(
        const std::vector<Edge>& edges,
        std::unordered_map<EdgeId, EdgeLayout>& edgeLayouts,
        const std::unordered_map<NodeId, NodeLayout>& nodeLayouts) = 0;
    
    // 알고리즘 이름
    virtual std::string algorithmName() const = 0;
    
    // Bend point 재생성
    virtual void regenerateBendPoints(
        const std::vector<Edge>& edges,
        std::unordered_map<EdgeId, EdgeLayout>& edgeLayouts,
        const std::unordered_map<NodeId, NodeLayout>& nodeLayouts) = 0;
    
    // Penalty 평가
    virtual bool passesHardConstraints(
        const EdgeLayout& layout,
        const PenaltyContext& context) const = 0;
    virtual int calculatePenalty(
        const EdgeLayout& layout,
        const PenaltyContext& context) const = 0;
};
```

### 9.9 변경의 범위

```
변경되는 파일:
├── src/layout/sugiyama/ObstacleMap.cpp    ← 비용 기반으로 확장
├── src/layout/sugiyama/ObstacleMap.h      ← 비용 기반으로 확장
├── src/layout/sugiyama/AStarPathFinder.cpp ← 동적 비용 사용
└── src/layout/sugiyama/EdgeRouting.cpp    ← Rip-up and Reroute 추가

변경되지 않는 파일:
├── include/arborvia/layout/IEdgePenalty.h        ← 유지
├── include/arborvia/layout/EdgePenaltySystem.h   ← 유지
├── include/arborvia/layout/BuiltinPenalties.h    ← 유지
├── include/arborvia/layout/IEdgeOptimizer.h      ← 유지
├── src/layout/sugiyama/AStarEdgeOptimizer.cpp    ← 유지 (내부에서 ObstacleMap 사용)
└── src/layout/sugiyama/GeometricEdgeOptimizer.cpp ← 유지
```

### 9.10 Constraint Gateway 아키텍처

#### 9.10.1 문제 정의

기존 시스템의 문제점:
1. **제약 우회 가능**: 28개 파일에서 bendPoints 직접 수정 가능
2. **중복 로직**: 같은 제약이 여러 곳에서 구현됨 (DRY 위반)
3. **분산된 검증**: 각 알고리즘이 독자적으로 제약 검증

#### 9.10.2 해결 방안: 단일 진입점 아키텍처

```
┌─────────────────────────────────────────────────────────────────┐
│              ConstraintGateway (단일 진입점)                     │
│  - validate(): 모든 제약 검증                                   │
│  - 어떤 알고리즘도 이 게이트웨이를 통해서만 검증                 │
├─────────────────────────────────────────────────────────────────┤
│              ConstraintRegistry (제약 저장소)                    │
│  - OrthogonalityConstraint         : 모든 세그먼트가 수평/수직   │
│  - DirectionalSourcePenetrationConstraint: 중간 세그먼트→소스 노드 관통 금지│
│  - DirectionalTargetPenetrationConstraint: 중간 세그먼트→타겟 노드 관통 금지│
├─────────────────────────────────────────────────────────────────┤
│              ConstraintPenaltyAdapter (통합 레이어)              │
│  - IEdgeConstraint를 IEdgePenalty로 래핑                        │
│  - EdgePenaltySystem에서 동일한 제약 재사용                     │
├─────────────────────────────────────────────────────────────────┤
│              PathGenerators (알고리즘들)                         │
│  - AStarEdgeOptimizer                                           │
│  - GeometricEdgeOptimizer                                       │
│  - 새 알고리즘 추가 시 자동으로 제약 적용                       │
└─────────────────────────────────────────────────────────────────┘
```

#### 9.10.3 핵심 컴포넌트

**IEdgeConstraint 인터페이스**
```cpp
class IEdgeConstraint {
public:
    virtual ~IEdgeConstraint() = default;
    
    // 제약 검증 - 위반 목록 반환
    virtual std::vector<ConstraintViolation> check(
        const EdgeLayout& layout,
        const EdgeConstraintContext& context) const = 0;
    
    // 제약 이름
    virtual std::string name() const = 0;
    
    // Hard Constraint 여부
    virtual bool isHardConstraint() const { return true; }
};
```

**ConstraintPenaltyAdapter**
```cpp
class ConstraintPenaltyAdapter : public IEdgePenalty {
public:
    explicit ConstraintPenaltyAdapter(
        std::shared_ptr<IEdgeConstraint> constraint,
        float tolerance = 1.0f);
    
    // IEdgePenalty 구현
    int calculatePenalty(const EdgeLayout& candidate,
                         const PenaltyContext& context) const override;
    
    std::string name() const override {
        return constraint_->name();  // 원본 제약 이름 사용
    }
    
    bool isHardConstraint() const override {
        return constraint_->isHardConstraint();
    }
};
```

#### 9.10.4 공유 유틸리티

중복 제거를 위한 공통 기하학 함수:
```cpp
namespace geometry {
    // 세그먼트가 노드 내부를 관통하는지 검사
    // tolerance로 경계 터치 허용 (기본 1.0f)
    bool segmentPenetratesNodeInterior(
        const Point& p1, const Point& p2,
        const NodeLayout& node,
        float tolerance = 1.0f);
}
```

사용 위치:
- `DirectionalSourcePenetrationConstraint`
- `DirectionalTargetPenetrationConstraint`
- `ConstraintPenaltyAdapter` (내부적으로 위 제약 재사용)

#### 9.10.5 파일 구조

```
include/arborvia/layout/constraints/
├── ConstraintViolation.h         # 위반 타입 및 구조체
├── EdgeConstraintContext.h       # 제약 검증 컨텍스트
├── IEdgeConstraint.h             # 제약 인터페이스
├── ConstraintRegistry.h          # 제약 저장소
├── ConstraintGateway.h           # 단일 진입점 API
├── ConstraintPenaltyAdapter.h    # IEdgeConstraint→IEdgePenalty 어댑터
└── builtins/
    ├── OrthogonalityConstraint.h
    └── DirectionalPenetrationConstraint.h

src/layout/constraints/
├── ConstraintViolation.cpp
├── ConstraintRegistry.cpp
├── ConstraintGateway.cpp
├── ConstraintPenaltyAdapter.cpp
└── builtins/
    ├── OrthogonalityConstraint.cpp
    └── DirectionalPenetrationConstraint.cpp

include/arborvia/core/GeometryUtils.h
└── segmentPenetratesNodeInterior()  # 공유 유틸리티
```

#### 9.10.6 장점

| 항목 | 기존 | 개선 후 |
|------|------|---------|
| 제약 정의 | 2곳 (Constraint + Penalty) | 1곳 (Constraint만) |
| 중복 코드 | ~80줄 | 제거됨 |
| 새 제약 추가 | 2개 클래스 작성 필요 | 1개 클래스만 작성 |
| 제약 우회 | 가능 | 불가능 (Gateway 통과 필수) |
| 테스트 | 각각 테스트 필요 | 제약만 테스트하면 됨 |

---

## 10. 용어 정리

| 용어 | 정의 | 사용 위치 |
|------|------|----------|
| **ObstacleMap Cost** | A* 탐색 시 셀 이동 비용 | AStarPathFinder |
| **EdgePenalty** | 경로 품질 평가 점수 | EdgePenaltySystem |
| **Hard Constraint** | 위반 시 경로 거부 (200,000점) | IEdgePenalty |
| **Soft Penalty** | 위반 시 감점 (1,000점 등) | IEdgePenalty |
| **Rip-up** | 기존 경로 제거 | EdgeRouting |
| **Reroute** | 새 경로 재계산 | EdgeRouting |
| **Channel** | 경로의 중간 세그먼트 좌표 | 수학적 개념 |

---

## 11. 테스트 전략

### 11.1 단위 테스트

```cpp
// ObstacleMap 비용 테스트
TEST(ObstacleMapTest, GetCost_EmptyCell_ReturnsFree) {
    ObstacleMap map;
    EXPECT_EQ(map.getCost({5, 5}), ObstacleMap::COST_FREE);
}

TEST(ObstacleMapTest, MarkEdgePath_IncreaseCost) {
    ObstacleMap map;
    map.markEdgePath(1, {{5, 5}, {5, 6}, {5, 7}});
    EXPECT_EQ(map.getCost({5, 6}), ObstacleMap::COST_EDGE_PATH);
}

// Rip-up and Reroute 테스트
TEST(EdgeRoutingTest, RipUpAndReroute_ReducesOverlap) {
    // 의도적으로 겹치는 경로 생성
    // Rip-up and Reroute 실행
    // 겹침 감소 확인
}
```

### 11.2 통합 테스트

```cpp
// 50 드래그 테스트
TEST(EdgeRoutingIntegrationTest, FiftyDrags_ZeroVisualOverlap) {
    // 초기 레이아웃
    // 50번 무작위 드래그
    // 각 드래그 후 시각적 overlap 카운트
    EXPECT_EQ(visualOverlapCount, 0);
}
```

### 11.3 성능 테스트

```cpp
// Rip-up 수렴 테스트
TEST(PerformanceTest, RipUpConverges_WithinMaxIterations) {
    // 복잡한 그래프 생성
    // Rip-up and Reroute 실행
    // maxIterations 내에 수렴하는지 확인
}
```

---

**버전:** 0.4.0
**최종 업데이트:** 2025-12-12
**상태:** 구현 완료

**구현 상태:**
- ✅ IEdgePenalty, EdgePenaltySystem, BuiltinPenalties
- ✅ IEdgeOptimizer, AStarEdgeOptimizer, GeometricEdgeOptimizer
- ✅ ObstacleMap (bool 기반 blocking + cost API 구현)
- ✅ AStarPathFinder (균일 비용 - 성능 최적화)
- ✅ ObstacleMap 동적 비용 API (구현됨, A*에서는 미사용 - 성능 이유)
- ✅ Rip-up and Reroute (AStarEdgeOptimizer.regenerateBendPoints에 구현)
- ✅ LayoutOptions 연동 (enableRipUpAndReroute 등)
- ✅ ConstraintGateway, ConstraintRegistry (제약 검증 단일 진입점)
- ✅ IEdgeConstraint, ConstraintViolation (제약 인터페이스)
- ✅ OrthogonalityConstraint, DirectionalPenetrationConstraint (Built-in 제약)
- ✅ ConstraintPenaltyAdapter (IEdgeConstraint→IEdgePenalty 통합)
- ✅ geometry::segmentPenetratesNodeInterior (공유 유틸리티)

**아키텍처 변경 사항:**
- 원래 계획: A*에서 동적 비용(getCostForDirection) 사용
- 실제 구현: Blocking 기반 A* + Rip-up and Reroute
- 이유: 동적 비용 방식은 A* 탐색 공간을 과도하게 확장 (726ms → 9ms 성능 차이)
- 결과: blocking으로 overlap 방지

**v0.4.0 변경 사항 (Constraint Gateway):**
- ConstraintGateway: 제약 검증 단일 진입점 구현
- ConstraintPenaltyAdapter: IEdgeConstraint를 IEdgePenalty로 래핑하여 Single Source of Truth 확보
- 중복 제거: OrthogonalityPenalty, DirectionalPenetrationPenalty 삭제 (~80줄)
- 공유 유틸리티: geometry::segmentPenetratesNodeInterior()로 관통 검사 통합
