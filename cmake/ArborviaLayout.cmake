# ArborVia Layout Module
# Layout algorithms and results

set(ARBORVIA_LAYOUT_SOURCES
    # Core layout files
    src/layout/LayoutResult.cpp
    src/layout/SugiyamaLayout.cpp
    src/layout/LayoutUtils.cpp
    src/layout/LayoutSerializer.cpp
    src/layout/ConstraintConfig.cpp

    # Sugiyama algorithm phases
    src/layout/sugiyama/phases/CycleRemoval.cpp
    src/layout/sugiyama/phases/LayerAssignment.cpp
    src/layout/sugiyama/phases/CrossingMinimization.cpp
    src/layout/sugiyama/phases/CoordinateAssignment.cpp

    # Sugiyama edge routing
    src/layout/sugiyama/routing/EdgeRouting.cpp
    src/layout/sugiyama/routing/PathCalculator.cpp
    src/layout/sugiyama/routing/EdgePathFixer.cpp
    src/layout/sugiyama/routing/RoutingOptimizer.cpp
    src/layout/sugiyama/routing/ChannelRouter.cpp
    src/layout/sugiyama/routing/EdgeValidator.cpp
    src/layout/sugiyama/routing/PathIntersection.cpp
    src/layout/sugiyama/routing/SelfLoopRouter.cpp
    src/layout/sugiyama/routing/PathCleanup.cpp

    # Pathfinding
    src/layout/pathfinding/AStarPathFinder.cpp
    src/layout/pathfinding/LShapedPathFinder.cpp
    src/layout/pathfinding/ObstacleMap.cpp

    # Optimization
    src/layout/optimization/EdgePenaltySystem.cpp
    src/layout/optimization/OptimizerConfig.cpp
    src/layout/optimization/OptimizerRegistry.cpp
    src/layout/optimization/BuiltinPenalties.cpp
    src/layout/optimization/astar/AStarEdgeOptimizer.cpp
    src/layout/optimization/astar/AStarOptimizerFactory.cpp
    src/layout/optimization/geometric/GeometricEdgeOptimizer.cpp
    src/layout/optimization/geometric/GeometricOptimizerFactory.cpp

    # Snap point system
    src/layout/snap/GridSnapCalculator.cpp
    src/layout/snap/SnapIndexManager.cpp
    src/layout/snap/SnapDistributor.cpp
    src/layout/snap/SnapPositionUpdater.cpp

    # Interactive
    src/layout/interactive/ManualLayoutManager.cpp
    src/layout/interactive/PathRoutingCoordinator.cpp
    src/layout/interactive/SnapPointController.cpp
    src/layout/interactive/ConstraintManager.cpp
    src/layout/interactive/ValidRegionCalculator.cpp
    src/layout/interactive/DragOptimizationHandler.cpp

    # Constraints
    src/layout/constraints/MinDistanceConstraint.cpp
    src/layout/constraints/ConstraintSolver.cpp

    # API (centralized controllers)
    src/layout/api/LayoutController.cpp

    # Routing utilities
    src/layout/routing/OrthogonalRouter.cpp
    src/layout/routing/EdgeNudger.cpp
    src/layout/routing/CooperativeRerouter.cpp
    src/layout/routing/UnifiedRetryChain.cpp
)

set(ARBORVIA_LAYOUT_HEADERS
    # Public API - Main entry point
    include/arborvia/layout/SugiyamaLayout.h

    # Public API - Interfaces
    include/arborvia/layout/api/ILayout.h
    include/arborvia/layout/api/IEdgeOptimizer.h
    include/arborvia/layout/api/IPathFinder.h
    include/arborvia/layout/api/IEdgeOptimizerFactory.h
    include/arborvia/layout/api/IEdgePenalty.h
    include/arborvia/layout/api/IObstacleProvider.h
    include/arborvia/layout/api/IDragConstraint.h
    include/arborvia/layout/api/ICycleRemoval.h
    include/arborvia/layout/api/ILayerAssignment.h
    include/arborvia/layout/api/ICrossingMinimization.h
    include/arborvia/layout/api/ICoordinateAssignment.h

    # Public API - Config types
    include/arborvia/layout/config/LayoutOptions.h
    include/arborvia/layout/config/LayoutResult.h
    include/arborvia/layout/config/LayoutTypes.h
    include/arborvia/layout/config/LayoutEnums.h
    include/arborvia/layout/config/ManualLayoutState.h
    include/arborvia/layout/config/ConstraintConfig.h
    include/arborvia/layout/config/OptimizerConfig.h
    include/arborvia/layout/config/MoveDirection.h

    # Public API - Constraints
    include/arborvia/layout/constraints/ConstraintSolver.h

    # Public API - Interactive
    include/arborvia/layout/interactive/ManualLayoutManager.h
    include/arborvia/layout/interactive/PathRoutingCoordinator.h
    include/arborvia/layout/interactive/SnapPointController.h

    # Public API - Utilities
    include/arborvia/layout/util/LayoutUtils.h
    include/arborvia/layout/util/LayoutSerializer.h
)
