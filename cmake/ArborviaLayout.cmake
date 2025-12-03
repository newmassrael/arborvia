# ArborVia Layout Module
# Layout algorithms and results

set(ARBORVIA_LAYOUT_SOURCES
    src/layout/LayoutResult.cpp
    src/layout/SugiyamaLayout.cpp
    src/layout/ManualLayoutManager.cpp
    src/layout/LayoutUtils.cpp
    src/layout/LayoutSerializer.cpp
    src/layout/OrthogonalRouter.cpp
    src/layout/sugiyama/CycleRemoval.cpp
    src/layout/sugiyama/LayerAssignment.cpp
    src/layout/sugiyama/CrossingMinimization.cpp
    src/layout/sugiyama/CoordinateAssignment.cpp
    src/layout/sugiyama/EdgeRouting.cpp
    src/layout/sugiyama/SnapIndexManager.cpp
    src/layout/sugiyama/ObstacleMap.cpp
    src/layout/sugiyama/PathFinder.cpp
)

set(ARBORVIA_LAYOUT_HEADERS
    include/arborvia/layout/LayoutOptions.h
    include/arborvia/layout/LayoutResult.h
    include/arborvia/layout/LayoutTypes.h
    include/arborvia/layout/LayoutEnums.h
    include/arborvia/layout/ManualLayoutState.h
    include/arborvia/layout/ILayout.h
    include/arborvia/layout/SugiyamaLayout.h
    include/arborvia/layout/ManualLayout.h
    include/arborvia/layout/ManualLayoutManager.h
    include/arborvia/layout/LayoutUtils.h
    include/arborvia/layout/LayoutSerializer.h
    include/arborvia/layout/OrthogonalRouter.h
)
