# ArborVia Layout Module
# Layout algorithms and results

set(ARBORVIA_LAYOUT_SOURCES
    src/layout/LayoutResult.cpp
    src/layout/SugiyamaLayout.cpp
    src/layout/sugiyama/CycleRemoval.cpp
    src/layout/sugiyama/LayerAssignment.cpp
    src/layout/sugiyama/CrossingMinimization.cpp
    src/layout/sugiyama/CoordinateAssignment.cpp
    src/layout/sugiyama/EdgeRouting.cpp
)

set(ARBORVIA_LAYOUT_HEADERS
    include/arborvia/layout/LayoutOptions.h
    include/arborvia/layout/LayoutResult.h
    include/arborvia/layout/ILayout.h
    include/arborvia/layout/SugiyamaLayout.h
)
