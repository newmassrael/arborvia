cmake_minimum_required(VERSION 3.16)
project(arborvia VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Options
option(ARBORVIA_ENABLE_SVG_EXPORT "Enable SVG export module" ON)
option(ARBORVIA_BUILD_TESTS "Build tests" ON)
option(ARBORVIA_BUILD_EXAMPLES "Build examples" ON)

# Include module definitions
include(cmake/ArborviaCore.cmake)
include(cmake/ArborviaLayout.cmake)
include(cmake/ArborviaExport.cmake)

# Build library with core and layout modules
add_library(arborvia
    ${ARBORVIA_CORE_SOURCES}
    ${ARBORVIA_LAYOUT_SOURCES}
)

# Add export module if enabled
if(ARBORVIA_ENABLE_SVG_EXPORT)
    target_sources(arborvia PRIVATE ${ARBORVIA_EXPORT_SOURCES})
    target_compile_definitions(arborvia PUBLIC ARBORVIA_ENABLE_SVG_EXPORT)
endif()

# nlohmann/json (header-only library)
set(NLOHMANN_JSON_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/third_party/nlohmann_json/include)

target_include_directories(arborvia
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
        ${CMAKE_CURRENT_SOURCE_DIR}/src/layout
        ${NLOHMANN_JSON_INCLUDE_DIR}
)

# Compiler warnings
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(arborvia PRIVATE -Wall -Wextra -Wpedantic)
endif()

# Tests
if(ARBORVIA_BUILD_TESTS)
    enable_testing()
    include(FetchContent)
    FetchContent_Declare(
        googletest
        GIT_REPOSITORY https://github.com/google/googletest.git
        GIT_TAG v1.14.0
    )
    set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
    FetchContent_MakeAvailable(googletest)

    add_executable(arborvia_tests
        tests/core/GraphTest.cpp
        tests/core/CompoundGraphTest.cpp
        tests/layout/SugiyamaLayoutTest.cpp
        tests/layout/LayoutResultTest.cpp
        tests/layout/LayoutUtilsTest.cpp
        tests/layout/ManualLayoutTest.cpp
        tests/layout/SnapPointControllerTest.cpp
        tests/layout/sugiyama/SnapIndexManagerTest.cpp
        tests/layout/sugiyama/EdgeRoutingTransitionTest.cpp
        tests/layout/sugiyama/EdgeOptimizerTest.cpp
        tests/layout/sugiyama/SnapPointOverlapTest.cpp
        tests/layout/sugiyama/GridSnapCalculatorTest.cpp
        tests/layout/routing/CooperativeRerouterTest.cpp
        tests/layout/pathfinding/ObstacleMapTest.cpp
        tests/layout/manual/BendPointTest.cpp
        tests/layout/manual/OrthogonalDragTest.cpp
        tests/export/SvgExportTest.cpp
        tests/interactive/DragBehaviorTest.cpp
        tests/integration/DiagonalReproductionTest.cpp
    )
    target_link_libraries(arborvia_tests PRIVATE arborvia GTest::gtest_main)

    # Integration tests (require running demo subprocess)
    add_executable(arborvia_integration_tests
        tests/integration/DemoApiTest.cpp
    )
    target_include_directories(arborvia_integration_tests PRIVATE ${NLOHMANN_JSON_INCLUDE_DIR})
    target_link_libraries(arborvia_integration_tests PRIVATE GTest::gtest_main)

    include(GoogleTest)
    gtest_discover_tests(arborvia_tests)
    # Integration tests are not auto-discovered (require manual run with display)
endif()

# Examples (includes interactive demo with SDL3 + ImGui)
if(ARBORVIA_BUILD_EXAMPLES)
    set(EXAMPLES_OUTPUT_DIR ${CMAKE_BINARY_DIR}/examples)

    # Simple examples
    add_executable(visual_demo examples/visual_demo/main.cpp)
    target_link_libraries(visual_demo PRIVATE arborvia)
    set_target_properties(visual_demo PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${EXAMPLES_OUTPUT_DIR})

    add_executable(drag_demo examples/drag_demo/main.cpp)
    target_link_libraries(drag_demo PRIVATE arborvia)
    set_target_properties(drag_demo PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${EXAMPLES_OUTPUT_DIR})

    # Fetch SDL3
    FetchContent_Declare(
        SDL3
        GIT_REPOSITORY https://github.com/libsdl-org/SDL.git
        GIT_TAG release-3.2.0
        GIT_SHALLOW TRUE
    )
    set(SDL_SHARED OFF CACHE BOOL "" FORCE)
    set(SDL_STATIC ON CACHE BOOL "" FORCE)
    set(SDL_TEST OFF CACHE BOOL "" FORCE)
    FetchContent_MakeAvailable(SDL3)

    # Fetch ImGui
    FetchContent_Declare(
        imgui
        GIT_REPOSITORY https://github.com/ocornut/imgui.git
        GIT_TAG v1.91.6
        GIT_SHALLOW TRUE
    )
    FetchContent_MakeAvailable(imgui)

    # ImGui library
    add_library(imgui STATIC
        ${imgui_SOURCE_DIR}/imgui.cpp
        ${imgui_SOURCE_DIR}/imgui_draw.cpp
        ${imgui_SOURCE_DIR}/imgui_tables.cpp
        ${imgui_SOURCE_DIR}/imgui_widgets.cpp
        ${imgui_SOURCE_DIR}/imgui_demo.cpp
        ${imgui_SOURCE_DIR}/backends/imgui_impl_sdl3.cpp
        ${imgui_SOURCE_DIR}/backends/imgui_impl_sdlrenderer3.cpp
    )
    target_include_directories(imgui PUBLIC
        ${imgui_SOURCE_DIR}
        ${imgui_SOURCE_DIR}/backends
    )
    target_link_libraries(imgui PUBLIC SDL3::SDL3-static)

    # Interactive demo
    add_executable(interactive_demo examples/interactive_demo/main.cpp)
    target_link_libraries(interactive_demo PRIVATE arborvia imgui SDL3::SDL3-static)
    set_target_properties(interactive_demo PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${EXAMPLES_OUTPUT_DIR})

    if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
        target_compile_options(interactive_demo PRIVATE -Wall -Wextra)
    endif()
endif()

# Install
install(TARGETS arborvia EXPORT arboviaTargets)
install(DIRECTORY include/ DESTINATION include)
