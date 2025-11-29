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

target_include_directories(arborvia
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
        ${CMAKE_CURRENT_SOURCE_DIR}/src/layout
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
    )
    target_link_libraries(arborvia_tests PRIVATE arborvia GTest::gtest_main)

    include(GoogleTest)
    gtest_discover_tests(arborvia_tests)
endif()

# Examples
if(ARBORVIA_BUILD_EXAMPLES)
    add_executable(visual_test examples/visual_test.cpp)
    target_link_libraries(visual_test PRIVATE arborvia)
endif()

# Install
install(TARGETS arborvia EXPORT arboviaTargets)
install(DIRECTORY include/ DESTINATION include)
