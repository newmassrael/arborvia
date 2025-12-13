cmake_minimum_required(VERSION 3.16)

# ============================================================================
# Dependencies
# ============================================================================
find_package(Threads REQUIRED)

# ============================================================================
# Google Benchmark Setup
# ============================================================================
find_package(benchmark QUIET)

if(NOT benchmark_FOUND)
    message(STATUS "Google Benchmark not found, using FetchContent to download")

    include(FetchContent)
    FetchContent_Declare(
        googlebenchmark
        GIT_REPOSITORY https://github.com/google/benchmark.git
        GIT_TAG v1.8.3
        GIT_SHALLOW TRUE
        GIT_PROGRESS TRUE
    )

    # Benchmark options
    set(BENCHMARK_ENABLE_TESTING OFF CACHE BOOL "" FORCE)
    set(BENCHMARK_ENABLE_INSTALL OFF CACHE BOOL "" FORCE)
    set(BENCHMARK_ENABLE_GTEST_TESTS OFF CACHE BOOL "" FORCE)
    set(BENCHMARK_DOWNLOAD_DEPENDENCIES ON CACHE BOOL "" FORCE)

    FetchContent_MakeAvailable(googlebenchmark)
else()
    message(STATUS "Using system-installed Google Benchmark")
endif()

# ============================================================================
# Benchmark Executables
# ============================================================================

# Async Optimization Benchmarks
add_executable(benchmark_async_optimization
    AsyncOptimizationBenchmark.cpp
)
target_link_libraries(benchmark_async_optimization
    PRIVATE
        arborvia
        benchmark::benchmark
        benchmark::benchmark_main
        Threads::Threads
)
target_include_directories(benchmark_async_optimization
    PRIVATE
        ${CMAKE_SOURCE_DIR}/src
        ${CMAKE_SOURCE_DIR}/src/layout
)

# ============================================================================
# CTest Integration
# ============================================================================

# Quick benchmark test (runs with ctest)
add_test(
    NAME benchmark_async_optimization_quick
    COMMAND benchmark_async_optimization
        --benchmark_min_time=0.1s
        --benchmark_filter=Quick
        --benchmark_format=console
)

# Full benchmark test (manual execution only)
add_test(
    NAME benchmark_async_optimization_full
    COMMAND benchmark_async_optimization
)

# Configure test properties
set_tests_properties(benchmark_async_optimization_quick PROPERTIES
    LABELS "benchmark;quick;performance"
    TIMEOUT 300
)

set_tests_properties(benchmark_async_optimization_full PROPERTIES
    LABELS "benchmark;full;performance;manual"
    TIMEOUT 600
    DISABLED TRUE
)

# ============================================================================
# Custom Targets
# ============================================================================

# Run all benchmarks
add_custom_target(run_benchmarks
    COMMAND benchmark_async_optimization --benchmark_format=console
    DEPENDS benchmark_async_optimization
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
)

# Generate JSON results for comparison
add_custom_target(benchmark_json
    COMMAND ${CMAKE_COMMAND} -E make_directory ${CMAKE_BINARY_DIR}/benchmark_results
    COMMAND benchmark_async_optimization
        --benchmark_format=json
        --benchmark_out=${CMAKE_BINARY_DIR}/benchmark_results/async_optimization.json
    DEPENDS benchmark_async_optimization
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
)

# Quick performance check
add_custom_target(benchmark_quick
    COMMAND benchmark_async_optimization
        --benchmark_min_time=0.1s
        --benchmark_filter=Quick
    DEPENDS benchmark_async_optimization
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
)
