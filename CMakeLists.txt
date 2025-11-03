cmake_minimum_required(VERSION 3.15)
project(ccplot2d LANGUAGES CXX)

# === Header-only library ===
add_library(cpplot2d INTERFACE)

# Make "include/" available to anyone linking this lib
target_include_directories(cpplot2d INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# Require C++17 or higher
target_compile_features(cpplot2d INTERFACE cxx_std_17)

# === Integration test executable ===
add_executable(test_plot
    tests/integration/test_plot.cpp
)

target_link_libraries(test_plot PRIVATE cpplot2d)

# === Unit tests with Catch2 ===
# Option 1: Use FetchContent to grab Catch2 automatically
include(FetchContent)
FetchContent_Declare(
    catch2
    GIT_REPOSITORY https://github.com/catchorg/Catch2.git
    GIT_TAG v3.7.0  # use a stable version
)
FetchContent_MakeAvailable(catch2)

add_executable(test_cpplot2d
    tests/unit/test_cpplot2d.cpp
)

target_link_libraries(test_cpplot2d PRIVATE cpplot2d Catch2::Catch2WithMain)

# === Enable CTest ===
include(CTest)
include(Catch)
catch_discover_tests(test_cpplot2d)

# === Benchmarks with Google Benchmark ===
FetchContent_Declare(
    benchmark
    GIT_REPOSITORY https://github.com/google/benchmark.git
    GIT_TAG v1.8.3
    FIND_PACKAGE_ARGS NAMES benchmark
)
# Disable tests and install for Google Benchmark
set(BENCHMARK_ENABLE_TESTING OFF)
set(BENCHMARK_ENABLE_INSTALL OFF)
FetchContent_MakeAvailable(benchmark)

# Create benchmark executable
add_executable(bench_cpplot2d
    benchmarks/bench_cpplot2d.cpp
)
target_link_libraries(bench_cpplot2d PRIVATE 
    cpplot2d 
    benchmark::benchmark
)