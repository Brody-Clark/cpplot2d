cmake_minimum_required(VERSION 3.15)
project(cpplot2d LANGUAGES CXX)

# Only enable OBJCXX if we are on an Apple platform
if (APPLE)
    enable_language(OBJCXX)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# === Header-only library ===
add_library(cpplot2d INTERFACE)

# Make "include/" available to anyone linking this lib
target_include_directories(cpplot2d INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# Require C++17 or higher
target_compile_features(cpplot2d INTERFACE cxx_std_17)

# === Fetch Catch2 for tests ===
include(FetchContent)
FetchContent_Declare(
    catch2
    GIT_REPOSITORY https://github.com/catchorg/Catch2.git
    GIT_TAG v3.7.0  # use a stable version
)
FetchContent_MakeAvailable(catch2)

# === Integration test executable ===
if (APPLE)
    add_executable(test_plot_mac tests/integration/test_plot.mm)

    target_link_libraries(test_plot_mac PRIVATE
        cpplot2d
        "-framework Cocoa"
        "-framework CoreGraphics" 
        Catch2::Catch2WithMain
    )
elseif (WIN32)
    add_executable(test_plot tests/integration/test_plot.cpp)
    target_link_libraries(test_plot PRIVATE cpplot2d gdi32 user32 Catch2::Catch2WithMain)
else()
    find_package(X11 REQUIRED)
    
    target_link_libraries(cpplot2d INTERFACE ${X11_LIBRARIES})
    target_include_directories(cpplot2d INTERFACE ${X11_INCLUDE_DIR})
    
    add_executable(test_plot tests/integration/test_plot.cpp)
    target_link_libraries(test_plot PRIVATE cpplot2d Catch2::Catch2WithMain)
endif()


#=== Unit test executable ===
if (APPLE)
    add_executable(test_cpplot2d tests/unit/test_cpplot2d.cpp)

    target_link_libraries(test_cpplot2d PRIVATE
        cpplot2d
        "-framework Cocoa"
        "-framework CoreGraphics" 
    )
elseif (WIN32)
    add_executable(test_cpplot2d tests/unit/test_cpplot2d.cpp)
    target_link_libraries(test_cpplot2d PRIVATE cpplot2d gdi32 user32)
else()
    find_package(X11 REQUIRED)
    
    target_link_libraries(cpplot2d INTERFACE ${X11_LIBRARIES})
    target_include_directories(cpplot2d INTERFACE ${X11_INCLUDE_DIR})
    
    add_executable(test_cpplot2d tests/unit/test_cpplot2d.cpp)
    target_link_libraries(test_cpplot2d PRIVATE cpplot2d)
endif()


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
