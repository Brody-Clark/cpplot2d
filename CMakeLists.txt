cmake_minimum_required(VERSION 3.15)
project(ccplot2d LANGUAGES CXX OBJCXX)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# === Header-only library ===
add_library(cpplot2d INTERFACE)

# Make "include/" available to anyone linking this lib
target_include_directories(cpplot2d INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# Require C++17 or higher
target_compile_features(cpplot2d INTERFACE cxx_std_17)

# === Integration test executable ===
# Integration tests
if (APPLE)
    add_executable(test_plot_mac tests/integration/test_plot.mm)

    target_link_libraries(test_plot_mac PRIVATE
        cpplot2d
        "-framework Cocoa"
        "-framework CoreGraphics" 
    )

else()
    add_executable(test_plot tests/integration/test_plot.cpp)
    target_link_libraries(test_plot PRIVATE cpplot2d)
endif()

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

# # Define a custom target for the C++ wrapper
# add_executable(tidy_cpp ${CMAKE_CURRENT_SOURCE_DIR}/clang_tidy_cpp.cpp)
# target_link_libraries(tidy_cpp PRIVATE cpplot2d)
# set_target_properties(tidy_cpp PROPERTIES CXX_STANDARD 17)

# # Define a custom target for the Objective-C++ wrapper
# add_executable(tidy_objc ${CMAKE_CURRENT_SOURCE_DIR}/clang_tidy_objc.mm)
# target_link_libraries(tidy_objc PRIVATE cpplot2d)
# target_link_libraries(tidy_objc PRIVATE
#     "-framework Cocoa"
# )
# set_property(TARGET tidy_objc PROPERTY LANGUAGE OBJCXX)
# set_target_properties(tidy_objc PROPERTIES CXX_STANDARD 17)

# # Apply the Clang-Tidy property only to these wrapper targets:
# set_target_properties(tidy_cpp tidy_objc PROPERTIES
#     # Run the clang-tidy executable with the correct config
#     CMAKE_CXX_CLANG_TIDY "clang-tidy;
#     --config-file=${CMAKE_SOURCE_DIR}/.clang-tidy;"
# )

# # --- CMake Tidy Setup: Recursive Check ---

# # 1. FUNCTION: Runs Clang-Tidy recursively on all files in a directory
# function(ADD_TIDY_CHECKS_RECURSIVE TARGET_NAME DIR_PATH)
#     # Recursively find all C++ and Objective-C++ files
#     file(GLOB_RECURSE SOURCES
#         "${DIR_PATH}/*.cpp"
#         "${DIR_PATH}/*.cxx"
#         "${DIR_PATH}/*.cc"
#         "${DIR_PATH}/*.mm"
#     )

#     # Add a custom target to run clang-tidy on the discovered files
#     add_custom_target(${TARGET_NAME}
#         COMMAND clang-tidy 
#         # Pass the full list of discovered source files to clang-tidy
#         ${SOURCES}
#         # Tell clang-tidy where to find the compile_commands.json
#         -p=${CMAKE_BINARY_DIR}
#         # Pass the header filter and config file (as a single string)
#         --config-file=${CMAKE_SOURCE_DIR}/.clang-tidy
#         --header-filter=.*
#         # Tell CMake that this target does not create an output file
#         VERBATIM
#         USES_TERMINAL
#     )
# endfunction()


# # 2. INVOCATION: Create targets for tests and benchmarks
# ADD_TIDY_CHECKS_RECURSIVE(tidy-tests tests)
# ADD_TIDY_CHECKS_RECURSIVE(tidy-benchmarks benchmarks)


# # Create a single, convenient target to run both checks:
# add_custom_target(tidy-check ALL 
#     # DEPENDS on the wrapper files AND the recursive checks
#     DEPENDS 
#         tidy_cpp 
#         tidy_objc 
#         tidy-tests             # New recursive test check
#         tidy-benchmarks        # New recursive benchmark check
#     COMMENT "Running Clang-Tidy checks on wrapper files and recursive source files."
# )