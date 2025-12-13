# Collect all test .cpp files
file(GLOB TEST_SOURCES CONFIGURE_DEPENDS *.cpp)

# Create test executable
add_executable(cpplot2d_tests ${TEST_SOURCES})

# Link Catch2
target_link_libraries(cpplot2d_tests PRIVATE Catch2::Catch2WithMain)

# Include library headers
target_include_directories(cpplot2d_tests PRIVATE ../../include)

# Register tests with CTest
include(CTest)
include(Catch)
catch_discover_tests(cpplot2d_tests)
