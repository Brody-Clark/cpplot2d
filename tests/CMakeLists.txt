# Add Catch2 (downloaded automatically if not present)
include(FetchContent)
FetchContent_Declare(
  catch2
  GIT_REPOSITORY https://github.com/catchorg/Catch2.git
  GIT_TAG v3.5.0
)
FetchContent_MakeAvailable(catch2)

# Automatically find all .cpp files in tests/
file(GLOB TEST_SOURCES CONFIGURE_DEPENDS *.cpp)

# Add test executable
add_executable(cxpplot_tests ${TEST_SOURCES})

# Link Catch2
target_link_libraries(cxpplot_tests PRIVATE Catch2::Catch2WithMain)

# Include library headers
target_include_directories(cxpplot_tests PRIVATE ../include)

# Register with CTest
include(CTest)
include(Catch)
catch_discover_tests(cxpplot_tests)
