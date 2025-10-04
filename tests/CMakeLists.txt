# Download Catch2 (or Doctest, if you prefer)
include(FetchContent)
FetchContent_Declare(
  catch2
  GIT_REPOSITORY https://github.com/catchorg/Catch2.git
  GIT_TAG v3.5.0
)
FetchContent_MakeAvailable(catch2)

# Add unit tests
add_subdirectory(unit)
