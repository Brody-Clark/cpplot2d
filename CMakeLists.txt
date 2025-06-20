cmake_minimum_required(VERSION 3.14)
project(cxpplot LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 14)

# Library headers
include_directories(include)

# Enable testing
enable_testing()
add_subdirectory(tests)
