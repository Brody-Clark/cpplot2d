cmake_minimum_required(VERSION 3.14)
project(cpplot2d LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Library headers
include_directories(include)

# Enable testing
enable_testing()
add_subdirectory(tests)
