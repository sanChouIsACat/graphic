cmake_minimum_required (VERSION 2.8.11)
project (Transformation)

find_package(Eigen3 REQUIRED)
#add_subdirectory(.. parent)

set(CMAKE_BINARY_DIR "${CMAKE_CURRENT_SOURCE_DIR}/build/${PROJECT_NAME}")

add_executable (Transformation main.cpp)

