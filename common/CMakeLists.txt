cmake_minimum_required(VERSION 3.30)
project(common)

# for testing
enable_testing()
find_package(GTest CONFIG REQUIRED)
add_subdirectory(test)


# 将common作为库进行构建
file(GLOB COMMON_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp")
file(GLOB COMMON_HEADERS "${CMAKE_CURRENT_SOURCE_DIR}/include/*.hpp")

# compiled for release
add_library(common_lib ${COMMON_SOURCES} ${COMMON_HEADERS})
target_include_directories(common_lib PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")

# compiled for trace
add_library(common_lib_trace ${COMMON_SOURCES} ${COMMON_HEADERS})
# set trace macro. trace log is most used in unit test.
target_compile_definitions(common_lib_trace PRIVATE TRACE_GRAPHICS)
target_include_directories(common_lib_trace PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")

