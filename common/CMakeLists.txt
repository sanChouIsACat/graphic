cmake_minimum_required(VERSION 3.30)
project(common)

# for testing
enable_testing()
find_package(GTest CONFIG REQUIRED)
add_subdirectory(test)


# 将common作为库进行构建
file(GLOB COMMON_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp")
file(GLOB COMMON_HEADERS "${CMAKE_CURRENT_SOURCE_DIR}/include/*.hpp")

add_library(common_lib ${COMMON_SOURCES} ${COMMON_HEADERS})

# 添加 include 路径，以便其他项目能够找到头文件
target_include_directories(common_lib PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")

