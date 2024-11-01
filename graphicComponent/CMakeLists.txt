cmake_minimum_required(VERSION 3.10)
project(graphic_component)

# for testing
enable_testing()
find_package(GTest CONFIG REQUIRED)
add_subdirectory(test)

find_package(OpenCV REQUIRED)

include_directories(/usr/local/include)
include_directories(../common/include)

file(GLOB G_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp")
file(GLOB G_HEADERS "${CMAKE_CURRENT_SOURCE_DIR}/include/*.hpp")

add_library(graphic_component ${G_SOURCES} ${G_HEADERS})
target_include_directories(common_lib PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")
target_link_libraries(graphic_component ${OpenCV_LIBRARIES} common_lib)

