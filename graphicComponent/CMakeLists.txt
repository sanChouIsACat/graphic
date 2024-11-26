cmake_minimum_required(VERSION 3.10)
project(graphic_component)

# for testing
enable_testing()
find_package(GTest CONFIG REQUIRED)
add_subdirectory(test)


include_directories(/usr/local/include)
include_directories(../common/include)
include_directories(./include)

file(GLOB G_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp")
file(GLOB G_HEADERS "${CMAKE_CURRENT_SOURCE_DIR}/include/*.hpp")

add_library(graphic_component ${G_SOURCES} ${G_HEADERS})
add_library(graphic_component_trace ${G_SOURCES} ${G_HEADERS})
target_link_libraries(
    graphic_component 
    ${OpenCV_LIBRARIES}
    common_lib
)
target_link_libraries(
    graphic_component_trace
    ${OpenCV_LIBRARIES}
    common_lib_trace
)
target_compile_definitions(graphic_component_trace PRIVATE TRACE_GRAPHICS)

