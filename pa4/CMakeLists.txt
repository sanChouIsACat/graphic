cmake_minimum_required(VERSION 3.10)
project(BezierCurve)

find_package(OpenCV REQUIRED)


include_directories(/usr/local/include ./include)

file(GLOB COMMON_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp")
file(GLOB COMMON_HEADERS "${CMAKE_CURRENT_SOURCE_DIR}/include/*.hpp")


add_executable(BezierCurve main.cpp 
${COMMON_SOURCES}
${COMMON_HEADERS} 
)

target_link_libraries(BezierCurve ${OpenCV_LIBRARIES})
target_link_libraries(BezierCurve common_lib)
target_link_libraries(BezierCurve graphic_component)
