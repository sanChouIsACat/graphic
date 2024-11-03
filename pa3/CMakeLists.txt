cmake_minimum_required(VERSION 3.10)
project(Rasterizer)

find_package(OpenCV REQUIRED)

set(CMAKE_CXX_STANDARD 17)

include_directories(/usr/local/include ./include)

add_executable(Rasterizer3 main.cpp)
target_link_libraries(Rasterizer3 ${OpenCV_LIBRARIES})
target_link_libraries(Rasterizer3 common_lib)
target_link_libraries(Rasterizer3 graphic_component)

file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/models DESTINATION ${CMAKE_CURRENT_BINARY_DIR})