cmake_minimum_required(VERSION 3.30)
project(basicTexture)

# 源文件
set(SOURCES
    src/main.cpp
)

# 添加可执行文件
add_executable(basicTexture  ${SOURCES})
target_link_libraries(basicTexture PRIVATE common_libs)
file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/models DESTINATION ${CMAKE_CURRENT_BINARY_DIR})
file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/src/glsls DESTINATION ${CMAKE_CURRENT_BINARY_DIR})