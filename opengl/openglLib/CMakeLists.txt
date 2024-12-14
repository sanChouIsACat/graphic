cmake_minimum_required(VERSION 3.30)
project(opengl_lib)
# compiled for release
file(GLOB_RECURSE OPENGL_SOURCES 
"${CMAKE_CURRENT_SOURCE_DIR}/src/*.c" 
"${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp")

message(${ModelResourcePath})
add_library(opengl_lib ${OPENGL_SOURCES})
add_definitions(-DMODEL_RESOURCE_PATH="${ModelResourcePath}")
add_definitions(-DGLSLS_RESOURCE_PATH="${GlslsResourcePath}")
target_link_libraries(opengl_lib 
glfw
OpenGL::GL
glfw
)
target_include_directories(opengl_lib PUBLIC 
"${CMAKE_CURRENT_SOURCE_DIR}/include"
${GLFW_INCLUDE_DIRS})