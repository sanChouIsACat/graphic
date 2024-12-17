cmake_minimum_required(VERSION 3.30)
project(opengl)
# 查找 OpenGL 库
find_package(OpenGL REQUIRED)
find_package(GLFW3 REQUIRED)
set(ModelResourcePath "models")
set(GlslsResourcePath "glsls")

add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/openglLib")
add_library(common_libs INTERFACE)
target_link_libraries(common_libs INTERFACE
 common_lib
 graphic_component
 opengl_lib
 OpenGL::GL
 glfw
)

add_subdirectory(basicTexture)
add_subdirectory(cameraControl)

