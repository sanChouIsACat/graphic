cmake_minimum_required(VERSION 3.30)

# I'm using vcpkg, this will include the toolchain file. Remove this line if you're not using vcpkg
SET(CMAKE_TOOLCHAIN_FILE "C:/Program Files/vcpkg/scripts/buildsystems/vcpkg.cmake")
SET(CMAKE_CXX_STANDARD 17)
SET(CMAKE_CXX_STANDARD_REQUIRED ON)


project(graphics)

find_package(OpenCV REQUIRED)

enable_testing()
# set debug macro. This will be used in the code to enable/disable debug code
add_definitions(-DDEBUG_GRAPHICS)
# add_definitions(-DTRACE_GRAPHICS)

set(CMAKE_BINARY_DIR "${CMAKE_CURRENT_SOURCE_DIR}/build/${PROJECT_NAME}")

# include or find_package your own eigen3 library
include_directories("C:/Program Files (x86)/Eigen3/include")
# add boost libs.
include_directories("C:/Program Files (x86)/boost_1_86_0")

# add system libs
include_directories("./lib/include")

# common code like interpolation, logger, debugger, etc.
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/common)

# graphic component like rasterizer, texture, etc.
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/graphicComponent)




# homework 
include_directories(
    "./common/include"
    "./graphicComponent/include"
)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/pa0)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/pa1)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/pa2)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/pa3)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/pa4)
