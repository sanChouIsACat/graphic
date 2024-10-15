cmake_minimum_required(VERSION 3.30)

# I'm using vcpkg, this will include the toolchain file. Remove this line if you're not using vcpkg
SET(CMAKE_TOOLCHAIN_FILE "C:/Program Files/vcpkg/scripts/buildsystems/vcpkg.cmake")

project(graphics)
enable_testing()
# set debug macro. This will be used in the code to enable/disable debug code
add_definitions(-DDEBUG_GRAPHICS)
# add_definitions(-DTRACE_GRAPHICS)

set(CMAKE_BINARY_DIR "${CMAKE_CURRENT_SOURCE_DIR}/build/${PROJECT_NAME}")

# include or find_package your own eigen3 library
include_directories("C:/Program Files (x86)/Eigen3/include")

# common code
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/common)





# transformation 
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/pa0)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/pa1)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/pa2)
