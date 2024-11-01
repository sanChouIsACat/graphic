find_package(GTest CONFIG REQUIRED)
include_directories(${GTEST_INCLUDE_DIRS})
include_directories("${CMAKE_CURRENT_SOURCE_DIR}/../include")


add_executable(
  graphic_component_test
  rasterizer_test.cpp
)

target_link_libraries(
  graphic_component_test
  PRIVATE GTest::gtest GTest::gtest_main GTest::gmock GTest::gmock_main
)

add_test(AllTestsInMain common_lib_test)