find_package(GTest CONFIG REQUIRED)
include_directories(${GTEST_INCLUDE_DIRS})
include_directories(../../common/include)
include_directories("${CMAKE_CURRENT_SOURCE_DIR}/../include")
include_directories("${CMAKE_CURRENT_SOURCE_DIR}/../include/debug")


add_executable(
  graphic_component_test
  rasterizer_test.cpp
  eventLoopTest.cpp
  windowTest.cpp
  algoTest.cpp
  lineDebuggerTest.cpp
)

target_link_libraries(
  graphic_component_test
  PRIVATE GTest::gtest GTest::gtest_main GTest::gmock GTest::gmock_main
  ${OpenCV_LIBRARIES}
  common_lib_trace
  graphic_component_trace
)


add_test(AllTestsInMain common_lib_test)