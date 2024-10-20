
find_package(GTest CONFIG REQUIRED)
message("GTest found: ${GTEST_INCLUDE_DIRS}")
include_directories(${GTEST_INCLUDE_DIRS})
include_directories("${CMAKE_CURRENT_SOURCE_DIR}/../include")


add_executable(
  common_lib_test
  algebra_test.cpp
  transformation_test.cpp
  view_test.cpp
  interpolation_test.cpp
)

target_link_libraries(
  common_lib_test
  PRIVATE GTest::gtest GTest::gtest_main GTest::gmock GTest::gmock_main
  PRIVATE common_lib_trace
)

add_test(AllTestsInMain common_lib_test)
