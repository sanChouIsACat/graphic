
find_package(GTest CONFIG REQUIRED)
message("GTest found: ${GTEST_INCLUDE_DIRS}")
include_directories(${GTEST_INCLUDE_DIRS})
include_directories("${CMAKE_CURRENT_SOURCE_DIR}/../include")


add_executable(
  common_lib_test
  algebra_test.cpp
  transformation_test.cpp
  view_test.cpp
  "${CMAKE_CURRENT_SOURCE_DIR}/../algebra.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/../view.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/../transformation.cpp"
)

# set trace macro. trace log is most used in unit test.
target_compile_definitions(common_lib_test PRIVATE TRACE_GRAPHICS)
target_link_libraries(
  common_lib_test
  PRIVATE GTest::gtest GTest::gtest_main GTest::gmock GTest::gmock_main
)

add_test(AllTestsInMain common_lib_test)
