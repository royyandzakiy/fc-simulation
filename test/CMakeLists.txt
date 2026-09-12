# test/CMakeLists.txt
find_package(GTest CONFIG REQUIRED)

add_executable(
  unit_tests
  test_simple.cpp
  # ${PROJECT_SOURCE_DIR}/src/main.cpp
)

# target_include_directories(unit_tests PRIVATE ${PROJECT_SOURCE_DIR}/include)

target_link_libraries(unit_tests PRIVATE GTest::gtest_main GTest::gmock)

include(GoogleTest)
gtest_discover_tests(unit_tests)