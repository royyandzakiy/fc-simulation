find_package(GTest CONFIG REQUIRED)

add_executable(unit_tests
  test_link.cpp
  test_core.cpp
)

target_link_libraries(unit_tests PRIVATE fc::core GTest::gtest_main)

include(GoogleTest)
gtest_discover_tests(unit_tests)
