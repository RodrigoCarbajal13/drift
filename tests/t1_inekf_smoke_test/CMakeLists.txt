# Create the test binary
add_executable(${test_prefix}_t1_inekf_smoke_test t1_inekf_smoke_test.cpp)

# Link tests
target_link_libraries(
  ${test_prefix}_t1_inekf_smoke_test
  GTest::gtest_main
  drift
  yaml-cpp
)
gtest_discover_tests(${test_prefix}_t1_inekf_smoke_test)
