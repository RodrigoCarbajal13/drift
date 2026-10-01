# Create the test binary
add_executable(${test_prefix}_t1kinematics_test t1kinematics_test.cpp)

# Link tests
target_link_libraries(
  ${test_prefix}_t1kinematics_test
  GTest::gtest_main
  drift
)
gtest_discover_tests(${test_prefix}_t1kinematics_test)
