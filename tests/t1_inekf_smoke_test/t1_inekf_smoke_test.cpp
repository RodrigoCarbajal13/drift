// Smoke test: wires T1Kinematics + config/t1/*.yaml into a real
// InekfEstimator (IMU propagation + legged kinematics correction), the way a
// non-ROS, sim-only replay harness would. Confirms the T1 config files parse
// and the Init/RunOnce sequence produces a finite state -- it does not check
// filter accuracy (that needs a real trajectory, e.g. a teleop replay from
// booster-mjlab), only that the plumbing from this PR doesn't hang or NaN.
#include <gtest/gtest.h>
#include <cmath>
#include <memory>
#include "drift/estimator/inekf_estimator.h"
#include "drift/kinematics/t1_kinematics.h"

using namespace measurement;
using namespace estimator;
using namespace t1_kinematics;

TEST(T1InekfSmokeTest, InitAndOneRunOnceIsFinite) {
  inekf::ErrorType error_type = LeftInvariant;
  InekfEstimator inekf_estimator(error_type,
                                 "../config/t1/inekf_estimator.yaml");

  IMUQueue imu_buffer;
  IMUQueuePtr imu_buffer_ptr = std::make_shared<IMUQueue>(imu_buffer);
  LeggedKinQueue kin_buffer;
  LeggedKinQueuePtr kin_buffer_ptr
      = std::make_shared<LeggedKinQueue>(kin_buffer);

  // Two timesteps' worth of data: one to be consumed by Init*(), one for the
  // single RunOnce() call below.
  for (int i = 0; i < 2; ++i) {
    ImuMeasurement<double> imu;
    imu.set_angular_velocity(0, 0, 0);
    imu.set_lin_acc(0, 0, 9.81);
    imu.set_time(i * 0.02);
    imu_buffer_ptr->push(std::make_shared<ImuMeasurement<double>>(imu));

    Eigen::Matrix<double, 12, 1> encoders = Eigen::Matrix<double, 12, 1>::Zero();
    Eigen::Matrix<double, 12, 1> d_encoders
        = Eigen::Matrix<double, 12, 1>::Zero();
    Eigen::Matrix<bool, 2, 1> contacts;
    contacts << true, true;    // double stance, matches the neutral pose.
    auto kin = std::make_shared<kinematics::T1Kinematics>(encoders, d_encoders,
                                                           contacts);
    kin->set_time(i * 0.02);
    kin_buffer_ptr->push(kin);
  }

  auto imu_mutex = std::make_shared<std::mutex>();
  auto kin_mutex = std::make_shared<std::mutex>();
  inekf_estimator.add_imu_propagation(imu_buffer_ptr, imu_mutex,
                                      "../config/t1/imu_propagation.yaml");
  inekf_estimator.add_legged_kinematics_correction(
      kin_buffer_ptr, kin_mutex,
      "../config/t1/legged_kinematics_correction.yaml");

  // Mirrors tests/velocitycorrection_test.cpp's Init sequence.
  while (!inekf_estimator.is_enabled()) {
    if (inekf_estimator.BiasInitialized()) {
      inekf_estimator.InitState();
      inekf_estimator.EnableFilter();
    } else {
      inekf_estimator.InitBias();
    }
  }
  ASSERT_TRUE(inekf_estimator.is_enabled());

  inekf_estimator.RunOnce();

  const Eigen::MatrixXd X = inekf_estimator.get_state().get_X();
  for (int r = 0; r < X.rows(); ++r) {
    for (int c = 0; c < X.cols(); ++c) {
      EXPECT_TRUE(std::isfinite(X(r, c)))
          << "Non-finite state entry at (" << r << ", " << c << ")";
    }
  }
}
