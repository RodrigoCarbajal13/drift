#include <gtest/gtest.h>
#include <iostream>
#include "drift/kinematics/t1_kinematics.h"

using namespace t1_kinematics;
using namespace measurement;

TEST(t1kinematicstest, DefaultCtor) {
  kinematics::T1Kinematics kin_data;
  EXPECT_EQ(kin_data.get_type(), LEGGED_KINEMATICS);

  for (size_t i = 0; i < 3; i++) {
    EXPECT_EQ(kin_data.get_kin_pos(LEFT)(i), 0);
  }

  EXPECT_EQ(kin_data.get_J(LEFT).rows(), 3);
  EXPECT_EQ(kin_data.get_J(LEFT).cols(), 6);
  for (size_t i = 0; i < 18; i++) {
    EXPECT_EQ(kin_data.get_J(LEFT)(i), 0);
  }

  for (size_t i = 0; i < 2; i++) {
    EXPECT_EQ(kin_data.get_contact(i), 0);
  }

  for (size_t i = 0; i < 12; i++) {
    EXPECT_EQ(kin_data.get_joint_state(i), 0);
  }
}

TEST(t1kinematicstest, JointStateSetGet) {
  kinematics::T1Kinematics kin_data;
  Eigen::Matrix<double, 12, 1> v;
  v << 0.123, 0.234, 0.345, 0.456, 0.567, 0.678, 0.789, 0.900, 1.011, 1.123,
      1.234, 1.345;
  kin_data.set_joint_state(v);
  for (size_t i = 0; i < 12; i++) {
    EXPECT_EQ(kin_data.get_joint_state(i), v[i]);
  }
  EXPECT_EQ(kin_data.get_num_legs(), 2);
}

TEST(t1kinematicstest, ContactsSetGet) {
  kinematics::T1Kinematics kin_data;
  Eigen::Matrix<bool, 2, 1> c;
  c << 0, 1;
  kin_data.set_contact(c);
  EXPECT_EQ(kin_data.get_contact(LEFT), false);
  EXPECT_EQ(kin_data.get_contact(RIGHT), true);
}

// Golden values: the all-zero joint configuration's foot positions were
// cross-validated directly against MuJoCo loading the same T1 MJCF (both
// position and Jacobian matched to ~1e-16; see booster-mjlab's
// validate_pinocchio_t1.py). This pins that regression in DRIFT itself.
TEST(t1kinematicstest, NeutralPositionMatchesMuJoCo) {
  Eigen::Matrix<double, 12, 1> encoders = Eigen::Matrix<double, 12, 1>::Zero();
  Eigen::Matrix<double, 12, 1> d_encoders
      = Eigen::Matrix<double, 12, 1>::Zero();
  Eigen::Matrix<bool, 2, 1> contacts;
  contacts << 1, 1;
  kinematics::T1Kinematics kin_data(encoders, d_encoders, contacts);
  kin_data.ComputeKinematics();

  Eigen::Vector3d expected_left(0.0485, 0.10625, -0.643354);
  Eigen::Vector3d expected_right(0.0485, -0.10625, -0.643354);
  EXPECT_NEAR((kin_data.get_kin_pos(LEFT) - expected_left).norm(), 0.0, 1e-5);
  EXPECT_NEAR((kin_data.get_kin_pos(RIGHT) - expected_right).norm(), 0.0,
              1e-5);
}

TEST(t1kinematicstest, JacobianAndPositionDependOnlyOnOwnLeg) {
  Eigen::Matrix<double, 12, 1> js;
  js << 0.1, 0.05, 0.02, -0.3, 0.1, 0.0,       // left
      0.1, -0.05, -0.02, -0.3, 0.1, 0.0;       // right
  Eigen::Matrix<double, 12, 1> js_vel = Eigen::Matrix<double, 12, 1>::Zero();
  Eigen::Matrix<bool, 2, 1> ct;
  ct << 1, 1;
  kinematics::T1Kinematics kin_data(js, js_vel, ct);
  kin_data.ComputeKinematics();
  Eigen::Matrix<double, 3, 6> J_left_before = kin_data.get_J(LEFT);
  Eigen::Matrix<double, 3, 6> J_right_before = kin_data.get_J(RIGHT);
  Eigen::Vector3d p_left_before = kin_data.get_kin_pos(LEFT);
  Eigen::Vector3d p_right_before = kin_data.get_kin_pos(RIGHT);

  // Perturb only the left leg's joints.
  js.head<6>() << 0.3, 0.0, 0.0, -0.5, 0.0, 0.0;
  kin_data.set_joint_state(js);
  kin_data.ComputeKinematics();

  EXPECT_NE(kin_data.get_J(LEFT), J_left_before);
  EXPECT_EQ(kin_data.get_J(RIGHT), J_right_before);
  EXPECT_NE(kin_data.get_kin_pos(LEFT), p_left_before);
  EXPECT_EQ(kin_data.get_kin_pos(RIGHT), p_right_before);
}
