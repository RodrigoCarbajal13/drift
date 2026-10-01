/* ----------------------------------------------------------------------------
 * T1-specific addition (not part of upstream UMich-CURLY/drift).
 * -------------------------------------------------------------------------- */

/**
 *  @file   t1_kinematics.cpp
 *  @brief  Booster T1 specific kinematics solver and measurement container
 **/

#include "drift/kinematics/t1_kinematics.h"

#define NLEG 2    // number of legs
#define NAPL 6    // number of actuators per leg

using namespace t1_kinematics;
using namespace math;

namespace measurement::kinematics {
T1Kinematics::T1Kinematics() {
  positions_.setConstant(3, NLEG, 0);
  jacobians_.setConstant(3, NLEG * NAPL, 0);
  contacts_.setConstant(NLEG, 1, 0);
  encoders_.setConstant(NLEG * NAPL, 1, 0);
}

T1Kinematics::T1Kinematics(
    const Eigen::Matrix<double, Eigen::Dynamic, 1>& encoders,
    const Eigen::Matrix<double, Eigen::Dynamic, 1>& d_encoders,
    const Eigen::Matrix<bool, Eigen::Dynamic, 1>& contacts)
    : LeggedKinematicsMeasurement(encoders, d_encoders, contacts) {
  positions_.setConstant(3, NLEG, 0);
  jacobians_.setConstant(3, NLEG * NAPL, 0);
}

void T1Kinematics::ComputeKinematics() {
  positions_.col(LEFT) = p_Body_to_LeftFoot(encoders_);
  positions_.col(RIGHT) = p_Body_to_RightFoot(encoders_);
  jacobians_.block(0, LEFT * NAPL, 3, NAPL)
      = Jp_Body_to_LeftFoot(encoders_).block(0, LEFT * NAPL, 3, NAPL);
  jacobians_.block(0, RIGHT * NAPL, 3, NAPL)
      = Jp_Body_to_RightFoot(encoders_).block(0, RIGHT * NAPL, 3, NAPL);
}

const Eigen::Vector3d T1Kinematics::get_init_velocity(
    const Eigen::Vector3d& w) {
  Eigen::Vector3d velocity = Eigen::Vector3d::Zero();

  if (this->get_contact(LEFT) == 1) {
    Eigen::Vector3d pL = p_Body_to_LeftFoot(encoders_);    // {I}_p_{IL}
    Eigen::Matrix<double, 3, 12> J_pL = Jp_Body_to_LeftFoot(encoders_);
    velocity = -J_pL * d_encoders_ - lie_group::skew(w) * pL;    // {I}_v_{WI}
  } else if (this->get_contact(RIGHT) == 1) {
    Eigen::Vector3d pR = p_Body_to_RightFoot(encoders_);    // {I}_p_{IR}
    Eigen::Matrix<double, 3, 12> J_pR = Jp_Body_to_RightFoot(encoders_);
    velocity = -J_pR * d_encoders_ - lie_group::skew(w) * pR;    // {I}_v_{WI}
  }
  return velocity;
}

int T1Kinematics::get_num_legs() { return NLEG; }
}    // namespace measurement::kinematics
