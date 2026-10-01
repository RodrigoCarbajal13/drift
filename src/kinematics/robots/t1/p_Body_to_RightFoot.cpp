/* T1-specific addition (not part of upstream UMich-CURLY/drift). */

#include "drift/kinematics/robots/t1/p_Body_to_RightFoot.h"

#include <pinocchio/algorithm/frames.hpp>

#include "t1_pinocchio_model.h"

Eigen::Matrix<double, 3, 1> p_Body_to_RightFoot(
    const Eigen::Matrix<double, 12, 1>& encoders) {
  const pinocchio::Model& model = t1_kinematics::GetModel();
  pinocchio::Data& data = t1_kinematics::GetThreadLocalData();
  Eigen::VectorXd q = t1_kinematics::NeutralWithLeg(
      model, t1_kinematics::RightLegJointNames(), encoders.tail<6>());
  pinocchio::framesForwardKinematics(model, data, q);
  return data.oMf[t1_kinematics::RightFootFrameId()].translation();
}
