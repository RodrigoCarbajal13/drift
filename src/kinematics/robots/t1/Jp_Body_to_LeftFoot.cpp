/* T1-specific addition (not part of upstream UMich-CURLY/drift). */

#include "drift/kinematics/robots/t1/Jp_Body_to_LeftFoot.h"

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>

#include "t1_pinocchio_model.h"

Eigen::Matrix<double, 3, 12> Jp_Body_to_LeftFoot(
    const Eigen::Matrix<double, 12, 1>& encoders) {
  const pinocchio::Model& model = t1_kinematics::GetModel();
  pinocchio::Data& data = t1_kinematics::GetThreadLocalData();
  const auto& names = t1_kinematics::LeftLegJointNames();
  Eigen::VectorXd q =
      t1_kinematics::NeutralWithLeg(model, names, encoders.head<6>());

  pinocchio::computeJointJacobians(model, data, q);
  pinocchio::framesForwardKinematics(model, data, q);

  pinocchio::Data::Matrix6x J_full(6, model.nv);
  J_full.setZero();
  pinocchio::getFrameJacobian(model, data, t1_kinematics::LeftFootFrameId(),
                               pinocchio::LOCAL_WORLD_ALIGNED, J_full);

  Eigen::Matrix<double, 3, 12> J = Eigen::Matrix<double, 3, 12>::Zero();
  for (int i = 0; i < 6; ++i) {
    const pinocchio::JointIndex jid = model.getJointId(names[i]);
    J.col(i) = J_full.block(0, model.idx_vs[jid], 3, 1);
  }
  return J;
}
