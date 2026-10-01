/* ----------------------------------------------------------------------------
 * T1-specific addition (not part of upstream UMich-CURLY/drift).
 * -------------------------------------------------------------------------- */

#include "t1_pinocchio_model.h"

#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/parsers/mjcf.hpp>

#include <cstdlib>
#include <stdexcept>

namespace t1_kinematics {
namespace {

std::string ResolveMJCFPath() {
  if (const char* env = std::getenv("T1_MJCF_PATH")) {
    return std::string(env);
  }
  throw std::runtime_error(
      "T1_MJCF_PATH environment variable is not set. Point it at "
      "booster-mjlab's "
      "src/mjlab_playground/asset_zoo/robots/booster_t1/xmls/t1.xml");
}

pinocchio::Model BuildModel() {
  pinocchio::Model model;
  pinocchio::mjcf::buildModel(ResolveMJCFPath(), model);
  return model;
}

}    // namespace

const pinocchio::Model& GetModel() {
  static const pinocchio::Model model = BuildModel();
  return model;
}

pinocchio::Data& GetThreadLocalData() {
  thread_local pinocchio::Data data(GetModel());
  return data;
}

pinocchio::FrameIndex LeftFootFrameId() {
  static const pinocchio::FrameIndex id = GetModel().getFrameId("left_foot");
  return id;
}

pinocchio::FrameIndex RightFootFrameId() {
  static const pinocchio::FrameIndex id = GetModel().getFrameId("right_foot");
  return id;
}

const std::array<std::string, 6>& LeftLegJointNames() {
  static const std::array<std::string, 6> names = {
      "Left_Hip_Pitch",  "Left_Hip_Roll",   "Left_Hip_Yaw",
      "Left_Knee_Pitch", "Left_Ankle_Pitch", "Left_Ankle_Roll",
  };
  return names;
}

const std::array<std::string, 6>& RightLegJointNames() {
  static const std::array<std::string, 6> names = {
      "Right_Hip_Pitch",  "Right_Hip_Roll",   "Right_Hip_Yaw",
      "Right_Knee_Pitch", "Right_Ankle_Pitch", "Right_Ankle_Roll",
  };
  return names;
}

Eigen::VectorXd NeutralWithLeg(const pinocchio::Model& model,
                                const std::array<std::string, 6>& joint_names,
                                const Eigen::Matrix<double, 6, 1>& angles) {
  Eigen::VectorXd q = pinocchio::neutral(model);
  for (int i = 0; i < 6; ++i) {
    const pinocchio::JointIndex jid = model.getJointId(joint_names[i]);
    q[model.idx_qs[jid]] = angles[i];
  }
  return q;
}

}    // namespace t1_kinematics
