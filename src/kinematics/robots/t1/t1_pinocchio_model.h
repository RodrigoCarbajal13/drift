/* ----------------------------------------------------------------------------
 * T1-specific addition (not part of upstream UMich-CURLY/drift).
 * -------------------------------------------------------------------------- */

/**
 *  @file   t1_pinocchio_model.h
 *  @brief  Internal helper shared by the T1 per-leg kinematics/Jacobian
 *          functions. Unlike Mini Cheetah's Mathematica-generated closed-form
 *          expressions, the T1's forward kinematics and Jacobians are
 *          computed at runtime with Pinocchio, loading the same MJCF used by
 *          the booster-mjlab simulator -- cross-validated against MuJoCo
 *          directly (position and Jacobian both matched to ~1e-16) before
 *          this was written.
 **/

#ifndef KINEMATICS_ROBOTS_T1_PINOCCHIO_MODEL_H
#define KINEMATICS_ROBOTS_T1_PINOCCHIO_MODEL_H

#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>

#include <array>
#include <string>

namespace t1_kinematics {

// Lazily builds the T1 Pinocchio model once (path from the T1_MJCF_PATH
// environment variable -- point it at booster-mjlab's
// src/mjlab_playground/asset_zoo/robots/booster_t1/xmls/t1.xml). Throws
// std::runtime_error if the variable is unset. Thread-safe: C++11 guarantees
// static local initialization happens exactly once even under concurrent
// first calls.
const pinocchio::Model& GetModel();

// pinocchio::Data holds mutable scratch buffers that are not safe to share
// across threads; each calling thread gets its own, built against the one
// shared Model.
pinocchio::Data& GetThreadLocalData();

pinocchio::FrameIndex LeftFootFrameId();
pinocchio::FrameIndex RightFootFrameId();

// Joint order matches contact_estimator.py's LEFT_LEG_JOINTS/RIGHT_LEG_JOINTS
// in booster-mjlab, and therefore the encoder vector layout every T1
// kinematics function in this directory assumes: [left 6, right 6].
const std::array<std::string, 6>& LeftLegJointNames();
const std::array<std::string, 6>& RightLegJointNames();

// Returns a full-model neutral configuration with just this leg's 6 joint
// angles set (free-flyer and every other joint left at neutral/zero) -- the
// T1's hip-to-foot chain doesn't depend on anything else, same assumption
// Mini Cheetah's generated functions make.
Eigen::VectorXd NeutralWithLeg(const pinocchio::Model& model,
                                const std::array<std::string, 6>& joint_names,
                                const Eigen::Matrix<double, 6, 1>& angles);

}    // namespace t1_kinematics

#endif    // KINEMATICS_ROBOTS_T1_PINOCCHIO_MODEL_H
