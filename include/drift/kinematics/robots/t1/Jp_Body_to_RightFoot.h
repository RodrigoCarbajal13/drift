/*
 * T1-specific addition (not part of upstream UMich-CURLY/drift).
 * Computed at runtime via Pinocchio from the T1 MJCF, not Mathematica
 * closed-form (see src/kinematics/robots/t1/t1_pinocchio_model.h).
 */

#ifndef KINEMATICS_ROBOTS_T1_JP_BODY_TO_RIGHTFOOT_H
#define KINEMATICS_ROBOTS_T1_JP_BODY_TO_RIGHTFOOT_H
#include <Eigen/Dense>

// Position Jacobian of the right foot w.r.t. all 12 encoders (body frame).
// Nonzero only in columns 6-11 (this leg's own joints); columns 0-5 (the
// other leg) are exactly zero, since the T1's hip-to-foot chain for one leg
// does not depend on the other leg's joint angles.
Eigen::Matrix<double, 3, 12> Jp_Body_to_RightFoot(
    const Eigen::Matrix<double, 12, 1>& encoders);

#endif
