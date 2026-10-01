/*
 * T1-specific addition (not part of upstream UMich-CURLY/drift).
 * Computed at runtime via Pinocchio from the T1 MJCF, not Mathematica
 * closed-form (see src/kinematics/robots/t1/t1_pinocchio_model.h).
 */

#ifndef KINEMATICS_ROBOTS_T1_P_BODY_TO_LEFTFOOT_H
#define KINEMATICS_ROBOTS_T1_P_BODY_TO_LEFTFOOT_H
#include <Eigen/Dense>

// encoders: 12x1, [Left_Hip_Pitch, Left_Hip_Roll, Left_Hip_Yaw,
// Left_Knee_Pitch, Left_Ankle_Pitch, Left_Ankle_Roll, <same 6 for Right>].
// Depends only on this leg's own 6 encoders (entries 0-5).
Eigen::Matrix<double, 3, 1> p_Body_to_LeftFoot(
    const Eigen::Matrix<double, 12, 1>& encoders);

#endif
