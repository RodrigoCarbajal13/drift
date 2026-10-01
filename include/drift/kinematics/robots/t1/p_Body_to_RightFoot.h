/*
 * T1-specific addition (not part of upstream UMich-CURLY/drift).
 * Computed at runtime via Pinocchio from the T1 MJCF, not Mathematica
 * closed-form (see src/kinematics/robots/t1/t1_pinocchio_model.h).
 */

#ifndef KINEMATICS_ROBOTS_T1_P_BODY_TO_RIGHTFOOT_H
#define KINEMATICS_ROBOTS_T1_P_BODY_TO_RIGHTFOOT_H
#include <Eigen/Dense>

// encoders: 12x1, [<6 for Left>, Right_Hip_Pitch, Right_Hip_Roll,
// Right_Hip_Yaw, Right_Knee_Pitch, Right_Ankle_Pitch, Right_Ankle_Roll].
// Depends only on this leg's own 6 encoders (entries 6-11).
Eigen::Matrix<double, 3, 1> p_Body_to_RightFoot(
    const Eigen::Matrix<double, 12, 1>& encoders);

#endif
