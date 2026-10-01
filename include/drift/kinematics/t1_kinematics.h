/* ----------------------------------------------------------------------------
 * T1-specific addition (not part of upstream UMich-CURLY/drift).
 * -------------------------------------------------------------------------- */

/**
 *  @file   t1_kinematics.h
 *  @brief  Booster T1 specific kinematics solver and measurement container.
 *
 *  Structurally mirrors MiniCheetahKinematics (2 legs instead of 4, 6
 *  actuated joints per leg instead of 3), but computes forward kinematics
 *  and Jacobians at runtime via Pinocchio (loaded from the T1's MJCF) rather
 *  than Mathematica-generated closed-form expressions -- see
 *  src/kinematics/robots/t1/t1_pinocchio_model.h for why, and the
 *  cross-validation against MuJoCo that preceded this.
 **/

#ifndef KINEMATICS_T1_KIN_H
#define KINEMATICS_T1_KIN_H

#include "drift/kinematics/robots/t1/Jp_Body_to_LeftFoot.h"
#include "drift/kinematics/robots/t1/Jp_Body_to_RightFoot.h"
#include "drift/kinematics/robots/t1/p_Body_to_LeftFoot.h"
#include "drift/kinematics/robots/t1/p_Body_to_RightFoot.h"
#include "drift/math/lie_group.h"
#include "drift/measurement/legged_kinematics.h"

namespace t1_kinematics {
enum Leg { LEFT, RIGHT };
}

using namespace t1_kinematics;
using namespace math;

namespace measurement::kinematics {
/**
 * @class T1Kinematics
 * @brief Booster T1 specific kinematics solver and measurement container
 *
 * Derived measurement class containing T1 specific kinematics information.
 */
class T1Kinematics : public LeggedKinematicsMeasurement {
 public:
  /// @name Constructor
  /// @{
  /**
   * @brief Default constructor. Will generate an empty measurement.
   */
  T1Kinematics();

  /**
   * @brief Constructor with encoder and contact information
   * @param[in] encoders Joint encoder values, [left 6, right 6]
   * @param[in] d_encoders Joint encoder velocity values
   * @param[in] contacts Contact information, [left, right]
   */
  T1Kinematics(const Eigen::Matrix<double, Eigen::Dynamic, 1>& encoders,
               const Eigen::Matrix<double, Eigen::Dynamic, 1>& d_encoders,
               const Eigen::Matrix<bool, Eigen::Dynamic, 1>& contacts);

  /// @}

  /**
   * @brief Compute kinematics and store in measurement
   */
  void ComputeKinematics() override;

  /**
   * @brief Get number of legs
   * @return Number of legs
   */
  int get_num_legs() override;

  /**
   * @brief Get initial velocity of the robot based on encoder values and
   * initial angular velocity
   * @param[in] w Initial angular velocity of the robot (rad/s)
   * @return Initial velocity
   */
  const Eigen::Vector3d get_init_velocity(const Eigen::Vector3d& w) override;
};
}    // namespace measurement::kinematics

#endif    // KINEMATICS_T1_KIN_H
