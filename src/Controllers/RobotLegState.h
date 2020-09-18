/*! @file RobotLegState.h
 *  @brief Data from joint sensors
 */

#ifndef _ROBOTLEGSTATE_H
#define _ROBOTLEGSTATE_H

#include "Utilities/cppTypes.h"
#include "Dynamics/Quadruped.h"

struct LegData {
  float q_abad[4];
  float q_hip[4];
  float q_knee[4];
  float qd_abad[4];
  float qd_hip[4];
  float qd_knee[4];
};

struct LegCommand {
  float q_des_abad[4];
  float q_des_hip[4];
  float q_des_knee[4];

  float qd_des_abad[4];
  float qd_des_hip[4];
  float qd_des_knee[4];

  float kp_abad[4];
  float kp_hip[4];
  float kp_knee[4];

  float kd_abad[4];
  float kd_hip[4];
  float kd_knee[4];

  float tau_abad_ff[4];
  float tau_hip_ff[4];
  float tau_knee_ff[4];

  int32_t flags[4];

};

#endif