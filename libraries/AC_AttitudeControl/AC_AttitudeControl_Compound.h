#pragma once

/// @file    AC_AttitudeControl_Compound.h
/// @brief   ArduCopter attitude control library

// control library for forward thrust compound system
// possibly able to used for some new airframe with lateral thruster

#include "AC_AttitudeControl.h"
#include "AC_PosControl.h"
#include <AP_Motors/AP_MotorsMulticopter.h>

class AC_AttitudeControl_Compound : public AC_AttitudeControl_Multi {
public:
  AC_AttitudeControl_Compound(AP_AHRS_View &ahrs, const AP_Vehicle::MultiCopter &aparm, AP_MotorsMulticopter& motors, float dt);
	AC_AttitudeControl_Multi(AP_AHRS_View &ahrs, const AP_Vehicle::MultiCopter &aparm, AP_MotorsMulticopter& motors, float dt);
  AC_PosControl(const AP_AHRS_View& ahrs, const AP_InertialNav& inav,
                const AP_Motors& motors, AC_AttitudeControl& attitude_control,
                AC_P& p_pos_z, AC_P& p_vel_z, AC_PID& pid_accel_z,
                AC_P& p_pos_xy, AC_PI_2D& pi_vel_xy);

  AC_PID& throttle_pid;

  // empty destructor to suppress compiler warning
  virtual ~AC_AttitudeControl_Compound() {}

  //enable use of thruster from radio inputs;
  void AC_AttitudeControl_Compound::set_radio_passthrough_auxiliary_thruster();
  //switch to accel forward to use thruster instead of pitch down.
  void AC_AttitudeControl_Compound::accel_to_thrust();

  // accel forward command to rear thruster throttle
  void AC_AttitudeControl_Compound::run_auxiliary_thruster_controller();

  //float _thrust_target = 0.0f; // used in booster controller
  float _thrust_out = 0.0f;  // scaled throttle to be pass to motor controller.
