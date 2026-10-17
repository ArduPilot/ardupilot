#pragma once

#include "AC_CustomControl_config.h"

#if AP_COPTER_CUSTOMCONTROL_INDI_ENABLED

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <AP_INDI/AP_INDI.h>

#include "AC_CustomControl_Backend.h"

/*
  INDI rate loop with online control-effectiveness identification,
  see libraries/AP_INDI/SPEC.md. The attitude loop is a fixed-gain P
  on the attitude error from the stock attitude controller's target,
  over-driven while the effectiveness is uncertain. Axes the
  identification gives up on are reported through get_fallback_mask()
  and the frontend leaves them to the stock rate PID.

  This backend builds on the earlier, unmerged INDI work by Emre
  Saldiran: the INDI custom controller backend of ArduPilot/ardupilot
  pull request 21699 (which this one replaces in the same type 3 slot)
  and the INDI controller of pull request 20511. Its optional motor-speed
  feedback (ACT_SRC 1) is the RPM feedback of 21699; see SPEC.md, prior
  art.
 */
class AC_CustomControl_INDI : public AC_CustomControl_Backend {
public:
    AC_CustomControl_INDI(AC_CustomControl& frontend, AP_AHRS_View*& ahrs, AC_AttitudeControl*& att_control, AP_MotorsMulticopter*& motors, float dt);

    Vector3f update() override;
    void reset(void) override;
    uint8_t get_fallback_mask() const override { return _indi.get_fallback_mask(); }

    static const struct AP_Param::GroupInfo var_info[];

protected:
    void init_indi();

    // actuator state from measured motor speed, in roll, pitch and yaw
    // command units; false if the frame or the telemetry cannot supply it
    bool motor_speed_actuator_state(Vector3f &u_measured);
    // assign measured motor speed to _motor_speed_meas_radps; false if
    // any of the four motors has no valid RPM
    bool get_motor_speed(void);

    float _dt;
    AP_INDI _indi;

    AP_Float _g_prior_rp;      // roll and pitch effectiveness prior
    AP_Float _g_prior_yaw;     // yaw effectiveness prior
    AP_Float _g_rel_sigma;     // trust in the prior
    AP_Float _rate_gain;       // rate loop gain, 1/s
    AP_Float _ang_bw;          // nominal attitude loop P, 1/s
    AP_Float _overdrive_max;   // attitude over-drive while uncertain
    AP_Float _act_tau;         // actuator lag time constant, s
    AP_Float _act_delay;       // actuator plus sensor delay, s
    AP_Float _du_min;          // minimum stroke, normalised
    AP_Int8 _pulses;           // excitation pulses enabled
    AP_Float _pulse_alt;       // minimum height above home for pulses, m
    AP_Int8 _act_src;          // 0 modelled actuator state, 1 motor speed when available
    AP_Float _throttle2motor_speed;  // motor speed at full throttle, rad/s (THR2RTRSPD of PR 21699)

    float _motor_speed_meas_radps[4];  // current motor speed in rad/s, quad-X order of PR 21699
    bool _act_src_warned;      // the frame-not-supported warning has been sent
};

#endif // AP_COPTER_CUSTOMCONTROL_INDI_ENABLED
