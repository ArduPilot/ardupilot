#pragma once

#include "AP_CustomControl_config.h"

#if AP_PLANE_CUSTOMCONTROL_INDI_ENABLED

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <AP_INDI/AP_INDI.h>

#include "AP_CustomControl_Backend.h"

/*
  INDI rate loop with online control-effectiveness identification for a
  fixed-wing aircraft, see libraries/AP_INDI/SPEC.md. The three virtual
  actuators are aileron, elevator and rudder. Commands are scaled by
  dynamic pressure relative to a reference airspeed so the identified
  effectiveness is airspeed independent (SPEC.md section 4). Axes the
  identification gives up on are left to the stock controllers.

  The INDI law and the custom controller framework it plugs into follow
  the earlier, unmerged Copter INDI work by Emre Saldiran
  (ArduPilot/ardupilot pull requests 20511 and 21699); see SPEC.md,
  prior art.
 */
class AP_CustomControl_INDI : public AP_CustomControl_Backend {
public:
    AP_CustomControl_INDI(AP_CustomControl& frontend, float dt);

    void update() override;
    void reset(void) override;

    static const struct AP_Param::GroupInfo var_info[];

private:
    void init_indi();

    AP_INDI _indi;
    float _q_scale;            // dynamic pressure relative to the reference, last step
    bool _was_flying;
    bool _rate_warned;        // the loop-rate warning has been sent
    uint32_t _airborne_ms;    // when the vehicle left the ground, 0 while on it
    bool _airborne_latched;   // has been above the launch height this flight

    AP_Float _g_roll;          // aileron effectiveness prior at the reference airspeed
    AP_Float _g_pitch;         // elevator effectiveness prior at the reference airspeed
    AP_Float _g_yaw;           // rudder effectiveness prior at the reference airspeed
    AP_Float _aspd_ref;        // reference airspeed for the priors, m/s
    AP_Float _g_rel_sigma;     // trust in the priors
    AP_Float _rate_gain;       // rate loop gain, 1/s
    AP_Float _ang_bw;          // nominal attitude loop P, 1/s
    AP_Float _overdrive_max;   // attitude over-drive while uncertain
    AP_Float _act_tau;         // servo lag time constant, s
    AP_Float _act_delay;       // servo plus sensor delay, s
    AP_Float _du_min;          // minimum stroke, normalised
    AP_Int8 _pulses;           // excitation pulses enabled
    AP_Float _rate_max;        // largest roll or pitch rate the attitude loop asks for, deg/s
};

#endif // AP_PLANE_CUSTOMCONTROL_INDI_ENABLED
