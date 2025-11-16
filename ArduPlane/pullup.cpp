#include "Plane.h"
/*
  support for pullup after an alitude wait. Used for high altitude gliders
 */

#if AP_PLANE_GLIDER_PULLUP_ENABLED

// Pullup control parameters
const AP_Param::GroupInfo GliderPullup::var_info[] = {

    // @Param: ENABLE
    // @DisplayName: Enable pullup after altitude wait
    // @Description: Enable pullup after altitude wait
    // @Values: 0:Disabled, 1:Enabled
    // @User: Advanced
    AP_GROUPINFO_FLAGS("ENABLE", 1, GliderPullup,  enable, 0, AP_PARAM_FLAG_ENABLE),

    // @Param: ELEV_OFS
    // @DisplayName: Elevator deflection used before starting pullup
    // @Description: Elevator deflection offset from -1 to 1 while waiting for airspeed to rise before starting close loop control of the pullup.
    // @Range: -1.0 1.0
    // @User: Advanced
    AP_GROUPINFO("ELEV_OFS", 2, GliderPullup,  elev_offset, 0.1f),

    // @Param: NG_LIM
    // @DisplayName: Maximum normal load factor during pullup
    // @Description: This is the nominal maximum value of normal load factor used during the closed loop pitch rate control of the pullup.
    // @Range: 1.0 4.0
    // @User: Advanced
    AP_GROUPINFO("NG_LIM", 3, GliderPullup,  ng_limit, 2.0f),

    // @Param: NG_JERK_LIM
    // @DisplayName: Maximum normal load factor rate of change during pullup
    // @Description: The normal load factor used for closed loop pitch rate control of the pullup will be ramped up to the value set by PUP_NG_LIM at the rate of change set by this parameter. The parameter value specified will be scaled internally by 1/EAS2TAS.
    // @Units: 1/s
    // @Range: 0.1 10.0
    // @User: Advanced
    AP_GROUPINFO("NG_JERK_LIM", 4, GliderPullup,  ng_jerk_limit, 4.0f),

    // @Param: PITCH
    // @DisplayName: Target pitch angle during pullup
    // @Description: The vehicle will attempt achieve this pitch angle during the pull-up maneouvre.
    // @Units: deg
    // @Range: -5 15
    // @User: Advanced
    AP_GROUPINFO("PITCH", 5, GliderPullup,  pitch_dem, 3),

    // @Param: ARSPD_START
    // @DisplayName: Pullup target airspeed
    // @Description: Target airspeed for initial airspeed wait
    // @Units: m/s
    // @Range: 0 100
    // @User: Advanced
    AP_GROUPINFO("ARSPD_START", 6, GliderPullup,  airspeed_start, 30),

    // @Param: PITCH_START
    // @DisplayName: Pullup target pitch
    // @Description: Target pitch for initial pullup
    // @Units: deg
    // @Range: -80 0
    // @User: Advanced
    AP_GROUPINFO("PITCH_START", 7, GliderPullup,  pitch_start, -60),
    
    AP_GROUPEND
};

// constructor
GliderPullup::GliderPullup(void)
{
    AP_Param::setup_object_defaults(this, var_info);
}

/*
  return true if in a pullup manoeuvre at the end of NAV_ALTITUDE_WAIT
*/
bool GliderPullup::in_pullup(void) const
{
    return plane.control_mode == &plane.mode_auto &&
        plane.mission.get_current_nav_cmd().id == MAV_CMD_NAV_ALTITUDE_WAIT &&
        stage != Stage::NONE;
}

/*
  start a pullup maneouvre, called when NAV_ALTITUDE_WAIT has reached
  altitude or exceeded descent rate
*/
bool GliderPullup::pullup_start(void)
{
    if (enable != 1) {
        return false;
    }

    // release balloon
    SRV_Channels::set_output_scaled(SRV_Channel::k_lift_release, 100);

    stage = Stage::WAIT_AIRSPEED;
    plane.auto_state.idle_mode = false;
    float aspeed;
    if (!plane.ahrs.airspeed_EAS(aspeed)) {
        aspeed = -1;
    }
    gcs().send_text(MAV_SEVERITY_INFO, "Start pullup airspeed %.1fm/s at %.1fm AMSL", aspeed, plane.current_loc.alt*0.01);
    return true;
}

/*
  first stage pullup from balloon release, verify completion
 */
bool GliderPullup::verify_pullup(void)
{
    const auto &ahrs = plane.ahrs;
    const auto &current_loc = plane.current_loc;
    const auto &aparm = plane.aparm;

    switch (stage) {
    case Stage::WAIT_AIRSPEED: {
        float aspeed;
        if (ahrs.airspeed_EAS(aspeed) && (aspeed > airspeed_start || ahrs.get_pitch_deg() > pitch_start)) {
            gcs().send_text(MAV_SEVERITY_INFO, "Pullup airspeed %.1fm/s alt %.1fm AMSL", aspeed, current_loc.alt*0.01);
            stage = Stage::WAIT_PITCH;
        }
        return false;
    }

    case Stage::WAIT_PITCH: {
        if (ahrs.get_pitch_deg() > pitch_start && fabsf(ahrs.get_roll_deg()) < 90) {
            gcs().send_text(MAV_SEVERITY_INFO, "Pullup pitch p=%.1f r=%.1f alt %.1fm AMSL",
                            ahrs.get_pitch_deg(),
                            ahrs.get_roll_deg(),
                            current_loc.alt*0.01);
            stage = Stage::WAIT_LEVEL;
        }
        return false;
    }

    case Stage::PUSH_NOSE_DOWN: {
        if (fabsf(ahrs.get_roll_deg()) < aparm.roll_limit) {
            stage = Stage::WAIT_LEVEL;
        }
        return false;
    }

    case Stage::WAIT_LEVEL: {
        // When pitch has raised past lower limit used by speed controller, wait for airspeed to approach
        // target value before handing over control of pitch demand to speed controller
        bool pitchup_complete = ahrs.get_pitch_deg() > MIN(0, aparm.pitch_limit_min);
        const float pitch_lag_time = 1.0f * sqrtf(ahrs.get_EAS2TAS());
        float aspeed;
        const float aspeed_derivative = (ahrs.get_accel().x + GRAVITY_MSS * ahrs.get_DCM_rotation_body_to_ned().c.x) / ahrs.get_EAS2TAS();
        bool airspeed_low = ahrs.airspeed_EAS(aspeed) ? (aspeed + aspeed_derivative * pitch_lag_time) < 0.01f * (float)plane.target_airspeed_cm : true;
        bool roll_control_lost = fabsf(ahrs.get_roll_deg()) > aparm.roll_limit;
        if (pitchup_complete && airspeed_low && !roll_control_lost) {
                gcs().send_text(MAV_SEVERITY_INFO, "Pullup level r=%.1f p=%.1f alt %.1fm AMSL",
                                ahrs.get_roll_deg(), ahrs.get_pitch_deg(), current_loc.alt*0.01);
                break;
        } else if (pitchup_complete && roll_control_lost) {
                // push nose down and wait to get roll control back
                gcs().send_text(MAV_SEVERITY_ALERT, "Pullup level roll bad r=%.1f p=%.1f",
                                ahrs.get_roll_deg(),
                                ahrs.get_pitch_deg());
                stage = Stage::PUSH_NOSE_DOWN;
        }
        return false;
    }
    case Stage::NONE:
        break;
    }

    // all done
    stage = Stage::NONE;
    return true;
}

/*
  stabilize during pullup from balloon drop
 */
void GliderPullup::stabilize_pullup(void)
{
    const float speed_scaler = plane.get_speed_scaler();
    switch (stage) {
    case Stage::WAIT_AIRSPEED: {
        plane.pitchController.reset_I();
        plane.yawController.reset_I();
        SRV_Channels::set_output_scaled(SRV_Channel::k_elevator, elev_offset*4500);
        SRV_Channels::set_output_scaled(SRV_Channel::k_rudder, 0);
        plane.nav_pitch_cd = 0;
        plane.nav_roll_cd = 0;
        SRV_Channels::set_output_scaled(SRV_Channel::k_aileron, plane.rollController.get_rate_out(0, speed_scaler));
        ng_demand = 0.0;
        break;
    }
    case Stage::WAIT_PITCH: {
        plane.yawController.reset_I();
        plane.nav_roll_cd = 0;
        plane.nav_pitch_cd = 0;
        SRV_Channels::set_output_scaled(SRV_Channel::k_rudder, 0);
        SRV_Channels::set_output_scaled(SRV_Channel::k_aileron, plane.rollController.get_rate_out(0, speed_scaler));
        float aspeed;
        const auto &ahrs = plane.ahrs;
        if (ahrs.airspeed_EAS(aspeed)) {
            // apply a rate of change limit to the ng pullup demand
            ng_demand += MAX(ng_jerk_limit / ahrs.get_EAS2TAS(), 0.1f) * plane.scheduler.get_loop_period_s();
            ng_demand = MIN(ng_demand, ng_limit);
            const float VTAS_ref = ahrs.get_EAS2TAS() * aspeed;
            const float pullup_accel = ng_demand * GRAVITY_MSS;
            const float demanded_rate_dps = degrees(pullup_accel / VTAS_ref);
            const uint32_t elev_trim_offset_cd = 4500.0f * elev_offset * (1.0f - ng_demand / ng_limit);
            SRV_Channels::set_output_scaled(SRV_Channel::k_elevator, elev_trim_offset_cd + plane.pitchController.get_rate_out(demanded_rate_dps, speed_scaler));
        } else {
            SRV_Channels::set_output_scaled(SRV_Channel::k_elevator, elev_offset*4500);
        }
        break;
    }
    case Stage::PUSH_NOSE_DOWN: {
        plane.nav_pitch_cd = plane.aparm.pitch_limit_min*100;
        plane.stabilize_pitch();
        plane.nav_roll_cd = 0;
        plane.stabilize_roll();
        plane.stabilize_yaw();
        ng_demand = 0.0f;
        break;
    }
    case Stage::WAIT_LEVEL:
        plane.nav_pitch_cd = MAX((plane.aparm.pitch_limit_min + 5), pitch_dem)*100;
        plane.stabilize_pitch();
        plane.nav_roll_cd = 0;
        plane.stabilize_roll();
        plane.stabilize_yaw();
        ng_demand = 0.0f;
        break;
    case Stage::NONE:
        break;
    }

    // we have done stabilisation
    plane.last_stabilize_ms = AP_HAL::millis();
}

#endif // AP_PLANE_GLIDER_PULLUP_ENABLED
