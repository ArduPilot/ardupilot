#include "Copter.h"

void Copter::update_ground_effect_detector(void)
{
    if(!g2.gndeffect_comp_enabled || !motors->armed()) {
        // disarmed - disable ground effect and return
        gndeffect_state.takeoff_expected = false;
        gndeffect_state.touchdown_expected = false;
        ahrs.set_takeoff_expected(gndeffect_state.takeoff_expected);
        ahrs.set_touchdown_expected(gndeffect_state.touchdown_expected);
        return;
    }

    // variable initialization
    uint32_t tnow_ms = millis();
    float xy_des_speed_cms = 0.0f;
    float xy_speed_cms = 0.0f;
    float des_climb_rate_cms = pos_control->get_vel_desired_cms().z;

    if (pos_control->is_active_xy()) {
        Vector3f vel_target = pos_control->get_vel_target_cms();
        vel_target.z = 0.0f;
        xy_des_speed_cms = vel_target.length();
    }

    if (position_ok() || ekf_has_relative_position()) {
        Vector3f vel = inertial_nav.get_velocity_neu_cms();
        vel.z = 0.0f;
        xy_speed_cms = vel.length();
    }

    // takeoff logic

    if (flightmode->mode_number() == Mode::Number::THROW) {
        // throw mode never wants the takeoff expected EKF code
        gndeffect_state.takeoff_expected = false;
    } else if (motors->armed() && ap.land_complete) {
        // if we are armed and haven't yet taken off then we expect an imminent takeoff
        gndeffect_state.takeoff_expected = true;
    }

    // if we aren't taking off yet, reset the takeoff timer, altitude and complete flag
    const bool throttle_up = flightmode->has_manual_throttle() && channel_throttle->get_control_in() > 0;
    if (!throttle_up && ap.land_complete) {
        gndeffect_state.takeoff_time_ms = tnow_ms;
        gndeffect_state.takeoff_alt_cm = inertial_nav.get_position_z_up_cm();
    }

    // if we are in takeoff_expected and we meet the conditions for having taken off
    // end the takeoff_expected state
    if (gndeffect_state.takeoff_expected && (tnow_ms-gndeffect_state.takeoff_time_ms > 5000 || inertial_nav.get_position_z_up_cm()-gndeffect_state.takeoff_alt_cm > 50.0f)) {
        gndeffect_state.takeoff_expected = false;
    }

    // landing logic
    Vector3f angle_target_rad = attitude_control->get_att_target_euler_cd() * radians(0.01f);
    bool small_angle_request = cosf(angle_target_rad.x)*cosf(angle_target_rad.y) > cosf(radians(7.5f));
    bool xy_speed_low = (position_ok() || ekf_has_relative_position()) && xy_speed_cms <= 125.0f;
    bool xy_speed_demand_low = pos_control->is_active_xy() && xy_des_speed_cms <= 125.0f;
    bool slow_horizontal = xy_speed_demand_low || (xy_speed_low && !pos_control->is_active_xy()) || (flightmode->mode_number() == Mode::Number::ALT_HOLD && small_angle_request);

    bool descent_demanded = pos_control->is_active_z() && des_climb_rate_cms < 0.0f;
    bool slow_descent_demanded = descent_demanded && des_climb_rate_cms >= -100.0f;
    bool z_speed_low = fabsf(inertial_nav.get_velocity_z_up_cms()) <= 60.0f;
    bool slow_descent = (slow_descent_demanded || (z_speed_low && descent_demanded));

    gndeffect_state.touchdown_expected = slow_horizontal && slow_descent;

    // Prepare the EKF for ground effect if either takeoff or touchdown is expected.
    ahrs.set_takeoff_expected(gndeffect_state.takeoff_expected);
    ahrs.set_touchdown_expected(gndeffect_state.touchdown_expected);
}

// update ekf terrain height stable setting
// when set to true, this allows the EKF to stabilize the normally barometer based altitude using a rangefinder
// this is not related to terrain following
void Copter::update_ekf_terrain_height_stable()
{
    // set to false if no position estimate
    if (!position_ok() && !ekf_has_relative_position()) {
        ahrs.set_terrain_hgt_stable(false);
        return;
    }

    // consider terrain height stable if vehicle is taking off or landing
    ahrs.set_terrain_hgt_stable(flightmode->is_taking_off() || flightmode->is_landing());
}
