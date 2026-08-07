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
    float des_speed_ne_ms = 0.0f;
    float des_climb_rate_ms = pos_control->get_vel_desired_U_ms();

    if (pos_control->NE_is_active()) {
        des_speed_ne_ms = pos_control->get_vel_target_NED_ms().xy().length();
    }

    // takeoff logic

    if (flightmode->mode_number() == Mode::Number::THROW) {
        // throw mode never wants the takeoff expected EKF code
        gndeffect_state.takeoff_expected = false;
    } else if (motors->armed() && ap.land_complete) {
        // if we are armed and haven't yet taken off then we expect an imminent takeoff
        gndeffect_state.takeoff_expected = true;
    }

    // get altitude estimate
    float pos_d_m = 0;
    UNUSED_RESULT(AP::ahrs().get_relative_position_D_origin_float(pos_d_m));

    // if we aren't taking off yet, reset the takeoff timer, altitude and complete flag
    const bool throttle_up = flightmode->has_manual_throttle() && channel_throttle->get_control_in() > 0;
    if (!throttle_up && ap.land_complete) {
        gndeffect_state.takeoff_time_ms = tnow_ms;
        gndeffect_state.takeoff_alt_m = -pos_d_m;
    }

    // if we are in takeoff_expected and we meet the conditions for having taken off
    // end the takeoff_expected state
    if (gndeffect_state.takeoff_expected && (tnow_ms - gndeffect_state.takeoff_time_ms > 5000 || (-pos_d_m - gndeffect_state.takeoff_alt_m) > 0.50)) {
        gndeffect_state.takeoff_expected = false;
    }

    // landing logic
    Vector3f angle_target_rad = attitude_control->get_att_target_euler_rad();
    bool small_angle_request = cosf(angle_target_rad.x) * cosf(angle_target_rad.y) > cosf(radians(7.5f));
    Vector3f vel_ned_ms;
    bool xy_speed_low = AP::ahrs().get_velocity_NED(vel_ned_ms) && (vel_ned_ms.xy().length() < 1.25);
    bool xy_speed_demand_low = pos_control->NE_is_active() && des_speed_ne_ms <= 1.25;
    bool slow_horizontal = xy_speed_demand_low || (xy_speed_low && !pos_control->NE_is_active()) || (flightmode->mode_number() == Mode::Number::ALT_HOLD && small_angle_request);

    bool descent_demanded = pos_control->D_is_active() && des_climb_rate_ms < 0.0f;
    bool slow_descent_demanded = descent_demanded && des_climb_rate_ms >= -1.00;
    bool speed_low_d_ms = AP::ahrs().get_velocity_D(vel_ned_ms.z, vibration_check.high_vibes) && fabsf(vel_ned_ms.z) <= 0.6f;
    bool slow_descent = (slow_descent_demanded || (speed_low_d_ms && descent_demanded));

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
