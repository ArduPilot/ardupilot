#include "Copter.h"

#if AP_GROUNDEFFECT_ENABLED
void Copter::update_ground_effect_detector(void)
{
    AP_GroundEffect &gndeff = g2.ground_effect;

    gndeff.set_takeoff_expected(flightmode->mode_number() != Mode::Number::THROW);
    gndeff.set_high_vibrations(vibration_check.high_vibes);

    // ALT_HOLD has manual attitude and no NE controller, so a near-level
    // attitude target stands in for "pilot is asking for slow horizontal"
    bool pilot_slow_horizontal = false;
    if (flightmode->mode_number() == Mode::Number::ALT_HOLD) {
        const Vector3f angle_target_rad = attitude_control->get_att_target_euler_rad();
        pilot_slow_horizontal = cosf(angle_target_rad.x) * cosf(angle_target_rad.y) > cosf(radians(7.5f));
    }
    gndeff.set_pilot_demanding_slow_horizontal(pilot_slow_horizontal);

    const bool throttle_up = flightmode->has_manual_throttle() && channel_throttle->get_control_in() > 0;
    gndeff.update(motors->armed(), ap.land_complete, throttle_up);
}
#endif  // AP_GROUNDEFFECT_ENABLED

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
