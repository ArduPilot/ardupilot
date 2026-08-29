#include "Copter.h"

#if MODE_VALT_ENABLED

/*
 * VALT (velocity alt hold) flight mode.
 *
 * AltHold with the Flying state replaced: surface tracking is skipped and
 * pos_desired is snapped towards the estimate off-centre, so the position P
 * loop is bypassed and the pilot commands a rate.  Any baro offset picked up
 * during the manoeuvre cancels, because both sides of the error saw it.
 */

bool ModeVelAltHold::init(bool ignore_checks)
{
    if (!ModeAltHold::init(ignore_checks)) {
        return false;
    }

    // VALT does not run surface tracking, so nothing would update or time out
    // a terrain offset inherited from the previous mode.  It would otherwise
    // sit in the position error for the whole flight and defeat the snap
    // below, which assumes the error goes to zero.
    pos_control->init_pos_terrain_D_m(0);
#if AP_RANGEFINDER_ENABLED
    // a quick return to surface tracking would otherwise slew the cleared
    // offset back in on top of pos_desired instead of re-initialising it
    copter.surface_tracking.reset_target_on_next_update();
#endif

    ekf_pos_d_reset_count = ahrs.get_position_D_reset_count();

    return true;
}

// velocity-controlled Flying state
void ModeVelAltHold::alt_hold_run_flying(float &target_roll_rad, float &target_pitch_rad, float target_climb_rate_ms)
{
#if AP_AVOIDANCE_ALTHOLD_ENABLED
    // apply avoidance
    copter.avoid.adjust_roll_pitch_rad(target_roll_rad, target_pitch_rad, attitude_control->lean_angle_max_rad());
#endif

    // get avoidance adjusted climb rate
    target_climb_rate_ms = get_avoidance_adjusted_climbrate_ms(target_climb_rate_ms);

    // Send the commanded climb rate to the position controller
    pos_control->D_set_pos_target_from_climb_rate_ms(target_climb_rate_ms);

    // D_update_controller() moves pos_desired by an EKF height reset after
    // this, so snapping to the post-reset estimate would apply it twice
    const uint16_t reset_count = ahrs.get_position_D_reset_count();
    if (reset_count != ekf_pos_d_reset_count) {
        ekf_pos_d_reset_count = reset_count;
        return;
    }

    // Snap pos_desired towards the estimate so the position P loop does not
    // fight the pilot's stick input.  When the stick returns to centre the
    // snap stops, pos_desired freezes and position P holds height.
    if (!is_zero(target_climb_rate_ms)) {
        pos_control->set_pos_desired_U_m(pos_control->get_pos_estimate_U_m());
    }
}

#endif  // MODE_VALT_ENABLED
