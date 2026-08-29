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

    return true;
}

// velocity-controlled Flying state
void ModeVelAltHold::alt_hold_run_flying(float &target_roll_rad, float &target_pitch_rad, float target_climb_rate_ms)
{
    // stick deflection is the pilot's demand, before avoidance clips it
    const float pilot_climb_rate_ms = target_climb_rate_ms;

#if AP_AVOIDANCE_ALTHOLD_ENABLED
    // apply avoidance
    copter.avoid.adjust_roll_pitch_rad(target_roll_rad, target_pitch_rad, attitude_control->lean_angle_max_rad());
#endif

    // get avoidance adjusted climb rate
    target_climb_rate_ms = get_avoidance_adjusted_climbrate_ms(target_climb_rate_ms);

    // Send the commanded climb rate to the position controller
    pos_control->D_set_pos_target_from_climb_rate_ms(target_climb_rate_ms);

    // Snap pos_desired towards the estimate so the position P loop does not
    // fight the pilot's stick input.  VALT_POS_EXPO weights the snap with
    // stick deflection; 0 keeps the original hard cutoff.
    const float expo = g2.valt_pos_expo;
    if (!is_positive(expo)) {
        if (!is_zero(target_climb_rate_ms)) {
            pos_control->set_pos_desired_U_m(pos_control->get_pos_estimate_U_m());
        }
        return;
    }

    // normalised stick deflection from centre, 0..1
    const float up_max = get_pilot_speed_up_ms();
    const float dn_max = get_pilot_speed_dn_ms();
    float deflection = 0.0f;
    if (is_positive(pilot_climb_rate_ms) && is_positive(up_max)) {
        deflection = pilot_climb_rate_ms / up_max;
    } else if (is_negative(pilot_climb_rate_ms) && is_positive(dn_max)) {
        deflection = -pilot_climb_rate_ms / dn_max;
    }
    deflection = constrain_float(deflection, 0.0f, 1.0f);

    // position-authority weight: a valley in deflection, 1 at centre and edge.
    // expo at or below 1 leaves this at 1 throughout, i.e. full position
    // authority everywhere (see VALT_POS_EXPO)
    const float w_pos = constrain_float(powf(1.0f - deflection, expo) + powf(deflection, expo), 0.0f, 1.0f);
    const float pos_est_m = pos_control->get_pos_estimate_U_m();
    const float pos_des_m = pos_control->get_pos_desired_U_m();
    pos_control->set_pos_desired_U_m(pos_est_m + w_pos * (pos_des_m - pos_est_m));
}

#endif  // MODE_VALT_ENABLED
