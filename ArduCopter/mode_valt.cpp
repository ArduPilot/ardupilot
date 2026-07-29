#include "Copter.h"

#if MODE_VALT_ENABLED

/*
 * VALT (velocity alt hold) flight mode.
 *
 * Inherits from AltHold, overriding only the Flying state.  Surface
 * tracking is skipped so its offsets do not fight pilot stick input, and
 * pos_desired is snapped towards the current position off-centre so the
 * position P loop is bypassed (velocity control).  VALT_POS_EXPO blends how
 * hard that snap is with stick deflection, so position authority returns near
 * centre (altitude hold) and at full deflection (position backstop); 0 keeps
 * the original hard cutoff.
 */

// velocity-controlled Flying state
void ModeVelAltHold::alt_hold_run_flying(float &target_roll_rad, float &target_pitch_rad, float target_climb_rate_ms)
{
    // get avoidance adjusted climb rate
    target_climb_rate_ms = get_avoidance_adjusted_climbrate_ms(target_climb_rate_ms);

    // Send the commanded climb rate to the position controller
    pos_control->D_set_pos_target_from_climb_rate_ms(target_climb_rate_ms);

    // Snap pos_desired towards the current position so the position P loop
    // does not fight the pilot's stick input -- this is what turns AltHold
    // into velocity control.  How hard it snaps is blended with stick
    // deflection so position authority is full at centre (altitude hold),
    // backed off in the mid range (clean velocity-rate feel), and returns at
    // full deflection (a held command leaves pos_desired marching so a
    // growing position error backstops a stuck velocity loop).  VALT_POS_EXPO
    // = 0 keeps the original hard cutoff.
    const float expo = g2.valt_pos_expo;
    if (expo <= 0) {
        if (!is_zero(target_climb_rate_ms)) {
            pos_control->set_pos_desired_U_m(pos_control->get_pos_estimate_U_m());
        }
        return;
    }

    // normalised stick deflection from centre, 0..1
    const float up_max = get_pilot_speed_up_ms();
    const float dn_max = get_pilot_speed_dn_ms();
    float deflection = 0.0f;
    if (is_positive(target_climb_rate_ms) && is_positive(up_max)) {
        deflection = target_climb_rate_ms / up_max;
    } else if (is_negative(target_climb_rate_ms) && is_positive(dn_max)) {
        deflection = -target_climb_rate_ms / dn_max;
    }
    deflection = constrain_float(deflection, 0.0f, 1.0f);

    // position-authority weight: a valley in deflection, 1 at centre and edge
    const float w_pos = constrain_float(powf(1.0f - deflection, expo) + powf(deflection, expo), 0.0f, 1.0f);
    const float pos_est_m = pos_control->get_pos_estimate_U_m();
    const float pos_des_m = pos_control->get_pos_desired_U_m();
    pos_control->set_pos_desired_U_m(pos_est_m + w_pos * (pos_des_m - pos_est_m));
}

#endif  // MODE_VALT_ENABLED
