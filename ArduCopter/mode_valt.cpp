#include "Copter.h"

#if MODE_VALT_ENABLED

/*
 * VALT (velocity alt hold) flight mode.
 *
 * Inherits from AltHold, overriding only the Flying state.  Surface
 * tracking is skipped so its offsets do not fight pilot stick input,
 * and pos_desired is overridden with the current position when the
 * stick is off-centre so the position P loop is bypassed.  When the
 * stick returns to centre pos_desired freezes and position P gently
 * holds height.
 */

// velocity-controlled Flying state
void ModeVelAltHold::alt_hold_run_flying(float &target_roll_rad, float &target_pitch_rad, float target_climb_rate_ms)
{
    // get avoidance adjusted climb rate
    target_climb_rate_ms = get_avoidance_adjusted_climbrate_ms(target_climb_rate_ms);

    // Send the commanded climb rate to the position controller
    pos_control->D_set_pos_target_from_climb_rate_ms(target_climb_rate_ms);

    // Override pos_desired with the current position so the position P
    // loop does not fight the pilot's stick input.  When the stick
    // returns to centre (zero climb rate), stop overriding so that
    // pos_desired freezes at the current altitude and position P gently
    // holds height.
    if (!is_zero(target_climb_rate_ms)) {
        pos_control->set_pos_desired_U_m(pos_control->get_pos_estimate_U_m());
    }
}

#endif  // MODE_VALT_ENABLED
