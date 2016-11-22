#include "Plane.h"

/*
  landing logic
 */

/*
  update navigation for landing. Called when on landing approach or
  final flare
 */
bool Plane::verify_land()
{
    // use rangefinder to correct if possible
    float height = height_above_target() - rangefinder_correction();

    return landing.verify_land(flight_stage, prev_WP_loc, next_WP_loc, current_loc,
            auto_state.takeoff_altitude_rel_cm, height, auto_state.sink_rate, auto_state.wp_proportion, auto_state.last_flying_ms, arming.is_armed(), is_flying(), rangefinder_state.in_range, throttle_suppressed);
}

