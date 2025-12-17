#include "Sub.h"

/*
 * SURFTRAK (surface tracking) -- a variation on ALT_HOLD (depth hold)
 *
 * SURFTRAK starts in the "reset" state (rangefinder_target_cm < 0). SURFTRAK exits the reset state when these
 * conditions are met:
 * -- There is a good rangefinder reading (the rangefinder is healthy, the reading is between min and max, etc.)
 * -- The sub is below SURFTRAK_DEPTH
 *
 * During normal operation, SURFTRAK sets the offset target to the current terrain altitude estimate and calls
 * AC_PosControl to do the rest.
 *
 * We generally do not want to reset SURFTRAK if the rangefinder glitches, since that will result in a new rangefinder
 * target. E.g., if a pilot is running 1m above the seafloor, there is a glitch, and the next rangefinder reading shows
 * 1.1m, the desired behavior is to move 10cm closer to the seafloor, vs setting a new target of 1.1m above the
 * seafloor.
 *
 * If the pilot takes control, SURFTRAK uses the change in depth readings to adjust the rangefinder target. This
 * minimizes the "bounce back" that can happen as the slower rangefinder catches up to the quicker barometer.
 */

#define INVALID_TARGET (-1)
#define HAS_VALID_TARGET (rangefinder_target_cm > 0)

ModeSurftrak::ModeSurftrak() :
        rangefinder_target_cm(INVALID_TARGET),
        pilot_in_control(false),
        pilot_control_start_z_cm(0)
{ }

bool ModeSurftrak::init(bool ignore_checks)
{
    if (!ModeAlthold::init(ignore_checks)) {
        return false;
    }

    reset();

    if (!sub.rangefinder_alt_ok()) {
        sub.gcs().send_text(MAV_SEVERITY_INFO, "waiting for a rangefinder reading");
#if AP_RANGEFINDER_ENABLED
    } else if (sub.inertial_nav.get_position_z_up_cm() >= sub.g.surftrak_depth) {
        sub.gcs().send_text(MAV_SEVERITY_WARNING, "descend below %f meters to hold range", sub.g.surftrak_depth * 0.01f);
#endif
    }

    return true;
}

void ModeSurftrak::run()
{
    run_pre();

    if (!motors.armed()) {
        // Forget rangefinder target
        reset();
    } else {
        control_range();
    }

    run_post();
}

/*
 * Set the rangefinder target, return true if successful. This may be called from scripting so run a few extra checks.
 */
bool ModeSurftrak::set_rangefinder_target_cm(float target_cm)
{
    bool success = false;

#if AP_RANGEFINDER_ENABLED
    if (sub.control_mode != Number::SURFTRAK) {
        sub.gcs().send_text(MAV_SEVERITY_WARNING, "wrong mode, rangefinder target not set");
    } else if (sub.inertial_nav.get_position_z_up_cm() >= sub.g.surftrak_depth) {
        sub.gcs().send_text(MAV_SEVERITY_WARNING, "descend below %f meters to set rangefinder target", sub.g.surftrak_depth * 0.01f);
    } else if (target_cm < sub.rangefinder_state.min*100) {
        sub.gcs().send_text(MAV_SEVERITY_WARNING, "rangefinder target below minimum, ignored");
    } else if (target_cm > sub.rangefinder_state.max*100) {
        sub.gcs().send_text(MAV_SEVERITY_WARNING, "rangefinder target above maximum, ignored");
    } else {
        success = true;
    }

    if (success) {
        rangefinder_target_cm = target_cm;
        sub.gcs().send_text(MAV_SEVERITY_INFO, "rangefinder target is %.2f meters", rangefinder_target_cm * 0.01f);

        // Initialize the terrain offset
        auto terrain_offset_cm = sub.inertial_nav.get_position_z_up_cm() - rangefinder_target_cm;
        sub.pos_control.init_pos_terrain_U_cm(terrain_offset_cm);

    } else {
        reset();
    }
#endif

    return success;
}

void ModeSurftrak::reset()
{
    rangefinder_target_cm = INVALID_TARGET;

    // Reset the terrain offset
    sub.pos_control.init_pos_terrain_U_cm(0);
}

/*
 * Main controller, call at 100hz+
 */
void ModeSurftrak::control_range() {
    float target_climb_rate_cms = sub.get_pilot_desired_climb_rate(channel_throttle->get_control_in());
    target_climb_rate_cms = constrain_float(target_climb_rate_cms, -sub.get_pilot_speed_dn(), g.pilot_speed_up);

    // Desired_climb_rate returns 0 when within the deadzone
    if (fabsf(target_climb_rate_cms) < 0.05f)  {
        if (pilot_in_control) {
            // Pilot has released control; apply the delta to the rangefinder target
            set_rangefinder_target_cm(rangefinder_target_cm + inertial_nav.get_position_z_up_cm() - pilot_control_start_z_cm);
            pilot_in_control = false;
        }
        if (sub.ap.at_surface) {
            // Set target depth to 5 cm below SURFACE_DEPTH and reset
            position_control->set_pos_desired_U_cm(MIN(position_control->get_pos_desired_U_cm(), g.surface_depth - 5.0f));
            reset();
        } else if (sub.ap.at_bottom) {
            // Set target depth to 10 cm above bottom and reset
            position_control->set_pos_desired_U_cm(MAX(inertial_nav.get_position_z_up_cm() + 10.0f, position_control->get_pos_desired_U_cm()));
            reset();
        } else {
            // Typical operation
            update_surface_offset();
        }
    } else if (HAS_VALID_TARGET && !pilot_in_control) {
        // Pilot has taken control; note the current depth
        pilot_control_start_z_cm = inertial_nav.get_position_z_up_cm();
        pilot_in_control = true;
    }

    // Set the target altitude from the climb rate and the terrain offset
    position_control->D_set_pos_target_from_climb_rate_cms(target_climb_rate_cms);

    // Run the PID controllers
    position_control->D_update_controller();
}

/*
 * Update the AC_PosControl terrain offset if we have a good rangefinder reading
 */
void ModeSurftrak::update_surface_offset()
{
#if AP_RANGEFINDER_ENABLED
    if (sub.rangefinder_alt_ok()) {
        // Get the latest terrain offset
        float rangefinder_terrain_offset_cm = sub.rangefinder_state.rangefinder_terrain_offset_cm;

        // Handle the first reading or a reset
        if (!HAS_VALID_TARGET && sub.rangefinder_state.inertial_alt_cm < sub.g.surftrak_depth) {
            set_rangefinder_target_cm(sub.rangefinder_state.inertial_alt_cm - rangefinder_terrain_offset_cm);
        }

        if (HAS_VALID_TARGET) {
            // Will the new offset target cause the sub to ascend above SURFTRAK_DEPTH?
            float desired_z_cm = rangefinder_terrain_offset_cm + rangefinder_target_cm;
            if (desired_z_cm >= sub.g.surftrak_depth) {
                // Adjust the terrain offset to stay below SURFTRAK_DEPTH, this should avoid "at_surface" events
                rangefinder_terrain_offset_cm += sub.g.surftrak_depth - desired_z_cm;
            }

            // Set the offset target, AC_PosControl will do the rest
            sub.pos_control.set_pos_terrain_target_U_cm(rangefinder_terrain_offset_cm);
        }
    }
#endif  // AP_RANGEFINDER_ENABLED
}
