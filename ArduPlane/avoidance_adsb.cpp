
#include <stdio.h>
#include "Plane.h"

void Plane::avoidance_adsb_update(void)
{
    adsb.update();
    avoidance_adsb.update();
}


MAV_COLLISION_ACTION AP_Avoidance_Plane::handle_avoidance(const AP_Avoidance::Obstacle *obstacle, MAV_COLLISION_ACTION requested_action)
{
    MAV_COLLISION_ACTION actual_action = requested_action;
    bool failsafe_state_change = false;

    // check for changes in failsafe
    if (!plane.failsafe.adsb) {
        plane.failsafe.adsb = true;
        failsafe_state_change = true;
        // record flight mode in case it's required for the recovery
        prev_control_mode = plane.control_mode->mode_number();
    }

    // take no action in some flight modes
    if (plane.control_mode == &plane.mode_manual ||
        (plane.control_mode == &plane.mode_auto && !plane.auto_state.takeoff_complete) ||
        (plane.flight_stage == AP_Vehicle::FixedWing::FLIGHT_LAND) || // TODO: consider allowing action during approach
        plane.control_mode == &plane.mode_autotune ||
        plane.control_mode == &plane.mode_qland) {
        actual_action = MAV_COLLISION_ACTION_NONE;
    }

    // take action based on requested action
    switch (actual_action) {

        case MAV_COLLISION_ACTION_RTL:
            if (failsafe_state_change) {
                plane.set_mode(plane.mode_rtl, MODE_REASON_AVOIDANCE);
            }
            break;

        case MAV_COLLISION_ACTION_HOVER:
            if (failsafe_state_change) {
                if (plane.quadplane.is_flying()) {
                    plane.set_mode(plane.mode_qloiter, MODE_REASON_AVOIDANCE);
                } else {
                    plane.set_mode(plane.mode_loiter, MODE_REASON_AVOIDANCE);
                }
            }
            break;

        case MAV_COLLISION_ACTION_ASCEND_OR_DESCEND:
            // climb or descend to avoid obstacle
            if (handle_avoidance_vertical(obstacle, failsafe_state_change)) {
                plane.set_guided_WP();
            } else {
                actual_action = MAV_COLLISION_ACTION_NONE;
            }
            break;

        case MAV_COLLISION_ACTION_MOVE_HORIZONTALLY:
            // move horizontally to avoid obstacle
            if (handle_avoidance_horizontal(obstacle, failsafe_state_change)) {
                plane.set_guided_WP();
            } else {
                actual_action = MAV_COLLISION_ACTION_NONE;
            }
            break;

        case MAV_COLLISION_ACTION_MOVE_PERPENDICULAR:
        {
            // move horizontally and vertically to avoid obstacle
            const bool success_vert = handle_avoidance_vertical(obstacle, failsafe_state_change);
            const bool success_hor = handle_avoidance_horizontal(obstacle, failsafe_state_change);
            if (success_vert || success_hor) {
                plane.set_guided_WP();
            } else {
                actual_action = MAV_COLLISION_ACTION_NONE;
            }
        }
            break;

        // unsupported actions and those that require no response
        case MAV_COLLISION_ACTION_NONE:
            return actual_action;
        case MAV_COLLISION_ACTION_REPORT:
        default:
            break;
    }

    if (failsafe_state_change) {
        gcs().send_text(MAV_SEVERITY_ALERT, "Avoid: Performing action: %d", actual_action);
    }

    // return with action taken
    return actual_action;
}

void AP_Avoidance_Plane::handle_recovery(uint8_t recovery_action)
{
    // check we are coming out of failsafe
    if (plane.failsafe.adsb) {
        plane.failsafe.adsb = false;
        gcs().send_text(MAV_SEVERITY_INFO, "Avoid: Resuming with action: %d", recovery_action);

        // restore flight mode if requested and user has not changed mode since
        if (plane.control_mode_reason == MODE_REASON_AVOIDANCE) {
            switch (recovery_action) {

            case AP_AVOIDANCE_RECOVERY_REMAIN_IN_AVOID_ADSB:
                // do nothing, we'll stay in the AVOID_ADSB mode which is guided which will loiter
                break;

            case AP_AVOIDANCE_RECOVERY_RESUME_PREVIOUS_FLIGHTMODE: {
                Mode *new_mode = plane.mode_from_mode_num((enum Mode::Number)prev_control_mode);
                if (new_mode != nullptr) {
                    plane.set_mode(*new_mode, MODE_REASON_AVOIDANCE_RECOVERY);
                }
                }
                break;

            case AP_AVOIDANCE_RECOVERY_RTL:
                plane.set_mode(plane.mode_rtl, MODE_REASON_AVOIDANCE_RECOVERY);
                break;

            case AP_AVOIDANCE_RECOVERY_RESUME_IF_AUTO_ELSE_LOITER:
                if (prev_control_mode == plane.mode_auto) {
                    plane.set_mode(plane.mode_auto, MODE_REASON_AVOIDANCE_RECOVERY);
                }
                // else do nothing, same as AP_AVOIDANCE_RECOVERY_LOITER
                break;

            default:
                break;
            } // switch
        }
    }
}

// check flight mode is avoid_adsb
bool AP_Avoidance_Plane::check_flightmode(bool allow_mode_change)
{
    // ensure plane is in avoid_adsb mode
    if (allow_mode_change && plane.control_mode != &plane.mode_avoidADSB) {
        plane.set_mode(plane.mode_avoidADSB, MODE_REASON_AVOIDANCE);
    }

    // check flight mode
    return (plane.control_mode == &plane.mode_avoidADSB);
}

bool AP_Avoidance_Plane::handle_avoidance_vertical(const AP_Avoidance::Obstacle *obstacle, bool allow_mode_change)
{
    // ensure copter is in avoid_adsb mode
     if (!check_flightmode(allow_mode_change)) {
         return false;
     }

     // get best vector away from obstacle
     if (plane.current_loc.alt > obstacle->_location.alt) {
         // should climb
         plane.guided_WP_loc.alt = plane.current_loc.alt + 1000; // set alt demand to be 10m above us, climb rate will be TECS_CLMB_MAX
         return true;

     } else if (plane.current_loc.alt > plane.g.RTL_altitude_cm) {
         // should descend while above RTL alt
         // TODO: consider using a lower altitude than RTL_altitude_cm since it's default (100m) is quite high
         plane.guided_WP_loc.alt = plane.current_loc.alt - 1000; // set alt demand to be 10m below us, sink rate will be TECS_SINK_MAX
         return true;
     }

     return false;
}

bool AP_Avoidance_Plane::handle_avoidance_horizontal(const AP_Avoidance::Obstacle *obstacle, bool allow_mode_change)
{
    // ensure plane is in avoid_adsb mode
    if (!check_flightmode(allow_mode_change)) {
        return false;
    }

    // get best vector away from obstacle
    Vector3f velocity_neu;
    if (get_vector_perpendicular(obstacle, velocity_neu)) {
        // remove vertical component
        velocity_neu.z = 0.0f;

        // check for divide by zero
        if (is_zero(velocity_neu.x) && is_zero(velocity_neu.y)) {
            return false;
        }

        // re-normalize
        velocity_neu.normalize();

        // push vector further away.
        velocity_neu *= 10000;

        // set target
        location_offset(plane.guided_WP_loc, velocity_neu.x, velocity_neu.y);
        return true;
    }

    // if we got this far we failed to set the new target
    return false;
}

