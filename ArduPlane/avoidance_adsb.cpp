
#include <stdio.h>
#include "Plane.h"

#if HAL_ADSB_ENABLED
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
        prev_control_mode_number = plane.control_mode->mode_number();
    }

    // take no action in some flight modes
    bool flightmode_prohibits_action = false;
    if (plane.control_mode == &plane.mode_manual ||
        (plane.control_mode == &plane.mode_auto && !plane.auto_state.takeoff_complete) ||
        (plane.flight_stage == AP_FixedWing::FlightStage::LAND) || // TODO: consider allowing action during approach
        plane.control_mode == &plane.mode_autotune) {
        flightmode_prohibits_action = true;
    }
#if HAL_QUADPLANE_ENABLED
    if (plane.control_mode == &plane.mode_qland) {
        flightmode_prohibits_action = true;
    }
#endif
    if (flightmode_prohibits_action) {
        actual_action = MAV_COLLISION_ACTION_NONE;
    }

    // take action based on requested action
    switch (actual_action) {

        case MAV_COLLISION_ACTION_RTL:
            if (failsafe_state_change) {
                plane.set_mode(plane.mode_rtl, ModeReason::AVOIDANCE);
            }
            break;

        case MAV_COLLISION_ACTION_HOVER:
            if (failsafe_state_change) {
#if HAL_QUADPLANE_ENABLED
                if (plane.quadplane.is_flying()) {
                    plane.set_mode(plane.mode_qloiter, ModeReason::AVOIDANCE);
                    break;
                }
#endif
                plane.set_mode(plane.mode_loiter, ModeReason::AVOIDANCE);
            }
            break;

        case MAV_COLLISION_ACTION_ASCEND_OR_DESCEND: {
            // climb or descend to avoid obstacle
            Location loc = plane.next_WP_loc;
            if (handle_avoidance_vertical(obstacle, failsafe_state_change, loc)) {
                plane.set_guided_WP(loc);
            } else {
                actual_action = MAV_COLLISION_ACTION_NONE;
            }
            break;
        }
        case MAV_COLLISION_ACTION_MOVE_HORIZONTALLY: {
            // move horizontally to avoid obstacle
            Location loc = plane.next_WP_loc;
            if (handle_avoidance_horizontal(obstacle, failsafe_state_change, loc)) {
                plane.set_guided_WP(loc);
            } else {
                actual_action = MAV_COLLISION_ACTION_NONE;
            }
            break;
        }
        case MAV_COLLISION_ACTION_MOVE_PERPENDICULAR:
        {
            // move horizontally and vertically to avoid obstacle
            Location loc = plane.next_WP_loc;
            const bool success_vert = handle_avoidance_vertical(obstacle, failsafe_state_change, loc);
            const bool success_hor = handle_avoidance_horizontal(obstacle, failsafe_state_change, loc);
            if (success_vert || success_hor) {
                plane.set_guided_WP(loc);
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

void AP_Avoidance_Plane::handle_recovery(RecoveryAction recovery_action)
{
    // check we are coming out of failsafe
    if (plane.failsafe.adsb) {
        plane.failsafe.adsb = false;
        gcs().send_text(MAV_SEVERITY_INFO, "Avoid: Resuming with action: %u", (unsigned)recovery_action);

        // restore flight mode if requested and user has not changed mode since
        if (plane.control_mode_reason == ModeReason::AVOIDANCE) {
            switch (recovery_action) {

            case RecoveryAction::REMAIN_IN_AVOID_ADSB:
                // do nothing, we'll stay in the AVOID_ADSB mode which is guided which will loiter
                break;

            case RecoveryAction::RESUME_PREVIOUS_FLIGHTMODE:
                plane.set_mode_by_number(prev_control_mode_number, ModeReason::AVOIDANCE_RECOVERY);
                break;

            case RecoveryAction::RTL:
                plane.set_mode(plane.mode_rtl, ModeReason::AVOIDANCE_RECOVERY);
                break;

            case RecoveryAction::RESUME_IF_AUTO_ELSE_LOITER:
                if (prev_control_mode_number == Mode::Number::AUTO) {
                    plane.set_mode(plane.mode_auto, ModeReason::AVOIDANCE_RECOVERY);
                } else {
                    // let ModeAvoidADSB continue in its guided
                    // behaviour, but reset the loiter location,
                    // rather than where the avoidance location was
                    plane.set_guided_WP(plane.current_loc);
                }
                break;

            default:
                // user has specified an invalid recovery action;
                // loiter where we are
                plane.set_guided_WP(plane.current_loc);
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
        plane.set_mode(plane.mode_avoidADSB, ModeReason::AVOIDANCE);
    }

    // check flight mode
    return (plane.control_mode == &plane.mode_avoidADSB);
}

bool AP_Avoidance_Plane::handle_avoidance_vertical(const AP_Avoidance::Obstacle *obstacle, bool allow_mode_change, Location &new_loc)
{
    // ensure copter is in avoid_adsb mode
     if (!check_flightmode(allow_mode_change)) {
         return false;
     }

     // get best vector away from obstacle
     if (plane.current_loc.alt > obstacle->_location.alt) {
         // should climb
         new_loc.alt = plane.current_loc.alt + 1000; // set alt demand to be 10m above us, climb rate will be TECS_CLMB_MAX
         return true;

     } else if (plane.current_loc.alt > plane.g.RTL_altitude*100) {
         // should descend while above RTL alt
         // TODO: consider using a lower altitude than RTL_altitude since it's default (100m) is quite high
         new_loc.alt = plane.current_loc.alt - 1000; // set alt demand to be 10m below us, sink rate will be TECS_SINK_MAX
         return true;
     }

     return false;
}

bool AP_Avoidance_Plane::handle_avoidance_horizontal(const AP_Avoidance::Obstacle *obstacle, bool allow_mode_change, Location &new_loc)
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
        new_loc.offset(velocity_neu.x, velocity_neu.y);
        return true;
    }

    // if we got this far we failed to set the new target
    return false;
}

#endif // HAL_ADSB_ENABLED

