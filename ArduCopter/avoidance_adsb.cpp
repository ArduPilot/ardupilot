#include "Copter.h"
#include <AP_Notify/AP_Notify.h>

#if HAL_ADSB_ENABLED
void Copter::avoidance_adsb_update(void)
{
    adsb.update();
    avoidance_adsb.update();
}

#include <stdio.h>

MAV_COLLISION_ACTION AP_Avoidance_Copter::handle_avoidance(const AP_Avoidance::Obstacle *obstacle, MAV_COLLISION_ACTION requested_action)
{
    MAV_COLLISION_ACTION actual_action = requested_action;
    bool failsafe_state_change = false;

    // check for changes in failsafe
    if (!copter.failsafe.adsb) {
        copter.failsafe.adsb = true;
        failsafe_state_change = true;
        // record flight mode in case it's required for the recovery
        prev_control_mode = copter.flightmode->mode_number();
    }

    // take no action in some flight modes
    if (copter.flightmode->mode_number() == Mode::Number::LAND ||
#if MODE_THROW_ENABLED
        copter.flightmode->mode_number() == Mode::Number::THROW ||
#endif
        copter.flightmode->mode_number() == Mode::Number::FLIP) {
        actual_action = MAV_COLLISION_ACTION_NONE;
    }

    // if landed and we will take some kind of action, just disarm
    if ((actual_action > MAV_COLLISION_ACTION_REPORT) && copter.should_disarm_on_failsafe()) {
        copter.arming.disarm(AP_Arming::Method::ADSBCOLLISIONACTION);
        actual_action = MAV_COLLISION_ACTION_NONE;
    } else {

        // take action based on requested action
        switch (actual_action) {

            case MAV_COLLISION_ACTION_RTL:
                // attempt to switch to RTL, if this fails (i.e. flying in manual mode with bad position) do nothing
                if (failsafe_state_change) {
                    if (!copter.set_mode(Mode::Number::RTL, ModeReason::AVOIDANCE)) {
                        actual_action = MAV_COLLISION_ACTION_NONE;
                    }
                }
                break;

            case MAV_COLLISION_ACTION_HOVER:
                // attempt to switch to Loiter, if this fails (i.e. flying in manual mode with bad position) do nothing
                if (failsafe_state_change) {
                    if (!copter.set_mode(Mode::Number::LOITER, ModeReason::AVOIDANCE)) {
                        actual_action = MAV_COLLISION_ACTION_NONE;
                    }
                }
                break;

            case MAV_COLLISION_ACTION_ASCEND_OR_DESCEND:
                // climb or descend to avoid obstacle
                if (!handle_avoidance_vertical(obstacle, failsafe_state_change)) {
                    actual_action = MAV_COLLISION_ACTION_NONE;
                }
                break;

            case MAV_COLLISION_ACTION_MOVE_HORIZONTALLY:
                // move horizontally to avoid obstacle
                if (!handle_avoidance_horizontal(obstacle, failsafe_state_change)) {
                    actual_action = MAV_COLLISION_ACTION_NONE;
                }
                break;

            case MAV_COLLISION_ACTION_MOVE_PERPENDICULAR:
                if (!handle_avoidance_perpendicular(obstacle, failsafe_state_change)) {
                    actual_action = MAV_COLLISION_ACTION_NONE;
                }
                break;

            // unsupported actions and those that require no response
            case MAV_COLLISION_ACTION_NONE:
                return actual_action;
            case MAV_COLLISION_ACTION_REPORT:
            default:
                break;
        }
    }

#if HAL_LOGGING_ENABLED
    if (failsafe_state_change) {
        AP::logger().Write_Error(LogErrorSubsystem::FAILSAFE_ADSB,
                                 LogErrorCode(actual_action));
    }
#endif

    // return with action taken
    return actual_action;
}

void AP_Avoidance_Copter::handle_recovery(RecoveryAction recovery_action)
{
    // check we are coming out of failsafe
    if (copter.failsafe.adsb) {
        copter.failsafe.adsb = false;
        LOGGER_WRITE_ERROR(LogErrorSubsystem::FAILSAFE_ADSB,
                           LogErrorCode::ERROR_RESOLVED);

        // restore flight mode if requested and user has not changed mode since
        if (copter.control_mode_reason == ModeReason::AVOIDANCE) {
            switch (recovery_action) {

            case RecoveryAction::REMAIN_IN_AVOID_ADSB:
                // do nothing, we'll stay in the AVOID_ADSB mode which is guided which will loiter forever
                break;

            case RecoveryAction::RESUME_PREVIOUS_FLIGHTMODE:
                set_mode_else_try_RTL_else_LAND(prev_control_mode);
                break;

            case RecoveryAction::RTL:
                set_mode_else_try_RTL_else_LAND(Mode::Number::RTL);
                break;

            case RecoveryAction::RESUME_IF_AUTO_ELSE_LOITER:
                if (prev_control_mode == Mode::Number::AUTO) {
                    set_mode_else_try_RTL_else_LAND(Mode::Number::AUTO);
                }
                break;

            default:
                break;
            } // switch
        }
    }
}

void AP_Avoidance_Copter::set_mode_else_try_RTL_else_LAND(Mode::Number mode)
{
    if (!copter.set_mode(mode, ModeReason::AVOIDANCE_RECOVERY)) {
        // on failure RTL or LAND
        if (!copter.set_mode(Mode::Number::RTL, ModeReason::AVOIDANCE_RECOVERY)) {
            copter.set_mode(Mode::Number::LAND, ModeReason::AVOIDANCE_RECOVERY);
        }
    }
}

int32_t AP_Avoidance_Copter::get_altitude_minimum() const
{
#if MODE_RTL_ENABLED
    // do not descend if below RTL alt
    return copter.g.rtl_altitude;
#else
    return 0;
#endif
}

// check flight mode is avoid_adsb
bool AP_Avoidance_Copter::check_flightmode(bool allow_mode_change)
{
    // ensure copter is in avoid_adsb mode
    if (allow_mode_change && copter.flightmode->mode_number() != Mode::Number::AVOID_ADSB) {
        if (!copter.set_mode(Mode::Number::AVOID_ADSB, ModeReason::AVOIDANCE)) {
            // failed to set mode so exit immediately
            return false;
        }
    }

    // check flight mode
    return (copter.flightmode->mode_number() == Mode::Number::AVOID_ADSB);
}

bool AP_Avoidance_Copter::handle_avoidance_vertical(const AP_Avoidance::Obstacle *obstacle, bool allow_mode_change)
{
    // ensure copter is in avoid_adsb mode
    if (!check_flightmode(allow_mode_change)) {
        return false;
    }

    // decide on whether we should climb or descend
    bool should_climb = false;
    Location my_loc;
    if (AP::ahrs().get_location(my_loc)) {
        should_climb = my_loc.alt > obstacle->_location.alt;
    }

    // get best vector away from obstacle
    Vector3f velocity_neu;
    if (should_climb) {
        velocity_neu.z = copter.wp_nav->get_default_speed_up();
    } else {
        velocity_neu.z = -copter.wp_nav->get_default_speed_down();
        // do not descend if below minimum altitude
        if (copter.current_loc.alt < get_altitude_minimum()) {
            velocity_neu.z = 0.0f;
        }
    }

    // send target velocity
    copter.mode_avoid_adsb.set_velocity(velocity_neu);
    return true;
}

bool AP_Avoidance_Copter::handle_avoidance_horizontal(const AP_Avoidance::Obstacle *obstacle, bool allow_mode_change)
{
    // ensure copter is in avoid_adsb mode
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
        // re-normalise
        velocity_neu.normalize();
        // convert horizontal components to velocities
        velocity_neu.x *= copter.wp_nav->get_default_speed_xy();
        velocity_neu.y *= copter.wp_nav->get_default_speed_xy();
        // send target velocity
        copter.mode_avoid_adsb.set_velocity(velocity_neu);
        return true;
    }

    // if we got this far we failed to set the new target
    return false;
}

bool AP_Avoidance_Copter::handle_avoidance_perpendicular(const AP_Avoidance::Obstacle *obstacle, bool allow_mode_change)
{
    // ensure copter is in avoid_adsb mode
    if (!check_flightmode(allow_mode_change)) {
        return false;
    }

    // get best vector away from obstacle
    Vector3f velocity_neu;
    if (get_vector_perpendicular(obstacle, velocity_neu)) {
        // convert horizontal components to velocities
        velocity_neu.x *= copter.wp_nav->get_default_speed_xy();
        velocity_neu.y *= copter.wp_nav->get_default_speed_xy();
        // use up and down waypoint speeds
        if (velocity_neu.z > 0.0f) {
            velocity_neu.z *= copter.wp_nav->get_default_speed_up();
        } else {
            velocity_neu.z *= copter.wp_nav->get_default_speed_down();
            // do not descend if below minimum altitude
            if (copter.current_loc.alt < get_altitude_minimum()) {
                velocity_neu.z = 0.0f;
            }
        }
        // send target velocity
        copter.mode_avoid_adsb.set_velocity(velocity_neu);
        return true;
    }

    // if we got this far we failed to set the new target
    return false;
}
#endif
