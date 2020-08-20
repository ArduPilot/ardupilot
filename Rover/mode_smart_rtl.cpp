#include "mode.h"
#include "Rover.h"

bool ModeSmartRTL::_enter()
{
    // SmartRTL requires EKF (not DCM)
    Location ekf_origin;
    if (!ahrs.get_origin(ekf_origin)) {
        return false;
    }

    // refuse to enter SmartRTL if smart RTL's home has not been set
    if (!g2.smart_rtl.is_active()) {
        return false;
    }

    // set desired location to reasonable stopping point
    if (!g2.wp_nav.set_desired_location_to_stopping_location()) {
        return false;
    }

    // initialise waypoint speed
    if (is_positive(g2.rtl_speed)) {
        g2.wp_nav.set_desired_speed(g2.rtl_speed);
    } else {
        g2.wp_nav.set_desired_speed_to_default();
    }

    // init state
    smart_rtl_state = SmartRTL_WaitForPathCleanup;
    _loitering = false;

    return true;
}

void ModeSmartRTL::update()
{
    switch (smart_rtl_state) {
        case SmartRTL_WaitForPathCleanup:
            // check if return path is computed and if yes, begin journey home
            if (g2.smart_rtl.request_thorough_cleanup()) {
                smart_rtl_state = SmartRTL_PathFollow;
                _load_point = true;
            }
            // Note: this may lead to an unnecessary 20ms slow down of the vehicle (but it is unlikely)
            stop_vehicle();
            break;

        case SmartRTL_PathFollow:
            // load point if required
            if (_load_point) {
                Vector3f next_point;
                if (!g2.smart_rtl.pop_point(next_point)) {
                    // if not more points, we have reached home
                    gcs().send_text(MAV_SEVERITY_INFO, "Reached destination");
                    smart_rtl_state = SmartRTL_StopAtHome;
                    break;
                }
                _load_point = false;
                // set target destination to new point
                if (!g2.wp_nav.set_desired_location_NED(next_point)) {
                    // this failure should never happen but we add it just in case
                    gcs().send_text(MAV_SEVERITY_INFO, "SmartRTL: failed to set destination");
                    smart_rtl_state = SmartRTL_Failure;
                }
            }
            // update navigation controller
            navigate_to_waypoint();

            // check if we've reached the next point
            if (g2.wp_nav.reached_destination()) {
                _load_point = true;
            }
            break;

        case SmartRTL_StopAtHome:
        case SmartRTL_Failure:
            _reached_destination = true;
            // we have reached the destination
            // boats loiters, rovers stop
            if (!rover.is_boat()) {
               stop_vehicle();
            } else {
                // if not loitering yet, start loitering
                if (!_loitering) {
                    _loitering = rover.mode_loiter.enter();
                }
                if (_loitering) {
                    rover.mode_loiter.update();
                } else {
                    stop_vehicle();
               }
            }
            break;
    }
}

// get desired location
bool ModeSmartRTL::get_desired_location(Location& destination) const
{
    switch (smart_rtl_state) {
    case SmartRTL_WaitForPathCleanup:
        return false;
    case SmartRTL_PathFollow:
        if (g2.wp_nav.is_destination_valid()) {
            destination = g2.wp_nav.get_destination();
            return true;
        }
        return false;
    case SmartRTL_StopAtHome:
    case SmartRTL_Failure:
        return false;
    }
    // should never reach here but just in case
    return false;
}

// set desired speed in m/s
bool ModeSmartRTL::set_desired_speed(float speed)
{
    if (is_negative(speed)) {
        return false;
    }
    g2.wp_nav.set_desired_speed(speed);
    return true;
}

// save current position for use by the smart_rtl flight mode
void ModeSmartRTL::save_position()
{
    const bool save_pos = (rover.control_mode != &rover.mode_smartrtl);
    g2.smart_rtl.update(true, save_pos);
}
