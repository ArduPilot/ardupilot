#include "avoidance_handler.h"

#include "Plane.h"

bool AvoidanceHandler__ModeChange::set_mode(const FlightMode _mode)
{
    // make sure we don't auto trim the surfaces on this mode change
    const int8_t saved_auto_trim = plane.g.auto_trim;
    plane.g.auto_trim.set(0);
    plane.set_mode(_mode);
    plane.g.auto_trim.set(saved_auto_trim);

    if (plane.control_mode != _mode) {
        return false;
    }
    return true;
}


bool AvoidanceHandler__ModeChange::enter(class AP_Avoidance::Obstacle &threat, class AvoidanceHandler *old_handler)
{
    if (!AvoidanceHandler::enter(threat, old_handler)) {
        return false;
    }
    _old_flight_mode = plane.control_mode;

    old_switch_position = plane.oldSwitchPosition;
    guided_WP_loc = plane.guided_WP_loc;

    if (!set_mode(mode())) {
        return false;
    }

    return true;
}

bool AvoidanceHandler__ModeChange::user_has_taken_control()
{
    if (plane.control_mode != mode()) {
        // user has switched modes
        return true;
    }
    if (plane.control_mode == GUIDED &&
        !locations_are_same(guided_WP_loc, plane.guided_WP_loc)) {
        // user has changed the guided destination
        return true;
    }

    return false;
}

// change mode every <n> seconds (this gives the pilot opportunity to
// change modes and fly manually, for example)
bool AvoidanceHandler__ModeChange::update()
{
    uint32_t now = AP_HAL::millis();
    // if something else has taken control, take it back again....
    if (user_has_taken_control()) {
        // but give whatever decided to make the change time to
        // resolve the situation:
        if (now - _last_mode_change < _mode_change_hysteresis) {
            return true;
        }
        if (!set_mode(mode())) {
            return false;
        }
        // question as to whether we should reset _old_flight_mode,
        // but if it was us fighting e.g. Fence for control of the
        // copter returning to the origin _old_flight_mode makes more
        // sense
        _last_mode_change = now;
    }
    return true;
}

void AvoidanceHandler__ModeChange::exit_revert_flight_mode()
{
    if (user_has_taken_control()) {
        // something else has taken control.  Don't change things on them
        return;
    }

    if (set_mode(_old_flight_mode)) {
        // All good, Plane returned to what it was doing.
        return;
    }

    GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_WARNING, "AVOID: Failed to revert flight mode");

    plane.set_mode(RTL);
}

void AvoidanceHandler__ModeChange::exit()
{
    exit_revert_flight_mode();
    AvoidanceHandler::exit();
}
