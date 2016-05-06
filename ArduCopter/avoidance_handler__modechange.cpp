#include "avoidance_handler.h"

#include "Copter.h"

bool AvoidanceHandler__ModeChange::enter(class AP_Avoidance::Obstacle &threat, class AvoidanceHandler *old_handler)
{
    if (!AvoidanceHandler::enter(threat, old_handler)) {
        return false;
    }
    _old_flight_mode = copter.control_mode;

    if (!copter.set_mode(mode(), MODE_REASON_AVOIDANCE)) {
        return false;
    }

    return true;
}

// change mode every <n> seconds (this gives the pilot opportunity to
// change modes and fly manually, for example)
bool AvoidanceHandler__ModeChange::update()
{
    uint32_t now = AP_HAL::millis();
    // if the copter has switched out of our mode switch back in:
    if (copter.control_mode != mode()) {
        // but give whatever decided to make the change time to
        // resolve the situation:
        if (now - _last_mode_change < _mode_change_hysteresis) {
            return true;
        }
        if (!copter.set_mode(mode(), MODE_REASON_AVOIDANCE)) {
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
    if (copter.control_mode_reason != MODE_REASON_AVOIDANCE) {
        // we did not set the current flight mode (e.g. the user
        // switched away from us to Stabilize and resolved the issue
        // themselves).  Leave them in that mode:
        return;
    }

    if (copter.set_mode(_old_flight_mode, MODE_REASON_AVOIDANCE)) { // should we revert the reason?
        // All good, Copter returned to what it was doing.
        return;
    }

    GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_WARNING, "AVOID: Failed to revert flight mode");

    copter.set_mode_RTL_or_land_with_pause(MODE_REASON_AVOIDANCE);
}

void AvoidanceHandler__ModeChange::exit()
{
    exit_revert_flight_mode();
    AvoidanceHandler::exit();
}
