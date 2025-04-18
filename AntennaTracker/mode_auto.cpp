#include "mode.h"

#include "Tracker.h"

void ModeAuto::update()
{
    if (tracker.vehicle.location_valid) {
        update_auto();
        switch_state(target_range_acceptable() ? State::TRACKING : State::SUPPRESSED);
    } else if (!option_is_set(OPTION::DO_NOT_SCAN) && (tracker.target_set || option_is_set(OPTION::SCAN_FOR_ANY_TARGET))) {
        update_scan();
        switch_state(State::SCANNING);
    } else {
        switch_state(State::IDLE);
    }
}

void ModeAuto::switch_state(ModeAuto::State st)
{
    // Announce the state change if necessary
    if (st != _state) {
        switch (st) {
            case State::TRACKING:
                gcs().send_text(MAV_SEVERITY_INFO, "Tracker locked on the vehicle %i", tracker.g.sysid_target.get());
                break;

            case State::SCANNING:
                if (option_is_set(OPTION::SCAN_FOR_ANY_TARGET)) {
                    gcs().send_text(MAV_SEVERITY_INFO, "Tracker scanning for a vehicle");
                } else {
                    gcs().send_text(MAV_SEVERITY_INFO, "Tracker scanning for the vehicle %i", tracker.g.sysid_target.get());
                }
                break;

            case State::SUPPRESSED:
                gcs().send_text(MAV_SEVERITY_INFO, "Tracker suppressed while the target is within %i m", tracker.g.distance_min.get());
                break;

            case State::IDLE:
                gcs().send_text(MAV_SEVERITY_INFO, "Tracker idle");
                break;

            default:
                break;
        }
        _state = st;
    }
}

bool ModeAuto::option_is_set(ModeAuto::OPTION option) const
{
    return (tracker.g.auto_opts.get() & int32_t(option)) != 0;
}
