#include "mode.h"

#include "Tracker.h"

void ModeAuto::update()
{
    if (tracker.vehicle.location_valid) {
        if (!tracker.nav_status.manual_control_yaw) {
            tracker.nav_status.bearing = tracker.nav_status.bearing_to_target;
        }
        if (!tracker.nav_status.manual_control_pitch) {
            tracker.nav_status.pitch = tracker.nav_status.pitch_to_target;
        }
        update_auto();
    } else if (tracker.target_set || (tracker.g.auto_opts.get() & (1 << 0)) != 0) {
        update_scan();
    }
}
