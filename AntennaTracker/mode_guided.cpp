#include "mode.h"

#include "Tracker.h"
#include <GCS_MAVLink/GCS.h>

void ModeGuided::update()
{
    struct Tracker::NavStatus &nav_status = tracker.nav_status;

    static uint32_t last_debug;
    const uint32_t now = AP_HAL::millis();
    float target_roll, target_pitch, target_yaw;
    _target_att.to_euler(target_roll, target_pitch, target_yaw);
    nav_status.bearing = degrees(target_yaw);
    nav_status.pitch = degrees(target_pitch);
    // TODO: we probably don't need this STATUS_TEXT anymore
    if (now - last_debug > 5000) {
        last_debug = now;
        gcs().send_text(MAV_SEVERITY_INFO, "target_yaw=%f target_pitch=%f", nav_status.bearing, nav_status.pitch);
    }
    calc_angle_error(nav_status.pitch*100, nav_status.bearing*100, false);
    float bf_pitch;
    float bf_yaw;
    convert_ef_to_bf(target_pitch, target_yaw, bf_pitch, bf_yaw);
    tracker.update_pitch_servo(bf_pitch);
    tracker.update_yaw_servo(bf_yaw);
}
