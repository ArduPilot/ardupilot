#include "mode.h"

#include "Tracker.h"
#include <GCS_MAVLink/GCS.h>

void ModeGuided::update()
{
    static uint32_t last_debug;
    float target_roll, target_pitch, target_yaw;
    _target_att.to_euler(target_roll, target_pitch, target_yaw);
    // Announce the angles if there was a change or if it's a first run
    if (_last_set_angle_ms > last_debug || _last_set_angle_ms == 0) {
        last_debug = AP_HAL::millis();
        gcs().send_text(MAV_SEVERITY_INFO, "Tracker target_yaw=%f target_pitch=%f", degrees(target_yaw), degrees(target_pitch));

        // Mark as having announced
        if (_last_set_angle_ms == 0) {
            _last_set_angle_ms = last_debug;
        }
    }
    calc_angle_error(degrees(target_pitch)*100, degrees(target_yaw)*100, false);
    float bf_pitch;
    float bf_yaw;
    convert_ef_to_bf(target_pitch, target_yaw, bf_pitch, bf_yaw);
    tracker.update_pitch_servo(bf_pitch);
    tracker.update_yaw_servo(bf_yaw);
}
