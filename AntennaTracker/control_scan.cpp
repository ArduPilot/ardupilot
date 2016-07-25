// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include "Tracker.h"

/*
 * Scan control mode
 */

/*
 * update_scan - runs the scan controller
 *  called at 50hz while control_mode is 'SCAN'
 */
void Tracker::update_scan(void)
{
    if (!nav_status.manual_control_yaw) {
        float yaw_delta = g.scan_speed * 0.02f;
        nav_status.bearing   += yaw_delta   * (nav_status.scan_reverse_yaw?-1:1);
        if (nav_status.bearing < 0 && nav_status.scan_reverse_yaw) {
            nav_status.scan_reverse_yaw = false;
        }
        if (nav_status.bearing > 360 && !nav_status.scan_reverse_yaw) {
            nav_status.scan_reverse_yaw = true;
        }
        nav_status.bearing = constrain_float(nav_status.bearing, 0, 360);
    }

    if (!nav_status.manual_control_pitch) {
        float pitch_delta = g.scan_speed * 0.02f;
        nav_status.pitch += pitch_delta * (nav_status.scan_reverse_pitch?-1:1);
        if (nav_status.pitch < -90 && nav_status.scan_reverse_pitch) {
            nav_status.scan_reverse_pitch = false;
        }
        if (nav_status.pitch > 90 && !nav_status.scan_reverse_pitch) {
            nav_status.scan_reverse_pitch = true;
        }
        nav_status.pitch = constrain_float(nav_status.pitch, -90, 90);
    }

    update_auto();
}
