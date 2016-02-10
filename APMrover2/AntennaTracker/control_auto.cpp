// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include "Tracker.h"

/*
 * control_auto.pde - auto control mode
 */

/*
 * update_auto - runs the auto controller
 *  called at 50hz while control_mode is 'AUTO'
 */
void Tracker::update_auto(void)
{
    // exit immediately if we do not have a valid vehicle position
    if (!vehicle.location_valid) {
        return;
    }

    float yaw = wrap_180_cd((nav_status.bearing+g.yaw_trim)*100) * 0.01f;
    float pitch = constrain_float(nav_status.pitch+g.pitch_trim, -90, 90);

    // only move servos if target is at least distance_min away
    if ((g.distance_min <= 0) || (nav_status.distance >= g.distance_min)) {
        update_pitch_servo(pitch);
        update_yaw_servo(yaw);
    }
}
