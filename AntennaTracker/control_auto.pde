// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/*
 * control_auto.pde - auto control mode
 */

/*
 * update_auto - runs the auto controller
 *  called at 50hz while control_mode is 'AUTO'
 */
static void update_auto(void)
{
    // exit immediately if we do not have a valid vehicle position
    if (!vehicle.location_valid) {
        return;
    }

    float yaw = wrap_180_cd((nav_status.bearing+g.yaw_trim)*100) * 0.01f;
    float pitch = constrain_float(nav_status.pitch+g.pitch_trim, -90, 90);
    update_pitch_servo(pitch);
    update_yaw_servo(yaw);
}
