// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include "Tracker.h"

/*
 * Auto control mode
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

    float yaw = wrap_180_cd((nav_status.bearing+g.yaw_trim)*100); // target yaw in centidegrees
    float pitch = constrain_float(nav_status.pitch+g.pitch_trim, -90, 90) * 100; // target pitch in centidegrees

    calc_angle_error(pitch, yaw);

    float bf_pitch;
    float bf_yaw;
    calc_body_frame_target(pitch, yaw, bf_pitch, bf_yaw);

    // only move servos if target is at least distance_min away
    if ((g.distance_min <= 0) || (nav_status.distance >= g.distance_min)) {
        update_pitch_servo(bf_pitch);
        update_yaw_servo(bf_yaw);
    }
}
void Tracker::calc_angle_error( float pitch, float yaw)
{
	// Pitch angle error in centidegrees
	// Positive error means the target is above current pitch
    // Negative error means the target is below current pitch
    float ahrs_pitch = ahrs.pitch_sensor;
    nav_status.angle_error_pitch = pitch - ahrs_pitch;

    // Yaw angle error in centidegrees
	// Positive error means the target is right of current yaw
    // Negative error means the target is left of current yaw
    int32_t ahrs_yaw_cd = wrap_180_cd(ahrs.yaw_sensor);
    nav_status.angle_error_yaw = wrap_180_cd(yaw - ahrs_yaw_cd);

    // earth frame to body frame angle error conversion
    float bf_pitch_err   = ahrs.cos_roll()     * nav_status.angle_error_pitch    + ahrs.sin_roll()  * ahrs.cos_pitch() * nav_status.angle_error_yaw;
    float bf_yaw_err     = -ahrs.sin_roll()    * nav_status.angle_error_pitch    + ahrs.cos_pitch() * ahrs.cos_roll()  * nav_status.angle_error_yaw;
    nav_status.angle_error_pitch = bf_pitch_err;
    nav_status.angle_error_yaw = bf_yaw_err;
}
void Tracker::calc_body_frame_target(float pitch, float yaw, float& bf_pitch, float& bf_yaw)
{
	// earth frame to body frame pitch and yaw conversion
    bf_pitch   = ahrs.cos_roll()     * pitch    + ahrs.sin_roll()  * ahrs.cos_pitch() * yaw;
    bf_yaw     = -ahrs.sin_roll()    * pitch    + ahrs.cos_pitch() * ahrs.cos_roll()  * yaw;
}
