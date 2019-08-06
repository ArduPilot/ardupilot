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
    float yaw = wrap_180_cd((nav_status.bearing+g.yaw_trim)*100); // target yaw in centidegrees
    float pitch = constrain_float(nav_status.pitch+g.pitch_trim, g.pitch_min, g.pitch_max) * 100; // target pitch in centidegrees

    bool direction_reversed = get_ef_yaw_direction();

    calc_angle_error(pitch, yaw, direction_reversed);

    float bf_pitch;
    float bf_yaw;
    convert_ef_to_bf(pitch, yaw, bf_pitch, bf_yaw);

    // only move servos if target is at least distance_min away if we  have a target
    if ((g.distance_min <= 0) || (nav_status.distance >= g.distance_min) || !vehicle.location_valid) {
        update_pitch_servo(bf_pitch);
        update_yaw_servo(bf_yaw);
    }
}

void Tracker::calc_angle_error(float pitch, float yaw, bool direction_reversed)
{
    // Pitch angle error in centidegrees
    // Positive error means the target is above current pitch
    // Negative error means the target is below current pitch
    float ahrs_pitch = ahrs.pitch_sensor;
    int32_t ef_pitch_angle_error = pitch - ahrs_pitch;

    // Yaw angle error in centidegrees
    // Positive error means the target is right of current yaw
    // Negative error means the target is left of current yaw
    int32_t ahrs_yaw_cd = wrap_180_cd(ahrs.yaw_sensor);
    int32_t ef_yaw_angle_error = wrap_180_cd(yaw - ahrs_yaw_cd);
    if (direction_reversed) {
        if (ef_yaw_angle_error > 0) {
            ef_yaw_angle_error = (yaw - ahrs_yaw_cd) - 36000;
        } else {
            ef_yaw_angle_error = 36000 + (yaw - ahrs_yaw_cd);
        }
    }

    // earth frame to body frame angle error conversion
    float bf_pitch_err;
    float bf_yaw_err;
    convert_ef_to_bf(ef_pitch_angle_error, ef_yaw_angle_error, bf_pitch_err, bf_yaw_err);
    nav_status.angle_error_pitch = bf_pitch_err;
    nav_status.angle_error_yaw = bf_yaw_err;

    // set actual and desired for logging, note we are using angles not rates
    g.pidPitch2Srv.set_target_rate(pitch * 0.01);
    g.pidPitch2Srv.set_actual_rate(ahrs_pitch * 0.01);
    g.pidYaw2Srv.set_target_rate(yaw * 0.01);
    g.pidYaw2Srv.set_actual_rate(ahrs_yaw_cd * 0.01);
}

void Tracker::convert_ef_to_bf(float pitch, float yaw, float& bf_pitch, float& bf_yaw)
{
	// earth frame to body frame pitch and yaw conversion
    bf_pitch = ahrs.cos_roll() * pitch + ahrs.sin_roll() * ahrs.cos_pitch() * yaw;
    bf_yaw = -ahrs.sin_roll() * pitch + ahrs.cos_pitch() * ahrs.cos_roll() * yaw;
}

bool Tracker::convert_bf_to_ef(float pitch, float yaw, float& ef_pitch, float& ef_yaw)
{
    // avoid divide by zero
    if (is_zero(ahrs.cos_pitch())) {
        return false;
    }
    // convert earth frame angle or rates to body frame
    ef_pitch = ahrs.cos_roll() * pitch - ahrs.sin_roll() * yaw;
    ef_yaw = (ahrs.sin_roll() / ahrs.cos_pitch()) * pitch + (ahrs.cos_roll() / ahrs.cos_pitch()) * yaw;
    return true;
}

// return value is true if taking the long road to the target, false if normal, shortest direction should be used
bool Tracker::get_ef_yaw_direction()
{
    // calculating distances from current pitch/yaw to lower and upper limits in centi-degrees
    float yaw_angle_limit_lower =   (-g.yaw_range * 100.0f / 2.0f) - yaw_servo_out_filt.get();
    float yaw_angle_limit_upper =   (g.yaw_range * 100.0f / 2.0f) - yaw_servo_out_filt.get();
    float pitch_angle_limit_lower = (g.pitch_min * 100.0f) - pitch_servo_out_filt.get();
    float pitch_angle_limit_upper = (g.pitch_max * 100.0f) - pitch_servo_out_filt.get();

    // distances to earthframe angle limits in centi-degrees
    float ef_yaw_limit_lower = yaw_angle_limit_lower;
    float ef_yaw_limit_upper = yaw_angle_limit_upper;
    float ef_pitch_limit_lower = pitch_angle_limit_lower;
    float ef_pitch_limit_upper = pitch_angle_limit_upper;
    convert_bf_to_ef(pitch_angle_limit_lower, yaw_angle_limit_lower, ef_pitch_limit_lower, ef_yaw_limit_lower);
    convert_bf_to_ef(pitch_angle_limit_upper, yaw_angle_limit_upper, ef_pitch_limit_upper, ef_yaw_limit_upper);

    float ef_yaw_current = wrap_180_cd(ahrs.yaw_sensor);
    float ef_yaw_target = wrap_180_cd((nav_status.bearing+g.yaw_trim)*100);
    float ef_yaw_angle_error = wrap_180_cd(ef_yaw_target - ef_yaw_current);

    // calculate angle error to target in both directions (i.e. moving up/right or lower/left)
    float yaw_angle_error_upper;
    float yaw_angle_error_lower;
    if (ef_yaw_angle_error >= 0) {
        yaw_angle_error_upper = ef_yaw_angle_error;
        yaw_angle_error_lower = ef_yaw_angle_error - 36000;
    } else {
        yaw_angle_error_upper = 36000 + ef_yaw_angle_error;
        yaw_angle_error_lower = ef_yaw_angle_error;
    }

    // checks that the vehicle is outside the tracker's range
    if ((yaw_angle_error_lower < ef_yaw_limit_lower) && (yaw_angle_error_upper > ef_yaw_limit_upper)) {
        // if the tracker is trying to move clockwise to reach the vehicle,
        // but the tracker could get closer to the vehicle by moving counter-clockwise then set direction_reversed to true
        if (ef_yaw_angle_error>0 && ((ef_yaw_limit_lower - yaw_angle_error_lower) < (yaw_angle_error_upper - ef_yaw_limit_upper))) {
            return true;
        }
        // if the tracker is trying to move counter-clockwise to reach the vehicle,
        // but the tracker could get closer to the vehicle by moving then set direction_reversed to true
        if (ef_yaw_angle_error<0 && ((ef_yaw_limit_lower - yaw_angle_error_lower) > (yaw_angle_error_upper - ef_yaw_limit_upper))) {
            return true;
        }
    }

    // if we get this far we can use the regular, shortest path to the target
    return false;
}
