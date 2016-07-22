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

    float yaw = wrap_180_cd((nav_status.bearing+g.yaw_trim)*100); // target yaw in centidegrees
    float pitch = constrain_float(nav_status.pitch+g.pitch_trim, -90, 90) * 100; // target pitch in centidegrees

    bool direction_reversed = false;
    set_servo_direction(direction_reversed);

    calc_angle_error(pitch, yaw, direction_reversed);

    float bf_pitch;
    float bf_yaw;
    calc_body_frame_target(pitch, yaw, bf_pitch, bf_yaw);

    // only move servos if target is at least distance_min away
    if ((g.distance_min <= 0) || (nav_status.distance >= g.distance_min)) {
        update_pitch_servo(bf_pitch);
        update_yaw_servo(bf_yaw);
    }
}
void Tracker::calc_angle_error( float pitch, float yaw, bool direction_reversed)
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
    if(direction_reversed == true){
    	if (nav_status.angle_error_yaw>0){
    	nav_status.angle_error_yaw = (yaw - ahrs_yaw_cd) - 36000;
    	}
    	else if (nav_status.angle_error_yaw<=0){
    		nav_status.angle_error_yaw = 36000 + (yaw - ahrs_yaw_cd);
    	}
    }

    // earth frame to body frame angle error conversion
    float bf_pitch_err;
    float bf_yaw_err;
    calc_body_frame_target(nav_status.angle_error_pitch, nav_status.angle_error_yaw, bf_pitch_err, bf_yaw_err);
    nav_status.angle_error_pitch = bf_pitch_err;
    nav_status.angle_error_yaw = bf_yaw_err;
}
void Tracker::calc_body_frame_target(float pitch, float yaw, float& bf_pitch, float& bf_yaw)
{
	// earth frame to body frame pitch and yaw conversion
    bf_pitch   = ahrs.cos_roll()     * pitch    + ahrs.sin_roll()  * ahrs.cos_pitch() * yaw;
    bf_yaw     = -ahrs.sin_roll()    * pitch    + ahrs.cos_pitch() * ahrs.cos_roll()  * yaw;
}
bool Tracker::convert_bf_to_ef(float pitch, float yaw, float& ef_pitch, float& ef_yaw)
{
    // avoid divide by zero
    if (is_zero(ahrs.cos_pitch())) {
        return false;
    }
    // convert earth frame angle or rates to body frame
    ef_pitch =  ahrs.cos_roll()                     * pitch -  ahrs.sin_roll()                     * yaw; //pitch
    ef_yaw =   (ahrs.sin_roll() / ahrs.cos_pitch()) * pitch + (ahrs.cos_roll() / ahrs.cos_pitch()) * yaw; //yaw
    return true;
}
void Tracker::set_servo_direction(bool& direction_reversed)
{
    // calculating distances from current pitch/yaw to lower and upper limits in centi-degrees
    float yaw_angle_limit_lower =   100 * (channel_yaw.get_servo_out()-(-9000))  / (9000-(-9000)) * g.yaw_range;
    float yaw_angle_limit_upper =   100 * (9000-channel_yaw.get_servo_out())    / (9000-(-9000)) * g.yaw_range;
    float pitch_angle_limit_lower = 100 * (channel_pitch.get_servo_out()-(-9000)) / (9000-(-9000))   * (g.pitch_max - g.pitch_min);
    float pitch_angle_limit_upper = 100 * (9000-channel_pitch.get_servo_out())   / (9000-(-9000))   * (g.pitch_max - g.pitch_min);

    // distances to earthframe angle limits in centi-degrees
    float ef_yaw_limit_lower = yaw_angle_limit_lower;
    float ef_yaw_limit_upper = yaw_angle_limit_upper;
    float ef_pitch_limit_lower = pitch_angle_limit_lower;
    float ef_pitch_limit_upper = pitch_angle_limit_upper;
    if (convert_bf_to_ef(pitch_angle_limit_lower, yaw_angle_limit_lower, ef_pitch_limit_lower, ef_yaw_limit_lower)){}
    if (convert_bf_to_ef(pitch_angle_limit_upper, yaw_angle_limit_upper, ef_pitch_limit_upper, ef_yaw_limit_upper)){}

    float ef_yaw_current = wrap_180_cd(ahrs.yaw_sensor);
    float ef_yaw_target = wrap_180_cd((nav_status.bearing+g.yaw_trim)*100);
    float ef_yaw_angle_error = wrap_180_cd(ef_yaw_target - ef_yaw_current);

    // sets lower and upper angle errors
    float upper_angle_error;
    float lower_angle_error;
    if (ef_yaw_angle_error >= 0){
    	upper_angle_error = ef_yaw_angle_error;
    	lower_angle_error = 36000 - ef_yaw_angle_error;
    }
    else{
    	upper_angle_error = 36000 + ef_yaw_angle_error;
    	lower_angle_error = -ef_yaw_angle_error;
    }

    // reverses direction of earth frame yaw angle error if vehicle is past ef yaw limits (in the blind spot) and
    // the vehicle is closer to the far side of the blind spot
    if ((lower_angle_error >= ef_yaw_limit_lower) && (upper_angle_error >= ef_yaw_limit_upper)){
    	if (ef_yaw_angle_error>0 && ((lower_angle_error - ef_yaw_limit_lower) < (upper_angle_error - ef_yaw_limit_upper))){
    		direction_reversed = true;
    	}
    	if (ef_yaw_angle_error<0 && ((lower_angle_error - ef_yaw_limit_lower) > (upper_angle_error - ef_yaw_limit_upper))){
    		direction_reversed = true;
    	}
    }
}
