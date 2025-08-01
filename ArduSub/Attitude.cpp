#include "Sub.h"

// get_pilot_desired_angle - transform pilot's roll or pitch input into a desired lean angle
// returns desired angle in centi-degrees
void Sub::get_pilot_desired_lean_angles(float roll_in, float pitch_in, float &roll_out, float &pitch_out, float angle_max)
{
    // sanity check angle max parameter
    aparm.angle_max.set(constrain_int16(aparm.angle_max,1000,8000));

    // limit max lean angle
    angle_max = constrain_float(angle_max, 1000, aparm.angle_max);

    // scale roll_in, pitch_in to ANGLE_MAX parameter range
    float scaler = aparm.angle_max/(float)ROLL_PITCH_INPUT_MAX;
    roll_in *= scaler;
    pitch_in *= scaler;

    // do circular limit
    float total_in = norm(pitch_in, roll_in);
    if (total_in > angle_max) {
        float ratio = angle_max / total_in;
        roll_in *= ratio;
        pitch_in *= ratio;
    }

    // do lateral tilt to euler roll conversion
    roll_in = (18000/M_PI) * atanf(cosf(pitch_in*(M_PI/18000))*tanf(roll_in*(M_PI/18000)));

    // return
    roll_out = roll_in;
    pitch_out = pitch_in;
}

// get_pilot_desired_heading - transform pilot's yaw input into a
// desired yaw rate
// returns desired yaw rate in centi-degrees per second
float Sub::get_pilot_desired_yaw_rate(int16_t stick_angle) const
{
    // convert pilot input to the desired yaw rate
    return stick_angle * g.acro_yaw_p;
}

// check for ekf yaw reset and adjust target heading
void Sub::check_ekf_yaw_reset()
{
    float yaw_angle_change_rad;
    uint32_t new_ekfYawReset_ms = ahrs.getLastYawResetAngle(yaw_angle_change_rad);
    if (new_ekfYawReset_ms != ekfYawReset_ms) {
        attitude_control.inertial_frame_reset();
        ekfYawReset_ms = new_ekfYawReset_ms;
    }
}

/*************************************************************
 * yaw controllers
 *************************************************************/

// get_roi_yaw - returns heading towards location held in roi_WP
// should be called at 100hz
float Sub::get_roi_yaw()
{
    static uint8_t roi_yaw_counter = 0;     // used to reduce update rate to 100hz

    roi_yaw_counter++;
    if (roi_yaw_counter >= 4) {
        roi_yaw_counter = 0;
        yaw_look_at_WP_bearing = get_bearing_cd(inertial_nav.get_position_xy_cm(), roi_WP.xy());
    }

    return yaw_look_at_WP_bearing;
}

float Sub::get_look_ahead_yaw()
{
    const Vector3f& vel = inertial_nav.get_velocity_neu_cms();
    const float speed_sq = vel.xy().length_squared();
    // Commanded Yaw to automatically look ahead.
    if (position_ok() && (speed_sq > (YAW_LOOK_AHEAD_MIN_SPEED * YAW_LOOK_AHEAD_MIN_SPEED))) {
        yaw_look_ahead_bearing = degrees(atan2f(vel.y,vel.x))*100.0f;
    }
    return yaw_look_ahead_bearing;
}

/*************************************************************
 *  throttle control
 ****************************************************************/

// get_pilot_desired_climb_rate - transform pilot's throttle input to climb rate in cm/s
// without any deadzone at the bottom
float Sub::get_pilot_desired_climb_rate(float throttle_control)
{
    // throttle failsafe check
    if (failsafe.pilot_input) {
        return 0.0f;
    }

    // ensure a reasonable throttle value
    throttle_control = constrain_float(throttle_control,0.0f,1000.0f);

    // ensure a reasonable deadzone
    g.throttle_deadzone.set(constrain_int16(g.throttle_deadzone, 0, 400));

    float mid_stick = channel_throttle->get_control_mid();
    float deadband_top = mid_stick + g.throttle_deadzone * gain;
    float deadband_bottom = mid_stick - g.throttle_deadzone * gain;

    // check throttle is above, below or in the deadband
    if (throttle_control < deadband_bottom) {
        // below the deadband
        return get_pilot_speed_dn() * (throttle_control-deadband_bottom) / deadband_bottom;
    } else if (throttle_control > deadband_top) {
        // above the deadband
        return g.pilot_speed_up * (throttle_control-deadband_top) / (1000.0f-deadband_top);
    } else {
        // must be in the deadband
        return 0.0f;
    }
}

// behavior is similar to Sub::get_pilot_desired_climb_rate
float Sub::get_pilot_desired_horizontal_rate(RC_Channel *channel) const
{
    if (failsafe.pilot_input) {
        return 0;
    }

    // forward and lateral sticks have center trim, unlike throttle
    auto control = channel->norm_input();

    // normalize deadzone
    auto dz = (float)g.throttle_deadzone * 2.0f / (float)(channel->get_radio_max() - channel->get_radio_min());
    auto deadband_top = dz * gain;
    auto deadband_bottom = -dz * gain;

    if (control < deadband_bottom) {
        // below the deadband
        return (float)g.pilot_speed * (control - deadband_bottom);
    } else if (control > deadband_top) {
        // above the deadband
        return (float)g.pilot_speed * (control - deadband_top);
    } else {
        // must be in the deadband
        return 0;
    }
}

// rotate vector from vehicle's perspective to North-East frame
void Sub::rotate_body_frame_to_NE(float &x, float &y)
{
    float ne_x = x*ahrs.cos_yaw() - y*ahrs.sin_yaw();
    float ne_y = x*ahrs.sin_yaw() + y*ahrs.cos_yaw();
    x = ne_x;
    y = ne_y;
}

// It will return the PILOT_SPEED_DN value if non zero, otherwise if zero it returns the PILOT_SPEED_UP value.
uint16_t Sub::get_pilot_speed_dn() const
{
    if (g.pilot_speed_dn == 0) {
        return abs(g.pilot_speed_up);
    }
    return abs(g.pilot_speed_dn);
}
