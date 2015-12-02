/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include "Copter.h"

// get_smoothing_gain - returns smoothing gain to be passed into attitude_control.angle_ef_roll_pitch_rate_ef_yaw_smooth
//      result is a number from 2 to 12 with 2 being very sluggish and 12 being very crisp
float Copter::get_smoothing_gain()
{
    return (2.0f + (float)g.rc_feel_rp/10.0f);
}

// get_pilot_desired_angle - transform pilot's roll or pitch input into a desired lean angle
// returns desired angle in centi-degrees
void Copter::get_pilot_desired_lean_angles(float roll_in, float pitch_in, float &roll_out, float &pitch_out, float angle_max)
{
    // sanity check angle max parameter
    aparm.angle_max = constrain_int16(aparm.angle_max,1000,8000);

    // limit max lean angle
    angle_max = constrain_float(angle_max, 1000, aparm.angle_max);

    // scale roll_in, pitch_in to ANGLE_MAX parameter range
    float scaler = aparm.angle_max/(float)ROLL_PITCH_INPUT_MAX;
    roll_in *= scaler;
    pitch_in *= scaler;

    // do circular limit
    float total_in = pythagorous2((float)pitch_in, (float)roll_in);
    if (total_in > angle_max) {
        float ratio = angle_max / total_in;
        roll_in *= ratio;
        pitch_in *= ratio;
    }

    // do lateral tilt to euler roll conversion
    roll_in = (18000/M_PI_F) * atanf(cosf(pitch_in*(M_PI_F/18000))*tanf(roll_in*(M_PI_F/18000)));

    // return
    roll_out = roll_in;
    pitch_out = pitch_in;
}

// get_pilot_desired_heading - transform pilot's yaw input into a
// desired yaw rate
// returns desired yaw rate in centi-degrees per second
float Copter::get_pilot_desired_yaw_rate(int16_t stick_angle)
{
    // convert pilot input to the desired yaw rate
    return stick_angle * g.acro_yaw_p;
}

// check for ekf yaw reset and adjust target heading
void Copter::check_ekf_yaw_reset()
{
    float yaw_angle_change_rad = 0.0f;
    uint32_t new_ekfYawReset_ms = ahrs.getLastYawResetAngle(yaw_angle_change_rad);
    if (new_ekfYawReset_ms != ekfYawReset_ms) {
        attitude_control.shift_ef_yaw_target(ToDeg(yaw_angle_change_rad) * 100.0f);
        ekfYawReset_ms = new_ekfYawReset_ms;
    }
}

/*************************************************************
 * yaw controllers
 *************************************************************/

// get_roi_yaw - returns heading towards location held in roi_WP
// should be called at 100hz
float Copter::get_roi_yaw()
{
    static uint8_t roi_yaw_counter = 0;     // used to reduce update rate to 100hz

    roi_yaw_counter++;
    if (roi_yaw_counter >= 4) {
        roi_yaw_counter = 0;
        yaw_look_at_WP_bearing = pv_get_bearing_cd(inertial_nav.get_position(), roi_WP);
    }

    return yaw_look_at_WP_bearing;
}

float Copter::get_look_ahead_yaw()
{
    const Vector3f& vel = inertial_nav.get_velocity();
    float speed = pythagorous2(vel.x,vel.y);
    // Commanded Yaw to automatically look ahead.
    if (position_ok() && (speed > YAW_LOOK_AHEAD_MIN_SPEED)) {
        yaw_look_ahead_bearing = degrees(atan2f(vel.y,vel.x))*100.0f;
    }
    return yaw_look_ahead_bearing;
}

/*************************************************************
 *  throttle control
 ****************************************************************/

// update_thr_average - update estimated throttle required to hover (if necessary)
//  should be called at 100hz
void Copter::update_thr_average()
{
    // ensure throttle_average has been initialised
    if( is_zero(throttle_average) ) {
        throttle_average = g.throttle_mid;
        // update position controller
        pos_control.set_throttle_hover(throttle_average);
    }

    // if not armed or landed exit
    if (!motors.armed() || ap.land_complete) {
        return;
    }

    // get throttle output
    float throttle = motors.get_throttle();

    // calc average throttle if we are in a level hover
    if (throttle > g.throttle_min && abs(climb_rate) < 60 && labs(ahrs.roll_sensor) < 500 && labs(ahrs.pitch_sensor) < 500) {
        throttle_average = throttle_average * 0.99f + throttle * 0.01f;
        // update position controller
        pos_control.set_throttle_hover(throttle_average);
    }
}

// set_throttle_takeoff - allows parents to tell throttle controller we are taking off so I terms can be cleared
void Copter::set_throttle_takeoff()
{
    // tell position controller to reset alt target and reset I terms
    pos_control.init_takeoff();

    // tell motors to do a slow start
    motors.slow_start(true);
}

// get_pilot_desired_throttle - transform pilot's throttle input to make cruise throttle mid stick
// used only for manual throttle modes
// returns throttle output 0 to 1000
int16_t Copter::get_pilot_desired_throttle(int16_t throttle_control)
{
    int16_t throttle_out;

    int16_t mid_stick = channel_throttle->get_control_mid();

    // ensure reasonable throttle values
    throttle_control = constrain_int16(throttle_control,0,1000);
    g.throttle_mid = constrain_int16(g.throttle_mid,g.throttle_min+50,700);

    // check throttle is above, below or in the deadband
    if (throttle_control < mid_stick) {
        // below the deadband
        throttle_out = g.throttle_min + ((float)(throttle_control-g.throttle_min))*((float)(g.throttle_mid - g.throttle_min))/((float)(mid_stick-g.throttle_min));
    }else if(throttle_control > mid_stick) {
        // above the deadband
        throttle_out = g.throttle_mid + ((float)(throttle_control-mid_stick)) * (float)(1000-g.throttle_mid) / (float)(1000-mid_stick);
    }else{
        // must be in the deadband
        throttle_out = g.throttle_mid;
    }

    return throttle_out;
}

// get_pilot_desired_climb_rate - transform pilot's throttle input to
// climb rate in cm/s.  we use radio_in instead of control_in to get the full range
// without any deadzone at the bottom
float Copter::get_pilot_desired_climb_rate(float throttle_control)
{
    // throttle failsafe check
    if( failsafe.radio ) {
        return 0.0f;
    }

    float desired_rate = 0.0f;
    float mid_stick = channel_throttle->get_control_mid();
    float deadband_top = mid_stick + g.throttle_deadzone;
    float deadband_bottom = mid_stick - g.throttle_deadzone;

    // ensure a reasonable throttle value
    throttle_control = constrain_float(throttle_control,g.throttle_min,1000.0f);

    // ensure a reasonable deadzone
    g.throttle_deadzone = constrain_int16(g.throttle_deadzone, 0, 400);

    // check throttle is above, below or in the deadband
    if (throttle_control < deadband_bottom) {
        // below the deadband
        desired_rate = g.pilot_velocity_z_max * (throttle_control-deadband_bottom) / (deadband_bottom-g.throttle_min);
    }else if (throttle_control > deadband_top) {
        // above the deadband
        desired_rate = g.pilot_velocity_z_max * (throttle_control-deadband_top) / (1000.0f-deadband_top);
    }else{
        // must be in the deadband
        desired_rate = 0.0f;
    }

    // desired climb rate for logging
    desired_climb_rate = desired_rate;

    return desired_rate;
}

// get_non_takeoff_throttle - a throttle somewhere between min and mid throttle which should not lead to a takeoff
float Copter::get_non_takeoff_throttle()
{
    return (g.throttle_mid / 2.0f);
}

float Copter::get_takeoff_trigger_throttle()
{
    return channel_throttle->get_control_mid() + g.takeoff_trigger_dz;
}

// get_throttle_pre_takeoff - convert pilot's input throttle to a throttle output before take-off
// used only for althold, loiter, hybrid flight modes
// returns throttle output 0 to 1000
float Copter::get_throttle_pre_takeoff(float input_thr)
{
    // exit immediately if input_thr is zero
    if (input_thr <= 0.0f) {
        return 0.0f;
    }

    // TODO: does this parameter sanity check really belong here?
    g.throttle_mid = constrain_int16(g.throttle_mid,g.throttle_min+50,700);

    float in_min = g.throttle_min;
    float in_max = get_takeoff_trigger_throttle();

#if FRAME_CONFIG == HELI_FRAME
    // helicopters swash will move from bottom to 1/2 of mid throttle
    float out_min = 0;
#else
    // multicopters will output between spin-when-armed and 1/2 of mid throttle
    float out_min = motors.get_throttle_warn();
#endif
    float out_max = get_non_takeoff_throttle();

    if ((g.throttle_behavior & THR_BEHAVE_FEEDBACK_FROM_MID_STICK) != 0) {
        in_min = channel_throttle->get_control_mid();
    }

    float input_range = in_max-in_min;
    float output_range = out_max-out_min;

    // sanity check ranges
    if (input_range <= 0.0f || output_range <= 0.0f) {
        return 0.0f;
    }

    return constrain_float(out_min + (input_thr-in_min)*output_range/input_range, out_min, out_max);
}

// get_surface_tracking_climb_rate - hold copter at the desired distance above the ground
//      returns climb rate (in cm/s) which should be passed to the position controller
float Copter::get_surface_tracking_climb_rate(int16_t target_rate, float current_alt_target, float dt)
{
    static uint32_t last_call_ms = 0;
    float distance_error;
    float velocity_correction;
    float current_alt = inertial_nav.get_altitude();

    uint32_t now = millis();

    // reset target altitude if this controller has just been engaged
    if (now - last_call_ms > SONAR_TIMEOUT_MS) {
        target_sonar_alt = sonar_alt + current_alt_target - current_alt;
    }
    last_call_ms = now;

    // adjust sonar target alt if motors have not hit their limits
    if ((target_rate<0 && !motors.limit.throttle_lower) || (target_rate>0 && !motors.limit.throttle_upper)) {
        target_sonar_alt += target_rate * dt;
    }

    // do not let target altitude get too far from current altitude above ground
    // Note: the 750cm limit is perhaps too wide but is consistent with the regular althold limits and helps ensure a smooth transition
    target_sonar_alt = constrain_float(target_sonar_alt,sonar_alt-pos_control.get_leash_down_z(),sonar_alt+pos_control.get_leash_up_z());

    // calc desired velocity correction from target sonar alt vs actual sonar alt (remove the error already passed to Altitude controller to avoid oscillations)
    distance_error = (target_sonar_alt - sonar_alt) - (current_alt_target - current_alt);
    velocity_correction = distance_error * g.sonar_gain;
    velocity_correction = constrain_float(velocity_correction, -THR_SURFACE_TRACKING_VELZ_MAX, THR_SURFACE_TRACKING_VELZ_MAX);

    // return combined pilot climb rate + rate to correct sonar alt error
    return (target_rate + velocity_correction);
}

// set_accel_throttle_I_from_pilot_throttle - smoothes transition from pilot controlled throttle to autopilot throttle
void Copter::set_accel_throttle_I_from_pilot_throttle(int16_t pilot_throttle)
{
    // shift difference between pilot's throttle and hover throttle into accelerometer I
    g.pid_accel_z.set_integrator(pilot_throttle-throttle_average);
}

// updates position controller's maximum altitude using fence and EKF limits
void Copter::update_poscon_alt_max()
{
    float alt_limit_cm = 0.0f;  // interpreted as no limit if left as zero

#if AC_FENCE == ENABLED
    // set fence altitude limit in position controller
    if ((fence.get_enabled_fences() & AC_FENCE_TYPE_ALT_MAX) != 0) {
        alt_limit_cm = pv_alt_above_origin(fence.get_safe_alt()*100.0f);
    }
#endif

    // get alt limit from EKF (limited during optical flow flight)
    float ekf_limit_cm = 0.0f;
    if (inertial_nav.get_hgt_ctrl_limit(ekf_limit_cm)) {
        if ((alt_limit_cm <= 0.0f) || (ekf_limit_cm < alt_limit_cm)) {
            alt_limit_cm = ekf_limit_cm;
        }
    }

    // pass limit to pos controller
    pos_control.set_alt_max(alt_limit_cm);
}

// rotate vector from vehicle's perspective to North-East frame
void Copter::rotate_body_frame_to_NE(float &x, float &y)
{
    float ne_x = x*ahrs.cos_yaw() - y*ahrs.sin_yaw();
    float ne_y = x*ahrs.sin_yaw() + y*ahrs.cos_yaw();
    x = ne_x;
    y = ne_y;
}
