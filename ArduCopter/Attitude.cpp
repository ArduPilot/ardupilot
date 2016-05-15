/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/*
 *  Copyright (c) BirdsEyeView Aerobotics, LLC, 2016.
 *
 *  This program is free software: you can redistribute it and/or modify it
 *  under the terms of the GNU General Public License version 3 as published
 *  by the Free Software Foundation.
 *
 *  This program is distributed in the hope that it will be useful, but
 *  WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General
 *  Public License version 3 for more details.
 *
 *  You should have received a copy of the GNU General Public License version
 *  3 along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

// get_smoothing_gain - returns smoothing gain to be passed into attitude_control.angle_ef_roll_pitch_rate_ef_yaw_smooth
//      result is a number from 2 to 12 with 2 being very sluggish and 12 being very crisp
float get_smoothing_gain()
{
    //BEV hardcoded in 6.5, which works out to rc_feel_rp = 45
    //this keeps users from accidentally setting 'dagnerous' parameters
    return 6.5;
    //return (2.0f + (float)g.rc_feel_rp/10.0f);
}

// get_pilot_desired_angle - transform pilot's roll or pitch input into a desired lean angle
// returns desired angle in centi-degrees
//BEV hard coded in 12 degrees roll, 12 deg nose up, 25 deg nose down
static void get_pilot_desired_lean_angles(int16_t roll_in, int16_t pitch_in, int16_t &roll_out, int16_t &pitch_out)
{
    // range check the input
    roll_in = constrain_int16(roll_in, -ROLL_PITCH_INPUT_MAX, ROLL_PITCH_INPUT_MAX);
    pitch_in = constrain_int16(pitch_in, -ROLL_PITCH_INPUT_MAX, ROLL_PITCH_INPUT_MAX);

    // convert pilot input to lean angle
    roll_out = (int16_t)((float)roll_in * 0.2666); //1200/4500 = 0.2666
    if(pitch_in > 0) {
        pitch_out = (int16_t)((float)pitch_in * 0.2666); //1200/4500 = 0.2666
    } else {
        pitch_out = (int16_t)((float)pitch_in * 0.4444); //2000/4500 = 0.4444
    }
}

// get_pilot_desired_heading - transform pilot's yaw input into a desired heading
// returns desired angle in centi-degrees
// To-Do: return heading as a float?
static float get_pilot_desired_yaw_rate(int16_t stick_angle)
{
    int32_t roll2yawff = 0;

    //BEV add in bank angle feed forward which will tend to turn vehicle into the wind
    return stick_angle * g.acro_yaw_p + get_roll2yaw_ff();
}

//BEV returns yaw rate which will turn vehicle into the wind
static int32_t get_roll2yaw_ff()
{
    //default
    int32_t ret = 0;

    //use 40% dead band on nav_roll when applying roll2yaw ff
    if(nav_roll_cd > 480) { // 4.8 degrees
        ret = 3*(nav_roll_cd - 480);
    } else if (nav_roll_cd < -480) { //-4.8 degrees
        ret = 3*(nav_roll_cd + 480);
    }

    return constrain_int32(ret,-2000,2000);
}

/*************************************************************
 * yaw controllers
 *************************************************************/

// get_roi_yaw - returns heading towards location held in roi_WP
// should be called at 100hz
static float get_roi_yaw()
{
    static uint8_t roi_yaw_counter = 0;     // used to reduce update rate to 10hz

    roi_yaw_counter++;
    if (roi_yaw_counter >= 10) {
        roi_yaw_counter = 0;
        yaw_look_at_WP_bearing = pv_get_bearing_cd(inertial_nav.get_position(), roi_WP);
    }

    return yaw_look_at_WP_bearing;
}

static float get_look_ahead_yaw()
{
    // Commanded Yaw to automatically look ahead.
    if (gps.status() >= AP_GPS::GPS_OK_FIX_2D && gps.ground_speed_cm() > YAW_LOOK_AHEAD_MIN_SPEED) {
        yaw_look_ahead_bearing = gps.ground_course_cd();
    }
    return yaw_look_ahead_bearing;
}

/*************************************************************
 *  throttle control
 ****************************************************************/

// update_thr_cruise - update throttle cruise if necessary
//  should be called at 100hz
static void update_thr_cruise()
{
    //BEV only do this if full copter
    if(is_plane_nav_active()) {
        return;
    }

    // ensure throttle_avg has been initialised
    if( fabs(throttle_avg)  < 0.01 ) {
        throttle_avg = g.throttle_cruise;
        // update position controller
        pos_control.set_throttle_hover(throttle_avg);
    }

    // if not armed or landed exit
    if (!motors.armed() || ap.land_complete) {
        return;
    }

    // get throttle output
    int16_t throttle = g.rc_3.servo_out;

    // calc average throttle if we are in a level hover
    if (throttle > g.throttle_min && abs(climb_rate) < 60 && labs(ahrs.roll_sensor) < 500 && labs(ahrs.pitch_sensor) < 500) {
        throttle_avg = throttle_avg * 0.99f + (float)throttle * 0.01f;
        g.throttle_cruise = throttle_avg;
        // update position controller
        pos_control.set_throttle_hover(throttle_avg);
    }
}

// set_throttle_takeoff - allows parents to tell throttle controller we are taking off so I terms can be cleared
static void
set_throttle_takeoff()
{
    // tell position controller to reset alt target and reset I terms
    pos_control.init_takeoff();

    // tell motors to do a slow start
    motors.slow_start(true);
}

// get_pilot_desired_throttle - transform pilot's throttle input to make cruise throttle mid stick
// used only for manual throttle modes
// returns throttle output 0 to 1000
#define THROTTLE_IN_MIDDLE 500          // the throttle mid point
static int16_t get_pilot_desired_throttle(int16_t throttle_control)
{
    int16_t throttle_out;

    // exit immediately in the simple cases
    if( throttle_control == 0 || g.throttle_mid == 500) {
        return throttle_control;
    }

    // ensure reasonable throttle values
    throttle_control = constrain_int16(throttle_control,0,1000);
    g.throttle_mid = constrain_int16(g.throttle_mid,300,700);

    // check throttle is above, below or in the deadband
    if (throttle_control < THROTTLE_IN_MIDDLE) {
        // below the deadband
        throttle_out = g.throttle_min + ((float)(throttle_control-g.throttle_min))*((float)(g.throttle_mid - g.throttle_min))/((float)(500-g.throttle_min));
    }else if(throttle_control > THROTTLE_IN_MIDDLE) {
        // above the deadband
        throttle_out = g.throttle_mid + ((float)(throttle_control-500))*(float)(1000-g.throttle_mid)/500.0f;
    }else{
        // must be in the deadband
        throttle_out = g.throttle_mid;
    }

    return throttle_out;
}

// get_pilot_desired_climb_rate - transform pilot's throttle input to
// climb rate in cm/s.  we use radio_in instead of control_in to get the full range
// without any deadzone at the bottom
#define THROTTLE_IN_DEADBAND_TOP (THROTTLE_IN_MIDDLE+g.throttle_deadzone)  // top of the deadband
#define THROTTLE_IN_DEADBAND_BOTTOM (THROTTLE_IN_MIDDLE-g.throttle_deadzone)  // bottom of the deadband
static int16_t get_pilot_desired_climb_rate(int16_t throttle_control)
{
    int16_t desired_rate = 0;

    // throttle failsafe check
    if(!throttle_input_valid() ) {
        return 0;
    }

    // ensure a reasonable throttle value
    throttle_control = constrain_int16(throttle_control,0,1000);

    // ensure a reasonable deadzone
    g.throttle_deadzone = constrain_int16(g.throttle_deadzone, 0, 400);

    // check throttle is above, below or in the deadband
    if (throttle_control < THROTTLE_IN_DEADBAND_BOTTOM) {
        // below the deadband
        desired_rate = (int32_t)g.pilot_velocity_z_max * (throttle_control-THROTTLE_IN_DEADBAND_BOTTOM) / (THROTTLE_IN_MIDDLE - g.throttle_deadzone);
    }else if (throttle_control > THROTTLE_IN_DEADBAND_TOP) {
        // above the deadband
        desired_rate = (int32_t)g.pilot_velocity_z_max * (throttle_control-THROTTLE_IN_DEADBAND_TOP) / (THROTTLE_IN_MIDDLE - g.throttle_deadzone);
    }else{
        // must be in the deadband
        desired_rate = 0;
    }

    // desired climb rate for logging
    desired_climb_rate = desired_rate;

    return desired_rate;
}

// get_non_takeoff_throttle - a throttle somewhere between min and mid throttle which should not lead to a takeoff
static int16_t get_non_takeoff_throttle()
{
    //BEV determined from analysis of many landings that 355 (35.5%) is a good number to put here.
    return (355);
}

// get_throttle_pre_takeoff - convert pilot's input throttle to a throttle output before take-off
// used only for althold, loiter, hybrid flight modes
// returns throttle output 0 to 1000
static int16_t get_throttle_pre_takeoff(int16_t throttle_control)
{
    int16_t throttle_out;

    // exit immediately if throttle_control is zero
    if (throttle_control <= 0) {
        return 0;
    }

    // sanity check throttle input
    throttle_control = constrain_int16(throttle_control,0,1000);

    // sanity check throttle_mid
    g.throttle_mid = constrain_int16(g.throttle_mid,300,700);

    // sanity check throttle_min vs throttle_mid
    if (g.throttle_min > get_non_takeoff_throttle()) {
        return g.throttle_min;
    }

    // check throttle is below top of deadband
    if (throttle_control < THROTTLE_IN_DEADBAND_TOP) {
        throttle_out = g.throttle_min + ((float)(throttle_control-g.throttle_min))*((float)(get_non_takeoff_throttle() - g.throttle_min))/((float)(THROTTLE_IN_DEADBAND_TOP-g.throttle_min));
    }else{
        // must be in the deadband
        throttle_out = get_non_takeoff_throttle();
    }

    return throttle_out;
}

// set_accel_throttle_I_from_pilot_throttle - smoothes transition from pilot controlled throttle to autopilot throttle
static void set_accel_throttle_I_from_pilot_throttle(int16_t pilot_throttle)
{
    // shift difference between pilot's throttle and hover throttle into accelerometer I
    g.pid_throttle_accel.set_integrator(pilot_throttle-g.throttle_cruise);
}

/****************************************************/
/***** BEGIN PLANE ATTITUDE ADDITIONS ***************/
/****************************************************/
/*
  get a speed scaling number for control surfaces. This is applied to
  PIDs to change the scaling of the PID with speed. At high speed we
  move the surfaces less, and at low speeds we move them more.
 */
static float get_speed_scaler(void)
{
    //BEV just use throttle position. No airspeed sensor necessary
    float speed_scaler;

    speed_scaler = 1.667f - throttle_percentage() / 100.0f;
    //bev constrained tighter to prevent wing rocking at low throttle settings
    speed_scaler = constrain_float(speed_scaler, 0.667, 1.400);

    return speed_scaler;
}

static void calc_nav_pitch()
{
    // Calculate the Pitch of the plane
    // --------------------------------
    nav_pitch_cd = SpdHgt_Controller.get_desired_pitch();
    nav_pitch_cd = constrain_int32(nav_pitch_cd, aparm.pitch_limit_min_cd.get(), aparm.pitch_limit_max_cd.get());
}


static void calc_nav_roll()
{
    nav_roll_cd = nav_controller->nav_roll_cd();
    nav_roll_cd = constrain_int32(nav_roll_cd, -g.roll_limit_cd, g.roll_limit_cd);
}


static void throttle_slew_limit(int16_t last_throttle)
{
    static uint32_t last_call_time = 0;
    float dt = (last_call_time - hal.scheduler->millis()) * 0.001f;
    if(dt > 0.1f) {
        dt = 0.1f;
    }

    // limit throttle change by the given percentage per second
    float temp = aparm.throttle_slewrate * dt;
    // allow a minimum change of 1 per cycle
    if (temp < 1.0f) {
        temp = 1.0f;
    }
    channel_throttle_out = constrain_int16(channel_throttle_out, last_throttle - temp, last_throttle + temp);
}

/*
  main stabilization function for all 3 axes
 */
static void plane_stabilize()
{
    float speed_scaler = get_speed_scaler();
    //outputs to servos are done in plane_set_servos();
    if(control_mode != ACRO) {
        stabilize_roll(speed_scaler);
        stabilize_pitch(speed_scaler);
        stabilize_yaw(speed_scaler);
    }

    //zero integrators when on ground.
    if (is_on_ground_maybe()) {
        // This prevents integrator buildup pre-takeoff.
        rollController.reset_I();
        pitchController.reset_I();
        yawController.reset_I();
    }
}

static void calc_throttle()
{
    if(motors.armed()) {
        //BEV get_desired_throttle is [0 100], but channel_throttle_out is [0 1000]. A factor of 10 fixes this
        channel_throttle_out = SpdHgt_Controller.get_desired_throttle()*10;
    } else {
        channel_throttle_out = 0;
    }
}

/*
  this is the main roll stabilization function. It takes the
  previously set nav_roll calculates roll servo_out to try to
  stabilize the plane at the given roll
 */
static void stabilize_roll(float speed_scaler)
{
    channel_roll_out = rollController.get_servo_out(nav_roll_cd - ahrs.roll_sensor,
                                                               speed_scaler,
                                                               false); //bev don't disable the integrator
}

/*
  this is the main pitch stabilization function. It takes the
  previously set nav_pitch and calculates servo_out values to try to
  stabilize the plane at the given attitude.
 */
static void stabilize_pitch(float speed_scaler)
{
    channel_pitch_out = pitchController.get_servo_out(nav_pitch_cd - ahrs.pitch_sensor,
                                                      speed_scaler,
                                                      false); //bev don't disable integrator
}

static void stabilize_yaw(float speed_scaler)
{
    //only use the feedback yaw controller when airborn. Otherwise, go open loop
    if(is_on_ground_maybe()) {
        //open loop for maximum steerability when on ground
        channel_rudder_out = g.rc_4.control_in;
        yawController.reset_I();
        return;
    }

    channel_rudder_out = yawController.get_servo_out(speed_scaler, false);

    //BEV don't allow user input when operating in an antonomous mode
    if(control_mode <= ALT_HOLD) {
        channel_rudder_out += g.rc_4.control_in / 2; //add user yaw on top of controller output
    }

    //constrain
    channel_rudder_out = constrain_int16(channel_rudder_out/2, -4500, 4500);
}

void set_target_altitude_current()
{
    alt_hold_gs_des_alt = inertial_nav.get_altitude();
}


/*
  return throttle percentage from 0 to 100
 */
static uint8_t throttle_percentage(void)
{
    //Channel_throttle_out is [0 1000]. Multiply by 0.1 to get [0 100]
    return channel_throttle_out*0.1;
}

//  handle CRUISE mode, locking heading to GPS course when we have
//  sufficient ground speed, and no aileron input
static void update_cruise()
{
    if (!cruise_state.locked_heading &&
            g.rc_1.control_in == 0 &&
            gps.status() >= AP_GPS::GPS_OK_FIX_2D &&
            gps.ground_speed() >= 3 &&
            cruise_state.lock_timer_ms == 0)
    {
        // user wants to lock the heading - start the timer
        cruise_state.lock_timer_ms = hal.scheduler->millis();
    }
    if (cruise_state.lock_timer_ms != 0 &&
        (hal.scheduler->millis() - cruise_state.lock_timer_ms) > 500) {
        // lock the heading after 0.5 seconds of zero heading input
        // from user
        cruise_state.locked_heading = true;
        cruise_state.lock_timer_ms = 0;
        cruise_state.locked_heading_cd = gps.ground_course_cd();
        prev_WP_loc = current_loc;
    }
    if (cruise_state.locked_heading) {
        next_WP_loc = prev_WP_loc;
        // always look 1km ahead
        location_update(next_WP_loc,
                        cruise_state.locked_heading_cd*0.01f,
                        get_distance(prev_WP_loc, current_loc) + 1000);
        nav_controller->update_waypoint(prev_WP_loc, next_WP_loc);
    }
}

static void setup_turn_angle(void)
{
    int32_t next_ground_course_cd = mission.get_next_ground_course_cd(-1);
    if (next_ground_course_cd == -1) {
        // the mission library can't determine a turn angle, assume 90 degrees
        auto_state.next_turn_angle = 90.0f;
    } else {
        // get the heading of the current leg
        int32_t ground_course_cd = get_bearing_cd(prev_WP_loc, next_WP_loc);

        // work out the angle we need to turn through
        auto_state.next_turn_angle = wrap_180_cd(next_ground_course_cd - ground_course_cd) * 0.01f;
    }
}
