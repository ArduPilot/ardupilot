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


#include <BEV_SpdHgt.h>
#include <BEV_TransitionState.h>
#include <BEV_Effectors.h>

//called on system startup
static void bev_uorb_init()
{
    bev_key.init();
    SpdHgt_Controller.init();
    payload_manager.init();
    //set the needed gimbal objects
    payload_manager.gimbal.set_ahrs_inav(&ahrs, &inertial_nav);
    servos.init();
}

//called at 50hz
static void bev_uorb_update()
{
    bev_key.update();
    payload_manager.update();
}

static void plane_50hz_tasks(void) {
    //Check to see if it's necesary to run plane control laws
    if (!motors.transition_is_plane_active()) {
        //we're entirely helicopter. No need for the runtime penalty
        return;
    }

    //verify the user didn't click the 'wingless' button when flying as plane
    if(g.wingless) {
        g.wingless.set_and_save(0);
    }

    //Update the speed height controller w/ desired altitude
    if (auto_throttle_mode) {
        SpdHgt_Controller.update(alt_hold_gs_des_alt);
    }

    plane_update_flight_mode();
    plane_stabilize();
    plane_set_servos();
}

/*****************************************
* Set the flight control servos based on the current calculated values
*****************************************/
static void plane_set_servos(void)
{
    static int16_t last_throttle = 0;

    if (control_mode == ACRO) {
        //bev direct pass through of radio in. Effectors class expects channels in [-4500 4500] (angle) so use control_in
        if(is_transitioning_get_angles(channel_roll_out, channel_pitch_out)) {
            //channel roll and pitch out set by above function call.
            channel_throttle_out = g.rc_3.control_in;
            last_throttle = channel_throttle_out;
        } else {
            //not transitioning. Direct pass through.
            channel_roll_out                = g.rc_1.control_in / 2; //half roll authority.
            channel_pitch_out               = g.rc_2.control_in;
            last_throttle = channel_throttle_out;
            channel_throttle_out            = g.rc_3.control_in;
            channel_rudder_out              = g.rc_4.control_in;
        }
    } else { //control_mode != ACRO
        //just to be certain throttle is in correct range
        channel_throttle_out = constrain_int16(channel_throttle_out, 0, 1000);

        if (control_mode == STABILIZE) {
            //manual throttle out. Reset last_throttle so that when the slew limiter is engaged
            //it starts at the current throttle setting
            channel_throttle_out = g.rc_3.control_in;
            last_throttle = channel_throttle_out;
        } else {
            //apply the throttle slewer
            throttle_slew_limit(last_throttle);
            last_throttle = channel_throttle_out;
        }
    }
}

static void plane_update_flight_mode(void) {

    switch (control_mode) {
    case AUTO:
        handle_auto_mode();
        break;

    case LOITER:
        update_target_alt();
        //fall through
    case RTL:
    case GUIDED:
        calc_nav_roll();
        calc_nav_pitch();
        calc_throttle();
        break;

    case LAND:
        //transition immediately if not already.
        if(motors.transition_get_direction() != BEV_TransitionState::DIRECTION_TO_COPTER) {
            motors.transition_to_copter();
            nav_roll_cd = nav_pitch_cd = 0;
        }

    case STABILIZE:
    { //bracket needed. See http://stackoverflow.com/questions/5685471/error-jump-to-case-label
        // set nav_roll and nav_pitch using sticks
        if(roll_pitch_input_valid()) {
            nav_roll_cd = g.rc_1.norm_input() * g.roll_limit_cd;
            nav_roll_cd = constrain_int32(nav_roll_cd, -g.roll_limit_cd, g.roll_limit_cd);
            float pitch_input = g.rc_2.norm_input();
            if (pitch_input > 0) {
                nav_pitch_cd = pitch_input * aparm.pitch_limit_max_cd;
            } else {
                nav_pitch_cd = -(pitch_input * aparm.pitch_limit_min_cd);
            }
            nav_pitch_cd = constrain_int32(nav_pitch_cd, aparm.pitch_limit_min_cd.get(), aparm.pitch_limit_max_cd.get());
        } else {
            nav_roll_cd = nav_pitch_cd = 0;
        }
        break;
    }

    case ALT_HOLD:
        if(roll_pitch_input_valid()) {
            nav_roll_cd = g.rc_1.norm_input() * g.roll_limit_cd;
        } else {
            nav_roll_cd = 0;
        }
        update_target_alt();
        break;

    case CIRCLE:
        nav_roll_cd = g.roll_limit_cd / 2;
        //BEV in our circle mode pilot can change altitude
        update_target_alt();
        calc_nav_pitch();
        calc_throttle();
        break;

    case ACRO:
        //bev this is taken care of in set_servos
        nav_roll_cd = 0;
        nav_pitch_cd = 0;
        break;

    }

    //if transitioning, set the desired pitch and roll open loop
    if (motors.transition_is_pitch_override()) {
        nav_pitch_cd = motors.transition_get_pitch_override();
        nav_roll_cd = 0;
    }
}

static void plane_update_navigation() {
    switch (control_mode) {
    case LAND:
        //if full plane or attempting to transition to plane, transition to copter.
        if (motors.transition_is_full_plane()
                || motors.transition_get_direction() == BEV_TransitionState::DIRECTION_TO_PLANE) {
            motors.transition_to_copter();
        }
        update_loiter();
        break;
    case RTL:
        //BEV if near home, transition to copter and complete the landing
        if (near_land_point_transition_copter() && motors.transition_is_full_plane()) {
            motors.transition_to_copter();
            rtl_force_wpnav = true;
        }
        //fall through intentional
    case LOITER:
    case GUIDED:
        update_loiter();
        break;
    case AUTO:
        //all updates called from master scheduler. hook is run_nav_updates
        break;
    }
}

//log every 10  seconds, unless transitioning then do it at 10hz
static void do_transition_logging()
{
    static uint32_t last_log_time = 0;
    uint16_t time_between_logs = 0;

    if(motors.transition_is_transitioning()) {
        time_between_logs = 100; //0.1 second
    } else {
        time_between_logs = 10000; //10 seconds
    }

    if(motors.armed() && can_force_log()) {
        if((hal.scheduler->millis() - last_log_time) > time_between_logs) {
            last_log_time = hal.scheduler->millis();
            //write the log
            Log_Write_Transition(motors.transition_get_direction(),
                                 0, //no longer used
                                 motors.transition_get_angle_deg(),
                                 motors.transition_is_nav_suppressed(),
                                 motors.transition_get_copter_percent(),
                                 motors.transition_get_plane_percent());
        }
    }
}

static bool is_on_ground_maybe()
{
    //tries to determine if the vehicle is on the ground
    return motors.gear_is_lowered() && //gear down
           inertial_nav.get_altitude() < 1000 && //alt < 10m
           gps.ground_speed() < 3; //ground speed is less than 3 m/s
}

//****************************************************************
// Function that will calculate the desired direction to fly and distance
//****************************************************************
static void plane_navigate()
{
    if(!motors.transition_is_plane_active()) {
        return;
    }

    if (next_WP_loc.lat == 0) {
        return;
    }

    // waypoint distance from plane
    // ----------------------------
    wp_distance = get_distance(current_loc, next_WP_loc) * 100; //putting it in cm for compatability w/ copter definitions

    // update total loiter angle
    loiter_angle_update();

    // control mode specific updates to navigation demands
    // ---------------------------------------------------
    plane_update_navigation();
}


/*
  pitch stick used to adjust altitude setpoint when in alt hold, circle, and loiter modes
 */
static void update_target_alt(void)
{
    static uint32_t last_call_time = 0;

    float dt = (hal.scheduler->millis() - last_call_time)*0.001;
    dt = constrain_float(dt, 0, 0.10);

    int16_t target_climb_rate = get_plane_pilot_desired_climb_rate(g.rc_3.control_in);

    //if commanding climb rate greater than 0.1 m/s, adjust the altitude setpoint
    if(abs(target_climb_rate) > 0.1) {
        alt_hold_gs_des_alt += target_climb_rate * dt;
        //don't let the error build to unreasonable levels
        alt_hold_gs_des_alt = inertial_nav.get_altitude() + constrain_float(alt_hold_gs_des_alt - inertial_nav.get_altitude(), -SpdHgt_Controller.get_max_alt_error_cm(), SpdHgt_Controller.get_max_alt_error_cm());
    }

    calc_throttle();
    calc_nav_pitch();
}
// get_plane_pilot_desired_climb_rate - transform pilot's throttle input to
// climb rate in cm/s.  we use radio_in instead of control_in to get the full range
// without any deadzone at the bottom
static int16_t get_plane_pilot_desired_climb_rate(int16_t throttle_control)
{
    int16_t desired_rate = 0;

    // throttle failsafe check
    if( !throttle_input_valid() ) {
        return 0;
    }

    // ensure a reasonable throttle value
    throttle_control = constrain_int16(throttle_control,0,1000);

    // ensure a reasonable deadzone
    g.throttle_deadzone = constrain_int16(g.throttle_deadzone, 0, 400);

    // check throttle is above, below or in the deadband
    if (throttle_control < THROTTLE_IN_DEADBAND_BOTTOM) {
        // below the deadband
        desired_rate = (int32_t)g.flybywire_climb_rate * (throttle_control-THROTTLE_IN_DEADBAND_BOTTOM) / (THROTTLE_IN_MIDDLE - g.throttle_deadzone);
    }else if (throttle_control > THROTTLE_IN_DEADBAND_TOP) {
        // above the deadband
        desired_rate = (int32_t)g.flybywire_climb_rate * (throttle_control-THROTTLE_IN_DEADBAND_TOP) / (THROTTLE_IN_MIDDLE - g.throttle_deadzone);
    }else{
        // must be in the deadband
        desired_rate = 0;
    }

    return desired_rate;
}

static void update_loiter()
{
    // allow loiter direction to be changed in flight
    if (g.plane_loiter_radius < 0) {
        nav_controller->update_loiter(next_WP_loc, abs(g.plane_loiter_radius), -1);
    } else {
        nav_controller->update_loiter(next_WP_loc, abs(g.plane_loiter_radius), 1);
    }
}
/*
  reset the total loiter angle
 */
static void loiter_angle_reset(void)
{
    loiter.sum_cd = 0;
    loiter.total_cd = 0;
}

/*
  update the total angle we have covered in a loiter. Used to support
  commands to do N circles of loiter
 */
static void loiter_angle_update(void)
{
    int32_t current_heading = ahrs.yaw_sensor; //centi-degrees from mag north
    int32_t current_delta = 0;

    if(loiter.sum_cd == 0) {
        //first call.
        current_delta = 1;
    } else {
        current_delta = current_heading - loiter.old_target_bearing_cd;
    }
    loiter.old_target_bearing_cd = current_heading;
    current_delta = wrap_180_cd(current_delta);
    loiter.sum_cd += current_delta;
}

/*
 main handling for AUTO mode
 */
static void handle_auto_mode(void) {
    calc_nav_roll();
    calc_nav_pitch();
    calc_throttle();
}

//called when copter first becomes active in a transition from plane to copter
void to_copter_callback() {
    //make sure the ahrs knows we're a copter
    ahrs.set_vehicle_class(AHRS_VEHICLE_COPTER);

    //reset the body axis rate targets to the current body axis rate. This enforces acceleration and velocity continuity when entering the new mode
    attitude_control.relax_bf_rate_controller();
    //clear the rate integrators.
    attitude_control.reset_rate_integrators();

    //float temp_alt;
    switch (control_mode) {
        case STABILIZE:
            //prepare to enter the control mode
            stabilize_init(true);
            break;
        case ALT_HOLD:
           althold_init(true);
           break;
       case LOITER:
           loiter_init(true);
        case GUIDED:
            guided_init(true);
            guided_set_destination(pv_location_to_vector(next_WP_loc));
            alt_hold_gs_des_alt = next_WP_loc.alt;
            break;
        case RTL:
            if(rtl_force_wpnav) {
                rtl_return_start();
            } else {
                rtl_init(true);
            }
            break;
        case AUTO:
            //auto_init(true);
            break;
        case LAND:
            land_init(true);
            break;

    }
}

void to_copternav_callback() {
    switch(control_mode) {
    case LOITER:
        loiter_init(true);
        break;
    case RTL:
        rtl_init(true);
        //this *should* reset the leash and keep copter from going backwards to hit the least point after a transition
        wp_nav.set_wp_destination(Vector3f(0,0,get_RTL_alt()));
        break;
    case GUIDED:
        guided_set_destination(pv_location_to_vector(next_WP_loc));
        alt_hold_gs_des_alt = next_WP_loc.alt;
        break;
    case AUTO:
        auto_init(true);
        //this *should* reset the leash and keep copter from going backwards to hit the least point after a transition
        wp_nav.set_wp_destination(wp_nav.get_destination());
        break;
    case LAND:
        land_init(true);
        break;
    }
}
//called when plane first becomes active in a transition from copter to plane
void to_plane_callback()
{
    rollController.reset_I();
    pitchController.reset_I();

    //disarm if on the ground, throttle at minimum, and armed. DOn't waot to have it accidentally 'fly away' when the alt_hold controller kicks in
    if( motors.armed() && is_on_ground_maybe() && (g.rc_3.control_in == 0) &&(control_mode == ALT_HOLD || control_mode == LOITER)) {
        init_disarm_motors();
    }

    switch(control_mode) {
    case STABILIZE:
        break;
    case ALT_HOLD:
        SpdHgt_Controller.initialize();
        alt_hold_gs_des_alt = current_loc.alt;
        break;
    case LOITER:
        //loiter about the current copter loiter position
        SpdHgt_Controller.initialize();
        next_WP_loc = current_loc;
        break;
    case GUIDED:
        SpdHgt_Controller.initialize();
        prev_WP_loc = current_loc;
        next_WP_loc = pv_vector_to_location(wp_nav.get_destination());
        alt_hold_gs_des_alt = next_WP_loc.alt;
        break;
    case RTL:
        SpdHgt_Controller.initialize();
        prev_WP_loc = current_loc;
        next_WP_loc = ahrs.get_home();
        alt_hold_gs_des_alt = get_RTL_alt();
        break;
    case AUTO:
        SpdHgt_Controller.initialize();
        prev_WP_loc = current_loc;
        break;
    case LAND:
        //bev don't allow transitioning to plane when in land mode
        motors.transition_to_copter();
        break;
    }
}


static bool is_plane_nav_active()
{
    return motors.transition_is_full_plane();
}

static bool is_copter_nav_active()
{
    return !is_plane_nav_active();
}

//determines if near enough to the set landing point to transition to copter. Used
//by RTL and land
static bool near_land_point_transition_copter()
{
    return (wp_distance < BEV_RTL_TRANSITION_DISTANCE) && motors.transition_is_full_plane();
}

static void override_transitions_to_plane()
{
    if(motors.transition_get_direction() == BEV_TransitionState::DIRECTION_TO_PLANE) { //to plane
        motors.transition_to_copter();
    }
}

static bool handle_bev_request(uint8_t request)
{
    //dont allow in wingless ops
    if(g.wingless) {
        gcs_send_text_P(SEVERITY_HIGH,PSTR("Disabled: Wingless flight"));
        return false;
    }

    if(request == BEV_REQUEST_SERVOS_TOGGLE) {
        servos.toggle();
        //BEV not intuitive, but have the  also toggle servos. This weird
        //bit comes from the limited number of joystuck buttons we have
        payload_manager.gimbal.point_here(0,0,0);
        return true;
    }

    if( (control_mode == AUTO)) {
        return false;
    } else if (get_key_level() < BEV_Key::KEY_SPORT) {
        //alert the user if they need a higher level key
        gcs_send_text_P(SEVERITY_HIGH,PSTR("Sport key needed"));
        return false;
    }

    switch(request)  {
        case BEV_REQUEST_NONE:
            break;
        case BEV_REQUEST_GEAR_UP:
            motors.gear_raise();
            break;
        case BEV_REQUEST_GEAR_DOWN:
            motors.gear_lower();
            break;
        case BEV_REQUEST_GEAR_TOGGLE:
            motors.gear_toggle();
            break;
        case BEV_REQUEST_TRANSITION_TO_COPTER:
            motors.transition_to_copter();
            break;
        case BEV_REQUEST_TRANSITION_TO_PLANE:
            motors.transition_to_plane();
            break;
        case BEV_REQUEST_TRANSITION_TOGGLE:
            motors.transition_toggle();
            break;
    }

    return true;
}

static void transition_toggle()
{
    //don't allow in wingless ops
    if(g.wingless)
        return;

    motors.transition_toggle();
}

static void gear_toggle()
{
    //don't allow in wingless ops
    if(g.wingless)
        return;

    motors.gear_toggle();
}

static void gear_raise()
{
    //don't allow in wingless ops
    if(g.wingless)
        return;

    motors.gear_raise();
}

static void gear_lower()
{
    //don't allow in wingless ops
    if(g.wingless)
        return;

    motors.gear_lower();
}

static void transition_to_copter()
{
    //don't allow in wingless ops
    if(g.wingless)
        return;

    motors.transition_to_copter();
}

static bool is_transitioning_get_angles(int16_t &target_roll, int16_t &target_pitch)
{
    //if transitioning, set the desired pitch and roll open loop
    if(motors.transition_is_pitch_override()) {
        target_pitch = motors.transition_get_pitch_override();
        target_roll = 0;
    } else if (motors.transition_is_nav_suppressed() && !(control_mode == ACRO || control_mode == STABILIZE || control_mode == ALT_HOLD)) {
        target_pitch  = 0;
        target_roll = 0;
    } else {
        return false;
    }

    //BEV allow roll control when in acro, stabilize, and alt_hold
    if( (control_mode == ACRO) ||
        (control_mode == STABILIZE) ||
        (control_mode == ALT_HOLD)) {
        int16_t temp_roll = 0, temp_pitch = 0;
        get_pilot_desired_lean_angles(g.rc_1.control_in, g.rc_2.control_in, temp_roll, temp_pitch);
        target_roll = constrain_int16(temp_roll,-1200,1200); //limit roll to 12 deg while transitioning
    }

    return true;
}

static void drive_elevons()
{
    //BEV command the elevons when plane is not active.
    if(!is_plane_nav_active()) {
        if(control_mode == ACRO) {
            if(!is_transitioning_get_angles(channel_roll_out, channel_pitch_out)) {
                channel_roll_out                = g.rc_1.control_in;
                channel_pitch_out               = g.rc_2.control_in;
            }
        } else {
            if(motors.transition_is_pitch_override()) {
                nav_pitch_cd = motors.transition_get_pitch_override();
                nav_roll_cd = 0;
            }else if(control_mode == STABILIZE || control_mode == ALT_HOLD) {
                //get pilot desired angle. THis works even if disarmed.
                int16_t temp_roll, temp_pitch;
                get_pilot_desired_lean_angles(g.rc_1.control_in, g.rc_2.control_in, temp_roll, temp_pitch);
                nav_roll_cd = temp_roll; //get_pilot_desired_lean_angles expects int16_t as arg, but nav_roll_cd is int32. This resolves
                nav_pitch_cd = temp_pitch; //get_pilot_desired_lean_angles expects int16_t as arg, but nav_roll_cd is int32. This resolves
            } else {
                //get the nav controllers desired roll and pitch
                const Vector3f &targets = attitude_control.angle_ef_targets();
                nav_roll_cd = targets.x;
                nav_pitch_cd = targets.y;
            }

            //BEV allow pilot to set bank angle when transitioning but only if in acro, stabilize, or alt_hold
            if( (motors.transition_is_pitch_override()) &&
                 ((control_mode == ACRO) || (control_mode == STABILIZE) || (control_mode == ALT_HOLD)) ) {
                int16_t temp_roll = 0, temp_pitch = 0;
                get_pilot_desired_lean_angles(g.rc_1.control_in, g.rc_2.control_in, temp_roll, temp_pitch);
                nav_roll_cd = constrain_int16(temp_roll,-1200,1200); //limit roll to 20 deg while transitioning
            }

            //BEV run the attitude controllers with speed scalar set to maximum possible (low airspeed), and disable the integrators
            channel_roll_out = rollController.get_servo_out(nav_roll_cd - ahrs.roll_sensor, 1.667, true);
            channel_pitch_out = pitchController.get_servo_out(nav_pitch_cd - ahrs.pitch_sensor, 1.667, true);
        }

        //BEV when in copter stabilize or alt_hold and disarmed drive the elevons open loop based on pilot input.
        //this allows the elevons to be trimmed in stabilize mode (0.60" trailing edge up) and affords an on ground
        //check that the elevons are functioning and rc channels aren't reversed
        if( (control_mode == STABILIZE || control_mode == ALT_HOLD) && (!motors.armed()) && (!motors.transition_is_pitch_override())) {
            channel_roll_out = g.rc_1.control_in/2; //half authority so it looks about right
            channel_pitch_out = g.rc_2.control_in/2;
        }
    }
}

static void bev_motors_output()
{
    if(get_key_level() < BEV_Key::KEY_SPORT) {
        //no transitions, gear, or axle ff if no key active
        motors.output(true, 0);
    } else if (g.wingless) {
        //don't allow transitions or gear if in wingless flight
        motors.output(true, get_desired_transition_axle_angle());
    }else if( (control_mode == AUTO) || !transition_gear_input_valid() ) {
        //don't allow gear and transition switches to be read if in auto mode, radio failsafe, or sport key unverified
        motors.output(true, get_desired_transition_axle_angle());
    } else {
        motors.output(false, get_desired_transition_axle_angle());
    }
}

//determine what the transition axle angle should be
int16_t get_desired_transition_axle_angle()
{
    //no transition axle angle in acro mode.
    if(control_mode == ACRO ) {
        return 0;
    } else {
        return nav_pitch_cd;
    }
}

//returns true if the pro-key matches the processor id's hash.
static uint8_t get_key_level()
{
    return bev_key.get_key_level();
}

static void say_key_level()
{
    switch(get_key_level()) {
    case BEV_Key::KEY_NONE:
        gcs_send_text_P(SEVERITY_HIGH,PSTR("No Key Active"));
        break;
    case BEV_Key::KEY_SPORT:
        gcs_send_text_P(SEVERITY_HIGH,PSTR("Sport Key Verified"));
        break;
    case BEV_Key::KEY_PRO:
        gcs_send_text_P(SEVERITY_HIGH,PSTR("Pro Key Verified"));
        break;
    case BEV_Key::KEY_MAPPING:
        gcs_send_text_P(SEVERITY_HIGH,PSTR("Mapping Key Verified"));
        break;
    }
}
