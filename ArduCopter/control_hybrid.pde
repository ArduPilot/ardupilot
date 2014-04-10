/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/*
 * control_hybrid.pde - init and run calls for hybrid flight mode
 *     hybrid tries to improve upon regular loiter by mixing the pilot input with the loiter controller
 */

#define HYBRID_SPEED_0                          10      //
#define HYBRID_LOITER_STAB_TIMER                300     // Must be higher than HYBRID_BRAKE_LOITER_MIX_TIMER (twice is a good deal) set it from 100 to 500, the number of centiseconds between loiter engage and getting wind_comp (once loiter stabilized)
#define HYBRID_BRAKE_LOITER_MIX_TIMER           150     // Number of cycles to transition from brake mode to loiter mode.  Must be lower than HYBRID_LOITER_STAB_TIMER
#define HYBRID_LOITER_MAN_MIX_TIMER             50      // Set it from 100 to 200, the number of centiseconds loiter and manual commands are mixed to make a smooth transition.
#define HYBRID_SMOOTH_RATE_FACTOR               0.04f   // controls the smoothness of the transition from ?? to ??.  0.04  = longer smoother transitions, 0.07 means faster transitions
#define HYBRID_STICK_RELEASE_SMOOTH_ANGLE       1800    // max angle required (in centi-degrees) after which the smooth stick release effect is applied
#define HYBRID_WIND_COMP_ESTIMATE_SPEED_MAX     10      // wind compensation estimates will only run when velocity is at or below this speed in cm/s

// declare some function to keep compiler happy
static void hybrid_update_wind_comp_estimate();
static void hybrid_get_wind_comp_lean_angles(int16_t &roll_angle, int16_t &pitch_angle);

// mission state enumeration
enum hybrid_rp_mode {
    HYBRID_PILOT_OVERRIDE=0,
    HYBRID_BRAKE=1,
    HYBRID_BRAKE_TO_LOITER=2,
    HYBRID_LOITER=3,
    HYBRID_LOITER_TO_PILOT_OVERRIDE=4
};

static struct {
    hybrid_rp_mode roll_mode    : 2;    // roll mode: pilot override, brake or loiter
    hybrid_rp_mode pitch_mode   : 2;    // pitch mode: pilot override, brake or loiter
    uint8_t loiter_engaged      : 1;    // 1 if loiter target has been set and loiter controller is running
} hybrid;

// wind compensation related variables
static Vector2f wind_comp_ef;                               // wind compensation in earth frame, filtered lean angles from position controller
static int16_t wind_offset_roll, wind_offset_pitch;         // wind offsets for pitch/roll
static int8_t  wind_offset_timer;                           // counter to reduce wind_offset calcs to 10hz
static int16_t wind_comp_start_timer;                       // counter to delay start of wind compensation calcs until loiter has settled

// breaking related variables
static float brake_gain;                                    // gain used during conversion of vehicle's velocity to lean angle during braking (calculated from brake_rate)
static int16_t brake_roll = 0, brake_pitch = 0;             // target roll and pitch angles during both pilot-override and braking periods.  Updated during pilot override to match pilot's input
static int16_t brake_timeout_roll, brake_timeout_pitch;     // time in seconds allowed for the braking to complete, this timeout will be updated at half-braking
static int16_t brake_roll_max, brake_pitch_max;             // used to detect half braking
static float brake_loiter_mix;                              // varies from 0 to 1, allows a smooth loiter engage
static bool brake_timeout_roll_updated, brake_timeout_pitch_updated;    // Allow the timeout to be updated only once per braking.

// loiter related variables
static float loiter_man_mix;                                // varies from 0 to 1, allow a smooth loiter to manual transition
static int16_t loiter_man_timer;
static int16_t loiter_roll, loiter_pitch;                   // store pitch/roll at loiter exit

// hybrid_init - initialise hybrid controller
static bool hybrid_init(bool ignore_checks)
{
    // fail to initialise hybrid mode if no GPS lock
    if (!GPS_ok() && !ignore_checks) {
        return false;
    }

    // set target to current position
    wp_nav.init_loiter_target();

    // clear pilot's feed forward for roll and pitch
    wp_nav.set_pilot_desired_acceleration(0, 0);

    // initialise altitude target to stopping point
    pos_control.set_target_to_stopping_point_z();

    // compute brake_gain
    brake_gain = (15.0f * (float)wp_nav._brake_rate + 95.0f) / 100.0f;

    if (ap.land_complete) {
        // Loiter start
        hybrid.roll_mode = HYBRID_LOITER;
        hybrid.pitch_mode = HYBRID_LOITER;
    }else{
        // Alt_hold like to avoid hard twitch if hybrid enabled in flight
        hybrid.roll_mode = HYBRID_PILOT_OVERRIDE;
        hybrid.pitch_mode = HYBRID_PILOT_OVERRIDE;
    }

    // initialise wind_comp (ef) each time hybrid is switched on
    wind_comp_ef.zero();

    // initialise offset angles
    wind_offset_roll = wind_offset_pitch = 0;

    // initialise wind offset computation and loiter-stab transition timer
    wind_offset_timer = 0;
    loiter_stab_timer = HYBRID_LOITER_STAB_TIMER;

    return true;
}

// hybrid_exit - restore position controller
static void hybrid_exit()
{
    pos_control.init_I = true;    // restore reset I for normal behaviour
}

// hybrid_run - runs the hybrid controller
// should be called at 100hz or more
static void hybrid_run()
{
    int16_t target_roll, target_pitch;  // pilot's roll and pitch angle inputs
    int16_t pilot_throttle_scaled = 0;  // pilot's throttle input
    float target_yaw_rate = 0;          // pilot desired yaw rate in centi-degrees/sec
    float target_climb_rate = 0;        // pilot desired climb rate in centimeters/sec
    float vel_fw, vel_right;            // vehicle's current velocity in body-frame forward and right directions
    const Vector3f& vel = inertial_nav.get_velocity();

    // if not auto armed set throttle to zero and exit immediately
    if(!ap.auto_armed || !inertial_nav.position_ok()) {
        wp_nav.init_loiter_target();
        attitude_control.init_targets();
        attitude_control.set_throttle_out(0, false);
        return;
    }

    // process pilot inputs
    if (!failsafe.radio) {
        // apply SIMPLE mode transform to pilot inputs
        update_simple_mode();

        // get pilot's desired yaw rate
        target_yaw_rate = get_pilot_desired_yaw_rate(g.rc_4.control_in);

        // get pilot desired climb rate (for alt-hold mode and take-off)
        target_climb_rate = get_pilot_desired_climb_rate(g.rc_3.control_in);

        // check for pilot requested take-off
        if (ap.land_complete && target_climb_rate > 0) {
            // indicate we are taking off
            set_land_complete(false);
            // clear i term when we're taking off
            set_throttle_takeoff();
        }
    }

    // if landed initialise loiter targets, set throttle to zero and exit
    if (ap.land_complete) {
        wp_nav.init_loiter_target();
        attitude_control.init_targets();
        attitude_control.set_throttle_out(0, false);
        return;
    }else{
        // convert pilot input to lean angles
        get_pilot_desired_lean_angles(g.rc_1.control_in, g.rc_2.control_in, target_roll, target_pitch);

        // convert inertial nav earth-frame velocities to body-frame
        // To-Do: move this to AP_Math (or perhaps we already have a function to do this)
        vel_fw = vel.x*ahrs.cos_yaw() + vel.y*ahrs.sin_yaw();
        vel_right = -vel.x*ahrs.sin_yaw() + vel.y*ahrs.cos_yaw();

        // get roll stick input
        if (target_roll != 0) {
            // roll stick input detected, set roll mode to pilot override
            hybrid.roll_mode = HYBRID_PILOT_OVERRIDE;

        } else {
            // roll stick is centered
            // if still in pilot override mode and breaking will complete within 0.5seconds switch to braking mode
            if ((hybrid.roll_mode == HYBRID_PILOT_OVERRIDE) && (abs(brake_roll) < 2*wp_nav._brake_rate)) {
                hybrid.roll_mode = HYBRID_BRAKE;    // Set brake roll mode
                brake_roll = 0;                     // this avoid false brake_timeout computing
                brake_timeout_roll = 600;           // seconds*0.01 - time allowed for the braking to complete, updated at half-braking
                brake_timeout_roll_updated = false; // Allow the timeout to be updated only once
                brake_roll_max = 0;                 // used to detect half braking
            } else {    // manage brake-to-loiter transition
                // brake timeout
                if (brake_timeout_roll > 0) {
                    brake_timeout_roll--;
                }
                // Changed loiter engage : not once HYBRID_SPEED_0 is reached but after a little delay that let the copter stabilize if it remains some rate. (maybe compare omega.x/y rather)
                if ((fabs(vel_right) < HYBRID_SPEED_0) && (brake_timeout_roll>50)) {
                    brake_timeout_roll = 50; // let 0.5s between brake reaches HYBRID_SPEED_0 and loiter engage
                }
                if ((hybrid.roll_mode == HYBRID_BRAKE) && (brake_timeout_roll==0)){	 //stick released and transition finished (speed 0) or brake timeout => loiter mode
                    hybrid.roll_mode = HYBRID_LOITER;   // Set loiter roll mode
                }
            }
        }

        // get pitch stick input
        if (target_pitch != 0) {
            // pitch stick input detected set pitch mode to pilot override
            hybrid.pitch_mode = HYBRID_PILOT_OVERRIDE; // Set stab pitch mode
        }else{
            if((hybrid.pitch_mode == HYBRID_PILOT_OVERRIDE) && (abs(brake_pitch) < 2*wp_nav._brake_rate)){	    // stick released from stab and copter horizontal (at wind_comp) => transition mode
                hybrid.pitch_mode = HYBRID_BRAKE;   // Set brake pitch mode
                brake_pitch = 0;                    // this avoid false brake_timeout computing
                brake_timeout_pitch = 600;          // seconds*0.01 - time allowed for the braking to complete, updated at half-braking
                brake_timeout_pitch_updated = false;// Allow the timeout to be updated only once
                brake_pitch_max = 0;                // used to detect half braking
            }else{                          // manage brake-to-loiter transition
                // brake timeout
                if (brake_timeout_pitch > 0) {
                    brake_timeout_pitch--;
                }
                // Changed loiter engage : not once HYBRID_SPEED_0 is reached but after a little delay that let the copter stabilize if it remains some rate. (maybe compare omega.x/y rather)
                if ((fabs(vel_fw) < HYBRID_SPEED_0) && (brake_timeout_pitch > 50)) {
                    brake_timeout_pitch = 50; // let 0.5s between brake reaches HYBRID_SPEED_0 and loiter engage
                }
                if ((hybrid.pitch_mode == HYBRID_BRAKE) && (brake_timeout_pitch == 0)) {
                    hybrid.pitch_mode = HYBRID_LOITER;  // Set loiter pitch mode
                }
            }
        }

        // manual roll with smooth decrease filter
        if (hybrid.roll_mode == HYBRID_PILOT_OVERRIDE) {
            if (((int32_t)brake_roll * (int32_t)target_roll >= 0) && (abs(target_roll) < HYBRID_STICK_RELEASE_SMOOTH_ANGLE)) {
                // Smooth decrease only when we want to stop, not if we have to quickly change direction
                if (brake_roll > 0){ // we use brake_roll to save mem usage and also because it will be natural transition with brake mode.
                    // rate decrease
                    brake_roll -= max((float)brake_roll * HYBRID_SMOOTH_RATE_FACTOR, wp_nav._brake_rate);
                    // use the max value if we increase and because we could have a smoother manual decrease than this computed value
                    brake_roll = max(brake_roll,target_roll);
                }else{
                    brake_roll += max(-(float)brake_roll * HYBRID_SMOOTH_RATE_FACTOR, wp_nav._brake_rate);
                    brake_roll = min(brake_roll,target_roll);
                }
            } else {
                brake_roll = target_roll;
            }
        }

        // manual pitch with smooth decrease filter
        if (hybrid.pitch_mode == HYBRID_PILOT_OVERRIDE) {
            if (((int32_t)brake_pitch * (int32_t)target_pitch >= 0) && (abs(target_pitch) < HYBRID_STICK_RELEASE_SMOOTH_ANGLE)) { //Smooth decrease only when we want to stop, not if we have to quickly change direction
                if (brake_pitch > 0) { // we use brake_pitch to save mem usage and also because it will be natural transition with brake mode.
                    brake_pitch -= max((float)brake_pitch * HYBRID_SMOOTH_RATE_FACTOR, wp_nav._brake_rate); //rate decrease
                    brake_pitch = max(brake_pitch,target_pitch); // use the max value because we could have a smoother manual decrease than this computed value
                } else {
                    brake_pitch += max(-(float)brake_pitch * HYBRID_SMOOTH_RATE_FACTOR, wp_nav._brake_rate);
                    brake_pitch = min(brake_pitch,target_pitch);
                }
            } else {
                brake_pitch = target_pitch;
            }
        }

        // braking update: roll
        if (hybrid.roll_mode >= HYBRID_BRAKE) {      // Roll: allow braking update to run also during loiter
            if (vel_right >= 0) {         // negative roll = go left, positive roll = go right
                brake_roll = max(brake_roll-wp_nav._brake_rate, max((-brake_gain*vel_right*(1.0f+500.0f/(vel_right+60.0f))),-wp_nav._max_braking_angle));
            }else{
                brake_roll = min(brake_roll+wp_nav._brake_rate, min((-brake_gain*vel_right*(1.0f+500.0f/(-vel_right+60.0f))),wp_nav._max_braking_angle));
            }
            if (abs(brake_roll) > brake_roll_max) { // detect half braking and update timeout
                brake_roll_max = abs(brake_roll);
            } else if (!brake_timeout_roll_updated) {
                brake_timeout_roll = 1+(uint16_t)(15L*(int32_t)(abs(brake_roll))/(10L*(int32_t)wp_nav._brake_rate));  // the 1.2 (12/10) factor has to be tuned in flight, here it means 120% of the "normal" time.
                brake_timeout_roll_updated = true;
            }
        }
        // braking update: pitch
        if (hybrid.pitch_mode >= HYBRID_BRAKE) {          // Pitch: allow braking update to run also during loiter
            if (vel_fw >= 0) {                  // positive pitch = go backward, negative pitch = go forward
                brake_pitch = min(brake_pitch+wp_nav._brake_rate,min((brake_gain*vel_fw*(1.0f+(500.0f/(vel_fw+60.0f)))),wp_nav._max_braking_angle));  // centidegrees
            } else {
                brake_pitch = max(brake_pitch-wp_nav._brake_rate,max((brake_gain*vel_fw*(1.0f-(500.0f/(vel_fw-60.0f)))),-wp_nav._max_braking_angle)); // centidegrees
            }
            if (abs(brake_pitch)>brake_pitch_max){	// detect half braking and update timeout
                brake_pitch_max = abs(brake_pitch);
            } else if (!brake_timeout_pitch_updated){
                // Changes 12 by 15 to let the brake=>loiter 0.5s happens before this timeout ends
                brake_timeout_pitch = 1+(int16_t)(15L*(int32_t)(abs(brake_pitch))/(10L*(int32_t)wp_nav._brake_rate));  // the 1.2 (12/10) factor has to be tuned in flight, here it means 120% of the "normal" time.
                brake_timeout_pitch_updated = true;
            }
        }
        // loiter to manual mix
        if ((hybrid.pitch_mode==HYBRID_PILOT_OVERRIDE)||(hybrid.roll_mode==HYBRID_PILOT_OVERRIDE)) {
            if (loiter_man_timer !=0 ) {
                loiter_man_mix = constrain_float((float)(loiter_man_timer)/(float)HYBRID_LOITER_MAN_MIX_TIMER, 0, 1.0);
                loiter_man_timer--;
            }
        }
        // loitering/moving:
        if (hybrid.pitch_mode == HYBRID_LOITER && hybrid.roll_mode == HYBRID_LOITER) {
            // while loitering, updates average lat/lon wind offset angles from I terms
            if (hybrid.loiter_engaged) {
                if (loiter_stab_timer != 0) {
                    loiter_stab_timer--;
                } else {
                    // update wind compensation estimate
                    hybrid_update_wind_comp_estimate();
                }
                // Brake_Loiter commands mix factor
                brake_loiter_mix = constrain_float((float)(HYBRID_LOITER_STAB_TIMER-loiter_stab_timer)/(float)HYBRID_BRAKE_LOITER_MIX_TIMER, 0, 1.0);
            } else {
                hybrid.loiter_engaged = true;   // turns on NAV_HYBRID if both sticks are at rest
                pos_control.init_I = false;     // stop I terms from being cleared when init_loiter_target is called.  avoids the stop_and_go effect
                wp_nav.init_loiter_target();    // init loiter controller and sets XY stopping point
                pos_control.set_target_to_stopping_point_z();   // init altitude
                loiter_stab_timer = HYBRID_LOITER_STAB_TIMER;   // starts a 3 seconds timer
                brake_roll = 1;             // required for next mode_1 smooth stick release and to avoid twitch
                brake_pitch = 1;            // required for next mode_1 smooth stick release and to avoid twitch
            }
        } else {
            // transition from Loiter to Manual
            if (hybrid.loiter_engaged) {
                hybrid.loiter_engaged = false;
                loiter_man_timer = HYBRID_LOITER_MAN_MIX_TIMER;
                // save pitch/roll at loiter exit
                loiter_roll = wp_nav.get_roll();
                loiter_pitch = wp_nav.get_pitch();
            }
            if (wind_offset_timer == 0) {	// reduce update frequency of wind_offset to 10Hz
                // retrieve latest wind compensation lean angles
                hybrid_get_wind_comp_lean_angles(wind_offset_roll, wind_offset_pitch);
                wind_offset_timer = 10;
            } else {
                wind_offset_timer--;
            }
        }

        // if required, update loiter controller
        if(hybrid.loiter_engaged) {
            wp_nav.update_loiter();
        }
        // select output to stabilize controllers
        switch (hybrid.roll_mode) {
            // To-Do: try to mix loiter->manual using brake_loiter_mix variable as we are doing on loiter engage
            case HYBRID_PILOT_OVERRIDE:
                // Loiter-Manual mix at loiter exit
                target_roll = loiter_man_mix*(float)loiter_roll+(1.0f-loiter_man_mix)*(float)(brake_roll+wind_offset_roll);
                break;

            case HYBRID_BRAKE:
                target_roll = brake_roll + wind_offset_roll;
                break;

            case HYBRID_BRAKE_TO_LOITER:
                break;

            case HYBRID_LOITER:
                if (hybrid.loiter_engaged) {
                    // Brake_Loiter mix at loiter engage
                    target_roll = brake_loiter_mix*(float)wp_nav.get_roll()+(1.0f-brake_loiter_mix)*(float)(brake_roll+wind_offset_roll);
                }else {
                    target_roll = brake_roll + wind_offset_roll;
                }
                break;

            case HYBRID_LOITER_TO_PILOT_OVERRIDE:
                // Loiter-Manual mix at loiter exit
                target_roll = loiter_man_mix*(float)loiter_roll+(1.0f-loiter_man_mix)*(float)(brake_roll+wind_offset_roll);
                // To-Do: add handling of exiting this mode far far above
                break;
        }

        switch (hybrid.pitch_mode){
            case HYBRID_PILOT_OVERRIDE:
                //Loiter-Manual mix at loiter exit
                target_pitch = loiter_man_mix*(float)loiter_pitch+(1.0f-loiter_man_mix)*(float)(brake_pitch+wind_offset_pitch);
                break;

            case HYBRID_BRAKE:
                target_pitch = brake_pitch+wind_offset_pitch;
                break;

            case HYBRID_BRAKE_TO_LOITER:
                break;

            case HYBRID_LOITER:
                if(hybrid.loiter_engaged) {
                    // mix brake and loiter outputs
                    target_pitch = brake_loiter_mix*(float)wp_nav.get_pitch()+(1.0f-brake_loiter_mix)*(float)(brake_pitch+wind_offset_pitch);
                } else {
                    // only wind-compensate pitch (roll is likely under manual control)
                    target_pitch = brake_pitch+wind_offset_pitch;
                }
                break;

            case HYBRID_LOITER_TO_PILOT_OVERRIDE:
                // mix brake and loiter outputs
                target_pitch = brake_loiter_mix*(float)wp_nav.get_pitch()+(1.0f-brake_loiter_mix)*(float)(brake_pitch+wind_offset_pitch);
                // To-Do: add handling of exiting this mode far far above
                break;
        }

        // constrain target pitch/roll angles
        target_roll = constrain_int16(target_roll,-aparm.angle_max,aparm.angle_max);
        target_pitch = constrain_int16(target_pitch,-aparm.angle_max,aparm.angle_max);

        // update attitude controller targets
        attitude_control.angle_ef_roll_pitch_rate_ef_yaw(target_roll, target_pitch, target_yaw_rate);

        // throttle control
        if (sonar_alt_health >= SONAR_ALT_HEALTH_MAX) {
            // if sonar is ok, use surface tracking
            target_climb_rate = get_throttle_surface_tracking(target_climb_rate, pos_control.get_alt_target(), G_Dt);
        }
        // update altitude target and call position controller
        pos_control.set_alt_target_from_climb_rate(target_climb_rate, G_Dt);
        pos_control.update_z_controller();
    }
}

// hybrid_update_wind_comp_estimate - updates wind compensation estimate
//  should be called at the maximum loop rate when loiter is engaged
//  To-Do: adjust the filtering for 100hz and 400hz update rates
static void hybrid_update_wind_comp_estimate()
{
    if (inertial_nav.get_velocity_xy() > HYBRID_WIND_COMP_ESTIMATE_SPEED_MAX) {
        return;
    }

    // update wind compensation in earth-frame lean angles
    if (wind_comp_ef.x == 0) {
        // if wind compensation has not been initialised set it immediately to the pos controller's desired accel in north direction
        wind_comp_ef.x = pos_control.get_desired_acc_x();
    } else {
        // low pass filter the position controller's lean angle output
        wind_comp_ef.x = (0.97f*wind_comp_ef.x+0.03f*pos_control.get_desired_acc_x());
    }
    if (wind_comp_ef.y == 0) {
        // if wind compensation has not been initialised set it immediately to the pos controller's desired accel in north direction
        wind_comp_ef.y = pos_control.get_desired_acc_y();
    } else {
        // low pass filter the position controller's lean angle output
        wind_comp_ef.y = (0.97f*wind_comp_ef.y+0.03f*pos_control.get_desired_acc_y());
    }
}

// hybrid_get_wind_comp_lean_angles - retrieve wind compensation angles in body frame roll and pitch angles
static void hybrid_get_wind_comp_lean_angles(int16_t &roll_angle, int16_t &pitch_angle)
{
    // convert earth frame desired accelerations to body frame roll and pitch lean angles
    roll_angle = (float)fast_atan((-wind_comp_ef.x*ahrs.sin_yaw() + wind_comp_ef.y*ahrs.cos_yaw())/981)*(18000/M_PI);
    pitch_angle = (float)fast_atan(-(wind_comp_ef.x*ahrs.cos_yaw() + wind_comp_ef.y*ahrs.sin_yaw())/981)*(18000/M_PI);
}
