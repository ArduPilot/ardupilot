/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/*
 * control_hybrid.pde - init and run calls for hybrid flight mode
 *     hybrid tries to improve upon regular loiter by mixing the pilot input with the loiter controller
 */

#define SPEED_0 10
#define LOITER_STAB_TIMER               300         // ST-JD : Must be higher than BRAKE_LOIT_MIX_TIMER (twice is a good deal) set it from 100 to 500, the number of centiseconds between loiter engage and getting wind_comp (once loiter stabilized)
#define BRAKE_LOIT_MIX_TIMER            150         // ST-JD : Must be lower than LOITER_STAB_TIMER set it from 100 to 200, the number of centiseconds brake and loiter commands are mixed to make a smooth transition.
#define LOITER_MAN_MIX_TIMER			50			// ST-JD : set it from 100 to 200, the number of centiseconds loiter and manual commands are mixed to make a smooth transition.
#define HYBRID_THROTTLE_FACTOR 			1.3f     	// Need param? Used to define the min and max throttle from the throttle_cruise in hybrid mode. Should be between 1,1 (smooth) and 1,5 (strong)
#define THROTTLE_HYBRID_MAN 0
#define THROTTLE_HYBRID_AH 1
#define THROTTLE_HYBRID_BK 3

// mission state enumeration
enum hybrid_rp_mode {
    HYBRID_PILOT_OVERRIDE=0,
    HYBRID_BRAKE=1,
    HYBRID_LOITER=2
};

static struct {
    hybrid_rp_mode roll_mode    : 2;    // roll mode: pilot override, brake or loiter
    hybrid_rp_mode pitch_mode   : 2;    // pitch mode: pilot override, brake or loiter
    uint8_t loiter_engaged      : 1;    // 1 if loiter target has been set and loiter controller is running
} hybrid;

static int16_t brake_roll = 0,brake_pitch = 0;
static float K_brake;
static uint8_t throttle_mode=THROTTLE_HYBRID_MAN;

static float wind_comp_x, wind_comp_y;// ST-JD : wind compensation vector, averaged I terms from loiter controller
static int16_t wind_offset_roll,wind_offset_pitch;	// ST-JD : wind offsets for pitch/roll
static int16_t timeout_roll, timeout_pitch; 	// seconds - time allowed for the braking to complete, this timeout will be updated at half-braking
static int16_t loiter_stab_timer;		// loiter stabilization timer: we read pid's I terms in wind_comp only after this time from loiter start

static bool timeout_roll_updated, timeout_pitch_updated;    // Allow the timeout to be updated only once per braking.
static int16_t brake_max_roll, brake_max_pitch;             // used to detect half braking
static int16_t loiter_roll,loiter_pitch;                    // store pitch/roll at loiter exit
static float brake_loiter_mix;                              // varies from 0 to 1, allows a smooth loiter engage
static float loiter_man_mix;                                // varies from 0 to 1, allow a smooth loiter to manual transition
static int16_t loiter_man_timer;
static int8_t  update_wind_offset_timer;                    // update wind_offset decimator (10Hz)

// hybrid_init - initialise hybrid controller
static bool hybrid_init(bool ignore_checks)
{
/*
    if (GPS_ok() || ignore_checks) {

        // set target to current position
        wp_nav.init_loiter_target();

        // clear pilot's feed forward for roll and pitch
        wp_nav.set_pilot_desired_acceleration(0, 0);

        // initialise altitude target to stopping point
        pos_control.set_target_to_stopping_point_z();

        // compute K_brake
        K_brake=(15.0f*(float)wp_nav._brake_rate+95.0f)/100.0f;

        if (ap.land_complete) {
            // Loiter start
            hybrid.roll_mode = HYBRID_LOITER;
            hybrid.pitch_mode = HYBRID_LOITER;
        }else{
            // Alt_hold like to avoid hard twitch if hybrid enabled in flight
            hybrid.roll_mode = HYBRID_PILOT_OVERRIDE;
            hybrid.pitch_mode = HYBRID_PILOT_OVERRIDE;
        }

        wind_comp_x=wind_comp_y=0;          // Init wind_comp (ef). For now, resetted each time hybrid is switched on
        wind_offset_roll=0;                 // Init offset angles
        wind_offset_pitch=0;
        update_wind_offset_timer=0;         // Init wind offset computation timer
        loiter_stab_timer=LOITER_STAB_TIMER;
        return true;
    }else{
        return false;
    }
*/
return true;
}

// hybrid_exit - restore position controller
static void hybrid_exit()
{
/*
    pos_control.init_I=true;    // restore reset I for normal behaviour
*/
}

// hybrid_run - runs the hybrid controller
// should be called at 100hz or more
static void hybrid_run()
{
/*
    float target_yaw_rate = 0;
    float target_climb_rate = 0;
    const Vector3f& vel = inertial_nav.get_velocity();
    float vel_fw, vel_right;

    int16_t target_roll, target_pitch;
    int16_t pilot_throttle_scaled=0;

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

        // calculate earth-frame velocities
        vel_fw = vel.x*ahrs.cos_yaw() + vel.y*ahrs.sin_yaw();
        vel_right = -vel.x*ahrs.sin_yaw() + vel.y*ahrs.cos_yaw();

        // get roll stick input
        if (target_roll != 0) {
            // roll stick input detected set roll mode to pilot override
            hybrid.roll_mode = HYBRID_PILOT_OVERRIDE;    // Set stab roll mode

        } else {
            // stick released from stab and copter horizontal (at wind comp) => transition mode
            if ((hybrid.roll_mode == HYBRID_PILOT_OVERRIDE) && (abs(brake_roll) < 2*wp_nav._brake_rate)) {
                hybrid.roll_mode = HYBRID_BRAKE;   // Set brake roll mode
                brake_roll = 0;             // this avoid false brake_timeout computing
                timeout_roll = 600; 		// seconds*0.01 - time allowed for the braking to complete, updated at half-braking
                timeout_roll_updated = false;   // Allow the timeout to be updated only once
                brake_max_roll = 0; 		// used to detect half braking
            } else {                          // manage brake-to-loiter transition
                // brake timeout
                if (timeout_roll > 0) {
                    timeout_roll--;
                }
                // Changed loiter engage : not once speed_0 reached but after a little delay that let the copter stabilize if it remains some rate. (maybe compare omega.x/y rather)
                if ((fabs(vel_right) < SPEED_0) && (timeout_roll>50)) {
                    timeout_roll = 50; // let 0.5s between brake reaches speed_0 and loiter engage
                }
                if ((hybrid.roll_mode == HYBRID_BRAKE) && (timeout_roll==0)){	 //stick released and transition finished (speed 0) or brake timeout => loiter mode
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
                hybrid.pitch_mode = HYBRID_BRAKE;      // Set brake pitch mode
                brake_pitch = 0;             // this avoid false brake_timeout computing
                timeout_pitch=600;		    // seconds*0.01 - time allowed for the braking to complete, updated at half-braking
                timeout_pitch_updated = false;   // Allow the timeout to be updated only once
                brake_max_pitch=0; 		    // used to detect half braking
            }else{                          // manage brake-to-loiter transition
                // brake timeout
                if (timeout_pitch>0) timeout_pitch--;
                // Changed loiter engage : not once speed_0 reached but after a little delay that let the copter stabilize if it remains some rate. (maybe compare omega.x/y rather)
                if ((fabs(vel_fw)<SPEED_0) && (timeout_pitch>50)) {
                    timeout_pitch = 50; // let 0.5s between brake reaches speed_0 and loiter engage
                }
                if ((hybrid.pitch_mode == HYBRID_BRAKE) && (timeout_pitch == 0)) {
                    hybrid.pitch_mode = HYBRID_LOITER;  // Set loiter pitch mode
                }
            }
        }

        // manual roll/pitch with smooth decrease filter
        // roll
        if (hybrid.roll_mode == HYBRID_PILOT_OVERRIDE) {
            if (((long)brake_roll*(long)target_roll>=0) && (abs(target_roll)<STICK_RELEASE_SMOOTH_ANGLE)) {
                // Smooth decrease only when we want to stop, not if we have to quickly change direction
                if (brake_roll > 0){ // we use brake_roll to save mem usage and also because it will be natural transition with brake mode.
                    // rate decrease
                    brake_roll -= max((float)brake_roll*(float)wp_nav._smooth_rate_factor/100,wp_nav._brake_rate);
                    // use the max value if we increase and because we could have a smoother manual decrease than this computed value
                    brake_roll = max(brake_roll,target_roll);
                }else{
                    brake_roll += max(-(float)brake_roll*(float)wp_nav._smooth_rate_factor/100,wp_nav._brake_rate);
                    brake_roll = min(brake_roll,target_roll);
                }
            } else {
                brake_roll=target_roll;
            }
        }

        // pitch
        if (hybrid.pitch_mode == HYBRID_PILOT_OVERRIDE) {
            if (((long)brake_pitch*(long)target_pitch>=0)&&(abs(target_pitch)<STICK_RELEASE_SMOOTH_ANGLE)){ //Smooth decrease only when we want to stop, not if we have to quickly change direction
                if (brake_pitch>0){ // we use brake_pitch to save mem usage and also because it will be natural transition with brake mode.
                    brake_pitch-=max((float)brake_pitch*(float)wp_nav._smooth_rate_factor/100,wp_nav._brake_rate); //rate decrease
                    brake_pitch=max(brake_pitch,target_pitch); // use the max value because we could have a smoother manual decrease than this computed value
                } else {
                    brake_pitch += max(-(float)brake_pitch*(float)wp_nav._smooth_rate_factor/100,wp_nav._brake_rate);
                    brake_pitch = min(brake_pitch,target_pitch);
                }
            } else {
                brake_pitch=target_pitch;
            }
        }
        // braking update: roll
        if (hybrid.roll_mode >= HYBRID_BRAKE) {      // Roll: allow braking update to run also during loiter
            if (vel_right >= 0) {         // negative roll = go left, positive roll = go right
                brake_roll = max(brake_roll-wp_nav._brake_rate, max((-K_brake*vel_right*(1.0f+500.0f/(vel_right+60.0f))),-wp_nav._max_braking_angle)); // centidegrees
            }else{
                brake_roll = min(brake_roll+wp_nav._brake_rate, min((-K_brake*vel_right*(1.0f+500.0f/(-vel_right+60.0f))),wp_nav._max_braking_angle));   // centidegrees
            }
            if (abs(brake_roll) > brake_max_roll) { // detect half braking and update timeout
                brake_max_roll = abs(brake_roll);
            } else if (!timeout_roll_updated){
                timeout_roll = 1+(uint16_t)(15L*(long)(abs(brake_roll))/(10L*(long)wp_nav._brake_rate));  // the 1.2 (12/10) factor has to be tuned in flight, here it means 120% of the "normal" time.
                timeout_roll_updated = true;
            }
        }
        // braking update: pitch
        if (hybrid.pitch_mode>=HYBRID_BRAKE) {          // Pitch: allow braking update to run also during loiter
            if (vel_fw>=0) {                  // positive pitch = go backward, negative pitch = go forward
                brake_pitch = min(brake_pitch+wp_nav._brake_rate,min((K_brake*vel_fw*(1.0f+(500.0f/(vel_fw+60.0f)))),wp_nav._max_braking_angle));  // centidegrees
            } else {
                brake_pitch = max(brake_pitch-wp_nav._brake_rate,max((K_brake*vel_fw*(1.0f-(500.0f/(vel_fw-60.0f)))),-wp_nav._max_braking_angle)); // centidegrees
            }
            if (abs(brake_pitch)>brake_max_pitch){	// detect half braking and update timeout
                brake_max_pitch=abs(brake_pitch);
            } else if (!timeout_pitch_updated){
                // Changes 12 by 15 to let the brake=>loiter 0.5s happens before this timeout ends
                timeout_pitch = 1+(int16_t)(15L*(long)(abs(brake_pitch))/(10L*(long)wp_nav._brake_rate));  // the 1.2 (12/10) factor has to be tuned in flight, here it means 120% of the "normal" time.
                timeout_pitch_updated = true;
            }
        }
        // loiter to manual mix
        if ((hybrid.pitch_mode==HYBRID_PILOT_OVERRIDE)||(hybrid.roll_mode==HYBRID_PILOT_OVERRIDE)) {
            if (!ap.land_complete && loiter_man_timer!=0) {
                loiter_man_mix = constrain_float((float)(loiter_man_timer)/(float)LOITER_MAN_MIX_TIMER, 0, 1.0);//constrain_float((float)(LOITER_MAN_MIX_TIMER-loiter_man_timer)/(float)LOITER_MAN_MIX_TIMER, 0, 1.0);
                loiter_man_timer--;
            }
        }
        // loitering/moving:
        if (hybrid.pitch_mode == HYBRID_LOITER && hybrid.roll_mode == HYBRID_LOITER) {
            // while loitering, updates average lat/lon wind offset angles from I terms
            if (hybrid.loiter_engaged) {
                if (!ap.land_complete && loiter_stab_timer!=0) {
                    loiter_stab_timer--;
                } else if (max(fabs(vel.x),fabs(vel.y))<SPEED_0) { //Or maybe 2*, 3* speed_0...
                    if (wind_comp_x==0) wind_comp_x=pos_control.get_desired_acc_x(); else wind_comp_x=(0.97f*wind_comp_x+0.03f*pos_control.get_desired_acc_x());
                    if (wind_comp_y==0) wind_comp_y=pos_control.get_desired_acc_y(); else wind_comp_y=(0.97f*wind_comp_y+0.03f*pos_control.get_desired_acc_y());
                }
                // Brake_Loiter commands mix factor
                brake_loiter_mix = constrain_float((float)(LOITER_STAB_TIMER-loiter_stab_timer)/(float)BRAKE_LOIT_MIX_TIMER, 0, 1.0);
            } else {
                hybrid.loiter_engaged = true;    // turns on NAV_HYBRID if both sticks are at rest
                pos_control.init_I=false;    // restore previous i_terms in Reset_I() => to avoid the stop_and_go effect
                wp_nav.init_loiter_target(); // init loiter controller and sets XY stopping point
                pos_control.set_target_to_stopping_point_z();	// init altitude
                loiter_stab_timer=LOITER_STAB_TIMER;      // starts a 3 seconds timer
                brake_roll = 1;             // required for next mode_1 smooth stick release and to avoid twitch
                brake_pitch = 1;            // required for next mode_1 smooth stick release and to avoid twitch
            }
        } else {
            // transition from Loiter to Manual
            if (hybrid.loiter_engaged) {
                hybrid.loiter_engaged = false;
                loiter_man_timer=LOITER_MAN_MIX_TIMER;
                // save pitch/roll at loiter exit
                loiter_roll=wp_nav.get_roll();
                loiter_pitch=wp_nav.get_pitch();
            }
            if (update_wind_offset_timer==0) {	// reduce update frequency of wind_offset to 10Hz
                // compute wind_offset_roll/pitch frame referred lon/lat_i_term and yaw rotated
                // acceleration to angle
                wind_offset_pitch = (float)fast_atan(-(wind_comp_x*ahrs.cos_yaw() + wind_comp_y*ahrs.sin_yaw())/981)*(18000/M_PI);
                wind_offset_roll = (float)fast_atan((-wind_comp_x*ahrs.sin_yaw() + wind_comp_y*ahrs.cos_yaw())/981)*(18000/M_PI);
                update_wind_offset_timer=10;
            } else {
                update_wind_offset_timer--;
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
            case HYBRID_LOITER:
                if (hybrid.loiter_engaged) { // if nav_hybrid enabled...
                    // Brake_Loiter mix at loiter engage
                    target_roll = brake_loiter_mix*(float)wp_nav.get_roll()+(1.0f-brake_loiter_mix)*(float)(brake_roll+wind_offset_roll);
                }else {
                    target_roll = brake_roll + wind_offset_roll;
                }
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
            case HYBRID_LOITER:
                if(hybrid.loiter_engaged) { // if nav_hybrid enabled...
                    // Brake_Loiter mix at loiter engage
                    target_pitch = brake_loiter_mix*(float)wp_nav.get_pitch()+(1.0f-brake_loiter_mix)*(float)(brake_pitch+wind_offset_pitch);
                } else {
                    target_pitch = brake_pitch+wind_offset_pitch;
                }
                break;
        }

        // constrain target pitch/roll angles
        target_roll=constrain_int16(target_roll,-aparm.angle_max,aparm.angle_max);
        target_pitch=constrain_int16(target_pitch,-aparm.angle_max,aparm.angle_max);

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
*/
}
