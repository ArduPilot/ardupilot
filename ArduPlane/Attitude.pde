// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

//****************************************************************
// Function that controls aileron/rudder, elevator, rudder (if 4 channel control) and throttle to produce desired attitude and airspeed.
//****************************************************************


/*
  get a speed scaling number for control surfaces. This is applied to
  PIDs to change the scaling of the PID with speed. At high speed we
  move the surfaces less, and at low speeds we move them more.
 */
static float get_speed_scaler(void)
{
    float aspeed, speed_scaler;
    if (ahrs.airspeed_estimate(&aspeed)) {
        if (aspeed > 0) {
            speed_scaler = g.scaling_speed / aspeed;
        } else {
            speed_scaler = 2.0;
        }
        speed_scaler = constrain(speed_scaler, 0.5, 2.0);
    } else {
        if (g.channel_throttle.servo_out > 0) {
            speed_scaler = 0.5 + ((float)THROTTLE_CRUISE / g.channel_throttle.servo_out / 2.0);                 // First order taylor expansion of square root
            // Should maybe be to the 2/7 power, but we aren't goint to implement that...
        }else{
            speed_scaler = 1.67;
        }
        // This case is constrained tighter as we don't have real speed info
        speed_scaler = constrain(speed_scaler, 0.6, 1.67);
    }
    return speed_scaler;
}

/*
  return true if the current settings and mode should allow for stick mixing
 */
static bool stick_mixing_enabled(void)
{
    if (control_mode == CIRCLE || control_mode > FLY_BY_WIRE_B) {
        // we're in an auto mode. Check the stick mixing flag
        if (g.stick_mixing &&
            geofence_stickmixing() &&
            failsafe == FAILSAFE_NONE) {
            // we're in an auto mode, and haven't triggered failsafe
            return true;
        } else {
            return false;
        }
    }
    // non-auto mode. Always do stick mixing
    return true;
}


/*
  this is the main roll stabilization function. It takes the
  previously set nav_roll calculates roll servo_out to try to
  stabilize the plane at the given roll
 */
static void stabilize_roll(float speed_scaler)
{
    if (crash_timer > 0) {
        nav_roll_cd = 0;
    }

    if (inverted_flight) {
        // we want to fly upside down. We need to cope with wrap of
        // the roll_sensor interfering with wrap of nav_roll, which
        // would really confuse the PID code. The easiest way to
        // handle this is to ensure both go in the same direction from
        // zero
        nav_roll_cd += 18000;
        if (ahrs.roll_sensor < 0) nav_roll_cd -= 36000;
    }

#if APM_CONTROL == DISABLED
	// Calculate dersired servo output for the roll
	// ---------------------------------------------
    g.channel_roll.servo_out = g.pidServoRoll.get_pid((nav_roll_cd - ahrs.roll_sensor), speed_scaler);
#else // APM_CONTROL == ENABLED
    // calculate roll and pitch control using new APM_Control library
    g.channel_roll.servo_out = g.rollController.get_servo_out(nav_roll_cd, speed_scaler, control_mode == STABILIZE);
#endif
}

/*
  this is the main pitch stabilization function. It takes the
  previously set nav_pitch and calculates servo_out values to try to
  stabilize the plane at the given attitude.
 */
static void stabilize_pitch(float speed_scaler)
{
#if APM_CONTROL == DISABLED
    int32_t tempcalc = nav_pitch_cd +
        fabs(ahrs.roll_sensor * g.kff_pitch_compensation) +
        (g.channel_throttle.servo_out * g.kff_throttle_to_pitch) -
        (ahrs.pitch_sensor - g.pitch_trim_cd);
    if (inverted_flight) {
        // when flying upside down the elevator control is inverted
        tempcalc = -tempcalc;
    }
    g.channel_pitch.servo_out = g.pidServoPitch.get_pid(tempcalc, speed_scaler);
#else // APM_CONTROL == ENABLED
    g.channel_pitch.servo_out = g.pitchController.get_servo_out(nav_pitch_cd, speed_scaler, control_mode == STABILIZE);    
#endif
}

/*
  this gives the user control of the aircraft in stabilization modes
 */
static void stabilize_stick_mixing()
{
    if (!stick_mixing_enabled() ||
        control_mode == FLY_BY_WIRE_A ||
        control_mode == FLY_BY_WIRE_B ||
        control_mode == TRAINING) {
        return;
    }
    // do stick mixing on aileron/elevator
    float ch1_inf;
    float ch2_inf;
        
    ch1_inf = (float)g.channel_roll.radio_in - (float)g.channel_roll.radio_trim;
    ch1_inf = fabs(ch1_inf);
    ch1_inf = min(ch1_inf, 400.0);
    ch1_inf = ((400.0 - ch1_inf) /400.0);
        
    ch2_inf = (float)g.channel_pitch.radio_in - g.channel_pitch.radio_trim;
    ch2_inf = fabs(ch2_inf);
    ch2_inf = min(ch2_inf, 400.0);
    ch2_inf = ((400.0 - ch2_inf) /400.0);
        
    // scale the sensor input based on the stick input
    // -----------------------------------------------
    g.channel_roll.servo_out  *= ch1_inf;
    g.channel_pitch.servo_out *= ch2_inf;
            
    // Mix in stick inputs
    // -------------------
    g.channel_roll.servo_out  +=     g.channel_roll.pwm_to_angle();
    g.channel_pitch.servo_out +=    g.channel_pitch.pwm_to_angle();
}


/*
  stabilize the yaw axis
 */
static void stabilize_yaw(float speed_scaler)
{
    float ch4_inf = 1.0;

    if (stick_mixing_enabled()) {
        // stick mixing performed for rudder for all cases including FBW
        // important for steering on the ground during landing
        // -----------------------------------------------
        ch4_inf = (float)g.channel_rudder.radio_in - (float)g.channel_rudder.radio_trim;
        ch4_inf = fabs(ch4_inf);
        ch4_inf = min(ch4_inf, 400.0);
        ch4_inf = ((400.0 - ch4_inf) /400.0);
    }

    // Apply output to Rudder
    calc_nav_yaw(speed_scaler, ch4_inf);
    g.channel_rudder.servo_out *= ch4_inf;
    g.channel_rudder.servo_out += g.channel_rudder.pwm_to_angle();
}


/*
  a special stabilization function for training mode
 */
static void stabilize_training(float speed_scaler)
{
    if (training_manual_roll) {
        g.channel_roll.servo_out = g.channel_roll.control_in;
    } else {
        // calculate what is needed to hold
        stabilize_roll(speed_scaler);
        if ((nav_roll_cd > 0 && g.channel_roll.control_in < g.channel_roll.servo_out) ||
            (nav_roll_cd < 0 && g.channel_roll.control_in > g.channel_roll.servo_out)) {
            // allow user to get out of the roll
            g.channel_roll.servo_out = g.channel_roll.control_in;            
        }
    }

    if (training_manual_pitch) {
        g.channel_pitch.servo_out = g.channel_pitch.control_in;
    } else {
        stabilize_pitch(speed_scaler);
        if ((nav_pitch_cd > 0 && g.channel_pitch.control_in < g.channel_pitch.servo_out) ||
            (nav_pitch_cd < 0 && g.channel_pitch.control_in > g.channel_pitch.servo_out)) {
            // allow user to get back to level
            g.channel_pitch.servo_out = g.channel_pitch.control_in;            
        }
    }

    stabilize_stick_mixing();
    stabilize_yaw(speed_scaler);
}


/*
  main stabilization function for all 3 axes
 */
static void stabilize()
{
    float speed_scaler = get_speed_scaler();

    if (control_mode == TRAINING) {
        stabilize_training(speed_scaler);
    } else {
        stabilize_roll(speed_scaler);
        stabilize_pitch(speed_scaler);
        stabilize_stick_mixing();
        stabilize_yaw(speed_scaler);
    }
}


static void crash_checker()
{
    if(ahrs.pitch_sensor < -4500) {
        crash_timer = 255;
    }
    if(crash_timer > 0)
        crash_timer--;
}


static void calc_throttle()
{
    if (!alt_control_airspeed()) {
        int16_t throttle_target = g.throttle_cruise + throttle_nudge;

        // TODO: think up an elegant way to bump throttle when
        // groundspeed_undershoot > 0 in the no airspeed sensor case; PID
        // control?

        // no airspeed sensor, we use nav pitch to determine the proper throttle output
        // AUTO, RTL, etc
        // ---------------------------------------------------------------------------
        if (nav_pitch_cd >= 0) {
            g.channel_throttle.servo_out = throttle_target + (g.throttle_max - throttle_target) * nav_pitch_cd / g.pitch_limit_max_cd;
        } else {
            g.channel_throttle.servo_out = throttle_target - (throttle_target - g.throttle_min) * nav_pitch_cd / g.pitch_limit_min_cd;
        }

        g.channel_throttle.servo_out = constrain_int16(g.channel_throttle.servo_out, g.throttle_min.get(), g.throttle_max.get());
    } else {
        // throttle control with airspeed compensation
        // -------------------------------------------
        energy_error = airspeed_energy_error + altitude_error_cm * 0.098f;

        // positive energy errors make the throttle go higher
        g.channel_throttle.servo_out = g.throttle_cruise + g.pidTeThrottle.get_pid(energy_error);
        g.channel_throttle.servo_out += (g.channel_pitch.servo_out * g.kff_pitch_to_throttle);

        g.channel_throttle.servo_out = constrain_int16(g.channel_throttle.servo_out,
                                                       g.throttle_min.get(), g.throttle_max.get());
    }

}

/*****************************************
* Calculate desired roll/pitch/yaw angles (in medium freq loop)
*****************************************/

//  Yaw is separated into a function for heading hold on rolling take-off
// ----------------------------------------------------------------------
static void calc_nav_yaw(float speed_scaler, float ch4_inf)
{
    if (hold_course != -1) {
        // steering on or close to ground
        g.channel_rudder.servo_out = g.pidWheelSteer.get_pid(bearing_error_cd, speed_scaler) + 
            g.kff_rudder_mix * g.channel_roll.servo_out;
        return;
    }

#if APM_CONTROL == DISABLED
    // always do rudder mixing from roll
    g.channel_rudder.servo_out = g.kff_rudder_mix * g.channel_roll.servo_out;

    // a PID to coordinate the turn (drive y axis accel to zero)
    Vector3f temp = ins.get_accel();
    int32_t error = -temp.y*100.0;

    g.channel_rudder.servo_out += g.pidServoRudder.get_pid(error, speed_scaler);
#else // APM_CONTROL == ENABLED
    // use the new APM_Control library
	g.channel_rudder.servo_out = g.yawController.get_servo_out(speed_scaler, ch4_inf < 0.25) + g.channel_roll.servo_out * g.kff_rudder_mix;
#endif
}


static void calc_nav_pitch()
{
    // Calculate the Pitch of the plane
    // --------------------------------
    if (alt_control_airspeed()) {
        nav_pitch_cd = -g.pidNavPitchAirspeed.get_pid(airspeed_error_cm);
    } else {
        nav_pitch_cd = g.pidNavPitchAltitude.get_pid(altitude_error_cm);
    }
    nav_pitch_cd = constrain_int32(nav_pitch_cd, g.pitch_limit_min_cd.get(), g.pitch_limit_max_cd.get());
}
static void calc_nav_roll()
{  
    static int32_t raw_nav_roll;
    static int32_t raw_nav_roll_fixed; 
    static int32_t raw_nav_roll_pid;    
    static int16_t count_a;
    static int16_t count_b;
    float air_speed = airspeed.get_airspeed();  
    //if (ahrs.airspeed_estimate(&aspeed)) {
    //   air_speed = aspeed * 100;
    //}
    static float nav_bender = 0;
    //static float turn_radius;
    static int32_t reverse_turn_cd = 0;
    static int8_t reverse = 1;
    static float loiter_error;
#if DEBUG_NAV == ENABLED    // For debug printing only      
    static bool once_a = 1;
    static bool once_b = 1; 
#endif           
    if(nav_wp_sw || loiter_trig){
#if DEBUG_NAV == ENABLED      
        once_b = true;
#endif        
        //turn_radius = float(g.waypoint_radius);
        if(prev_control_mode != control_mode) turn_around = false;
        // Normal Left turn
        if((bearing_error_cd < -g.nav_pid_angle_cd || turn_step > 1) && lock != 1 && !loiter_trig){  
            switch(turn_step){
                case 1:
                    lock = -1;                             
                    if(turn_around){                  
                        reverse_turn_cd = wrap_360_cd(last_wp_bearing_cd + g.turn_back_angle_cd - g.turn_back_angle_comp_cd);                                  
                        reverse = -1; // Do the oposite of the normal
                        turn_step ++;
#if DEBUG_NAV == ENABLED                        
                        hal.uartC->printf("***acute angle trig L****  next hdg : %d\n", (reverse_turn_cd*.01));
#endif                        
                    }
                break;             
                case 2:                  
                    if(wrap_360_cd(reverse_turn_cd - calc_bearing_cd) < 500){
                        reverse_turn_cd = wrap_360_cd(current_wp_bearing_cd - g.turn_back_angle_cd + g.turn_back_angle_comp_cd);                         
                        reverse = 1;  //Return to normal turn
                        turn_step ++;
#if DEBUG_NAV == ENABLED                        
                        hal.uartC->printf("*** turn around *** next hdg : %d\n", (reverse_turn_cd*.01));
#endif                        
                    }                                                                                          
                break;
                case 3:
                    if(wrap_360_cd(calc_bearing_cd - reverse_turn_cd) < 500){                        
                        reverse = -1;
                        turn_step ++;
#if DEBUG_NAV == ENABLED                        
                        hal.uartC->printf("*** turn back ***\n");
#endif                        
                    }
                break;
                case 4:
                    if(bearing_error_cd < g.nav_pid_angle_cd){                    
                        turn_around = false;
                        turn_step = 1;                                                          
                    }
                break;
                default:                                                                                   
                    lock = -1;
                break;
             }                      
        } 
        //Normal Right turn
        else if((bearing_error_cd > g.nav_pid_angle_cd || turn_step > 1) && lock != -1 && !loiter_trig){   
            switch(turn_step){
                case 1:
                    lock = 1;
                    if(turn_around){
                        turn_step ++;
                        reverse_turn_cd = wrap_360_cd(last_wp_bearing_cd - g.turn_back_angle_cd + g.turn_back_angle_comp_cd);                                   
                        reverse = -1; // Do the oposite of the normal
#if DEBUG_NAV == ENABLED                        
                        hal.uartC->printf("***acute angle trig R****  hdg : %d\n", (reverse_turn_cd*.01));
#endif                        
                    }            
                break;             
                case 2:                  
                    if(wrap_360_cd(calc_bearing_cd - reverse_turn_cd) < 500){
                        turn_step ++;
                        reverse_turn_cd = wrap_360_cd(current_wp_bearing_cd + g.turn_back_angle_cd - g.turn_back_angle_comp_cd);                        
                        reverse = 1;  //Return to normal turn
#if DEBUG_NAV == ENABLED                        
                        hal.uartC->printf("*** turn around *** next hdg : %d\n",(reverse_turn_cd*.01));
#endif                        
                    }                                                                                                             
                break;
                case 3:
                    if(wrap_360_cd(reverse_turn_cd - calc_bearing_cd) < 500){
                        turn_step ++;
                        reverse = -1;
#if DEBUG_NAV == ENABLED                        
                        hal.uartC->printf("*** turn back ***\n");
#endif    
                    }
                break;
                case 4:
                    if(bearing_error_cd > -g.nav_pid_angle_cd){
                        turn_step = 1;
                        turn_around = false;                                                                                                         
                    }
                break;
                default:                                                                                   
                    lock = 1;                 
                break;
             }
        // Loiter
        }else if(loiter_trig){                     
            switch (turn_step){
                case 1:    // Set up for loiter entry
                    turn_step ++;
                    turn_around = false;
                    reverse = -1;                                                        
                break;
                case 2:    // Start loitering turn
                   //Left 
                    if(lock == -1){
                        if(wrap_360_cd(calc_bearing_cd - loiter_entry_bearing_cd) < 500){
                            turn_step ++;
                            reverse = 1;                            
                        }
                    }
                   //Right 
                    if(lock == 1){
                       if(wrap_360_cd(loiter_entry_bearing_cd - calc_bearing_cd) < 500){
                            turn_step ++;
                            reverse = 1;                            
                        }                      
                    } 
                break;
                case 3:    
                    // neccesary stuf, very important for loitering circles.                    
                    loiter_error = wp_distance - g.loiter_radius;
                    // located in navigation.pde
                    tune_loiter(loiter_error);  
                    // This is the NAV loiter controler right turn
                    if(lock == 1){                    
                        nav_bearing_cd -= 9000; 
                        nav_bearing_cd += (g.nav_data_latency * 0.075 * g_gps->ground_speed) / g.loiter_radius;
                        nav_bearing_cd += constrain(((loiter_error) * g.loiter_P * 100),-1500, 2000);                                                
                    }
                    // Left turn
                    if(lock == -1){    
                        nav_bearing_cd += 9000;
                        nav_bearing_cd -= (g.nav_data_latency * 0.075 * g_gps->ground_speed) / g.loiter_radius;
                        nav_bearing_cd -= constrain(((loiter_error) * g.loiter_P * 100),-2000, 1500);                                               
                    }                        
                    nav_bearing_cd = wrap_360_cd(nav_bearing_cd);                    
                    calc_bearing_error();
                break;
            }
            
         // If we have a waypoint switch while loitering, then stop loiter and turn to new wp.
            if(nav_wp_sw){  
                turn_step = 1;
                turn_around = false;
                loiter_trig = false;
                return;
            }
#if DEBUG_NAV == ENABLED            
            if(once_a){hal.uartC->printf("*** loiter trigered 2 ***");hal.uartC->printf("\n"); once_a = false;}
#endif            
        }
        else{          // Be sure to always fall thru to here!
            lock = 0;    //WARNING setting reverse or lock to zero can cause flyaways (nav_roll stuck on 0) if we don't get to the else. 
            reverse = 1;   // Relax, I know I set lock to zero. That's so the wp switch can choose left or right based on the bering error and next wp. 
            turn_step = 1;   // We alwas have a wp and the with all the wp trigers we should not fly away. I think.
            nav_wp_sw = false;
            turn_around = false;
            g.pidNavRoll.reset_I(); 

            if(prev_control_mode != control_mode || control_mode == GUIDED || prev_nav_cmd_index != nav_command_index || jump){  
                prev_WP = current_loc;
                current_wp_bearing_cd   = get_bearing_cd(&prev_WP, &next_WP);
                prev_control_mode = control_mode;                
                reset_crosstrack();
                navigate();
                calc_bearing_error();
#if DEBUG_NAV == ENABLED              
                hal.uartC->printf("** current bearing : %d\n", g_gps->ground_course);
                hal.uartC->printf("*** current loc set as from WP ***\n");
                hal.uartC->printf("bearing_error : %d\n", bearing_error_cd);
#endif                
            }            
         }                 
#if DEBUG_NAV == ENABLED       
         count_a ++;
         if(once_a){
             once_a = false;
             hal.uartC->printf("****** nav fixed ****** : %d\n", raw_nav_roll);
             //hal.uartC->printf(", next_WP.id : "); hal.uartC->printf(next_WP.id);
         }
         if(count_a > 49){
             count_a = 0;
             hal.uartC->printf("nav fixed : %d", raw_nav_roll);hal.uartC->printf(", rad : %d", g.waypoint_radius);hal.uartC->printf(", next_WP.id : %d\n", next_WP.id);hal.uartC->printf(", bender : %f\n", nav_bender);
             hal.uartC->printf(", turn step : %i", turn_step);hal.uartC->printf("    L:%i\n", loiter_trig); 
         }                  
#endif

     // Fixed nav roll tries to fly a circular ground path by maintaining a constant radius with a variable ground speed.
     // Air speed is not used in this method. Inputs are gps speed(2d) and turn radius and left/right. 
     // Roll angle = atan((Vg^2/(r*g)), where Vg is ground speed, r is turn radius, and g is gravity.        
         
         raw_nav_roll_fixed = degrees(atan(sq(g_gps->ground_speed*.01)/(g.waypoint_radius * gravity))) * 100 * lock * reverse;             
               
    }
     // Normal PID
    else{        
#if DEBUG_NAV == ENABLED          // For debug printing only 88888888888
         if(once_b){          
             hal.uartC->printf("***** nav pid until futher notice *****\n");                        
             once_a = true;
             once_b = false;
         }    
#endif
             lock = 0;
             reverse = 1;
             turn_step = 1;      
#if DEBUG_NAV == ENABLED       
         count_b ++;
         if(count_b > 49){
             count_b = 0;             
             hal.uartC->printf(", nav pid : %d", raw_nav_roll);hal.uartC->printf(", bender : %d\n", nav_bender);
             hal.uartC->printf(", mode : %d", control_mode);hal.uartC->printf(", turn step : %d\n", turn_step);
         
         }
#endif         
         //888888888888888888888888888888888888                               
    }
    float nav_gain_scaler = 0.01 * g_gps->ground_speed / g.scaling_speed;
    nav_gain_scaler = constrain(nav_gain_scaler, 0.2, 1.4);      
    raw_nav_roll_pid = g.pidNavRoll.get_pid(bearing_error_cd, nav_gain_scaler); //returns desired bank angle in degrees*100 

   // The Bender!  1 = 100% nav_roll_fixed, 0 = 100% nav_roll_pid
   // 1000 is 10 degrees. Bearing error > X deg(g.bender_angle) means 100% nav_fixed. Bearing error from zero to X deg is blended, zero is 100% nav_pid.
   //--------------------------------------------------------------------------------------------
    if(turn_step <= 3 && nav_wp_sw && turn_around && !loiter_trig){
        nav_bender = 1;
    }   
    else if((turn_step == 4 || (turn_step == 1 && !turn_around)) && nav_wp_sw && !loiter_trig){
        nav_bender = float(g.bender_angle_cd - abs(bearing_error_cd))/float(g.bender_angle_cd) - 1; 
        nav_bender = constrain(fabs(nav_bender),0,1);                        
    }
    else if(loiter_trig && turn_step < 3){
        nav_bender = 1;
    }
    else if(loiter_trig && turn_step >= 3){
        nav_bender = 0;        
    }
    else nav_bender = 0;   
    raw_nav_roll = (1 - nav_bender) * raw_nav_roll_pid + nav_bender * raw_nav_roll_fixed;                      
    nav_roll_cd = 0.9 * nav_roll_cd + 0.1 * raw_nav_roll;    // a little filtering?
    nav_roll_cd = raw_nav_roll; 
    if(control_mode == GUIDED || control_mode == RTL) nav_roll_cd = constrain(nav_roll_cd, (0.78 * -g.roll_limit_cd.get()), (0.78 * g.roll_limit_cd.get())); // Had to put a tighter leash on these modes as you can get there rairly, if you have a loiter as the last command.   
    else nav_roll_cd = constrain(nav_roll_cd, -g.roll_limit_cd.get(), g.roll_limit_cd.get());     
}
/*****************************************
* Roll servo slew limit
*****************************************/
/*
 *  float roll_slew_limit(float servo)
 *  {
 *       static float last;
 *       float temp = constrain(servo, last-ROLL_SLEW_LIMIT * delta_ms_fast_loop/1000.f, last + ROLL_SLEW_LIMIT * delta_ms_fast_loop/1000.f);
 *       last = servo;
 *       return temp;
 *  }*/

/*****************************************
* Throttle slew limit
*****************************************/
static void throttle_slew_limit(int16_t last_throttle)
{
    // if slew limit rate is set to zero then do not slew limit
    if (g.throttle_slewrate) {                   
        // limit throttle change by the given percentage per second
        float temp = g.throttle_slewrate * G_Dt * 0.01 * fabs(g.channel_throttle.radio_max - g.channel_throttle.radio_min);
        // allow a minimum change of 1 PWM per cycle
        if (temp < 1) {
            temp = 1;
        }
        g.channel_throttle.radio_out = constrain_int16(g.channel_throttle.radio_out, last_throttle - temp, last_throttle + temp);
    }
}


/* We want to supress the throttle if we think we are on the ground and in an autopilot controlled throttle mode.

   Disable throttle if following conditions are met:
   *       1 - We are in Circle mode (which we use for short term failsafe), or in FBW-B or higher
   *       AND
   *       2 - Our reported altitude is within 10 meters of the home altitude.
   *       3 - Our reported speed is under 5 meters per second.
   *       4 - We are not performing a takeoff in Auto mode
   *       OR
   *       5 - Home location is not set
*/
static bool suppress_throttle(void)
{
    if (!throttle_suppressed) {
        // we've previously met a condition for unsupressing the throttle
        return false;
    }
    if (control_mode != CIRCLE && control_mode <= FLY_BY_WIRE_A) {
        // the user controls the throttle
        throttle_suppressed = false;
        return false;
    }

    if (control_mode==AUTO && takeoff_complete == false) {
        // we're in auto takeoff 
        throttle_suppressed = false;
        return false;
    }
    
    if (labs(home.alt - current_loc.alt) >= 1000) {
        // we're more than 10m from the home altitude
        throttle_suppressed = false;
        return false;
    }

    if (g_gps != NULL && 
        g_gps->status() == GPS::GPS_OK && 
        g_gps->ground_speed >= 500) {
        // we're moving at more than 5 m/s
        throttle_suppressed = false;
        return false;        
    }

    // throttle remains suppressed
    return true;
}

/*****************************************
* Set the flight control servos based on the current calculated values
*****************************************/
static void set_servos(void)
{
    int16_t last_throttle = g.channel_throttle.radio_out;

    if (control_mode == MANUAL) {
        // do a direct pass through of radio values
        if (g.mix_mode == 0) {
            g.channel_roll.radio_out                = g.channel_roll.radio_in;
            g.channel_pitch.radio_out               = g.channel_pitch.radio_in;
        } else {
            g.channel_roll.radio_out                = hal.rcin->read(CH_ROLL);
            g.channel_pitch.radio_out               = hal.rcin->read(CH_PITCH);
        }
        g.channel_throttle.radio_out    = g.channel_throttle.radio_in;
        g.channel_rudder.radio_out              = g.channel_rudder.radio_in;

        // setup extra aileron channel. We want this to come from the
        // main aileron input channel, but using the 2nd channels dead
        // zone, reverse and min/max settings. We need to use
        // pwm_to_angle_dz() to ensure we don't trim the value for the
        // deadzone of the main aileron channel, otherwise the 2nd
        // aileron won't quite follow the first one
        int16_t aileron_in = g.channel_roll.pwm_to_angle_dz(0);
        RC_Channel_aux::set_servo_out(RC_Channel_aux::k_aileron, aileron_in);

        // this aileron variant assumes you have the corresponding
        // input channel setup in your transmitter for manual control
        // of the 2nd aileron
        RC_Channel_aux::copy_radio_in_out(RC_Channel_aux::k_aileron_with_input);

        // copy flap control from transmitter
        RC_Channel_aux::copy_radio_in_out(RC_Channel_aux::k_flap_auto);

        if (g.mix_mode != 0) {
            // set any differential spoilers to follow the elevons in
            // manual mode. 
            RC_Channel_aux::set_radio(RC_Channel_aux::k_dspoiler1, g.channel_roll.radio_out);
            RC_Channel_aux::set_radio(RC_Channel_aux::k_dspoiler2, g.channel_pitch.radio_out);
        }
    } else {
        if (g.mix_mode == 0) {
            // both types of secondary aileron are slaved to the roll servo out
            RC_Channel_aux::set_servo_out(RC_Channel_aux::k_aileron, g.channel_roll.servo_out);
            RC_Channel_aux::set_servo_out(RC_Channel_aux::k_aileron_with_input, g.channel_roll.servo_out);
        }else{
            /*Elevon mode*/
            float ch1;
            float ch2;
            ch1 = g.channel_pitch.servo_out - (BOOL_TO_SIGN(g.reverse_elevons) * g.channel_roll.servo_out);
            ch2 = g.channel_pitch.servo_out + (BOOL_TO_SIGN(g.reverse_elevons) * g.channel_roll.servo_out);

			/* Differential Spoilers
               If differential spoilers are setup, then we translate
               rudder control into splitting of the two ailerons on
               the side of the aircraft where we want to induce
               additional drag.
             */
			if (RC_Channel_aux::function_assigned(RC_Channel_aux::k_dspoiler1) && RC_Channel_aux::function_assigned(RC_Channel_aux::k_dspoiler2)) {
				float ch3 = ch1;
				float ch4 = ch2;
				if ( BOOL_TO_SIGN(g.reverse_elevons) * g.channel_rudder.servo_out < 0) {
				    ch1 += abs(g.channel_rudder.servo_out);
				    ch3 -= abs(g.channel_rudder.servo_out);
				} else {
					ch2 += abs(g.channel_rudder.servo_out);
				    ch4 -= abs(g.channel_rudder.servo_out);
				}
				RC_Channel_aux::set_servo_out(RC_Channel_aux::k_dspoiler1, ch3);
				RC_Channel_aux::set_servo_out(RC_Channel_aux::k_dspoiler2, ch4);
			}

            // directly set the radio_out values for elevon mode
            g.channel_roll.radio_out  =     elevon1_trim + (BOOL_TO_SIGN(g.reverse_ch1_elevon) * (ch1 * 500.0/ SERVO_MAX));
            g.channel_pitch.radio_out =     elevon2_trim + (BOOL_TO_SIGN(g.reverse_ch2_elevon) * (ch2 * 500.0/ SERVO_MAX));
        }

#if OBC_FAILSAFE == ENABLED
        // this is to allow the failsafe module to deliberately crash 
        // the plane. Only used in extreme circumstances to meet the
        // OBC rules
        if (obc.crash_plane()) {
            g.channel_roll.servo_out = -4500;
            g.channel_pitch.servo_out = -4500;
            g.channel_rudder.servo_out = -4500;
            g.channel_throttle.servo_out = 0;
        }
#endif
        

        // push out the PWM values
        if (g.mix_mode == 0) {
            g.channel_roll.calc_pwm();
            g.channel_pitch.calc_pwm();
        }
        g.channel_rudder.calc_pwm();

#if THROTTLE_OUT == 0
        g.channel_throttle.servo_out = 0;
#else
        // convert 0 to 100% into PWM
        g.channel_throttle.servo_out = constrain_int16(g.channel_throttle.servo_out, 
                                                       g.throttle_min.get(), 
                                                       g.throttle_max.get());

        if (suppress_throttle()) {
            // throttle is suppressed in auto mode
            g.channel_throttle.servo_out = 0;
            if (g.throttle_suppress_manual) {
                // manual pass through of throttle while throttle is suppressed
                g.channel_throttle.radio_out = g.channel_throttle.radio_in;
            } else {
                g.channel_throttle.calc_pwm();                
            }
        } else if (g.throttle_passthru_stabilize && 
                   (control_mode == STABILIZE || 
                    control_mode == TRAINING ||
                    control_mode == FLY_BY_WIRE_A)) {
            // manual pass through of throttle while in FBWA or
            // STABILIZE mode with THR_PASS_STAB set
            g.channel_throttle.radio_out = g.channel_throttle.radio_in;
        } else {
            // normal throttle calculation based on servo_out
            g.channel_throttle.calc_pwm();
        }
#endif
    }

    // Auto flap deployment
    if(control_mode < FLY_BY_WIRE_B) {
        RC_Channel_aux::copy_radio_in_out(RC_Channel_aux::k_flap_auto);
    } else if (control_mode >= FLY_BY_WIRE_B) {
        int16_t flapSpeedSource = 0;

        // FIXME: use target_airspeed in both FBW_B and g.airspeed_enabled cases - Doug?
        if (control_mode == FLY_BY_WIRE_B) {
            flapSpeedSource = target_airspeed_cm * 0.01;
        } else if (airspeed.use()) {
            flapSpeedSource = g.airspeed_cruise_cm * 0.01;
        } else {
            flapSpeedSource = g.throttle_cruise;
        }
        if ( g.flap_1_speed != 0 && flapSpeedSource > g.flap_1_speed) {
            RC_Channel_aux::set_servo_out(RC_Channel_aux::k_flap_auto, 0);
        } else if (g.flap_2_speed != 0 && flapSpeedSource > g.flap_2_speed) {
            RC_Channel_aux::set_servo_out(RC_Channel_aux::k_flap_auto, g.flap_1_percent);
        } else {
            RC_Channel_aux::set_servo_out(RC_Channel_aux::k_flap_auto, g.flap_2_percent);
        }
    }

    if (control_mode >= FLY_BY_WIRE_B) {
        /* only do throttle slew limiting in modes where throttle
         *  control is automatic */
        throttle_slew_limit(last_throttle);
    }

#if HIL_MODE == HIL_MODE_DISABLED || HIL_SERVOS
    // send values to the PWM timers for output
    // ----------------------------------------
    hal.rcout->write(CH_1, g.channel_roll.radio_out);     // send to Servos
    hal.rcout->write(CH_2, g.channel_pitch.radio_out);     // send to Servos
    hal.rcout->write(CH_3, g.channel_throttle.radio_out);     // send to Servos
    hal.rcout->write(CH_4, g.channel_rudder.radio_out);     // send to Servos
    // Route configurable aux. functions to their respective servos
    g.rc_5.output_ch(CH_5);
    g.rc_6.output_ch(CH_6);
    g.rc_7.output_ch(CH_7);
    g.rc_8.output_ch(CH_8);
 # if CONFIG_HAL_BOARD == HAL_BOARD_APM2
    g.rc_9.output_ch(CH_9);
    g.rc_10.output_ch(CH_10);
    g.rc_11.output_ch(CH_11);
 # endif
#endif
}

static bool demoing_servos;

static void demo_servos(uint8_t i) {

    while(i > 0) {
        gcs_send_text_P(SEVERITY_LOW,PSTR("Demo Servos!"));
        demoing_servos = true;
#if HIL_MODE == HIL_MODE_DISABLED || HIL_SERVOS
        hal.rcout->write(1, 1400);
        mavlink_delay(400);
        hal.rcout->write(1, 1600);
        mavlink_delay(200);
        hal.rcout->write(1, 1500);
#endif
        demoing_servos = false;
        mavlink_delay(400);
        i--;
    }
}

// return true if we should use airspeed for altitude/throttle control
static bool alt_control_airspeed(void)
{
    return airspeed.use() && g.alt_control_algorithm == ALT_CONTROL_DEFAULT;
}
