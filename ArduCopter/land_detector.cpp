#include "Copter.h"

// Code to detect a crash main ArduCopter code
#define LAND_CHECK_ANGLE_ERROR_DEG  30.0f       // maximum angle error to be considered landing
#define LAND_CHECK_LARGE_ANGLE_CD   1500.0f     // maximum angle target to be considered landing
#define LAND_CHECK_ACCEL_MOVING     3.0f        // maximum acceleration after subtracting gravity


// counter to verify landings
static uint32_t land_detector_count = 0;

// run land and crash detectors
// called at MAIN_LOOP_RATE
void Copter::update_land_and_crash_detectors()
{
    // update 1hz filtered acceleration
    Vector3f accel_ef = ahrs.get_accel_ef();
    accel_ef.z += GRAVITY_MSS;
    land_accel_ef_filter.apply(accel_ef, scheduler.get_loop_period_s());

    update_land_detector();

#if PARACHUTE == ENABLED
    // check parachute
    parachute_check();
#endif

    crash_check();
    thrust_loss_check();
    yaw_imbalance_check();
}

// update_land_detector - checks if we have landed and updates the ap.land_complete flag
// called at MAIN_LOOP_RATE
void Copter::update_land_detector()
{
    // land detector can not use the following sensors because they are unreliable during landing
    // barometer altitude :                 ground effect can cause errors larger than 4m
    // EKF vertical velocity or altitude :  poor barometer and large acceleration from ground impact
    // earth frame angle or angle error :   landing on an uneven surface will force the airframe to match the ground angle
    // gyro output :                        on uneven surface the airframe may rock back an forth after landing
    // range finder :                       tend to be problematic at very short distances
    // input throttle :                     in slow land the input throttle may be only slightly less than hover

    if (!motors->armed()) {
        // if disarmed, always landed.
        set_land_complete(true);
    } else if (ap.land_complete) {
#if FRAME_CONFIG == HELI_FRAME
        // if rotor speed and collective pitch are high then clear landing flag
        if (!flightmode->is_taking_off() && motors->get_takeoff_collective() && motors->get_spool_state() == AP_Motors::SpoolState::THROTTLE_UNLIMITED) {
#else
        // if throttle output is high then clear landing flag
        if (!flightmode->is_taking_off() && motors->get_throttle_out() > get_non_takeoff_throttle() && motors->get_spool_state() == AP_Motors::SpoolState::THROTTLE_UNLIMITED) {
            // this should never happen because take-off should be detected at the flight mode level
            // this here to highlight there is a bug or missing take-off detection
            INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
#endif
            set_land_complete(false);
        }
    } else if (standby_active) {
        // land detector will not run in standby mode
        land_detector_count = 0;
    } else {

        float land_trigger_sec = g.land_detector_trigger_sec; //since  LAND_DETECTOR_TRIGGER_SEC_DEFAULT has been added as a parameter, we grab the parameter instead for compatibility purpose..
		
		
		
#if FRAME_CONFIG == HELI_FRAME
        // check for both manual collective modes and modes that use altitude hold. For manual collective (called throttle
        // because multi's use throttle), check that collective pitch is below land min collective position or throttle stick is zero.
        // Including the throttle zero check will ensure the conditions where stabilize stick zero position was not below collective min. For modes
        // that use altitude hold, check that the pilot is commanding a descent and collective is at min allowed for altitude hold modes.

        // check if landing
        const bool landing = flightmode->is_landing();
        bool motor_at_lower_limit = (flightmode->has_manual_throttle() && (motors->get_below_land_min_coll() || heli_flags.coll_stk_low) && fabsf(ahrs.get_roll()) < M_PI/2.0f)
                                    || ((!force_flying || landing) && motors->limit.throttle_lower && pos_control->get_vel_desired_cms().z < 0.0f);
        bool throttle_mix_at_min = true;
#else
        // check that the average throttle output is near minimum (less than 12.5% hover throttle)
        bool motor_at_lower_limit = motors->limit.throttle_lower;
        bool throttle_mix_at_min = attitude_control->is_throttle_mix_min();
        // set throttle_mix_at_min to true because throttle is never at mix min in airmode
		
        // increase land_trigger_sec when using airmode
        if (flightmode->has_manual_throttle() && air_mode == AirMode::AIRMODE_ENABLED) {
            land_trigger_sec = LAND_AIRMODE_DETECTOR_TRIGGER_SEC_DEFAULT;
            throttle_mix_at_min = true;
        }
#endif

        uint8_t land_detector_scalar = 1;
#if LANDING_GEAR_ENABLED == ENABLED
        if (landinggear.get_wow_state() != AP_LandingGear::LG_WOW_UNKNOWN) {
            // we have a WoW sensor so lets loosen the strictness of the landing detector
            land_detector_scalar = 2;
        }
#endif

        // check that the airframe is not accelerating (not falling or braking after fast forward flight)
		float acceleration = land_accel_ef_filter.get().length(); // in order to be able to add this value in the log for investigation purpose
        bool accel_stationary = (acceleration <= g.land_detector_accel_max * land_detector_scalar);

        // check that vertical speed is within 1m/s of zero
		int32_t Zspeed = fabsf(inertial_nav.get_velocity_z_up_cms()); 	// in order to be able to add this value in the log for investigation purpose
        bool descent_rate_low = Zspeed < 100 * land_detector_scalar; 	// LAND_SPEED should be below 100 and LAND_SPEED_HIGH higher

        // if we have a healthy rangefinder only allow landing detection below 2 meters
		int32_t height = rangefinder_state.alt_cm_filt.get(); // in order to be able to add this value in the log for investigation purpose
        bool rangefinder_check = (!rangefinder_alt_ok() || height < LAND_RANGEFINDER_MIN_ALT_CM);
			
		// Check the Motor throttle and add 5% as limit (for use with range finder only)	
		float mot_throttle =  motors->get_throttle_out(); // in order to be able to add this value in the log for investigation purpose
		bool land_mot_low = mot_throttle <  g.land_detector_mot_low; // mot_low will be used instead of  motor_at_lower_limit this value must be between motor_at_lower_limit and hoover

		// Check if the range finder distance is below GNDCLEAR and > 0 : this condition will allow Landed_Complete regardless the AccStationary 
		int16_t gnd_clear = copter.rangefinder.ground_clearance_cm_orient(ROTATION_PITCH_270);  // in order to be able to add this value in the log for investigation purpose
		bool height_gnd_clear = height < gnd_clear && height > 0; // checking if height above ground is lower than RNGFND1_GNDCLEAR param, only usefull with a RNGFND and LAND_RNGFND = 1
			
		bool test_mode = g.land_detector_rngfnd==1;	//checks if we are in mode Land_detector (using) RNGFND
			
			
        // if we have weight on wheels (WoW) or ambiguous unknown. never no WoW
#if LANDING_GEAR_ENABLED == ENABLED
        const bool WoW_check = (landinggear.get_wow_state() == AP_LandingGear::LG_WOW || landinggear.get_wow_state() == AP_LandingGear::LG_WOW_UNKNOWN);
#else
        const bool WoW_check = true;
#endif
			    // support height based triggering using rangefinder or altitude above ground
	
						AP::logger().Write("LNDT", "TimeUS,MotLL,TMM,AcSt,DeRat,RfCk,RFLD,HGC,MotLo","---------","00000000", "Qffffffff", // loging of boolean values used for land detection
										AP_HAL::micros64(),
										(double)motor_at_lower_limit,	//This trigger will mean the drone is probably not flying as no thrust (possibly but unlikely descending)
                                        (double)throttle_mix_at_min,	//This trigger is always true on LAND mode
                                        (double)accel_stationary,	//This trigger means no accelaration anymore means the drone is probably on the ground or not common on big drone where vibrations keep that value high on ground
                                        (double)descent_rate_low,	//This trigger means no vertical speed means the drone is on ground only if the thrust is @ minimum (or close) at the mean time
                                        (double)rangefinder_check,	//This trigger means the range finder is below LAND_RANGEFINDER_MIN_ALT_CM (2m) or not healthy, 
                                        (double)test_mode,	//This recalls if RNGFND based Land_detector mode is currently used
										(double)height_gnd_clear,	//This trigger will mean the drone is close the ground below RNGFND1_GNDCLEAR it will be the altenative to accel_stationary if LAND_DET_RNGFND = 1
										(double)land_mot_low); //This trigger will switch when motor_at_lower_limit +5% used only if LAND_DET_RNGFND = 1
										
						AP::logger().Write("LNDV", "TimeUS,Acc,height,Zspd,GndClr,ThO,Ldc","-omnm%-","00BBB20", "Qffffff", // loging of variable values used for land detection
										AP_HAL::micros64(),
										(double)acceleration,	//will display acceleration which will reset detector if over LAND_DET_ACC_MAX will be ignored if LAND_DET_RNGFND = 1
                                        (double)height,	//Height from the ground 
                                        (double)Zspeed,	//vertical speed down
										(double)gnd_clear,	//Parameter RNGFND1_GNDCLEAR displayed here
										(float)mot_throttle,	//average motor throttle 
										(double)land_detector_count);	//This counts the number on cycles since Land detector conditions turned all true before LAND is completed

        if ((motor_at_lower_limit && accel_stationary && descent_rate_low && throttle_mix_at_min && rangefinder_check && WoW_check)|| 	// that condition will get TRUE as initial (probably not with large prop's)
			(test_mode && land_mot_low && descent_rate_low && throttle_mix_at_min && rangefinder_check && WoW_check && height_gnd_clear))	// g.land_detector_rngfnd==1 && AccStationary is not tested in RNGFND LAND MODE
			 {
            // landed criteria met - increment the counter and check if we've triggered
			
            if( land_detector_count < land_trigger_sec*scheduler.get_loop_rate_hz()) {
                land_detector_count++;
            } else {
                set_land_complete(true);
	            }
        } else {
            // we've sensed movement up or down so reset land_de	tector
            land_detector_count = 0;
        }
    }

    set_land_complete_maybe(ap.land_complete || (land_detector_count >= g.land_detector_maybe_trigger_sec*scheduler.get_loop_rate_hz()));
}

// set land_complete flag and disarm motors if disarm-on-land is configured
void Copter::set_land_complete(bool b)
{
    // if no change, exit immediately
    if( ap.land_complete == b )
        return;

    land_detector_count = 0;

    if(b){
        AP::logger().Write_Event(LogEvent::LAND_COMPLETE);
    } else {
        AP::logger().Write_Event(LogEvent::NOT_LANDED);
    }
    ap.land_complete = b;

#if STATS_ENABLED == ENABLED
    g2.stats.set_flying(!b);
#endif

    // tell AHRS flying state
    set_likely_flying(!b);
    
    // trigger disarm-on-land if configured
    bool disarm_on_land_configured = (g.throttle_behavior & THR_BEHAVE_DISARM_ON_LAND_DETECT) != 0;
    const bool mode_disarms_on_land = flightmode->allows_arming(AP_Arming::Method::LANDING) && !flightmode->has_manual_throttle();

    if (ap.land_complete && motors->armed() && disarm_on_land_configured && mode_disarms_on_land) {
        arming.disarm(AP_Arming::Method::LANDED);
    }
}

// set land complete maybe flag
void Copter::set_land_complete_maybe(bool b)
{
    // if no change, exit immediately
    if (ap.land_complete_maybe == b)
        return;

    if (b) {
        AP::logger().Write_Event(LogEvent::LAND_COMPLETE_MAYBE);
    }
    ap.land_complete_maybe = b;
}

// sets motors throttle_low_comp value depending upon vehicle state
//  low values favour pilot/autopilot throttle over attitude control, high values favour attitude control over throttle
//  has no effect when throttle is above hover throttle
void Copter::update_throttle_mix()
{
#if FRAME_CONFIG != HELI_FRAME
    // if disarmed or landed prioritise throttle
    if (!motors->armed() || ap.land_complete) {
        attitude_control->set_throttle_mix_min();
        return;
    }

    if (flightmode->has_manual_throttle()) {
        // manual throttle
        if (channel_throttle->get_control_in() <= 0 && air_mode != AirMode::AIRMODE_ENABLED) {
            attitude_control->set_throttle_mix_min();
        } else {
            attitude_control->set_throttle_mix_man();
        }
    } else {
        // autopilot controlled throttle

        // check for aggressive flight requests - requested roll or pitch angle below 15 degrees
        const Vector3f angle_target = attitude_control->get_att_target_euler_cd();
        bool large_angle_request = angle_target.xy().length() > LAND_CHECK_LARGE_ANGLE_CD;

        // check for large external disturbance - angle error over 30 degrees
        const float angle_error = attitude_control->get_att_error_angle_deg();
        bool large_angle_error = (angle_error > LAND_CHECK_ANGLE_ERROR_DEG);

        // check for large acceleration - falling or high turbulence
        const bool accel_moving = (land_accel_ef_filter.get().length() > LAND_CHECK_ACCEL_MOVING);

        // check for requested descent
        bool descent_not_demanded = pos_control->get_vel_desired_cms().z >= 0.0f;

        // check if landing
        const bool landing = flightmode->is_landing();

        if (((large_angle_request || force_flying) && !landing) || large_angle_error || accel_moving || descent_not_demanded) {
            attitude_control->set_throttle_mix_max(pos_control->get_vel_z_control_ratio());
        } else {
            attitude_control->set_throttle_mix_min();
        }
    }
#endif
}
