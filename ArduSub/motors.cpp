/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include "Sub.h"

#define ARM_DELAY               20  // called at 10hz so 2 seconds
#define DISARM_DELAY            20  // called at 10hz so 2 seconds
#define AUTO_TRIM_DELAY         100 // called at 10hz so 10 seconds
#define LOST_VEHICLE_DELAY      10  // called at 10hz so 1 second

static uint32_t auto_disarm_begin;

// arm_motors_check - checks for pilot input to arm or disarm the copter
// called at 10hz
void Sub::arm_motors_check()
{
<<<<<<< 6dafedb2d1ad5061d859a9c319fa4b69b4ac5dd9
    // Arm motors automatically
	if ( !motors.armed() ) {
    	init_arm_motors(false);
    }

	/*static int16_t arming_counter;
=======
    static int16_t arming_counter;
>>>>>>> Changed to ArduCopter as the base code.

    // ensure throttle is down
    if (channel_throttle->control_in > 0) {
        arming_counter = 0;
        return;
    }

    int16_t tmp = channel_yaw->control_in;

    // full right
    if (tmp > 4000) {

        // increase the arming counter to a maximum of 1 beyond the auto trim counter
        if( arming_counter <= AUTO_TRIM_DELAY ) {
            arming_counter++;
        }

        // arm the motors and configure for flight
        if (arming_counter == ARM_DELAY && !motors.armed()) {
            // reset arming counter if arming fail
            if (!init_arm_motors(false)) {
                arming_counter = 0;
            }
        }

        // arm the motors and configure for flight
        if (arming_counter == AUTO_TRIM_DELAY && motors.armed() && control_mode == STABILIZE) {
            auto_trim_counter = 250;
            // ensure auto-disarm doesn't trigger immediately
            auto_disarm_begin = millis();
        }

    // full left
    }else if (tmp < -4000) {
        if (!mode_has_manual_throttle(control_mode) && !ap.land_complete) {
            arming_counter = 0;
            return;
        }

        // increase the counter to a maximum of 1 beyond the disarm delay
        if( arming_counter <= DISARM_DELAY ) {
            arming_counter++;
        }

        // disarm the motors
        if (arming_counter == DISARM_DELAY && motors.armed()) {
            init_disarm_motors();
        }

    // Yaw is centered so reset arming counter
    }else{
        arming_counter = 0;
<<<<<<< 6dafedb2d1ad5061d859a9c319fa4b69b4ac5dd9
    }*/
=======
    }
>>>>>>> Changed to ArduCopter as the base code.
}

// auto_disarm_check - disarms the copter if it has been sitting on the ground in manual mode with throttle low for at least 15 seconds
void Sub::auto_disarm_check()
{
<<<<<<< 6dafedb2d1ad5061d859a9c319fa4b69b4ac5dd9
    /*uint32_t tnow_ms = millis();
=======
    uint32_t tnow_ms = millis();
>>>>>>> Changed to ArduCopter as the base code.
    uint32_t disarm_delay_ms = 1000*constrain_int16(g.disarm_delay, 0, 127);

    // exit immediately if we are already disarmed, or if auto
    // disarming is disabled
    if (!motors.armed() || disarm_delay_ms == 0 || control_mode == THROW) {
        auto_disarm_begin = tnow_ms;
        return;
    }

    // always allow auto disarm if using interlock switch or motors are Emergency Stopped
    if ((ap.using_interlock && !motors.get_interlock()) || ap.motor_emergency_stop) {
        // use a shorter delay if using throttle interlock switch or Emergency Stop, because it is less
        // obvious the copter is armed as the motors will not be spinning
        disarm_delay_ms /= 2;
    } else {
        bool sprung_throttle_stick = (g.throttle_behavior & THR_BEHAVE_FEEDBACK_FROM_MID_STICK) != 0;
        bool thr_low;
        if (mode_has_manual_throttle(control_mode) || !sprung_throttle_stick) {
            thr_low = ap.throttle_zero;
        } else {
            float deadband_top = g.rc_3.get_control_mid() + g.throttle_deadzone;
            thr_low = g.rc_3.control_in <= deadband_top;
        }

        if (!thr_low) {
            // reset timer
            auto_disarm_begin = tnow_ms;
        }
    }

    // disarm once timer expires
    if ((tnow_ms-auto_disarm_begin) >= disarm_delay_ms) {
        init_disarm_motors();
        auto_disarm_begin = tnow_ms;
<<<<<<< 6dafedb2d1ad5061d859a9c319fa4b69b4ac5dd9
    }*/
=======
    }
>>>>>>> Changed to ArduCopter as the base code.
}

// init_arm_motors - performs arming process including initialisation of barometer and gyros
//  returns false if arming failed because of pre-arm checks, arming checks or a gyro calibration failure
bool Sub::init_arm_motors(bool arming_from_gcs)
{
    static bool in_arm_motors = false;

    // exit immediately if already in this function
    if (in_arm_motors) {
        return false;
    }
    in_arm_motors = true;

    // run pre-arm-checks and display failures
    if (!all_arming_checks_passing(arming_from_gcs)) {
        AP_Notify::events.arming_failed = true;
        in_arm_motors = false;
        return false;
    }

    // disable cpu failsafe because initialising everything takes a while
    failsafe_disable();

    // reset battery failsafe
    set_failsafe_battery(false);

    // notify that arming will occur (we do this early to give plenty of warning)
    AP_Notify::flags.armed = true;
    // call update_notify a few times to ensure the message gets out
    for (uint8_t i=0; i<=10; i++) {
        update_notify();
    }

#if HIL_MODE != HIL_MODE_DISABLED || CONFIG_HAL_BOARD == HAL_BOARD_SITL
    gcs_send_text(MAV_SEVERITY_INFO, "Arming motors");
#endif

    // Remember Orientation
    // --------------------
    init_simple_bearing();

    initial_armed_bearing = ahrs.yaw_sensor;

    if (ap.home_state == HOME_UNSET) {
        // Reset EKF altitude if home hasn't been set yet (we use EKF altitude as substitute for alt above home)
        
        // Always use absolute altitude for ROV
		// ahrs.resetHeightDatum();
		// Log_Write_Event(DATA_EKF_ALT_RESET);
    } else if (ap.home_state == HOME_SET_NOT_LOCKED) {
        // Reset home position if it has already been set before (but not locked)
        set_home_to_current_location();
    }
    calc_distance_and_bearing();

    // enable gps velocity based centrefugal force compensation
    ahrs.set_correct_centrifugal(true);
    hal.util->set_soft_armed(true);

    // enable output to motors
    enable_motor_output();

    // finally actually arm the motors
    motors.armed(true);

    // log arming to dataflash
    Log_Write_Event(DATA_ARMED);

    // log flight mode in case it was changed while vehicle was disarmed
    DataFlash.Log_Write_Mode(control_mode, control_mode_reason);

    // reenable failsafe
    failsafe_enable();

    // perf monitor ignores delay due to arming
    perf_ignore_this_loop();

    // flag exiting this function
    in_arm_motors = false;

    // return success
    return true;
}

// init_disarm_motors - disarm motors
void Sub::init_disarm_motors()
{
    // return immediately if we are already disarmed
    if (!motors.armed()) {
        return;
    }

#if HIL_MODE != HIL_MODE_DISABLED || CONFIG_HAL_BOARD == HAL_BOARD_SITL
    gcs_send_text(MAV_SEVERITY_INFO, "Disarming motors");
#endif

    // save compass offsets learned by the EKF if enabled
    if (ahrs.use_compass() && compass.get_learn_type() == Compass::LEARN_EKF) {
        for(uint8_t i=0; i<COMPASS_MAX_INSTANCES; i++) {
            Vector3f magOffsets;
            if (ahrs.getMagOffsets(i, magOffsets)) {
                compass.set_and_save_offsets(i, magOffsets);
            }
        }
	 }

#if AUTOTUNE_ENABLED == ENABLED
    // save auto tuned parameters
    autotune_save_tuning_gains();
#endif

    // log disarm to the dataflash
    Log_Write_Event(DATA_DISARMED);

    // send disarm command to motors
    motors.armed(false);

    // reset the mission
    mission.reset();

    // suspend logging
    if (!(g.log_bitmask & MASK_LOG_WHEN_DISARMED)) {
        DataFlash.EnableWrites(false);
    }

    // disable gps velocity based centrefugal force compensation
    ahrs.set_correct_centrifugal(false);
    hal.util->set_soft_armed(false);
}

// motors_output - send output to motors library which will adjust and send to ESCs and servos
void Sub::motors_output()
{
    // check if we are performing the motor test
    if (ap.motor_test) {
        motor_test_output();
    } else {
        if (!ap.using_interlock){
            // if not using interlock switch, set according to Emergency Stop status
            // where Emergency Stop is forced false during arming if Emergency Stop switch
            // is not used. Interlock enabled means motors run, so we must
            // invert motor_emergency_stop status for motors to run.
            motors.set_interlock(!ap.motor_emergency_stop);
        }
        motors.output();
    }
}

// check for pilot stick input to trigger lost vehicle alarm
void Sub::lost_vehicle_check()
{
    static uint8_t soundalarm_counter;

    // disable if aux switch is setup to vehicle alarm as the two could interfere
    if (check_if_auxsw_mode_used(AUXSW_LOST_COPTER_SOUND)) {
        return;
    }

    // ensure throttle is down, motors not armed, pitch and roll rc at max. Note: rc1=roll rc2=pitch
    if (ap.throttle_zero && !motors.armed() && (channel_roll->control_in > 4000) && (channel_pitch->control_in > 4000)) {
        if (soundalarm_counter >= LOST_VEHICLE_DELAY) {
            if (AP_Notify::flags.vehicle_lost == false) {
                AP_Notify::flags.vehicle_lost = true;
                gcs_send_text(MAV_SEVERITY_NOTICE,"Locate Copter alarm");
            }
        } else {
            soundalarm_counter++;
        }
    } else {
        soundalarm_counter = 0;
        if (AP_Notify::flags.vehicle_lost == true) {
            AP_Notify::flags.vehicle_lost = false;
        }
    }
}

// translate wpnav roll/pitch outputs to lateral/forward
void Sub::translate_wpnav_rp(float &lateral_out, float &forward_out) {
	   // get roll and pitch targets in centidegrees
		int32_t lateral = wp_nav.get_roll();
		int32_t forward = -wp_nav.get_pitch(); // output is reversed

		// constrain target forward/lateral values
		// The outputs of wp_nav.get_roll and get_pitch should already be constrained to these values
		lateral = constrain_int16(lateral, -aparm.angle_max, aparm.angle_max);
		forward = constrain_int16(forward, -aparm.angle_max, aparm.angle_max);

		// Normalize
		lateral_out = (float)lateral/(float)aparm.angle_max;
		forward_out = (float)forward/(float)aparm.angle_max;
}
