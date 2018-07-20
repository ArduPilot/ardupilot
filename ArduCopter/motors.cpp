#include "Copter.h"

#define ARM_DELAY               20  // called at 10hz so 2 seconds
#define DISARM_DELAY            20  // called at 10hz so 2 seconds
#define AUTO_TRIM_DELAY         100 // called at 10hz so 10 seconds
#define LOST_VEHICLE_DELAY      10  // called at 10hz so 1 second

static uint32_t auto_disarm_begin;

// arm_motors_check - checks for pilot input to arm or disarm the copter
// called at 10hz
void Copter::arm_motors_check()
{
    static int16_t arming_counter;

#if TOY_MODE_ENABLED == ENABLED
    if (g2.toy_mode.enabled()) {
        // not armed with sticks in toy mode
        return;
    }
#endif
    
    // ensure throttle is down
    if (channel_throttle->get_control_in() > 0) {
        arming_counter = 0;
        return;
    }

    int16_t tmp = channel_yaw->get_control_in();

    // full right
    if (tmp > 4000) {

        // increase the arming counter to a maximum of 1 beyond the auto trim counter
        if( arming_counter <= AUTO_TRIM_DELAY ) {
            arming_counter++;
        }

        // arm the motors and configure for flight
        if (arming_counter == ARM_DELAY && !motors->armed()) {
            // reset arming counter if arming fail
            if (!init_arm_motors(AP_Arming::ArmingMethod::RUDDER)) {
                arming_counter = 0;
            }
        }

        // arm the motors and configure for flight
        if (arming_counter == AUTO_TRIM_DELAY && motors->armed() && control_mode == STABILIZE) {
            auto_trim_counter = 250;
            // ensure auto-disarm doesn't trigger immediately
            auto_disarm_begin = millis();
        }

    // full left
    }else if (tmp < -4000) {
        if (!flightmode->has_manual_throttle() && !ap.land_complete) {
            arming_counter = 0;
            return;
        }

        // increase the counter to a maximum of 1 beyond the disarm delay
        if( arming_counter <= DISARM_DELAY ) {
            arming_counter++;
        }

        // disarm the motors
        if (arming_counter == DISARM_DELAY && motors->armed()) {
            init_disarm_motors();
        }

    // Yaw is centered so reset arming counter
    }else{
        arming_counter = 0;
    }
}

// auto_disarm_check - disarms the copter if it has been sitting on the ground in manual mode with throttle low for at least 15 seconds
void Copter::auto_disarm_check()
{
    uint32_t tnow_ms = millis();
    uint32_t disarm_delay_ms = 1000*constrain_int16(g.disarm_delay, 0, 127);

    // exit immediately if we are already disarmed, or if auto
    // disarming is disabled
    if (!motors->armed() || disarm_delay_ms == 0 || control_mode == THROW) {
        auto_disarm_begin = tnow_ms;
        return;
    }

#if FRAME_CONFIG == HELI_FRAME
    // if the rotor is still spinning, don't initiate auto disarm
    if (motors->rotor_speed_above_critical()) {
        auto_disarm_begin = tnow_ms;
        return;
    }
#endif

    // always allow auto disarm if using interlock switch or motors are Emergency Stopped
    if ((ap.using_interlock && !motors->get_interlock()) || ap.motor_emergency_stop) {
#if FRAME_CONFIG != HELI_FRAME
        // use a shorter delay if using throttle interlock switch or Emergency Stop, because it is less
        // obvious the copter is armed as the motors will not be spinning
        disarm_delay_ms /= 2;
#endif
    } else {
        bool sprung_throttle_stick = (g.throttle_behavior & THR_BEHAVE_FEEDBACK_FROM_MID_STICK) != 0;
        bool thr_low;
        if (flightmode->has_manual_throttle() || !sprung_throttle_stick) {
            thr_low = ap.throttle_zero;
        } else {
            float deadband_top = get_throttle_mid() + g.throttle_deadzone;
            thr_low = channel_throttle->get_control_in() <= deadband_top;
        }

        if (!thr_low || !ap.land_complete) {
            // reset timer
            auto_disarm_begin = tnow_ms;
        }
    }

    // disarm once timer expires
    if ((tnow_ms-auto_disarm_begin) >= disarm_delay_ms) {
        init_disarm_motors();
        auto_disarm_begin = tnow_ms;
    }
}

// init_arm_motors - performs arming process including initialisation of barometer and gyros
//  returns false if arming failed because of pre-arm checks, arming checks or a gyro calibration failure
bool Copter::init_arm_motors(const AP_Arming::ArmingMethod method, const bool do_arming_checks)
{
    static bool in_arm_motors = false;

    // exit immediately if already in this function
    if (in_arm_motors) {
        return false;
    }
    in_arm_motors = true;

    // return true if already armed
    if (motors->armed()) {
        in_arm_motors = false;
        return true;
    }

    // run pre-arm-checks and display failures
    if (do_arming_checks && !arming.all_checks_passing(method)) {
        AP_Notify::events.arming_failed = true;
        in_arm_motors = false;
        return false;
    }

    // let dataflash know that we're armed (it may open logs e.g.)
    DataFlash_Class::instance()->set_vehicle_armed(true);

    // disable cpu failsafe because initialising everything takes a while
    failsafe_disable();

    // notify that arming will occur (we do this early to give plenty of warning)
    AP_Notify::flags.armed = true;
    // call notify update a few times to ensure the message gets out
    for (uint8_t i=0; i<=10; i++) {
        notify.update();
    }

#if HIL_MODE != HIL_MODE_DISABLED || CONFIG_HAL_BOARD == HAL_BOARD_SITL
    gcs().send_text(MAV_SEVERITY_INFO, "Arming motors");
#endif

    // Remember Orientation
    // --------------------
    init_simple_bearing();

    initial_armed_bearing = ahrs.yaw_sensor;

    if (!ahrs.home_is_set()) {
        // Reset EKF altitude if home hasn't been set yet (we use EKF altitude as substitute for alt above home)
        ahrs.resetHeightDatum();
        Log_Write_Event(DATA_EKF_ALT_RESET);

        // we have reset height, so arming height is zero
        arming_altitude_m = 0;        
    } else if (!ahrs.home_is_locked()) {
        // Reset home position if it has already been set before (but not locked)
        set_home_to_current_location(false);

        // remember the height when we armed
        arming_altitude_m = inertial_nav.get_altitude() * 0.01;
    }
    update_super_simple_bearing(false);

    // Reset SmartRTL return location. If activated, SmartRTL will ultimately try to land at this point
#if MODE_SMARTRTL_ENABLED == ENABLED
    g2.smart_rtl.set_home(position_ok());
#endif

    // enable gps velocity based centrefugal force compensation
    ahrs.set_correct_centrifugal(true);
    hal.util->set_soft_armed(true);

#if SPRAYER_ENABLED == ENABLED
    // turn off sprayer's test if on
    sprayer.test_pump(false);
#endif

    // enable output to motors
    enable_motor_output();

    // finally actually arm the motors
    motors->armed(true);

    // log arming to dataflash
    Log_Write_Event(DATA_ARMED);

    // log flight mode in case it was changed while vehicle was disarmed
    DataFlash.Log_Write_Mode(control_mode, control_mode_reason);

    // reenable failsafe
    failsafe_enable();

    // perf monitor ignores delay due to arming
    scheduler.perf_info.ignore_this_loop();

    // flag exiting this function
    in_arm_motors = false;

    // Log time stamp of arming event
    arm_time_ms = millis();

    // Start the arming delay
    ap.in_arming_delay = true;

    // assumed armed without a arming, switch. Overridden in switches.cpp
    ap.armed_with_switch = false;
    
    // return success
    return true;
}

// init_disarm_motors - disarm motors
void Copter::init_disarm_motors()
{
    // return immediately if we are already disarmed
    if (!motors->armed()) {
        return;
    }

#if HIL_MODE != HIL_MODE_DISABLED || CONFIG_HAL_BOARD == HAL_BOARD_SITL
    gcs().send_text(MAV_SEVERITY_INFO, "Disarming motors");
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
    mode_autotune.save_tuning_gains();
#endif

    // we are not in the air
    set_land_complete(true);
    set_land_complete_maybe(true);

    // log disarm to the dataflash
    Log_Write_Event(DATA_DISARMED);

    // send disarm command to motors
    motors->armed(false);

#if MODE_AUTO_ENABLED == ENABLED
    // reset the mission
    mission.reset();
#endif

    DataFlash_Class::instance()->set_vehicle_armed(false);

    // disable gps velocity based centrefugal force compensation
    ahrs.set_correct_centrifugal(false);
    hal.util->set_soft_armed(false);

    ap.in_arming_delay = false;
}

// motors_output - send output to motors library which will adjust and send to ESCs and servos
void Copter::motors_output()
{
#if ADVANCED_FAILSAFE == ENABLED
    // this is to allow the failsafe module to deliberately crash
    // the vehicle. Only used in extreme circumstances to meet the
    // OBC rules
    if (g2.afs.should_crash_vehicle()) {
        g2.afs.terminate_vehicle();
        return;
    }
#endif

    // Update arming delay state
    if (ap.in_arming_delay && (!motors->armed() || millis()-arm_time_ms > ARMING_DELAY_SEC*1.0e3f || control_mode == THROW)) {
        ap.in_arming_delay = false;
    }

    // output any servo channels
    SRV_Channels::calc_pwm();

    // cork now, so that all channel outputs happen at once
    SRV_Channels::cork();

    // update output on any aux channels, for manual passthru
    SRV_Channels::output_ch_all();

    // check if we are performing the motor test
    if (ap.motor_test) {
        motor_test_output();
    } else {
        bool interlock = motors->armed() && !ap.in_arming_delay && (!ap.using_interlock || ap.motor_interlock_switch) && !ap.motor_emergency_stop;
        if (!motors->get_interlock() && interlock) {
            motors->set_interlock(true);
            Log_Write_Event(DATA_MOTORS_INTERLOCK_ENABLED);
        } else if (motors->get_interlock() && !interlock) {
            motors->set_interlock(false);
            Log_Write_Event(DATA_MOTORS_INTERLOCK_DISABLED);
        }

        // send output signals to motors
        motors->output();
    }

    // push all channels
    SRV_Channels::push();
}

// check for pilot stick input to trigger lost vehicle alarm
void Copter::lost_vehicle_check()
{
    static uint8_t soundalarm_counter;

    // disable if aux switch is setup to vehicle alarm as the two could interfere
    if (check_if_auxsw_mode_used(AUXSW_LOST_COPTER_SOUND)) {
        return;
    }

    // ensure throttle is down, motors not armed, pitch and roll rc at max. Note: rc1=roll rc2=pitch
    if (ap.throttle_zero && !motors->armed() && (channel_roll->get_control_in() > 4000) && (channel_pitch->get_control_in() > 4000)) {
        if (soundalarm_counter >= LOST_VEHICLE_DELAY) {
            if (AP_Notify::flags.vehicle_lost == false) {
                AP_Notify::flags.vehicle_lost = true;
                gcs().send_text(MAV_SEVERITY_NOTICE,"Locate Copter alarm");
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
