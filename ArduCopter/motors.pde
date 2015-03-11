/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#define ARM_DELAY               20  // called at 10hz so 2 seconds
#define DISARM_DELAY            20  // called at 10hz so 2 seconds
#define AUTO_TRIM_DELAY         100 // called at 10hz so 10 seconds
#define AUTO_DISARMING_DELAY    15  // called at 1hz so 15 seconds

static uint8_t auto_disarming_counter;

// arm_motors_check - checks for pilot input to arm or disarm the copter
// called at 10hz
static void arm_motors_check()
{
    static int16_t arming_counter;

    // ensure throttle is down
    if (g.rc_3.control_in > 0) {
        arming_counter = 0;
        return;
    }

    int16_t tmp = g.rc_4.control_in;

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
            auto_disarming_counter = 0;
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
    }
}

// auto_disarm_check - disarms the copter if it has been sitting on the ground in manual mode with throttle low for at least 15 seconds
// called at 1hz
static void auto_disarm_check()
{
    // exit immediately if we are already disarmed or throttle is not zero
    if (!motors.armed() || !ap.throttle_zero) {
        auto_disarming_counter = 0;
        return;
    }

    // allow auto disarm in manual flight modes or Loiter/AltHold if we're landed
    if (mode_has_manual_throttle(control_mode) || ap.land_complete) {
        auto_disarming_counter++;

        if(auto_disarming_counter >= AUTO_DISARMING_DELAY) {
            init_disarm_motors();
            auto_disarming_counter = 0;
        }
    }else{
        auto_disarming_counter = 0;
    }
}

// init_arm_motors - performs arming process including initialisation of barometer and gyros
//  returns false if arming failed because of pre-arm checks, arming checks or a gyro calibration failure
static bool init_arm_motors(bool arming_from_gcs)
{
	// arming marker
    // Flag used to track if we have armed the motors the first time.
    // This is used to decide if we should run the ground_start routine
    // which calibrates the IMU
    static bool did_ground_start = false;
    static bool in_arm_motors = false;

    // exit immediately if already in this function
    if (in_arm_motors) {
        return false;
    }
    in_arm_motors = true;

    // run pre-arm-checks and display failures
    if(!pre_arm_checks(true) || !arm_checks(true, arming_from_gcs)) {
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

#if HIL_MODE != HIL_MODE_DISABLED || CONFIG_HAL_BOARD == HAL_BOARD_AVR_SITL
    gcs_send_text_P(SEVERITY_HIGH, PSTR("ARMING MOTORS"));
#endif

    // Remember Orientation
    // --------------------
    init_simple_bearing();

    initial_armed_bearing = ahrs.yaw_sensor;

    // Reset home position if it has already been set before (but not locked)
    if (ap.home_state == HOME_SET_NOT_LOCKED) {
        set_home_to_current_location();
    }
    calc_distance_and_bearing();

    if(did_ground_start == false) {
        startup_ground(true);
        // final check that gyros calibrated successfully
        if (((g.arming_check == ARMING_CHECK_ALL) || (g.arming_check & ARMING_CHECK_INS)) && !ins.gyro_calibrated_ok_all()) {
            gcs_send_text_P(SEVERITY_HIGH,PSTR("Arm: Gyro calibration failed"));
            AP_Notify::flags.armed = false;
            failsafe_enable();
            in_arm_motors = false;
            return false;
        }
        did_ground_start = true;
    }

    // fast baro calibration to reset ground pressure
    init_barometer(false);

    // go back to normal AHRS gains
    ahrs.set_fast_gains(false);

    // enable gps velocity based centrefugal force compensation
    ahrs.set_correct_centrifugal(true);
    hal.util->set_soft_armed(true);

    // set hover throttle
    motors.set_hover_throttle(g.throttle_mid);

#if SPRAYER == ENABLED
    // turn off sprayer's test if on
    sprayer.test_pump(false);
#endif

    // short delay to allow reading of rc inputs
    delay(30);

    // enable output to motors
    output_min();

    // finally actually arm the motors
    motors.armed(true);

    // log arming to dataflash
    Log_Write_Event(DATA_ARMED);

    // log flight mode in case it was changed while vehicle was disarmed
    DataFlash.Log_Write_Mode(control_mode);

    // reenable failsafe
    failsafe_enable();

    // perf monitor ignores delay due to arming
    perf_ignore_this_loop();

    // flag exiting this function
    in_arm_motors = false;

    // return success
    return true;
}

// perform pre-arm checks and set ap.pre_arm_check flag
//  return true if the checks pass successfully
static bool pre_arm_checks(bool display_failure)
{
    // exit immediately if already armed
    if (motors.armed()) {
        return true;
    }

    // exit immediately if we've already successfully performed the pre-arm check
    if (ap.pre_arm_check) {
        // run gps checks because results may change and affect LED colour
        // no need to display failures because arm_checks will do that if the pilot tries to arm
        pre_arm_gps_checks(false);
        return true;
    }

    // succeed if pre arm checks are disabled
    if(g.arming_check == ARMING_CHECK_NONE) {
        set_pre_arm_check(true);
        set_pre_arm_rc_check(true);
        return true;
    }

    // pre-arm rc checks a prerequisite
    pre_arm_rc_checks();
    if(!ap.pre_arm_rc_check) {
        if (display_failure) {
            gcs_send_text_P(SEVERITY_HIGH,PSTR("PreArm: RC not calibrated"));
        }
        return false;
    }

    // check Baro
    if ((g.arming_check == ARMING_CHECK_ALL) || (g.arming_check & ARMING_CHECK_BARO)) {
        // barometer health check
        if(!barometer.all_healthy()) {
            if (display_failure) {
                gcs_send_text_P(SEVERITY_HIGH,PSTR("PreArm: Barometer not healthy"));
            }
            return false;
        }
        // check Baro & inav alt are within 1m
        if(fabs(inertial_nav.get_altitude() - baro_alt) > PREARM_MAX_ALT_DISPARITY_CM) {
            if (display_failure) {
                gcs_send_text_P(SEVERITY_HIGH,PSTR("PreArm: Altitude disparity"));
            }
            return false;
        }
    }

    // check Compass
    if ((g.arming_check == ARMING_CHECK_ALL) || (g.arming_check & ARMING_CHECK_COMPASS)) {
        // check the primary compass is healthy
        if(!compass.healthy(0)) {
            if (display_failure) {
                gcs_send_text_P(SEVERITY_HIGH,PSTR("PreArm: Compass not healthy"));
            }
            return false;
        }

        // check compass learning is on or offsets have been set
        if(!compass.configured()) {
            if (display_failure) {
                gcs_send_text_P(SEVERITY_HIGH,PSTR("PreArm: Compass not calibrated"));
            }
            return false;
        }

        // check for unreasonable compass offsets
        Vector3f offsets = compass.get_offsets();
        if(offsets.length() > COMPASS_OFFSETS_MAX) {
            if (display_failure) {
                gcs_send_text_P(SEVERITY_HIGH,PSTR("PreArm: Compass offsets too high"));
            }
            return false;
        }

        // check for unreasonable mag field length
        float mag_field = compass.get_field().length();
        if (mag_field > COMPASS_MAGFIELD_EXPECTED*1.65 || mag_field < COMPASS_MAGFIELD_EXPECTED*0.35) {
            if (display_failure) {
                gcs_send_text_P(SEVERITY_HIGH,PSTR("PreArm: Check mag field"));
            }
            return false;
        }

#if COMPASS_MAX_INSTANCES > 1
        // check all compasses point in roughly same direction
        if (compass.get_count() > 1) {
            Vector3f prime_mag_vec = compass.get_field();
            prime_mag_vec.normalize();
            for(uint8_t i=0; i<compass.get_count(); i++) {
                // get next compass
                Vector3f mag_vec = compass.get_field(i);
                mag_vec.normalize();
                Vector3f vec_diff = mag_vec - prime_mag_vec;
                if (vec_diff.length() > COMPASS_ACCEPTABLE_VECTOR_DIFF) {
                    if (display_failure) {
                        gcs_send_text_P(SEVERITY_HIGH,PSTR("PreArm: inconsistent compasses"));
                    }
                    return false;
                }
            }
        }
#endif

    }

    // check GPS
    if (!pre_arm_gps_checks(display_failure)) {
        return false;
    }

#if AC_FENCE == ENABLED
    // check fence is initialised
    if(!fence.pre_arm_check()) {
        if (display_failure) {
            gcs_send_text_P(SEVERITY_HIGH,PSTR("PreArm: check fence"));
        }
        return false;
    }
#endif

    // check INS
    if ((g.arming_check == ARMING_CHECK_ALL) || (g.arming_check & ARMING_CHECK_INS)) {
        // check accelerometers have been calibrated
        if(!ins.calibrated()) {
            if (display_failure) {
                gcs_send_text_P(SEVERITY_HIGH,PSTR("PreArm: INS not calibrated"));
            }
            return false;
        }

        // check accels are healthy
        if(!ins.get_accel_health_all()) {
            if (display_failure) {
                gcs_send_text_P(SEVERITY_HIGH,PSTR("PreArm: Accelerometers not healthy"));
            }
            return false;
        }

#if INS_MAX_INSTANCES > 1
        // check all accelerometers point in roughly same direction
        if (ins.get_accel_count() > 1) {
            const Vector3f &prime_accel_vec = ins.get_accel();
            for(uint8_t i=0; i<ins.get_accel_count(); i++) {
                // get next accel vector
                const Vector3f &accel_vec = ins.get_accel(i);
                Vector3f vec_diff = accel_vec - prime_accel_vec;
                if (vec_diff.length() > PREARM_MAX_ACCEL_VECTOR_DIFF) {
                    if (display_failure) {
                        gcs_send_text_P(SEVERITY_HIGH,PSTR("PreArm: inconsistent Accelerometers"));
                    }
                    return false;
                }
            }
        }
#endif

        // check gyros are healthy
        if(!ins.get_gyro_health_all()) {
            if (display_failure) {
                gcs_send_text_P(SEVERITY_HIGH,PSTR("PreArm: Gyros not healthy"));
            }
            return false;
        }

#if INS_MAX_INSTANCES > 1
        // check all gyros are consistent
        if (ins.get_gyro_count() > 1) {
            for(uint8_t i=0; i<ins.get_gyro_count(); i++) {
                // get rotation rate difference between gyro #i and primary gyro
                Vector3f vec_diff = ins.get_gyro(i) - ins.get_gyro();
                if (vec_diff.length() > PREARM_MAX_GYRO_VECTOR_DIFF) {
                    if (display_failure) {
                        gcs_send_text_P(SEVERITY_HIGH,PSTR("PreArm: inconsistent Gyros"));
                    }
                    return false;
                }
            }
        }
#endif
    }
#if CONFIG_HAL_BOARD != HAL_BOARD_VRBRAIN
#ifndef CONFIG_ARCH_BOARD_PX4FMU_V1
    // check board voltage
    if ((g.arming_check == ARMING_CHECK_ALL) || (g.arming_check & ARMING_CHECK_VOLTAGE)) {
        if(hal.analogin->board_voltage() < BOARD_VOLTAGE_MIN || hal.analogin->board_voltage() > BOARD_VOLTAGE_MAX) {
            if (display_failure) {
                gcs_send_text_P(SEVERITY_HIGH,PSTR("PreArm: Check Board Voltage"));
            }
            return false;
        }
    }
#endif
#endif

    // check various parameter values
    if ((g.arming_check == ARMING_CHECK_ALL) || (g.arming_check & ARMING_CHECK_PARAMETERS)) {

        // ensure ch7 and ch8 have different functions
        if (check_duplicate_auxsw()) {
            if (display_failure) {
                gcs_send_text_P(SEVERITY_HIGH,PSTR("PreArm: Duplicate Aux Switch Options"));
            }
            return false;
        }

        // failsafe parameter checks
        if (g.failsafe_throttle) {
            // check throttle min is above throttle failsafe trigger and that the trigger is above ppm encoder's loss-of-signal value of 900
            if (g.rc_3.radio_min <= g.failsafe_throttle_value+10 || g.failsafe_throttle_value < 910) {
                if (display_failure) {
                    gcs_send_text_P(SEVERITY_HIGH,PSTR("PreArm: Check FS_THR_VALUE"));
                }
                return false;
            }
        }

        // lean angle parameter check
        if (aparm.angle_max < 1000 || aparm.angle_max > 8000) {
            if (display_failure) {
                gcs_send_text_P(SEVERITY_HIGH,PSTR("PreArm: Check ANGLE_MAX"));
            }
            return false;
        }

        // acro balance parameter check
        if ((g.acro_balance_roll > g.p_stabilize_roll.kP()) || (g.acro_balance_pitch > g.p_stabilize_pitch.kP())) {
            if (display_failure) {
                gcs_send_text_P(SEVERITY_HIGH,PSTR("PreArm: ACRO_BAL_ROLL/PITCH"));
            }
            return false;
        }
    }

    // if we've gotten this far then pre arm checks have completed
    set_pre_arm_check(true);
    return true;
}

// perform pre_arm_rc_checks checks and set ap.pre_arm_rc_check flag
static void pre_arm_rc_checks()
{
    // exit immediately if we've already successfully performed the pre-arm rc check
    if( ap.pre_arm_rc_check ) {
        return;
    }

    // set rc-checks to success if RC checks are disabled
    if ((g.arming_check != ARMING_CHECK_ALL) && !(g.arming_check & ARMING_CHECK_RC)) {
        set_pre_arm_rc_check(true);
        return;
    }

    // check if radio has been calibrated
    if(!g.rc_3.radio_min.load() && !g.rc_3.radio_max.load()) {
        return;
    }

    // check channels 1 & 2 have min <= 1300 and max >= 1700
    if (g.rc_1.radio_min > 1300 || g.rc_1.radio_max < 1700 || g.rc_2.radio_min > 1300 || g.rc_2.radio_max < 1700) {
        return;
    }

    // check channels 3 & 4 have min <= 1300 and max >= 1700
    if (g.rc_3.radio_min > 1300 || g.rc_3.radio_max < 1700 || g.rc_4.radio_min > 1300 || g.rc_4.radio_max < 1700) {
        return;
    }

    // check channels 1 & 2 have trim >= 1300 and <= 1700
    if (g.rc_1.radio_trim < 1300 || g.rc_1.radio_trim > 1700 || g.rc_2.radio_trim < 1300 || g.rc_2.radio_trim > 1700) {
        return;
    }

    // check channel 4 has trim >= 1300 and <= 1700
    if (g.rc_4.radio_trim < 1300 || g.rc_4.radio_trim > 1700) {
        return;
    }

    // if we've gotten this far rc is ok
    set_pre_arm_rc_check(true);
}

// performs pre_arm gps related checks and returns true if passed
static bool pre_arm_gps_checks(bool display_failure)
{
    // return true immediately if gps check is disabled
    if (!(g.arming_check == ARMING_CHECK_ALL || g.arming_check & ARMING_CHECK_GPS)) {
        AP_Notify::flags.pre_arm_gps_check = true;
        return true;
    }

    // check if flight mode requires GPS
    bool gps_required = mode_requires_GPS(control_mode);

#if AC_FENCE == ENABLED
    // if circular fence is enabled we need GPS
    if ((fence.get_enabled_fences() & AC_FENCE_TYPE_CIRCLE) != 0) {
        gps_required = true;
    }
#endif

    // return true if GPS is not required
    if (!gps_required) {
        AP_Notify::flags.pre_arm_gps_check = true;
        return true;
    }

    // ensure GPS is ok
    if (!position_ok()) {
        if (display_failure) {
            gcs_send_text_P(SEVERITY_HIGH,PSTR("PreArm: Need 3D Fix"));
        }
        AP_Notify::flags.pre_arm_gps_check = false;
        return false;
    }

    // check speed is below 50cm/s
    float speed_cms = inertial_nav.get_velocity().length();     // speed according to inertial nav in cm/s
    if (speed_cms == 0 || speed_cms > PREARM_MAX_VELOCITY_CMS) {
        if (display_failure) {
            gcs_send_text_P(SEVERITY_HIGH,PSTR("PreArm: Bad Velocity"));
        }
        AP_Notify::flags.pre_arm_gps_check = false;
        return false;
    }

    // check home and EKF origin are not too far
    if (far_from_EKF_origin(ahrs.get_home())) {
        if (display_failure) {
            gcs_send_text_P(SEVERITY_HIGH,PSTR("PreArm: EKF-home variance"));
        }
        AP_Notify::flags.pre_arm_gps_check = false;
        return false;
    }

    // warn about hdop separately - to prevent user confusion with no gps lock
    if (gps.get_hdop() > g.gps_hdop_good) {
        if (display_failure) {
            gcs_send_text_P(SEVERITY_HIGH,PSTR("PreArm: High GPS HDOP"));
        }
        AP_Notify::flags.pre_arm_gps_check = false;
        return false;
    }

    // if we got here all must be ok
    AP_Notify::flags.pre_arm_gps_check = true;
    return true;
}

// arm_checks - perform final checks before arming
//  always called just before arming.  Return true if ok to arm
//  has side-effect that logging is started
static bool arm_checks(bool display_failure, bool arming_from_gcs)
{
#if LOGGING_ENABLED == ENABLED
    // start dataflash
    start_logging();
#endif

    // always check if the current mode allows arming
    if (!mode_allows_arming(control_mode, arming_from_gcs)) {
        if (display_failure) {
            gcs_send_text_P(SEVERITY_HIGH,PSTR("Arm: Mode not armable"));
        }
        return false;
    }

    // always check if rotor is spinning on heli
    #if FRAME_CONFIG == HELI_FRAME
    // heli specific arming check
    if (!motors.allow_arming()){
        if (display_failure) {
            gcs_send_text_P(SEVERITY_HIGH,PSTR("Arm: Rotor not spinning"));
        }
        return false;
    }
    #endif  // HELI_FRAME

    // succeed if arming checks are disabled
    if (g.arming_check == ARMING_CHECK_NONE) {
        return true;
    }

    // check Baro & inav alt are within 1m
    if ((g.arming_check == ARMING_CHECK_ALL) || (g.arming_check & ARMING_CHECK_BARO)) {
        if(fabs(inertial_nav.get_altitude() - baro_alt) > PREARM_MAX_ALT_DISPARITY_CM) {
            if (display_failure) {
                gcs_send_text_P(SEVERITY_HIGH,PSTR("Arm: Altitude disparity"));
            }
            return false;
        }
    }

    // check gps
    if (!pre_arm_gps_checks(display_failure)) {
        return false;
    }

    // check lean angle
    if ((g.arming_check == ARMING_CHECK_ALL) || (g.arming_check & ARMING_CHECK_INS)) {
        if (labs(ahrs.roll_sensor) > aparm.angle_max || labs(ahrs.pitch_sensor) > aparm.angle_max) {
            if (display_failure) {
                gcs_send_text_P(SEVERITY_HIGH,PSTR("Arm: Leaning"));
            }
            return false;
        }
    }

    // check throttle
    if ((g.arming_check == ARMING_CHECK_ALL) || (g.arming_check & ARMING_CHECK_RC)) {
        // check throttle is not too low - must be above failsafe throttle
        if (g.failsafe_throttle != FS_THR_DISABLED && g.rc_3.radio_in < g.failsafe_throttle_value) {
            if (display_failure) {
                gcs_send_text_P(SEVERITY_HIGH,PSTR("Arm: Throttle below Failsafe"));
            }
            return false;
        }

        // check throttle is not too high - skips checks if arming from GCS in Guided
        if (!(arming_from_gcs && control_mode == GUIDED)) {
            // above top of deadband is too always high
            if (g.rc_3.control_in > (g.rc_3.get_control_mid() + g.throttle_deadzone)) {
                if (display_failure) {
                    gcs_send_text_P(SEVERITY_HIGH,PSTR("Arm: Throttle too high"));
                }
                return false;
            }
            // in manual modes throttle must be at zero
            if ((mode_has_manual_throttle(control_mode) || control_mode == DRIFT) && g.rc_3.control_in > 0) {
                if (display_failure) {
                    gcs_send_text_P(SEVERITY_HIGH,PSTR("Arm: Throttle too high"));
                }
                return false;
            }
        }
    }

    // check if safety switch has been pushed
    if (hal.util->safety_switch_state() == AP_HAL::Util::SAFETY_DISARMED) {
        if (display_failure) {
            gcs_send_text_P(SEVERITY_HIGH,PSTR("Arm: Safety Switch"));
        }
        return false;
    }

    // if we've gotten this far all is ok
    return true;
}

// init_disarm_motors - disarm motors
static void init_disarm_motors()
{
    // return immediately if we are already disarmed
    if (!motors.armed()) {
        return;
    }

#if HIL_MODE != HIL_MODE_DISABLED || CONFIG_HAL_BOARD == HAL_BOARD_AVR_SITL
    gcs_send_text_P(SEVERITY_HIGH, PSTR("DISARMING MOTORS"));
#endif

    motors.armed(false);

    // save offsets if automatic offset learning is on
    if (compass.learn_offsets_enabled()) {
        compass.save_offsets();
    }

#if AUTOTUNE_ENABLED == ENABLED
    // save auto tuned parameters
    autotune_save_tuning_gains();
#endif

    // we are not in the air
    set_land_complete(true);
    set_land_complete_maybe(true);

    // reset the mission
    mission.reset();

    // setup fast AHRS gains to get right attitude
    ahrs.set_fast_gains(true);

    // log disarm to the dataflash
    Log_Write_Event(DATA_DISARMED);

    // suspend logging
    if (!(g.log_bitmask & MASK_LOG_WHEN_DISARMED)) {
        DataFlash.EnableWrites(false);
    }

    // disable gps velocity based centrefugal force compensation
    ahrs.set_correct_centrifugal(false);
    hal.util->set_soft_armed(false);
}

// motors_output - send output to motors library which will adjust and send to ESCs and servos
static void motors_output()
{
    // check if we are performing the motor test
    if (ap.motor_test) {
        motor_test_output();
    } else {
        motors.output();
    }
}
