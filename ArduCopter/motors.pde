/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

// 10 = 1 second
#define ARM_DELAY 20
#define DISARM_DELAY 20
#define AUTO_TRIM_DELAY 100


// called at 10hz
static void arm_motors()
{
    static int16_t arming_counter;

    // don't allow arming/disarming in anything but manual
    if (g.rc_3.control_in > 0) {
        arming_counter = 0;
        return;
    }

    if ((control_mode > ACRO) && ((control_mode != TOY_A) && (control_mode != TOY_M))) {
        arming_counter = 0;
        return;
    }
	
	#if FRAME_CONFIG == HELI_FRAME
	if ((motors.rsc_mode > 0) && (g.rc_8.control_in >= 10)){
		arming_counter = 0;
		return;
	}
	#endif  // HELI_FRAME

#if TOY_EDF == ENABLED
    int16_t tmp = g.rc_1.control_in;
#else
    int16_t tmp = g.rc_4.control_in;
#endif

    // full right
    if (tmp > 4000) {

        // increase the arming counter to a maximum of 1 beyond the auto trim counter
        if( arming_counter <= AUTO_TRIM_DELAY ) {
            arming_counter++;
        }

        // arm the motors and configure for flight
        if (arming_counter == ARM_DELAY && !motors.armed()) {
////////////////////////////////////////////////////////////////////////////////
// Experimental AP_Limits library - set constraints, limits, fences, minima, maxima on various parameters
////////////////////////////////////////////////////////////////////////////////
#if AP_LIMITS == ENABLED
            if (limits.enabled() && limits.required()) {
                gcs_send_text_P(SEVERITY_LOW, PSTR("Limits - Running pre-arm checks"));

                // check only pre-arm required modules
                if (limits.check_required()) {
                    gcs_send_text_P(SEVERITY_LOW, PSTR("ARMING PREVENTED - Limit Breached"));
                    limits.set_state(LIMITS_TRIGGERED);
                    gcs_send_message(MSG_LIMITS_STATUS);

                    arming_counter++;                                 // restart timer by cycling
                }else{
                    init_arm_motors();
                }
            }else{
                init_arm_motors();
            }
#else  // without AP_LIMITS, just arm motors
            init_arm_motors();
#endif //AP_LIMITS_ENABLED
        }

        // arm the motors and configure for flight
        if (arming_counter == AUTO_TRIM_DELAY && motors.armed()) {
            auto_trim_counter = 250;
        }

    // full left
    }else if (tmp < -4000) {

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


static void init_arm_motors()
{
	// arming marker
    // Flag used to track if we have armed the motors the first time.
    // This is used to decide if we should run the ground_start routine
    // which calibrates the IMU
    static bool did_ground_start = false;

    // disable failsafe because initialising everything takes a while
    failsafe_disable();

    //cliSerial->printf("\nARM\n");
#if HIL_MODE != HIL_MODE_DISABLED || CONFIG_HAL_BOARD == HAL_BOARD_AVR_SITL
    gcs_send_text_P(SEVERITY_HIGH, PSTR("ARMING MOTORS"));
#endif

    // we don't want writes to the serial port to cause us to pause
    // mid-flight, so set the serial ports non-blocking once we arm
    // the motors
    hal.uartA->set_blocking_writes(false);
    if (gcs3.initialised) {
        hal.uartC->set_blocking_writes(false);
    }

#if COPTER_LEDS == ENABLED
    if ( bitRead(g.copter_leds_mode, 3) ) {
        piezo_beep();
        delay(50);
        piezo_beep();
    }
#endif

    // Remember Orientation
    // --------------------
    init_simple_bearing();

    // Reset home position
    // -------------------
    if(ap.home_is_set)
        init_home();

    // all I terms are invalid
    // -----------------------
    reset_I_all();

    if(did_ground_start == false) {
        did_ground_start = true;
        startup_ground();
    }

#if HIL_MODE != HIL_MODE_ATTITUDE
    // read Baro pressure at ground -
    // this resets Baro for more accuracy
    //-----------------------------------
    init_barometer();
#endif

    // temp hack
    ap_system.motor_light = true;
    digitalWrite(A_LED_PIN, LED_ON);

    // go back to normal AHRS gains
    ahrs.set_fast_gains(false);
#if SECONDARY_DMP_ENABLED == ENABLED
    ahrs2.set_fast_gains(false);
#endif

    // finally actually arm the motors
    motors.armed(true);
    set_armed(true);

    // reenable failsafe
    failsafe_enable();
}


static void init_disarm_motors()
{
#if HIL_MODE != HIL_MODE_DISABLED || CONFIG_HAL_BOARD == HAL_BOARD_AVR_SITL
    gcs_send_text_P(SEVERITY_HIGH, PSTR("DISARMING MOTORS"));
#endif

    motors.armed(false);
    set_armed(false);

    motors.auto_armed(false);
    set_auto_armed(false);

    compass.save_offsets();

    g.throttle_cruise.save();

#if INERTIAL_NAV_XY == ENABLED || INERTIAL_NAV_Z == ENABLED
    inertial_nav.save_params();
#endif

    // we are not in the air
    set_takeoff_complete(false);

#if COPTER_LEDS == ENABLED
    if ( bitRead(g.copter_leds_mode, 3) ) {
        piezo_beep();
    }
#endif

    // setup fast AHRS gains to get right attitude
    ahrs.set_fast_gains(true);
#if SECONDARY_DMP_ENABLED == ENABLED
    ahrs2.set_fast_gains(true);
#endif
}

/*****************************************
* Set the flight control servos based on the current calculated values
*****************************************/
static void
set_servos_4()
{
#if FRAME_CONFIG == TRI_FRAME
    // To-Do: implement improved stability patch for tri so that we do not need to limit throttle input to motors
    g.rc_3.servo_out = min(g.rc_3.servo_out, 800);
#endif
    motors.output();
}

