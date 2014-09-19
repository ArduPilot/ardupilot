// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/*****************************************************************************
*  esc_calibration.pde : functions to check and perform ESC calibration
*****************************************************************************/

#define ESC_CALIBRATION_HIGH_THROTTLE   950

// enum for ESC CALIBRATION
enum ESCCalibrationModes {
    ESCCAL_NONE = 0,
    ESCCAL_PASSTHROUGH_IF_THROTTLE_HIGH = 1,
    ESCCAL_PASSTHROUGH_ALWAYS = 2,
    ESCCAL_AUTO = 3
};

// check if we should enter esc calibration mode
static void esc_calibration_startup_check()
{
    // exit immediately if pre-arm rc checks fail
    pre_arm_rc_checks();
    if (!ap.pre_arm_rc_check) {
        // clear esc flag for next time
        if (g.esc_calibrate != ESCCAL_NONE) {
            g.esc_calibrate.set_and_save(ESCCAL_NONE);
        }
        return;
    }

    // check ESC parameter
    switch (g.esc_calibrate) {
        case ESCCAL_NONE:
            // check if throttle is high
            if (g.rc_3.control_in >= ESC_CALIBRATION_HIGH_THROTTLE) {
                // we will enter esc_calibrate mode on next reboot
                g.esc_calibrate.set_and_save(ESCCAL_PASSTHROUGH_IF_THROTTLE_HIGH);
                // send message to gcs
                gcs_send_text_P(SEVERITY_HIGH,PSTR("ESC Cal: restart board"));
                // turn on esc calibration notification
                AP_Notify::flags.esc_calibration = true;
                // block until we restart
                while(1) { delay(5); }
            }
            break;
        case ESCCAL_PASSTHROUGH_IF_THROTTLE_HIGH:
            // check if throttle is high
            if (g.rc_3.control_in >= ESC_CALIBRATION_HIGH_THROTTLE) {
                // pass through pilot throttle to escs
                esc_calibration_passthrough();
            }
            break;
        case ESCCAL_PASSTHROUGH_ALWAYS:
            // pass through pilot throttle to escs
            esc_calibration_passthrough();
            break;
        case ESCCAL_AUTO:
            // perform automatic ESC calibration
            esc_calibration_auto();
            break;
        default:
            // do nothing
            break;
    }
    // clear esc flag for next time
    g.esc_calibrate.set_and_save(ESCCAL_NONE);
}

// esc_calibration_passthrough - pass through pilot throttle to escs
static void esc_calibration_passthrough()
{
    // clear esc flag for next time
    g.esc_calibrate.set_and_save(ESCCAL_NONE);

    // reduce update rate to motors to 50Hz
    motors.set_update_rate(50);

    // send message to GCS
    gcs_send_text_P(SEVERITY_HIGH,PSTR("ESC Cal: passing pilot thr to ESCs"));

    while(1) {
        // arm motors
        motors.armed(true);
        motors.enable();

        // flash LEDS
        AP_Notify::flags.esc_calibration = true;

        // read pilot input
        read_radio();
        delay(10);

        // pass through to motors
        motors.throttle_pass_through(g.rc_3.radio_in);
    }
}

// esc_calibration_auto - calibrate the ESCs automatically using a timer and no pilot input
static void esc_calibration_auto()
{
    bool printed_msg = false;

    // reduce update rate to motors to 50Hz
    motors.set_update_rate(50);

    // send message to GCS
    gcs_send_text_P(SEVERITY_HIGH,PSTR("ESC Cal: auto calibration"));

    // arm and enable motors
    motors.armed(true);
    motors.enable();

    // flash LEDS
    AP_Notify::flags.esc_calibration = true;

    // raise throttle to maximum
    delay(10);
    motors.throttle_pass_through(g.rc_3.radio_max);

    // wait for safety switch to be pressed
    while (hal.util->safety_switch_state() == AP_HAL::Util::SAFETY_DISARMED) {
        if (!printed_msg) {
            gcs_send_text_P(SEVERITY_HIGH,PSTR("ESC Cal: push safety switch"));
            printed_msg = true;
        }
        delay(10);
    }

    // delay for 3 seconds
    delay(3000);

    // reduce throttle to minimum
    motors.throttle_pass_through(g.rc_3.radio_min);

    // clear esc parameter
    g.esc_calibrate.set_and_save(ESCCAL_NONE);

    // block until we restart
    while(1) { delay(5); }
}
