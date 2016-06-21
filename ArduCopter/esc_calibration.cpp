// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include "Copter.h"

/*****************************************************************************
*  esc_calibration.pde : functions to check and perform ESC calibration
*****************************************************************************/

#define ESC_CALIBRATION_HIGH_THROTTLE   950

// enum for ESC CALIBRATION
enum ESCCalibrationModes {
    ESCCAL_NONE = 0,
    ESCCAL_PASSTHROUGH_IF_THROTTLE_HIGH = 1,
    ESCCAL_PASSTHROUGH_ALWAYS = 2,
    ESCCAL_AUTO = 3,
    ESCCAL_DISABLED = 9,
};

// check if we should enter esc calibration mode
void Copter::esc_calibration_startup_check()
{
#if FRAME_CONFIG != HELI_FRAME
    // exit immediately if pre-arm rc checks fail
    pre_arm_rc_checks();
    if (!ap.pre_arm_rc_check) {
        // clear esc flag for next time
        if ((g.esc_calibrate != ESCCAL_NONE) && (g.esc_calibrate != ESCCAL_DISABLED)) {
            g.esc_calibrate.set_and_save(ESCCAL_NONE);
        }
        return;
    }

    // check ESC parameter
    switch (g.esc_calibrate) {
        case ESCCAL_NONE:
            // check if throttle is high
            if (channel_throttle->get_control_in() >= ESC_CALIBRATION_HIGH_THROTTLE) {
                // we will enter esc_calibrate mode on next reboot
                g.esc_calibrate.set_and_save(ESCCAL_PASSTHROUGH_IF_THROTTLE_HIGH);
                // send message to gcs
                gcs_send_text(MAV_SEVERITY_CRITICAL,"ESC calibration: Restart board");
                // turn on esc calibration notification
                AP_Notify::flags.esc_calibration = true;
                // block until we restart
                while(1) { delay(5); }
            }
            break;
        case ESCCAL_PASSTHROUGH_IF_THROTTLE_HIGH:
            // check if throttle is high
            if (channel_throttle->get_control_in() >= ESC_CALIBRATION_HIGH_THROTTLE) {
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
        case ESCCAL_DISABLED:
        default:
            // do nothing
            break;
    }

    // clear esc flag for next time
    if (g.esc_calibrate != ESCCAL_DISABLED) {
        g.esc_calibrate.set_and_save(ESCCAL_NONE);
    }
#endif  // FRAME_CONFIG != HELI_FRAME
}

// esc_calibration_passthrough - pass through pilot throttle to escs
void Copter::esc_calibration_passthrough()
{
#if FRAME_CONFIG != HELI_FRAME
    // clear esc flag for next time
    g.esc_calibrate.set_and_save(ESCCAL_NONE);

    // reduce update rate to motors to 50Hz
    motors.set_update_rate(50);

    // send message to GCS
    gcs_send_text(MAV_SEVERITY_INFO,"ESC calibration: Passing pilot throttle to ESCs");

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
        motors.set_throttle_passthrough_for_esc_calibration(channel_throttle->get_control_in() / 1000.0f);
    }
#endif  // FRAME_CONFIG != HELI_FRAME
}

// esc_calibration_auto - calibrate the ESCs automatically using a timer and no pilot input
void Copter::esc_calibration_auto()
{
#if FRAME_CONFIG != HELI_FRAME
    bool printed_msg = false;

    // reduce update rate to motors to 50Hz
    motors.set_update_rate(50);

    // send message to GCS
    gcs_send_text(MAV_SEVERITY_INFO,"ESC calibration: Auto calibration");

    // arm and enable motors
    motors.armed(true);
    motors.enable();

    // flash LEDS
    AP_Notify::flags.esc_calibration = true;

    // raise throttle to maximum
    delay(10);
    motors.set_throttle_passthrough_for_esc_calibration(1.0f);

    // wait for safety switch to be pressed
    while (hal.util->safety_switch_state() == AP_HAL::Util::SAFETY_DISARMED) {
        if (!printed_msg) {
            gcs_send_text(MAV_SEVERITY_INFO,"ESC calibration: Push safety switch");
            printed_msg = true;
        }
        delay(10);
    }

    // delay for 5 seconds
    delay(5000);

    // reduce throttle to minimum
    motors.set_throttle_passthrough_for_esc_calibration(0.0f);

    // clear esc parameter
    g.esc_calibrate.set_and_save(ESCCAL_NONE);

    // block until we restart
    while(1) { delay(5); }
#endif // FRAME_CONFIG != HELI_FRAME
}
