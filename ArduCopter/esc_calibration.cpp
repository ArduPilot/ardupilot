#include "Copter.h"

/*****************************************************************************
* Functions to check and perform ESC calibration
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
    // delay up to 2 second for first radio input
    uint8_t i = 0;
    while ((i++ < 100) && (last_radio_update_ms == 0)) {
        delay(20);
        read_radio();
    }

    // exit immediately if pre-arm rc checks fail
    if (!arming.rc_calibration_checks(true)) {
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
                gcs().send_text(MAV_SEVERITY_CRITICAL,"ESC calibration: Restart board");
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

    if (motors->get_pwm_type() >= AP_Motors::PWM_TYPE_ONESHOT) {
        // run at full speed for oneshot ESCs (actually done on push)
        motors->set_update_rate(g.rc_speed);
    } else {
        // reduce update rate to motors to 50Hz
        motors->set_update_rate(50);
    }

    // send message to GCS
    gcs().send_text(MAV_SEVERITY_INFO,"ESC calibration: Passing pilot throttle to ESCs");

    // disable safety if requested
    BoardConfig.init_safety();

    // arm motors
    motors->armed(true);
    motors->enable();
    SRV_Channels::enable_by_mask(motors->get_motor_mask());
    hal.util->set_soft_armed(true);

    while(1) {
        // flash LEDs
        esc_calibration_notify();

        // read pilot input
        read_radio();

        // we run at high rate to make oneshot ESCs happy. Normal ESCs
        // will only see pulses at the RC_SPEED
        delay(3);

        // pass through to motors
        hal.rcout->cork();
        motors->set_throttle_passthrough_for_esc_calibration(channel_throttle->get_control_in() / 1000.0f);
        hal.rcout->push();
    }
#endif  // FRAME_CONFIG != HELI_FRAME
}

// esc_calibration_auto - calibrate the ESCs automatically using a timer and no pilot input
void Copter::esc_calibration_auto()
{
#if FRAME_CONFIG != HELI_FRAME
    bool printed_msg = false;

    // clear esc flag for next time
    g.esc_calibrate.set_and_save(ESCCAL_NONE);

    if (motors->get_pwm_type() >= AP_Motors::PWM_TYPE_ONESHOT) {
        // run at full speed for oneshot ESCs (actually done on push)
        motors->set_update_rate(g.rc_speed);
    } else {
        // reduce update rate to motors to 50Hz
        motors->set_update_rate(50);
    }

    // send message to GCS
    gcs().send_text(MAV_SEVERITY_INFO,"ESC calibration: Auto calibration");

    // disable safety if requested
    BoardConfig.init_safety();

    // arm and enable motors
    motors->armed(true);
    motors->enable();
    SRV_Channels::enable_by_mask(motors->get_motor_mask());
    hal.util->set_soft_armed(true);

    // flash LEDs
    esc_calibration_notify();

    // raise throttle to maximum
    delay(10);

    // wait for safety switch to be pressed
    while (hal.util->safety_switch_state() == AP_HAL::Util::SAFETY_DISARMED) {
        if (!printed_msg) {
            gcs().send_text(MAV_SEVERITY_INFO,"ESC calibration: Push safety switch");
            printed_msg = true;
        }
        hal.rcout->cork();
        motors->set_throttle_passthrough_for_esc_calibration(1.0f);
        hal.rcout->push();
        esc_calibration_notify();
        delay(3);
    }

    // delay for 5 seconds while outputting pulses
    uint32_t tstart = millis();
    while (millis() - tstart < 5000) {
        motors->set_throttle_passthrough_for_esc_calibration(1.0f);
        esc_calibration_notify();
        delay(3);
    }

    // reduce throttle to minimum
    motors->set_throttle_passthrough_for_esc_calibration(0.0f);

    // block until we restart
    while(1) {
        motors->set_throttle_passthrough_for_esc_calibration(0.0f);
        esc_calibration_notify();
        delay(3);
    }
#endif // FRAME_CONFIG != HELI_FRAME
}

// flash LEDs to notify the user that ESC calibration is happening
void Copter::esc_calibration_notify()
{
    AP_Notify::flags.esc_calibration = true;
    uint32_t now = AP_HAL::millis();
    if (now - esc_calibration_notify_update_ms > 20) {
        esc_calibration_notify_update_ms = now;
        update_notify();
    }
}
