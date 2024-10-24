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

    // check if arming/disarm using rudder is allowed
    AP_Arming::RudderArming arming_rudder = arming.get_rudder_arming_type();
    if (arming_rudder == AP_Arming::RudderArming::IS_DISABLED) {
        arming_counter = 0;
        return;
    }

#if TOY_MODE_ENABLED
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

    int16_t yaw_in = channel_yaw->get_control_in();

    // full right
    if (yaw_in > 4000) {

        // increase the arming counter to a maximum of 1 beyond the auto trim counter
        if (arming_counter <= AUTO_TRIM_DELAY) {
            arming_counter++;
        }

        // arm the motors and configure for flight
        if (arming_counter == ARM_DELAY && !motors->armed()) {
            // reset arming counter if arming fail
            if (!arming.arm(AP_Arming::Method::RUDDER)) {
                arming_counter = 0;
            }
        }

        // arm the motors and configure for flight
        if (arming_counter == AUTO_TRIM_DELAY && motors->armed() && flightmode->mode_number() == Mode::Number::STABILIZE) {
            gcs().send_text(MAV_SEVERITY_INFO, "AutoTrim start");
            auto_trim_counter = 250;
            auto_trim_started = false;
            // ensure auto-disarm doesn't trigger immediately
            auto_disarm_begin = millis();
        }

    // full left and rudder disarming is enabled
    } else if ((yaw_in < -4000) && (arming_rudder == AP_Arming::RudderArming::ARMDISARM)) {
        if (!flightmode->has_manual_throttle() && !ap.land_complete) {
            arming_counter = 0;
            return;
        }

        // increase the counter to a maximum of 1 beyond the disarm delay
        if (arming_counter <= DISARM_DELAY) {
            arming_counter++;
        }

        // disarm the motors
        if (arming_counter == DISARM_DELAY && motors->armed()) {
            arming.disarm(AP_Arming::Method::RUDDER);
        }

    // Yaw is centered so reset arming counter
    } else {
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
    if (!motors->armed() || disarm_delay_ms == 0 || flightmode->mode_number() == Mode::Number::THROW) {
        auto_disarm_begin = tnow_ms;
        return;
    }

    // if the rotor is still spinning, don't initiate auto disarm
    if (motors->get_spool_state() > AP_Motors::SpoolState::GROUND_IDLE) {
        auto_disarm_begin = tnow_ms;
        return;
    }

    // always allow auto disarm if using interlock switch or motors are Emergency Stopped
    if ((ap.using_interlock && !motors->get_interlock()) || SRV_Channels::get_emergency_stop()) {
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
        arming.disarm(AP_Arming::Method::DISARMDELAY);
        auto_disarm_begin = tnow_ms;
    }
}

// motors_output - send output to motors library which will adjust and send to ESCs and servos
void Copter::motors_output(bool full_push)
{
#if ADVANCED_FAILSAFE
    // this is to allow the failsafe module to deliberately crash
    // the vehicle. Only used in extreme circumstances to meet the
    // OBC rules
    if (g2.afs.should_crash_vehicle()) {
        g2.afs.terminate_vehicle();
        if (!g2.afs.terminating_vehicle_via_landing()) {
            return;
        }
        // landing must continue to run the motors output
    }
#endif

    // Update arming delay state
    if (ap.in_arming_delay && (!motors->armed() || millis()-arm_time_ms > ARMING_DELAY_SEC*1.0e3f || flightmode->mode_number() == Mode::Number::THROW)) {
        ap.in_arming_delay = false;
    }

    // output any servo channels
    SRV_Channels::calc_pwm();

    // cork now, so that all channel outputs happen at once
    SRV_Channels::cork();

    // update output on any aux channels, for manual passthru
    SRV_Channels::output_ch_all();

    // update motors interlock state
    bool interlock = motors->armed() && !ap.in_arming_delay && (!ap.using_interlock || ap.motor_interlock_switch) && !SRV_Channels::get_emergency_stop();
    if (!motors->get_interlock() && interlock) {
        motors->set_interlock(true);
        LOGGER_WRITE_EVENT(LogEvent::MOTORS_INTERLOCK_ENABLED);
    } else if (motors->get_interlock() && !interlock) {
        motors->set_interlock(false);
        LOGGER_WRITE_EVENT(LogEvent::MOTORS_INTERLOCK_DISABLED);
    }

    if (ap.motor_test) {
        // check if we are performing the motor test
        motor_test_output();
    } else {
        // send output signals to motors
        flightmode->output_to_motors();
    }

    // push all channels
    if (full_push) {
        SRV_Channels::push();
    } else {
        hal.rcout->push();
    }
}

// motors_output from main thread
void Copter::motors_output_main()
{
    if (!using_rate_thread) {
        motors_output();
    }
}

// check for pilot stick input to trigger lost vehicle alarm
void Copter::lost_vehicle_check()
{
    static uint8_t soundalarm_counter;

    // disable if aux switch is setup to vehicle alarm as the two could interfere
    if (rc().find_channel_for_option(RC_Channel::AUX_FUNC::LOST_VEHICLE_SOUND)) {
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
