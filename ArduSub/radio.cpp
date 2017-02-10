#include "Sub.h"


// Function that will read the radio data, limit servos and trigger a failsafe
// ----------------------------------------------------------------------------

void Sub::default_dead_zones()
{
    channel_roll->set_default_dead_zone(30);
    channel_pitch->set_default_dead_zone(30);
    channel_throttle->set_default_dead_zone(30);
    channel_yaw->set_default_dead_zone(40);
    channel_forward->set_default_dead_zone(30);
    channel_lateral->set_default_dead_zone(30);
}

void Sub::init_rc_in()
{
    channel_pitch    = RC_Channels::rc_channel(0);
    channel_roll     = RC_Channels::rc_channel(1);
    channel_throttle = RC_Channels::rc_channel(2);
    channel_yaw      = RC_Channels::rc_channel(3);
    channel_forward  = RC_Channels::rc_channel(5);
    channel_lateral  = RC_Channels::rc_channel(6);

    // set rc channel ranges
    channel_roll->set_angle(ROLL_PITCH_INPUT_MAX);
    channel_pitch->set_angle(ROLL_PITCH_INPUT_MAX);
    channel_yaw->set_angle(ROLL_PITCH_INPUT_MAX);
    channel_throttle->set_range(1000);
    channel_forward->set_angle(ROLL_PITCH_INPUT_MAX);
    channel_lateral->set_angle(ROLL_PITCH_INPUT_MAX);

    // force throttle trim to 1100
    channel_throttle->set_radio_trim(1100);
    channel_throttle->save_eeprom();

    //set auxiliary servo ranges
    //    g.rc_5.set_range(0,1000);
    //    g.rc_6.set_range(0,1000);
    //    g.rc_7.set_range(0,1000);
    //    g.rc_8.set_range(0,1000);

    // set default dead zones
    default_dead_zones();

    // initialize rc input to 1500 on control channels (rather than 0)
    for (int i = 0; i < 7; i++) {
        if (i == 4) {
            hal.rcin->set_override(i, 1100); // Channel 5 mode selection
        } else {
            hal.rcin->set_override(i, 1500);
        }
    }

    // initialise throttle_zero flag
    ap.throttle_zero = true;
}

// init_rc_out -- initialise motors and check if pilot wants to perform ESC calibration
void Sub::init_rc_out()
{
    motors.set_update_rate(g.rc_speed);
    motors.set_loop_rate(scheduler.get_loop_rate_hz());
    motors.init((AP_Motors::motor_frame_class)g.frame_configuration.get(), (AP_Motors::motor_frame_type)0);

    for (uint8_t i = 0; i < 5; i++) {
        hal.scheduler->delay(20);
        read_radio();
    }

    // setup correct scaling for ESCs like the UAVCAN PX4ESC which
    // take a proportion of speed.
    hal.rcout->set_esc_scaling(channel_throttle->get_radio_min(), channel_throttle->get_radio_max());

    // check if we should enter esc calibration mode
    esc_calibration_startup_check();

    // enable output to motors
    pre_arm_rc_checks();
    if (ap.pre_arm_rc_check) {
        enable_motor_output();
    }

    // refresh auxiliary channel to function map
    SRV_Channels::update_aux_servo_function();
}

// enable_motor_output() - enable and output lowest possible value to motors
void Sub::enable_motor_output()
{
    // enable motors
    motors.enable();
    motors.output_min();
}

void Sub::read_radio()
{
    static uint32_t last_update_ms = 0;
    uint32_t tnow_ms = millis();

    if (hal.rcin->new_input()) {
        ap.new_radio_frame = true;
        RC_Channels::set_pwm_all();

        set_throttle_zero_flag(channel_throttle->get_control_in());

        // flag we must have an rc receiver attached
        if (!failsafe.rc_override_active) {
            ap.rc_receiver_present = true;
        }

        // update output on any aux channels, for manual passthru
        SRV_Channels::output_ch_all();

        // pass pilot input through to motors (used to allow wiggling servos while disarmed on heli, single, coax copters)
        radio_passthrough_to_motors();

        float dt = (tnow_ms - last_update_ms)*1.0e-3f;
        rc_throttle_control_in_filter.apply(channel_throttle->get_control_in(), dt);
        last_update_ms = tnow_ms;
    }
}

#define THROTTLE_ZERO_DEBOUNCE_TIME_MS 400
// set_throttle_zero_flag - set throttle_zero flag from debounced throttle control
// throttle_zero is used to determine if the pilot intends to shut down the motors
// Basically, this signals when we are not flying.  We are either on the ground
// or the pilot has shut down the vehicle in the air and it is free-falling
void Sub::set_throttle_zero_flag(int16_t throttle_control)
{
    static uint32_t last_nonzero_throttle_ms = 0;
    uint32_t tnow_ms = millis();

    // if not using throttle interlock and non-zero throttle and not E-stopped,
    // or using motor interlock and it's enabled, then motors are running,
    // and we are flying. Immediately set as non-zero
    if ((!ap.using_interlock && (throttle_control < 475 || throttle_control > 525) &&  !ap.motor_emergency_stop) || (ap.using_interlock && motors.get_interlock())) {
        last_nonzero_throttle_ms = tnow_ms;
        ap.throttle_zero = false;
    } else if (tnow_ms - last_nonzero_throttle_ms > THROTTLE_ZERO_DEBOUNCE_TIME_MS) {
        ap.throttle_zero = true;
    }
}

// pass pilot's inputs to motors library (used to allow wiggling servos while disarmed on heli, single, coax copters)
void Sub::radio_passthrough_to_motors()
{
    motors.set_radio_passthrough(channel_roll->get_control_in()/1000.0f, channel_pitch->get_control_in()/1000.0f, channel_throttle->get_control_in()/1000.0f, channel_yaw->get_control_in()/1000.0f);
}
