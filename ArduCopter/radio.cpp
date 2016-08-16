// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include "Copter.h"


// Function that will read the radio data, limit servos and trigger a failsafe
// ----------------------------------------------------------------------------

void Copter::default_dead_zones()
{
    channel_roll->set_default_dead_zone(30);
    channel_pitch->set_default_dead_zone(30);
#if FRAME_CONFIG == HELI_FRAME
    channel_throttle->set_default_dead_zone(10);
    channel_yaw->set_default_dead_zone(15);
    g.rc_8.set_default_dead_zone(10);
#else
    channel_throttle->set_default_dead_zone(30);
    channel_yaw->set_default_dead_zone(40);
#endif
    g.rc_6.set_default_dead_zone(0);
}

void Copter::init_rc_in()
{
    channel_roll     = RC_Channel::rc_channel(rcmap.roll()-1);
    channel_pitch    = RC_Channel::rc_channel(rcmap.pitch()-1);
    channel_throttle = RC_Channel::rc_channel(rcmap.throttle()-1);
    channel_yaw      = RC_Channel::rc_channel(rcmap.yaw()-1);

    // set rc channel ranges
    channel_roll->set_angle(ROLL_PITCH_INPUT_MAX);
    channel_pitch->set_angle(ROLL_PITCH_INPUT_MAX);
    channel_yaw->set_angle(4500);
    channel_throttle->set_range(0, 1000);

    channel_roll->set_type(RC_CHANNEL_TYPE_ANGLE_RAW);
    channel_pitch->set_type(RC_CHANNEL_TYPE_ANGLE_RAW);
    channel_yaw->set_type(RC_CHANNEL_TYPE_ANGLE_RAW);

    //set auxiliary servo ranges
    g.rc_5.set_range_in(0,1000);
    g.rc_6.set_range_in(0,1000);
    g.rc_7.set_range_in(0,1000);
    g.rc_8.set_range_in(0,1000);

    // set default dead zones
    default_dead_zones();

    // initialise throttle_zero flag
    ap.throttle_zero = true;
}

 // init_rc_out -- initialise motors and check if pilot wants to perform ESC calibration
void Copter::init_rc_out()
{
    motors.set_update_rate(g.rc_speed);
    motors.set_frame_orientation(g.frame_orientation);
    motors.set_loop_rate(scheduler.get_loop_rate_hz());
    motors.Init();                                              // motor initialisation
#if FRAME_CONFIG != HELI_FRAME
    motors.set_throttle_range(channel_throttle->get_radio_min(), channel_throttle->get_radio_max());
#endif

    for(uint8_t i = 0; i < 5; i++) {
        delay(20);
        read_radio();
    }

    // we want the input to be scaled correctly
    channel_throttle->set_range_out(0,1000);

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
    RC_Channel_aux::update_aux_servo_function();
}

// enable_motor_output() - enable and output lowest possible value to motors
void Copter::enable_motor_output()
{
    // enable motors
    motors.enable();
    motors.output_min();
}

void Copter::read_radio()
{
    uint32_t tnow_ms = millis();

    if (hal.rcin->new_input()) {
        ap.new_radio_frame = true;
        RC_Channel::set_pwm_all();

        set_throttle_and_failsafe(channel_throttle->get_radio_in());
        set_throttle_zero_flag(channel_throttle->get_control_in());

        // flag we must have an rc receiver attached
        if (!failsafe.rc_override_active) {
            ap.rc_receiver_present = true;
        }

        // update output on any aux channels, for manual passthru
        RC_Channel_aux::output_ch_all();

        // pass pilot input through to motors (used to allow wiggling servos while disarmed on heli, single, coax copters)
        radio_passthrough_to_motors();

        float dt = (tnow_ms - last_radio_update_ms)*1.0e-3f;
        rc_throttle_control_in_filter.apply(g.rc_3.get_control_in(), dt);
        last_radio_update_ms = tnow_ms;
    }else{
        uint32_t elapsed = tnow_ms - last_radio_update_ms;
        // turn on throttle failsafe if no update from the RC Radio for 500ms or 2000ms if we are using RC_OVERRIDE
        if (((!failsafe.rc_override_active && (elapsed >= FS_RADIO_TIMEOUT_MS)) || (failsafe.rc_override_active && (elapsed >= FS_RADIO_RC_OVERRIDE_TIMEOUT_MS))) &&
            (g.failsafe_throttle && (ap.rc_receiver_present||motors.armed()) && !failsafe.radio)) {
            Log_Write_Error(ERROR_SUBSYSTEM_RADIO, ERROR_CODE_RADIO_LATE_FRAME);
            set_failsafe_radio(true);
        }
    }
}

#define FS_COUNTER 3        // radio failsafe kicks in after 3 consecutive throttle values below failsafe_throttle_value
void Copter::set_throttle_and_failsafe(uint16_t throttle_pwm)
{
    // if failsafe not enabled pass through throttle and exit
    if(g.failsafe_throttle == FS_THR_DISABLED) {
        channel_throttle->set_pwm(throttle_pwm);
        return;
    }

    //check for low throttle value
    if (throttle_pwm < (uint16_t)g.failsafe_throttle_value) {

        // if we are already in failsafe or motors not armed pass through throttle and exit
        if (failsafe.radio || !(ap.rc_receiver_present || motors.armed())) {
            channel_throttle->set_pwm(throttle_pwm);
            return;
        }

        // check for 3 low throttle values
        // Note: we do not pass through the low throttle until 3 low throttle values are received
        failsafe.radio_counter++;
        if( failsafe.radio_counter >= FS_COUNTER ) {
            failsafe.radio_counter = FS_COUNTER;  // check to ensure we don't overflow the counter
            set_failsafe_radio(true);
            channel_throttle->set_pwm(throttle_pwm);   // pass through failsafe throttle
        }
    }else{
        // we have a good throttle so reduce failsafe counter
        failsafe.radio_counter--;
        if( failsafe.radio_counter <= 0 ) {
            failsafe.radio_counter = 0;   // check to ensure we don't underflow the counter

            // disengage failsafe after three (nearly) consecutive valid throttle values
            if (failsafe.radio) {
                set_failsafe_radio(false);
            }
        }
        // pass through throttle
        channel_throttle->set_pwm(throttle_pwm);
    }
}

#define THROTTLE_ZERO_DEBOUNCE_TIME_MS 400
// set_throttle_zero_flag - set throttle_zero flag from debounced throttle control
// throttle_zero is used to determine if the pilot intends to shut down the motors
// Basically, this signals when we are not flying.  We are either on the ground
// or the pilot has shut down the copter in the air and it is free-falling
void Copter::set_throttle_zero_flag(int16_t throttle_control)
{
    static uint32_t last_nonzero_throttle_ms = 0;
    uint32_t tnow_ms = millis();

    // if not using throttle interlock and non-zero throttle and not E-stopped,
    // or using motor interlock and it's enabled, then motors are running, 
    // and we are flying. Immediately set as non-zero
    if ((!ap.using_interlock && (throttle_control > 0) && !ap.motor_emergency_stop) || (ap.using_interlock && motors.get_interlock())) {
        last_nonzero_throttle_ms = tnow_ms;
        ap.throttle_zero = false;
    } else if (tnow_ms - last_nonzero_throttle_ms > THROTTLE_ZERO_DEBOUNCE_TIME_MS) {
        ap.throttle_zero = true;
    }
}

// pass pilot's inputs to motors library (used to allow wiggling servos while disarmed on heli, single, coax copters)
void Copter::radio_passthrough_to_motors()
{
    motors.set_radio_passthrough(channel_roll->get_control_in()/1000.0f, channel_pitch->get_control_in()/1000.0f, channel_throttle->get_control_in()/1000.0f, channel_yaw->get_control_in()/1000.0f);
}
