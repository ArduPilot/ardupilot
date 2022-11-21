#include "Copter.h"


// Function that will read the radio data, limit servos and trigger a failsafe
// ----------------------------------------------------------------------------

void Copter::default_dead_zones()
{
    channel_roll->set_default_dead_zone(20);
    channel_pitch->set_default_dead_zone(20);
#if FRAME_CONFIG == HELI_FRAME
    channel_throttle->set_default_dead_zone(10);
    channel_yaw->set_default_dead_zone(15);
#else
    channel_throttle->set_default_dead_zone(30);
    channel_yaw->set_default_dead_zone(20);
#endif
    rc().channel(CH_6)->set_default_dead_zone(0);
}

void Copter::init_rc_in()
{
    channel_roll     = rc().channel(rcmap.roll()-1);
    channel_pitch    = rc().channel(rcmap.pitch()-1);
    channel_throttle = rc().channel(rcmap.throttle()-1);
    channel_yaw      = rc().channel(rcmap.yaw()-1);

    // set rc channel ranges
    channel_roll->set_angle(ROLL_PITCH_YAW_INPUT_MAX);
    channel_pitch->set_angle(ROLL_PITCH_YAW_INPUT_MAX);
    channel_yaw->set_angle(ROLL_PITCH_YAW_INPUT_MAX);
    channel_throttle->set_range(1000);

    // set default dead zones
    default_dead_zones();

    // initialise throttle_zero flag
    ap.throttle_zero = true;
}

 // init_rc_out -- initialise motors
void Copter::init_rc_out()
{
    motors->set_loop_rate(scheduler.get_loop_rate_hz());
    motors->init((AP_Motors::motor_frame_class)g2.frame_class.get(), (AP_Motors::motor_frame_type)g.frame_type.get());

    // enable aux servos to cope with multiple output channels per motor
    SRV_Channels::enable_aux_servos();

    // update rate must be set after motors->init() to allow for motor mapping
    motors->set_update_rate(g.rc_speed);

#if FRAME_CONFIG != HELI_FRAME
    if (channel_throttle->configured()) {
        // throttle inputs setup, use those to set motor PWM min and max if not already configured
        motors->convert_pwm_min_max_param(channel_throttle->get_radio_min(), channel_throttle->get_radio_max());
    } else {
        // throttle inputs default, force set motor PWM min and max to defaults so they will not be over-written by a future change in RC min / max
        motors->convert_pwm_min_max_param(1000, 2000);
    }
    motors->update_throttle_range();
#else
    // setup correct scaling for ESCs like the UAVCAN ESCs which
    // take a proportion of speed.
    hal.rcout->set_esc_scaling(channel_throttle->get_radio_min(), channel_throttle->get_radio_max());
#endif

    // refresh auxiliary channel to function map
    SRV_Channels::update_aux_servo_function();

#if FRAME_CONFIG != HELI_FRAME
    /*
      setup a default safety ignore mask, so that servo gimbals can be active while safety is on
     */
    uint16_t safety_ignore_mask = (~copter.motors->get_motor_mask()) & 0x3FFF;
    BoardConfig.set_default_safety_ignore_mask(safety_ignore_mask);
#endif
}


// enable_motor_output() - enable and output lowest possible value to motors
void Copter::enable_motor_output()
{
    // enable motors
    motors->output_min();
}

void Copter::read_radio()
{
    const uint32_t tnow_ms = millis();

    if (rc().read_input()) {
        ap.new_radio_frame = true;

        set_throttle_and_failsafe(channel_throttle->get_radio_in());
        set_throttle_zero_flag(channel_throttle->get_control_in());

        // RC receiver must be attached if we've just got input
        ap.rc_receiver_present = true;

        // pass pilot input through to motors (used to allow wiggling servos while disarmed on heli, single, coax copters)
        radio_passthrough_to_motors();

        const float dt = (tnow_ms - last_radio_update_ms)*1.0e-3f;
        rc_throttle_control_in_filter.apply(channel_throttle->get_control_in(), dt);
        last_radio_update_ms = tnow_ms;
        return;
    }

    // No radio input this time
    if (failsafe.radio) {
        // already in failsafe!
        return;
    }

    // trigger failsafe if no update from the RC Radio for RC_FS_TIMEOUT seconds
    const uint32_t elapsed_ms = tnow_ms - last_radio_update_ms;
    if (elapsed_ms < rc().get_fs_timeout_ms()) {
        // not timed out yet
        return;
    }
    if (!g.failsafe_throttle) {
        // throttle failsafe not enabled
        return;
    }
    if (!ap.rc_receiver_present && !motors->armed()) {
        // we only failsafe if we are armed OR we have ever seen an RC receiver
        return;
    }

    // Log an error and enter failsafe.
    AP::logger().Write_Error(LogErrorSubsystem::RADIO, LogErrorCode::RADIO_LATE_FRAME);
    set_failsafe_radio(true);
}

#define FS_COUNTER 3        // radio failsafe kicks in after 3 consecutive throttle values below failsafe_throttle_value
void Copter::set_throttle_and_failsafe(uint16_t throttle_pwm)
{
    // if failsafe not enabled pass through throttle and exit
    if(g.failsafe_throttle == FS_THR_DISABLED) {
        return;
    }

    //check for low throttle value
    if (throttle_pwm < (uint16_t)g.failsafe_throttle_value) {

        // if we are already in failsafe or motors not armed pass through throttle and exit
        if (failsafe.radio || !(ap.rc_receiver_present || motors->armed())) {
            return;
        }

        // check for 3 low throttle values
        // Note: we do not pass through the low throttle until 3 low throttle values are received
        failsafe.radio_counter++;
        if( failsafe.radio_counter >= FS_COUNTER ) {
            failsafe.radio_counter = FS_COUNTER;  // check to ensure we don't overflow the counter
            set_failsafe_radio(true);
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
    if ((!ap.using_interlock && (throttle_control > 0) && !SRV_Channels::get_emergency_stop()) ||
        (ap.using_interlock && motors->get_interlock()) ||
        ap.armed_with_airmode_switch || air_mode == AirMode::AIRMODE_ENABLED) {
        last_nonzero_throttle_ms = tnow_ms;
        ap.throttle_zero = false;
    } else if (tnow_ms - last_nonzero_throttle_ms > THROTTLE_ZERO_DEBOUNCE_TIME_MS) {
        ap.throttle_zero = true;
    }
}

// pass pilot's inputs to motors library (used to allow wiggling servos while disarmed on heli, single, coax copters)
void Copter::radio_passthrough_to_motors()
{
    motors->set_radio_passthrough(channel_roll->norm_input(),
                                  channel_pitch->norm_input(),
                                  channel_throttle->get_control_in_zero_dz()*0.001f,
                                  channel_yaw->norm_input());
}

/*
  return the throttle input for mid-stick as a control-in value
 */
int16_t Copter::get_throttle_mid(void)
{
#if TOY_MODE_ENABLED == ENABLED
    if (g2.toy_mode.enabled()) {
        return g2.toy_mode.get_throttle_mid();
    }
#endif
    return channel_throttle->get_control_mid();
}
