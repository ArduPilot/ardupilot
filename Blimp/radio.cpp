#include "Blimp.h"


// Function that will read the radio data, limit servos and trigger a failsafe
// ----------------------------------------------------------------------------

void Blimp::default_dead_zones()
{
    channel_right->set_default_dead_zone(20);
    channel_front->set_default_dead_zone(20);
    channel_up->set_default_dead_zone(30);
    channel_yaw->set_default_dead_zone(20);
    rc().channel(CH_6)->set_default_dead_zone(0);
}

void Blimp::init_rc_in()
{
    channel_right = rc().channel(rcmap.roll()-1);
    channel_front = rc().channel(rcmap.pitch()-1);
    channel_up    = rc().channel(rcmap.throttle()-1);
    channel_yaw   = rc().channel(rcmap.yaw()-1);

    // set rc channel ranges
    channel_right->set_angle(RC_SCALE);
    channel_front->set_angle(RC_SCALE);
    channel_yaw->set_angle(RC_SCALE);
    channel_up->set_angle(RC_SCALE);

    // set default dead zones
    default_dead_zones();

    // initialise throttle_zero flag
    ap.throttle_zero = true;
}

// init_rc_out -- initialise motors
void Blimp::init_rc_out()
{
    // enable aux servos to cope with multiple output channels per motor
    SRV_Channels::enable_aux_servos();

    // refresh auxiliary channel to function map
    SRV_Channels::update_aux_servo_function();
}


// enable_motor_output() - enable and output lowest possible value to motors
void Blimp::enable_motor_output()
{
    // enable motors
    motors->output_min();
}

void Blimp::read_radio()
{
    const uint32_t tnow_ms = millis();

    if (rc().read_input()) {
        ap.new_radio_frame = true;

        set_throttle_and_failsafe(channel_up->get_radio_in());
        set_throttle_zero_flag(channel_up->get_control_in());

        // RC receiver must be attached if we've just got input
        ap.rc_receiver_present = true;

        const float dt = (tnow_ms - last_radio_update_ms)*1.0e-3f;
        rc_throttle_control_in_filter.apply(channel_up->get_control_in(), dt);
        last_radio_update_ms = tnow_ms;
        return;
    }

    // No radio input this time
    if (failsafe.radio) {
        // already in failsafe!
        return;
    }

    const uint32_t elapsed = tnow_ms - last_radio_update_ms;
    // turn on throttle failsafe if no update from the RC Radio for 500ms or 2000ms if we are using RC_OVERRIDE
    const uint32_t timeout = RC_Channels::has_active_overrides() ? FS_RADIO_RC_OVERRIDE_TIMEOUT_MS : FS_RADIO_TIMEOUT_MS;
    if (elapsed < timeout) {
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

    // Nobody ever talks to us.  Log an error and enter failsafe.
    LOGGER_WRITE_ERROR(LogErrorSubsystem::RADIO, LogErrorCode::RADIO_LATE_FRAME);
    set_failsafe_radio(true);
}

#define FS_COUNTER 3        // radio failsafe kicks in after 3 consecutive throttle values below failsafe_throttle_value
void Blimp::set_throttle_and_failsafe(uint16_t throttle_pwm)
{
    // if failsafe not enabled pass through throttle and exit
    if (g.failsafe_throttle == FS_THR_DISABLED) {
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
        if ( failsafe.radio_counter >= FS_COUNTER ) {
            failsafe.radio_counter = FS_COUNTER;  // check to ensure we don't overflow the counter
            set_failsafe_radio(true);
        }
    } else {
        // we have a good throttle so reduce failsafe counter
        failsafe.radio_counter--;
        if ( failsafe.radio_counter <= 0 ) {
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
// or the pilot has shut down the vehicle in the air and it is free-floating
void Blimp::set_throttle_zero_flag(int16_t throttle_control)
{
    static uint32_t last_nonzero_throttle_ms = 0;
    uint32_t tnow_ms = millis();

    // if not using throttle interlock and non-zero throttle and not E-stopped,
    // or using motor interlock and it's enabled, then motors are running,
    // and we are flying. Immediately set as non-zero
    if (throttle_control > 0) {
        last_nonzero_throttle_ms = tnow_ms;
        ap.throttle_zero = false;
    } else if (tnow_ms - last_nonzero_throttle_ms > THROTTLE_ZERO_DEBOUNCE_TIME_MS) {
        ap.throttle_zero = true;
    }
    //TODO: This may not be needed
}
