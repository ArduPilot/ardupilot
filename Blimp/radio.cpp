#include "Blimp.h"

#include <AP_RCProtocol/AP_RCProtocol.h>


// Function that will read the radio data, limit servos and trigger a failsafe
// ----------------------------------------------------------------------------

void Blimp::default_dead_zones()
{
    channel_right->set_default_dead_zone(20);
    channel_front->set_default_dead_zone(20);
    channel_up->set_default_dead_zone(30);
    channel_yaw->set_default_dead_zone(20);
}

void Blimp::init_rc_in()
{
    // the library guarantees that these are non-nullptr:
    channel_right = &rc().get_roll_channel();
    channel_front = &rc().get_pitch_channel();
    channel_up    = &rc().get_throttle_channel();
    channel_yaw   = &rc().get_yaw_channel();

    // set rc channel ranges
    channel_right->set_angle(INPUT_AND_OUTPUT_SCALING);
    channel_front->set_angle(INPUT_AND_OUTPUT_SCALING);
    channel_yaw->set_angle(INPUT_AND_OUTPUT_SCALING);
    channel_up->set_angle(INPUT_AND_OUTPUT_SCALING);

    // set default dead zones
    default_dead_zones();

    // initialise throttle_zero flag
    ap.throttle_zero = true;
}

// init_rc_out -- initialise motors
void Blimp::init_rc_out()
{
    // enable aux servos to cope with multiple output channels per motor
    AP::srv().enable_aux_servos();

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

#if AP_RCPROTOCOL_THROTTLE_FAILSAFE_ENABLED
    // update in case failsafe values have changed:
    if (g.failsafe_throttle == FS_THR_DISABLED) {
        // set flag values to disable filtering in AP_RCProtocol:
        AP::RC().set_throttle_channel_and_failsafe_value(UINT8_MAX, UINT16_MAX);
    } else {
        AP::RC().set_throttle_channel_and_failsafe_value(
            channel_up->ch(),
            g.failsafe_throttle_value
        );
    }
#endif  // AP_RCPROTOCOL_THROTTLE_FAILSAFE_ENABLED

    if (rc().read_input()) {
        if (rc().input_in_failsafe()) {
            // a failsafe has been identified in the input itself.  The
            // values remain available (e.g. for reporting), but are
            // not to be used to fly the vehicle:
            set_failsafe_radio(true);
            return;
        }
        if (rc().input_valid()) {
            ap.new_radio_frame = true;

            set_failsafe_radio(false);
            set_throttle_zero_flag(channel_up->get_control_in());

            const float dt = (tnow_ms - last_radio_update_ms)*1.0e-3f;
            rc_throttle_control_in_filter.apply(channel_up->get_control_in(), dt);
            last_radio_update_ms = tnow_ms;
            return;
        }
        // the input cannot be flown on; handle it as no input at all,
        // so that the timeout applies
    }

    // No radio input this time
    if (failsafe.radio) {
        // already in failsafe!
        return;
    }

    const uint32_t elapsed = tnow_ms - last_radio_update_ms;
    // turn on throttle failsafe if no update from the RC Radio for RC_FS_TIMEOUT seconds or 1000ms if we are using RC_OVERRIDE
    const uint32_t timeout = RC_Channels::has_active_overrides() ? FS_RADIO_RC_OVERRIDE_TIMEOUT_MS : rc().get_fs_timeout_ms();
    if (elapsed < timeout) {
        // not timed out yet
        return;
    }
    if (!g.failsafe_throttle) {
        // throttle failsafe not enabled
        return;
    }
    if (!rc().has_ever_seen_rc_input() && !motors->armed()) {
        // we only failsafe if we are armed OR we have ever seen an RC receiver
        return;
    }

    // Nobody ever talks to us.  Log an error and enter failsafe.
    LOGGER_WRITE_ERROR(LogErrorSubsystem::RADIO, LogErrorCode::RADIO_LATE_FRAME);
    set_failsafe_radio(true);
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
