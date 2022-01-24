#include "Plane.h"

/*
 *  failsafe support
 *  Andrew Tridgell, December 2011
 */

/*
 *  our failsafe strategy is to detect main loop lockup and switch to
 *  passing inputs straight from the RC inputs to RC outputs.
 */

/*
 *  this failsafe_check function is called from the core timer interrupt
 *  at 1kHz.
 */
void Plane::failsafe_check(void)
{
    static uint16_t last_ticks;
    static uint32_t last_timestamp;
    static bool in_failsafe;
    uint32_t tnow = micros();

    const uint16_t ticks = scheduler.ticks();
    if (ticks != last_ticks) {
        // the main loop is running, all is OK
        last_ticks = ticks;
        last_timestamp = tnow;
        in_failsafe = false;
        return;
    }

    if (tnow - last_timestamp > 200000) {
        // we have gone at least 0.2 seconds since the main loop
        // ran. That means we're in trouble, or perhaps are in
        // an initialisation routine or log erase. Start passing RC
        // inputs through to outputs
        in_failsafe = true;
    }

    if (in_failsafe && tnow - last_timestamp > 20000) {

        // ensure we have the latest RC inputs
        rc().read_input();

        last_timestamp = tnow;

        rc().read_input();

#if ADVANCED_FAILSAFE == ENABLED
        if (in_calibration) {
            // tell the failsafe system that we are calibrating
            // sensors, so don't trigger failsafe
            afs.heartbeat();
        }
#endif

        if (RC_Channels::get_valid_channel_count() < 5) {
            // we don't have any RC input to pass through
            return;
        }

        // pass RC inputs to outputs every 20ms
        RC_Channels::clear_overrides();

        float roll = roll_in_expo(false);
        float pitch = pitch_in_expo(false);
        float throttle = get_throttle_input(true);
        float rudder = rudder_in_expo(false);

        if (!hal.util->get_soft_armed()) {
            throttle = 0;
        }
        
        // setup secondary output channels that don't have
        // corresponding input channels
        SRV_Channels::set_output_scaled(SRV_Channel::k_aileron, roll);
        SRV_Channels::set_output_scaled(SRV_Channel::k_elevator, pitch);
        SRV_Channels::set_output_scaled(SRV_Channel::k_rudder, rudder);
        SRV_Channels::set_output_scaled(SRV_Channel::k_steering, rudder);
        SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, throttle);

        // this is to allow the failsafe module to deliberately crash 
        // the plane. Only used in extreme circumstances to meet the
        // OBC rules
#if ADVANCED_FAILSAFE == ENABLED
        if (afs.should_crash_vehicle()) {
            afs.terminate_vehicle();
            if (!afs.terminating_vehicle_via_landing()) {
                return;
            }
        }
#endif

        // setup secondary output channels that do have
        // corresponding input channels
        SRV_Channels::copy_radio_in_out(SRV_Channel::k_manual, true);
        SRV_Channels::set_output_scaled(SRV_Channel::k_flap, 0.0);
        SRV_Channels::set_output_scaled(SRV_Channel::k_flap_auto, 0.0);

        // setup flaperons
        flaperon_update();

        servos_output();

        // in SITL we send through the servo outputs so we can verify
        // we're manipulating surfaces
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
        GCS_MAVLINK *chan = gcs().chan(0);
        if (HAVE_PAYLOAD_SPACE(chan->get_chan(), SERVO_OUTPUT_RAW)) {
            chan->send_servo_output_raw();
        }
#endif
    }
}
