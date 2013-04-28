// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
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
void failsafe_check(uint32_t tnow)
{
    static uint16_t last_mainLoop_count;
    static uint32_t last_timestamp;
    static bool in_failsafe;

    if (mainLoop_count != last_mainLoop_count) {
        // the main loop is running, all is OK
        last_mainLoop_count = mainLoop_count;
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
        // pass RC inputs to outputs every 20ms
        last_timestamp = tnow;
        hal.rcin->clear_overrides();
        g.channel_roll.radio_out     = hal.rcin->read(CH_1);
        g.channel_pitch.radio_out    = hal.rcin->read(CH_2);
        g.channel_throttle.radio_out = hal.rcin->read(CH_3);
        g.channel_rudder.radio_out   = hal.rcin->read(CH_4);
        if (g.vtail_output != MIXING_DISABLED) {
            channel_output_mixer(g.vtail_output, g.channel_pitch.radio_out, g.channel_rudder.radio_out);
        } else if (g.elevon_output != MIXING_DISABLED) {
            channel_output_mixer(g.elevon_output, g.channel_pitch.radio_out, g.channel_roll.radio_out);
        }
        if (!demoing_servos) {
            servo_write(CH_1, g.channel_roll.radio_out);
        }
        servo_write(CH_2, g.channel_pitch.radio_out);
        servo_write(CH_3, g.channel_throttle.radio_out);
        servo_write(CH_4, g.channel_rudder.radio_out);

        RC_Channel_aux::copy_radio_in_out(RC_Channel_aux::k_manual, true);
        RC_Channel_aux::copy_radio_in_out(RC_Channel_aux::k_aileron_with_input, true);
        RC_Channel_aux::copy_radio_in_out(RC_Channel_aux::k_elevator_with_input, true);
    }
}
