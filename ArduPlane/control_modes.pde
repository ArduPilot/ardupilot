/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-


static void read_control_switch()
{
    static bool switch_debouncer;
    uint8_t switchPosition = readSwitch();

    // If switchPosition = 255 this indicates that the mode control channel input was out of range
    // If we get this value we do not want to change modes.
    if(switchPosition == 255) return;

    if (failsafe.ch3_failsafe || failsafe.ch3_counter > 0) {
        // when we are in ch3_failsafe mode then RC input is not
        // working, and we need to ignore the mode switch channel
        return;
    }

    if (hal.scheduler->millis() - failsafe.last_valid_rc_ms > 100) {
        // only use signals that are less than 0.1s old.
        return;
    }

    // we look for changes in the switch position. If the
    // RST_SWITCH_CH parameter is set, then it is a switch that can be
    // used to force re-reading of the control switch. This is useful
    // when returning to the previous mode after a failsafe or fence
    // breach. This channel is best used on a momentary switch (such
    // as a spring loaded trainer switch).
    if (oldSwitchPosition != switchPosition ||
        (g.reset_switch_chan != 0 &&
         hal.rcin->read(g.reset_switch_chan-1) > RESET_SWITCH_CHAN_PWM)) {

        if (switch_debouncer == false) {
            // this ensures that mode switches only happen if the
            // switch changes for 2 reads. This prevents momentary
            // spikes in the mode control channel from causing a mode
            // switch
            switch_debouncer = true;
            return;
        }

        set_mode((enum FlightMode)(flight_modes[switchPosition].get()));

        oldSwitchPosition = switchPosition;
        prev_WP_loc = current_loc;
    }

    if (g.reset_mission_chan != 0 &&
        hal.rcin->read(g.reset_mission_chan-1) > RESET_SWITCH_CHAN_PWM) {
        mission.start();
        prev_WP_loc = current_loc;
    }

    switch_debouncer = false;

    if (g.inverted_flight_ch != 0) {
        // if the user has configured an inverted flight channel, then
        // fly upside down when that channel goes above INVERTED_FLIGHT_PWM
        inverted_flight = (control_mode != MANUAL && hal.rcin->read(g.inverted_flight_ch-1) > INVERTED_FLIGHT_PWM);
    }

#if CONFIG_HAL_BOARD == HAL_BOARD_PX4
    if (g.override_channel > 0) {
        // if the user has configured an override channel then check it
        bool override = (hal.rcin->read(g.override_channel-1) >= PX4IO_OVERRIDE_PWM);
        if (override && !px4io_override_enabled) {
            // we only update the mixer if we are not armed. This is
            // important as otherwise we will need to temporarily
            // disarm to change the mixer
            if (ahrs.get_armed() || setup_failsafe_mixing()) {
                px4io_override_enabled = true;
                // disable output channels to force PX4IO override
                for (uint8_t i=0; i<16; i++) {
                    hal.rcout->disable_ch(i);
                }
                gcs_send_text_P(SEVERITY_LOW, PSTR("PX4IO Override enabled"));
            } else {
                // we'll try again next loop. The PX4IO code sometimes
                // rejects a mixer, probably due to it being busy in
                // some way?
                gcs_send_text_P(SEVERITY_LOW, PSTR("PX4IO Override enable failed"));
            }
        } else if (!override && px4io_override_enabled) {
            px4io_override_enabled = false;
            // re-enable output channels
            for (uint8_t i=0; i<8; i++) {
                hal.rcout->enable_ch(i);
            }
            RC_Channel_aux::enable_aux_servos();
            gcs_send_text_P(SEVERITY_LOW, PSTR("PX4IO Override disabled"));
        }
        if (px4io_override_enabled && 
            hal.util->safety_switch_state() != AP_HAL::Util::SAFETY_ARMED) {
            // we force safety off, so that if this override is used
            // with a in-flight reboot it gives a way for the pilot to
            // re-arm and take manual control
            hal.rcout->force_safety_off();
        }
    }
#endif // CONFIG_HAL_BOARD
}

static uint8_t readSwitch(void)
{
    uint16_t pulsewidth = hal.rcin->read(g.flight_mode_channel - 1);
    if (pulsewidth <= 900 || pulsewidth >= 2200) return 255;            // This is an error condition
    if (pulsewidth > 1230 && pulsewidth <= 1360) return 1;
    if (pulsewidth > 1360 && pulsewidth <= 1490) return 2;
    if (pulsewidth > 1490 && pulsewidth <= 1620) return 3;
    if (pulsewidth > 1620 && pulsewidth <= 1749) return 4;              // Software Manual
    if (pulsewidth >= 1750) return 5;                                                           // Hardware Manual
    return 0;
}

static void reset_control_switch()
{
    oldSwitchPosition = 254;
    read_control_switch();
}

/*
  called when entering autotune
 */
static void autotune_start(void)
{
    rollController.autotune_start();
    pitchController.autotune_start();
}

/*
  called when exiting autotune
 */
static void autotune_restore(void)
{
    rollController.autotune_restore();
    pitchController.autotune_restore();
}

/*
  are we flying inverted?
 */
static bool fly_inverted(void)
{
    if (g.inverted_flight_ch != 0 && inverted_flight) {
        // controlled with INVERTED_FLIGHT_CH
        return true;
    }
    if (control_mode == AUTO && auto_state.inverted_flight) {
        return true;
    }
    return false;
}
