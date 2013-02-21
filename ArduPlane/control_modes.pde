/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-


static void read_control_switch()
{
    static bool switch_debouncer;
    uint8_t switchPosition = readSwitch();

    // If switchPosition = 255 this indicates that the mode control channel input was out of range
    // If we get this value we do not want to change modes.
    if(switchPosition == 255) return;

    if (ch3_failsafe) {
        // when we are in ch3_failsafe mode then RC input is not
        // working, and we need to ignore the mode switch channel
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
        prev_WP = current_loc;
    }

    if (g.reset_mission_chan != 0 &&
        hal.rcin->read(g.reset_mission_chan-1) > RESET_SWITCH_CHAN_PWM) {
        // reset to first waypoint in mission
        prev_WP = current_loc;
        change_command(0);
    }

    switch_debouncer = false;

    if (g.inverted_flight_ch != 0) {
        // if the user has configured an inverted flight channel, then
        // fly upside down when that channel goes above INVERTED_FLIGHT_PWM
        inverted_flight = (control_mode != MANUAL && hal.rcin->read(g.inverted_flight_ch-1) > INVERTED_FLIGHT_PWM);
    }
}

static uint8_t readSwitch(void){
    uint16_t pulsewidth = hal.rcin->read(g.flight_mode_channel - 1);
    if (pulsewidth <= 910 || pulsewidth >= 2090) return 255;            // This is an error condition
    if (pulsewidth > 1230 && pulsewidth <= 1360) return 1;
    if (pulsewidth > 1360 && pulsewidth <= 1490) return 2;
    if (pulsewidth > 1490 && pulsewidth <= 1620) return 3;
    if (pulsewidth > 1620 && pulsewidth <= 1749) return 4;              // Software Manual
    if (pulsewidth >= 1750) return 5;                                                           // Hardware Manual
    return 0;
}

static void reset_control_switch()
{
    oldSwitchPosition = 0;
    read_control_switch();
}

