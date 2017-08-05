#include "Rover.h"

static const int16_t CH_7_PWM_TRIGGER = 1800;

Mode *Rover::control_mode_from_num(const enum mode num)
{
    Mode *ret = nullptr;
    switch (num) {
    case MANUAL:
        ret = &mode_manual;
        break;
    case LEARNING:
        ret = &mode_learning;
        break;
    case STEERING:
        ret = &mode_steering;
        break;
    case HOLD:
        ret = &mode_hold;
        break;
    case AUTO:
        ret = &mode_auto;
        break;
    case RTL:
        ret = &mode_rtl;
        break;
    case GUIDED:
       ret = &mode_guided;
        break;
    case INITIALISING:
        ret = &mode_initializing;
        break;
    default:
        break;
    }
    return ret;
}

void Rover::read_control_switch()
{
    static bool switch_debouncer;
    const uint8_t switchPosition = readSwitch();

    // If switchPosition = 255 this indicates that the mode control channel input was out of range
    // If we get this value we do not want to change modes.
    if (switchPosition == 255) {
        return;
    }

    if (AP_HAL::millis() - failsafe.last_valid_rc_ms > 100) {
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

        Mode *new_mode = control_mode_from_num((enum mode)modes[switchPosition].get());
        if (new_mode != nullptr) {
            set_mode(*new_mode, MODE_REASON_TX_COMMAND);
        }

        oldSwitchPosition = switchPosition;

        // reset speed integrator
        g.pidSpeedThrottle.reset_I();
    }

    switch_debouncer = false;
}

uint8_t Rover::readSwitch(void) {
    const uint16_t pulsewidth = hal.rcin->read(g.mode_channel - 1);
    if (pulsewidth <= 900 || pulsewidth >= 2200) {
        return 255;  // This is an error condition
    }
    if (pulsewidth <= 1230) {
        return 0;
    }
    if (pulsewidth <= 1360) {
        return 1;
    }
    if (pulsewidth <= 1490) {
        return 2;
    }
    if (pulsewidth <= 1620) {
        return 3;
    }
    if (pulsewidth <= 1749) {
        return 4;  // Software Manual
    }
    return 5;  // Hardware Manual
}

void Rover::reset_control_switch()
{
    oldSwitchPosition = 254;
    read_control_switch();
}

// read at 10 hz
// set this to your trainer switch
void Rover::read_trim_switch()
{
    switch ((enum ch7_option)g.ch7_option.get()) {
    case CH7_DO_NOTHING:
        break;
    case CH7_SAVE_WP:
        if (channel_learn->get_radio_in() > CH_7_PWM_TRIGGER) {
            // switch is engaged
            ch7_flag = true;
        } else {  // switch is disengaged
            if (ch7_flag) {
                ch7_flag = false;

                if (control_mode == &mode_manual) {
                    hal.console->printf("Erasing waypoints\n");
                    // if SW7 is ON in MANUAL = Erase the Flight Plan
                    mission.clear();
                    if (channel_steer->get_control_in() > 3000) {
                        // if roll is full right store the current location as home
                        set_home_to_current_location(false);
                    }
                } else if (control_mode == &mode_learning || control_mode == &mode_steering) {
                    // if SW7 is ON in LEARNING = record the Wp

                    // create new mission command
                    AP_Mission::Mission_Command cmd = {};

                    // set new waypoint to current location
                    cmd.content.location = current_loc;

                    // make the new command to a waypoint
                    cmd.id = MAV_CMD_NAV_WAYPOINT;

                    // save command
                    if (mission.add_cmd(cmd)) {
                        hal.console->printf("Learning waypoint %u", static_cast<uint32_t>(mission.num_commands()));
                    }
                } else if (control_mode == &mode_auto) {
                    // if SW7 is ON in AUTO = set to RTL
                    set_mode(mode_rtl, MODE_REASON_TX_COMMAND);
                    break;
                }
            }
        }
        break;
    }
}

bool Rover::motor_active()
{
    // Check if armed and output throttle servo is not neutral
    if (hal.util->get_soft_armed()) {
        if (!is_zero(g2.motors.get_throttle())) {
            return true;
        }
    }

    return false;
}
