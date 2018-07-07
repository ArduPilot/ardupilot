#include "Rover.h"

static const int16_t CH_7_PWM_TRIGGER = 1800;

Mode *Rover::mode_from_mode_num(const enum Mode::Number num)
{
    Mode *ret = nullptr;
    switch (num) {
    case Mode::Number::MANUAL:
        ret = &mode_manual;
        break;
    case Mode::Number::ACRO:
        ret = &mode_acro;
        break;
    case Mode::Number::STEERING:
        ret = &mode_steering;
        break;
    case Mode::Number::HOLD:
        ret = &mode_hold;
        break;
    case Mode::Number::LOITER:
        ret = &mode_loiter;
        break;
    case Mode::Number::FOLLOW:
        ret = &mode_follow;
        break;
    case Mode::Number::AUTO:
        ret = &mode_auto;
        break;
    case Mode::Number::RTL:
        ret = &mode_rtl;
        break;
    case Mode::Number::SMART_RTL:
        ret = &mode_smartrtl;
        break;
    case Mode::Number::GUIDED:
       ret = &mode_guided;
        break;
    case Mode::Number::INITIALISING:
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
         RC_Channels::get_radio_in(g.reset_switch_chan-1) > RESET_SWITCH_CHAN_PWM)) {
        if (switch_debouncer == false) {
            // this ensures that mode switches only happen if the
            // switch changes for 2 reads. This prevents momentary
            // spikes in the mode control channel from causing a mode
            // switch
            switch_debouncer = true;
            return;
        }

        Mode *new_mode = mode_from_mode_num((enum Mode::Number)modes[switchPosition].get());
        if (new_mode != nullptr) {
            set_mode(*new_mode, MODE_REASON_TX_COMMAND);
        }

        oldSwitchPosition = switchPosition;
    }

    switch_debouncer = false;
}

uint8_t Rover::readSwitch(void) {
    const uint16_t pulsewidth = RC_Channels::get_radio_in(g.mode_channel - 1);
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

// ready auxiliary switch's position
aux_switch_pos Rover::read_aux_switch_pos()
{
    const uint16_t radio_in = channel_aux->get_radio_in();
    if (radio_in < AUX_SWITCH_PWM_TRIGGER_LOW) return AUX_SWITCH_LOW;
    if (radio_in > AUX_SWITCH_PWM_TRIGGER_HIGH) return AUX_SWITCH_HIGH;
    return AUX_SWITCH_MIDDLE;
}

// initialise position of auxiliary switch
void Rover::init_aux_switch()
{
    aux_ch7 = read_aux_switch_pos();
}

void Rover::do_aux_function_change_mode(Mode &mode,
                                        const aux_switch_pos ch_flag)
{
    switch(ch_flag) {
    case AUX_SWITCH_HIGH:
        set_mode(mode, MODE_REASON_TX_COMMAND);
        break;
    case AUX_SWITCH_MIDDLE:
        // do nothing
        break;
    case AUX_SWITCH_LOW:
        if (control_mode == &mode) {
            reset_control_switch();
        }
    }
}

// read ch7 aux switch
void Rover::read_aux_switch()
{
    // do not consume input during rc or throttle failsafe
    if ((failsafe.bits & FAILSAFE_EVENT_THROTTLE) || (failsafe.bits & FAILSAFE_EVENT_RC)) {
        return;
    }

    // get ch7's current position
    aux_switch_pos aux_ch7_pos = read_aux_switch_pos();

    // return if no change to switch position
    if (aux_ch7_pos == aux_ch7) {
        return;
    }
    aux_ch7 = aux_ch7_pos;

    switch ((enum ch7_option)g.ch7_option.get()) {
    case CH7_DO_NOTHING:
        break;
    case CH7_SAVE_WP:
        if (aux_ch7 == AUX_SWITCH_HIGH) {
            // do nothing if in AUTO mode
            if (control_mode == &mode_auto) {
                return;
            }

            // if disarmed clear mission and set home to current location
            if (!arming.is_armed()) {
                mission.clear();
                set_home_to_current_location(false);
                return;
            }

            // record the waypoint if not in auto mode
            if (control_mode != &mode_auto) {
                // create new mission command
                AP_Mission::Mission_Command cmd = {};

                // set new waypoint to current location
                cmd.content.location = current_loc;

                // make the new command to a waypoint
                cmd.id = MAV_CMD_NAV_WAYPOINT;

                // save command
                if (mission.add_cmd(cmd)) {
                    hal.console->printf("Added waypoint %u", unsigned(mission.num_commands()));
                }
            }
        }
        break;

    // learn cruise speed and throttle
    case CH7_LEARN_CRUISE:
        if (aux_ch7 == AUX_SWITCH_HIGH) {
            cruise_learn_start();
        } else if (aux_ch7 == AUX_SWITCH_LOW) {
            cruise_learn_complete();
        }
        break;

    // arm or disarm the motors
    case CH7_ARM_DISARM:
        if (aux_ch7 == AUX_SWITCH_HIGH) {
            arm_motors(AP_Arming::RUDDER);
        } else if (aux_ch7 == AUX_SWITCH_LOW) {
            disarm_motors();
        }
        break;

    // set mode to Manual
    case CH7_MANUAL:
        do_aux_function_change_mode(rover.mode_manual, aux_ch7);
        break;

    // set mode to Acro
    case CH7_ACRO:
        do_aux_function_change_mode(rover.mode_acro, aux_ch7);
        break;

    // set mode to Steering
    case CH7_STEERING:
        do_aux_function_change_mode(rover.mode_steering, aux_ch7);
        break;

    // set mode to Hold
    case CH7_HOLD:
        do_aux_function_change_mode(rover.mode_hold, aux_ch7);
        break;

    // set mode to Auto
    case CH7_AUTO:
        do_aux_function_change_mode(rover.mode_auto, aux_ch7);
        break;

    // set mode to RTL
    case CH7_RTL:
        do_aux_function_change_mode(rover.mode_rtl, aux_ch7);
        break;

    // set mode to SmartRTL
    case CH7_SMART_RTL:
        do_aux_function_change_mode(rover.mode_smartrtl, aux_ch7);
        break;

    // set mode to Guided
    case CH7_GUIDED:
        do_aux_function_change_mode(rover.mode_guided, aux_ch7);
        break;

    // Set mode to LOITER
    case CH7_LOITER:
        do_aux_function_change_mode(rover.mode_loiter, aux_ch7);
        break;

    // Set mode to Follow
    case CH7_FOLLOW:
        do_aux_function_change_mode(rover.mode_follow, aux_ch7);
        break;

    }
}
