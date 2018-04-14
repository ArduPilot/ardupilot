#include "Rover.h"

static const int16_t CH_7_PWM_TRIGGER = 1800;

Mode *Rover::mode_from_mode_num(const enum mode num)
{
    Mode *ret = nullptr;
    switch (num) {
    case MANUAL:
        ret = &mode_manual;
        break;
    case ACRO:
        ret = &mode_acro;
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
    case SMART_RTL:
        ret = &mode_smartrtl;
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

        Mode *new_mode = mode_from_mode_num((enum mode)modes[switchPosition].get());
        if (new_mode != nullptr) {
            set_mode(*new_mode, MODE_REASON_TX_COMMAND);
        }

        oldSwitchPosition = switchPosition;
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
                    hal.console->printf("Added waypoint %u", static_cast<uint32_t>(mission.num_commands()));
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
        if (aux_ch7 == AUX_SWITCH_HIGH) {
            set_mode(mode_manual, MODE_REASON_TX_COMMAND);
        } else if ((aux_ch7 == AUX_SWITCH_LOW) && (control_mode == &mode_manual)) {
            reset_control_switch();
        }
        break;

    // set mode to Acro
    case CH7_ACRO:
        if (aux_ch7 == AUX_SWITCH_HIGH) {
            set_mode(mode_acro, MODE_REASON_TX_COMMAND);
        } else if ((aux_ch7 == AUX_SWITCH_LOW) && (control_mode == &mode_acro)) {
            reset_control_switch();
        }
        break;

    // set mode to Steering
    case CH7_STEERING:
        if (aux_ch7 == AUX_SWITCH_HIGH) {
            set_mode(mode_steering, MODE_REASON_TX_COMMAND);
        } else if ((aux_ch7 == AUX_SWITCH_LOW) && (control_mode == &mode_steering)) {
            reset_control_switch();
        }
        break;

    // set mode to Hold
    case CH7_HOLD:
        if (aux_ch7 == AUX_SWITCH_HIGH) {
            set_mode(mode_hold, MODE_REASON_TX_COMMAND);
        } else if ((aux_ch7 == AUX_SWITCH_LOW) && (control_mode == &mode_hold)) {
            reset_control_switch();
        }
        break;

    // set mode to Auto
    case CH7_AUTO:
        if (aux_ch7 == AUX_SWITCH_HIGH) {
            set_mode(mode_auto, MODE_REASON_TX_COMMAND);
        } else if ((aux_ch7 == AUX_SWITCH_LOW) && (control_mode == &mode_auto)) {
            reset_control_switch();
        }
        break;

    // set mode to RTL
    case CH7_RTL:
        if (aux_ch7 == AUX_SWITCH_HIGH) {
            set_mode(mode_rtl, MODE_REASON_TX_COMMAND);
        } else if ((aux_ch7 == AUX_SWITCH_LOW) && (control_mode == &mode_rtl)) {
            reset_control_switch();
        }
        break;

    // set mode to SmartRTL
    case CH7_SMART_RTL:
        if (aux_ch7 == AUX_SWITCH_HIGH) {
            set_mode(mode_smartrtl, MODE_REASON_TX_COMMAND);
        } else if ((aux_ch7 == AUX_SWITCH_LOW) && (control_mode == &mode_smartrtl)) {
            reset_control_switch();
        }
        break;

    // set mode to Guided
    case CH7_GUIDED:
        if (aux_ch7 == AUX_SWITCH_HIGH) {
            set_mode(mode_guided, MODE_REASON_TX_COMMAND);
        } else if ((aux_ch7 == AUX_SWITCH_LOW) && (control_mode == &mode_guided)) {
            reset_control_switch();
        }
        break;
    }
}

// return true if motors are moving
bool Rover::motor_active()
{
    // if soft disarmed, motors not active
    if (!hal.util->get_soft_armed()) {
        return false;
    }

    // check throttle is active
    if (!is_zero(g2.motors.get_throttle())) {
        return true;
    }

    // skid-steering vehicles active when steering
    if (g2.motors.have_skid_steering() && !is_zero(g2.motors.get_steering())) {
        return true;
    }

    return false;
}
