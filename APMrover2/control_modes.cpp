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
         RC_Channels::get_radio_in(g.reset_switch_chan-1) > RESET_SWITCH_CHAN_PWM)) {
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

void AP_RCSwitch_Rover::reset_control_switch()
{
    rover.oldSwitchPosition = 254;
    rover.read_control_switch();
}

// init_aux_switch_function - initialize aux functions
void AP_RCSwitch_Rover::init_aux_function(const aux_func_t ch_option, const aux_switch_pos_t ch_flag)
{
    // init channel options
    switch(ch_option) {
        // at the moment nothing needs initialisation:
        // do_aux_function(ch_option, ch_flag);
        break;
    default:
        AP_RCSwitch::init_aux_function(ch_option, ch_flag);
        break;
    }
}


bool AP_RCSwitch_Rover::in_rc_failsafe() const
{
    return ((rover.failsafe.bits & FAILSAFE_EVENT_THROTTLE) ||
            (rover.failsafe.bits & FAILSAFE_EVENT_RC));
}

void AP_RCSwitch_Rover::do_aux_function_change_mode(Mode &mode,
                                                    const aux_switch_pos_t ch_flag)
{
    switch(ch_flag) {
    case HIGH:
        rover.set_mode(mode, MODE_REASON_TX_COMMAND);
        break;
    case MIDDLE:
        // do nothing
        break;
    case LOW:
        if (rover.control_mode == &mode) {
            reset_control_switch();
        }
    }
}

void AP_RCSwitch_Rover::do_aux_function(const aux_func_t ch_option, const aux_switch_pos_t ch_flag)
{
    switch (ch_option) {
    case DO_NOTHING:
        break;
    case SAVE_WP:
        if (ch_flag == HIGH) {
            // do nothing if in AUTO mode
            if (rover.control_mode == &rover.mode_auto) {
                return;
            }

            // if disarmed clear mission and set home to current location
            if (!rover.arming.is_armed()) {
                rover.mission.clear();
                rover.set_home_to_current_location(false);
                return;
            }

            // record the waypoint if not in auto mode
            if (rover.control_mode != &rover.mode_auto) {
                // create new mission command
                AP_Mission::Mission_Command cmd = {};

                // set new waypoint to current location
                cmd.content.location = rover.current_loc;

                // make the new command to a waypoint
                cmd.id = MAV_CMD_NAV_WAYPOINT;

                // save command
                if (rover.mission.add_cmd(cmd)) {
                    hal.console->printf("Added waypoint %u", static_cast<uint32_t>(rover.mission.num_commands()));
                }
            }
        }
        break;

    // learn cruise speed and throttle
    case LEARN_CRUISE:
        if (ch_flag == HIGH) {
            rover.cruise_learn_start();
        } else if (ch_flag == LOW) {
            rover.cruise_learn_complete();
        }
        break;

    // arm or disarm the motors
    case ARMDISARM:
        if (ch_flag == HIGH) {
            rover.arm_motors(AP_Arming::RUDDER);
        } else if (ch_flag == LOW) {
            rover.disarm_motors();
        }
        break;

    // set mode to Manual
    case aux_func_t::MANUAL:
        do_aux_function_change_mode(rover.mode_manual, ch_flag);
        break;

    // set mode to Acro
    case aux_func_t::ACRO:
        do_aux_function_change_mode(rover.mode_acro, ch_flag);
        break;

    // set mode to Steering
    case aux_func_t::STEERING:
        do_aux_function_change_mode(rover.mode_steering, ch_flag);
        break;

    // set mode to Hold
    case aux_func_t::HOLD:
        do_aux_function_change_mode(rover.mode_hold, ch_flag);
        break;

    // set mode to Auto
    case aux_func_t::AUTO:
        do_aux_function_change_mode(rover.mode_auto, ch_flag);
        break;

    // set mode to RTL
    case aux_func_t::RTL:
        do_aux_function_change_mode(rover.mode_rtl, ch_flag);
        break;

    // set mode to SmartRTL
    case aux_func_t::SMART_RTL:
        do_aux_function_change_mode(rover.mode_smartrtl, ch_flag);
        break;

    // set mode to Guided
    case aux_func_t::GUIDED:
        do_aux_function_change_mode(rover.mode_guided, ch_flag);
        break;
    default:
        AP_RCSwitch::do_aux_function(ch_option, ch_flag);
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
