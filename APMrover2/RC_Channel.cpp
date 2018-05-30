#include "Rover.h"

#include "RC_Channel.h"

// defining these two macros and including the RC_Channels_VarInfo
// header defines the parameter information common to all vehicle
// types
#define RC_CHANNELS_SUBCLASS RC_Channels_Rover
#define RC_CHANNEL_SUBCLASS RC_Channel_Rover

#include <RC_Channel/RC_Channels_VarInfo.h>

int8_t RC_Channels_Rover::flight_mode_channel_number() const
{
    return rover.g.mode_channel;
}

void RC_Channel_Rover::mode_switch_changed(modeswitch_pos_t new_pos)
{
    if (new_pos < 0 || (uint8_t)new_pos > rover.num_modes) {
        // should not have been called
        return;
    }
    Mode *new_mode = rover.mode_from_mode_num((Mode::Number)rover.modes[new_pos].get());
    if (new_mode != nullptr) {
        rover.set_mode(*new_mode, MODE_REASON_TX_COMMAND);
    }
}

// init_aux_switch_function - initialize aux functions
void RC_Channel_Rover::init_aux_function(const aux_func_t ch_option, const aux_switch_pos_t ch_flag)
{
    // init channel options
    switch(ch_option) {
        // the following functions do not need initialising:
    case SAVE_WP:
    case LEARN_CRUISE:
    case ARMDISARM:
    case MANUAL:
    case ACRO:
    case STEERING:
    case HOLD:
    case AUTO:
    case GUIDED:
    case LOITER:
    case FOLLOW:
    case SAILBOAT_TACK:
        break;
    default:
        RC_Channel::init_aux_function(ch_option, ch_flag);
        break;
    }
}


bool RC_Channels_Rover::has_valid_input() const
{
    if (rover.failsafe.bits & FAILSAFE_EVENT_THROTTLE) {
        return false;
    }
    return true;
}

void RC_Channel_Rover::do_aux_function_change_mode(Mode &mode,
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
            rc().reset_mode_switch();
        }
    }
}

void RC_Channel_Rover::do_aux_function(const aux_func_t ch_option, const aux_switch_pos_t ch_flag)
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
                rover.mode_auto.mission.clear();
                if (!rover.set_home_to_current_location(false)) {
                    // ignored
                }
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
                if (rover.mode_auto.mission.add_cmd(cmd)) {
                    hal.console->printf("Added waypoint %u", (unsigned)rover.mode_auto.mission.num_commands());
                }
            }
        }
        break;

    // learn cruise speed and throttle
    case LEARN_CRUISE:
        if (ch_flag == HIGH) {
            rover.cruise_learn_start();
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
    case MANUAL:
        do_aux_function_change_mode(rover.mode_manual, ch_flag);
        break;

    // set mode to Acro
    case ACRO:
        do_aux_function_change_mode(rover.mode_acro, ch_flag);
        break;

    // set mode to Steering
    case STEERING:
        do_aux_function_change_mode(rover.mode_steering, ch_flag);
        break;

    // set mode to Hold
    case HOLD:
        do_aux_function_change_mode(rover.mode_hold, ch_flag);
        break;

    // set mode to Auto
    case AUTO:
        do_aux_function_change_mode(rover.mode_auto, ch_flag);
        break;

    // set mode to RTL
    case RTL:
        do_aux_function_change_mode(rover.mode_rtl, ch_flag);
        break;

    // set mode to SmartRTL
    case SMART_RTL:
        do_aux_function_change_mode(rover.mode_smartrtl, ch_flag);
        break;

    // set mode to Guided
    case GUIDED:
        do_aux_function_change_mode(rover.mode_guided, ch_flag);
        break;

    // Set mode to LOITER
    case LOITER:
        do_aux_function_change_mode(rover.mode_loiter, ch_flag);
        break;

    // Set mode to Follow
    case FOLLOW:
        do_aux_function_change_mode(rover.mode_follow, ch_flag);
        break;

    // set mode to Simple
    case SIMPLE:
        do_aux_function_change_mode(rover.mode_simple, ch_flag);
        break;

    // trigger sailboat tack
    case SAILBOAT_TACK:
        // any switch movement interpreted as request to tack
        rover.control_mode->handle_tack_request();
        break;

    default:
        RC_Channel::do_aux_function(ch_option, ch_flag);
        break;

    }
}
