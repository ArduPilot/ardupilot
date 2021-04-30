#include "Rover.h"

#include "AP_AuxFunc.h"

// init_aux_switch_function - initialize aux functions
bool AP_AuxFunc_Rover::init_function(const Function function, const SwitchPos pos)
{
    // init channel options
    switch (function) {
    // the following functions do not need initialising:
    case Function::ACRO:
    case Function::AUTO:
    case Function::FOLLOW:
    case Function::GUIDED:
    case Function::HOLD:
    case Function::LEARN_CRUISE:
    case Function::LOITER:
    case Function::MAINSAIL:
    case Function::MANUAL:
    case Function::PITCH:
    case Function::ROLL:
    case Function::WALKING_HEIGHT:
    case Function::RTL:
    case Function::SAILBOAT_TACK:
    case Function::SAVE_TRIM:
    case Function::SAVE_WP:
    case Function::SIMPLE:
    case Function::SMART_RTL:
    case Function::STEERING:
    case Function::WIND_VANE_DIR_OFSSET:
        return true;
    case Function::SAILBOAT_MOTOR_3POS:
        return run_function(function, pos, TriggerSource::INIT);
    default:
        return AP_AuxFunc::init_function(function, pos);
    }
}

void AP_AuxFunc_Rover::do_function_change_mode(Mode &mode,
        const SwitchPos pos)
{
    switch (pos) {
    case SwitchPos::HIGH:
        rover.set_mode(mode, ModeReason::RC_COMMAND);
        break;
    case SwitchPos::MIDDLE:
        // do nothing
        break;
    case SwitchPos::LOW:
        if (rover.control_mode == &mode) {
            rc().reset_mode_switch();
        }
    }
}

void AP_AuxFunc_Rover::add_waypoint_for_current_loc()
{
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

void AP_AuxFunc_Rover::do_function_sailboat_motor_3pos(const SwitchPos pos)
{
    switch (pos) {
    case SwitchPos::HIGH:
        rover.g2.sailboat.set_motor_state(Sailboat::UseMotor::USE_MOTOR_ALWAYS);
        break;
    case SwitchPos::MIDDLE:
        rover.g2.sailboat.set_motor_state(Sailboat::UseMotor::USE_MOTOR_ASSIST);
        break;
    case SwitchPos::LOW:
        rover.g2.sailboat.set_motor_state(Sailboat::UseMotor::USE_MOTOR_NEVER);
        break;
    }
}

bool AP_AuxFunc_Rover::do_function(const Function function, const SwitchPos pos)
{
    switch (function) {
    case Function::DO_NOTHING:
        break;
    case Function::SAVE_WP:
        if (pos == SwitchPos::HIGH) {
            // do nothing if in AUTO mode
            if (rover.control_mode == &rover.mode_auto) {
                break;
            }

            // if disarmed clear mission and set home to current location
            if (!rover.arming.is_armed()) {
                rover.mode_auto.mission.clear();
                if (!rover.set_home_to_current_location(false)) {
                    // ignored
                }
                break;
            }

            // record the waypoint if not in auto mode
            if (rover.control_mode != &rover.mode_auto) {
                if (rover.mode_auto.mission.num_commands() == 0) {
                    // add a home location....
                    add_waypoint_for_current_loc();
                }
                add_waypoint_for_current_loc();
            }
        }
        break;

    // learn cruise speed and throttle
    case Function::LEARN_CRUISE:
        if (pos == SwitchPos::HIGH) {
            rover.cruise_learn_start();
        }
        break;

    // set mode to Manual
    case Function::MANUAL:
        do_function_change_mode(rover.mode_manual, pos);
        break;

    // set mode to Acro
    case Function::ACRO:
        do_function_change_mode(rover.mode_acro, pos);
        break;

    // set mode to Steering
    case Function::STEERING:
        do_function_change_mode(rover.mode_steering, pos);
        break;

    // set mode to Hold
    case Function::HOLD:
        do_function_change_mode(rover.mode_hold, pos);
        break;

    // set mode to Auto
    case Function::AUTO:
        do_function_change_mode(rover.mode_auto, pos);
        break;

    // set mode to RTL
    case Function::RTL:
        do_function_change_mode(rover.mode_rtl, pos);
        break;

    // set mode to SmartRTL
    case Function::SMART_RTL:
        do_function_change_mode(rover.mode_smartrtl, pos);
        break;

    // set mode to Guided
    case Function::GUIDED:
        do_function_change_mode(rover.mode_guided, pos);
        break;

    // Set mode to LOITER
    case Function::LOITER:
        do_function_change_mode(rover.mode_loiter, pos);
        break;

    // Set mode to Follow
    case Function::FOLLOW:
        do_function_change_mode(rover.mode_follow, pos);
        break;

    // set mode to Simple
    case Function::SIMPLE:
        do_function_change_mode(rover.mode_simple, pos);
        break;

    // trigger sailboat tack
    case Function::SAILBOAT_TACK:
        // any switch movement interpreted as request to tack
        rover.control_mode->handle_tack_request();
        break;

    // sailboat motor state 3pos
    case Function::SAILBOAT_MOTOR_3POS:
        do_function_sailboat_motor_3pos(pos);
        break;

    // save steering trim
    case Function::SAVE_TRIM:
        if (!rover.g2.motors.have_skid_steering() && rover.arming.is_armed() &&
            (rover.control_mode != &rover.mode_loiter)
            && (rover.control_mode != &rover.mode_hold) && pos == SwitchPos::HIGH) {
            SRV_Channels::set_trim_to_servo_out_for(SRV_Channel::k_steering);
            gcs().send_text(MAV_SEVERITY_CRITICAL, "Steering trim saved!");
        }
        break;

    // manual input, nothing to do
    case Function::MAINSAIL:
    case Function::PITCH:
    case Function::ROLL:
    case Function::WALKING_HEIGHT:
    case Function::WIND_VANE_DIR_OFSSET:
        break;

    default:
        return AP_AuxFunc::do_function(function, pos);

    }

    return true;
}
