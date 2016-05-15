/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/*
 *  Copyright (c) BirdsEyeView Aerobotics, LLC, 2016.
 *
 *  This program is free software: you can redistribute it and/or modify it
 *  under the terms of the GNU General Public License version 3 as published
 *  by the Free Software Foundation.
 *
 *  This program is distributed in the hope that it will be useful, but
 *  WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General
 *  Public License version 3 for more details.
 *
 *  You should have received a copy of the GNU General Public License version
 *  3 along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

// set_mode - change flight mode and perform any necessary initialisation
// optional force parameter used to force the flight mode change (used only first time mode is set)
// returns true if mode was succesfully set
// ACRO, STABILIZE, ALTHOLD, LAND, DRIFT and SPORT can always be set successfully but the return state of other flight modes should be checked and the caller should deal with failures appropriately
static bool set_mode(uint8_t mode)
{
    // boolean to record if flight mode could be set
    bool success = false;
    bool ignore_checks = !motors.armed();   // allow switching to any mode if disarmed.  We rely on the arming check to perform

    // return immediately if we are already in the desired mode
    if (mode == control_mode) {
        return true;
    }

    //Check to see if the key is good. Don't allow into unauthorized flight modes without proper key
    if(get_key_level() < BEV_Key::KEY_PRO) {
        if( (mode != ACRO) && (mode != STABILIZE) && (mode != ALT_HOLD) && (mode != RTL) && (mode != LAND)) {
            //attempting to go into an unauthorized flight mode
            gcs_send_text_P(SEVERITY_HIGH,PSTR("Pro key needed"));
            return false;
        }
    }

    switch(mode) {
    //BEV no acro mode
    /*
        case ACRO:
            //copter
            success = acro_init(ignore_checks);

            //plane
            auto_throttle_mode = false;
            break;
     */

        case STABILIZE:
            //BEV allow stabilize mode as permitted by parameter
            if(!g.stabilize_allow) {
                gcs_send_text_P(SEVERITY_HIGH,PSTR("Stabilize Mode Disabled"));
                return false;
            }

            //copter
            success = stabilize_init(ignore_checks);

            //plane
            auto_throttle_mode = false;
            break;

        case ALT_HOLD:
            //copter
            success = althold_init(ignore_checks);

            //plane
            auto_throttle_mode = true;
            SpdHgt_Controller.initialize(); //this sets the target altitude appropriately
            break;

        case AUTO:
            success = auto_init(ignore_checks);

            //plane
            auto_throttle_mode = true;
            SpdHgt_Controller.initialize();
            break;

        //BEV removing circle mode
        /*
        case CIRCLE:
            success = circle_init(ignore_checks);

            //plane
            auto_throttle_mode = true;
            SpdHgt_Controller.initialize();
            break;
        */
        case LOITER:
            success = loiter_init(ignore_checks);

            //plane
            auto_throttle_mode = true;
            next_WP_loc = current_loc;

            break;

        case GUIDED:
            success = guided_init(ignore_checks);

            //plane
            auto_throttle_mode = true;
            next_WP_loc = current_loc; //just in case the desired location isn't updated
            break;

        case LAND:
            success = land_init(ignore_checks);

            //plane
            auto_throttle_mode = true;
            next_WP_loc = current_loc;
            break;

        case RTL:
            success = rtl_init(ignore_checks);

            //plane
            auto_throttle_mode = true;
            prev_WP_loc = current_loc;
            next_WP_loc = ahrs.get_home();
            alt_hold_gs_des_alt = get_RTL_alt();
            break;

        default:
            success = false;
            break;
    }

    // update flight mode
    if (success) {
        // perform any cleanup required by previous flight mode
        exit_mode(control_mode, mode);
        control_mode = mode;
        Log_Write_Mode(control_mode);
    }else{
        // Log error that we failed to enter desired flight mode
        Log_Write_Error(ERROR_SUBSYSTEM_FLIGHT_MODE,mode);
    }

    // return success or failure
    return success;
}

// update_flight_mode - calls the appropriate attitude controllers based on flight mode
// called at 100hz or more
static void update_flight_mode()
{
    //BEV this allows elevons to be active even when in copter mode
    drive_elevons();

    switch (control_mode) {
        case STABILIZE:
            stabilize_run();
            break;

        case ALT_HOLD:
            althold_run();
            break;

        case AUTO:
            auto_run();
            break;

        case LOITER:
            loiter_run();
            break;

        case GUIDED:
            guided_run();
            break;

        case LAND:
            land_run();
            break;

        case RTL:
            rtl_run();
            break;
    }

}

// exit_mode - high level call to organise cleanup as a flight mode is exited
static void exit_mode(uint8_t old_control_mode, uint8_t new_control_mode)
{

    // stop mission when we leave auto mode
    if (old_control_mode == AUTO) {
        wp_distance = 0;
        if (mission.state() == AP_Mission::MISSION_RUNNING) {
            mission.stop();
        }
        //BEV reset mission if final command was a loiter unlimited.
        if( (mission.get_current_nav_cmd().id == MAV_CMD_NAV_LOITER_UNLIM) ||
            (mission.get_current_nav_cmd().id == MAV_CMD_NAV_RETURN_TO_LAUNCH)) {
            mission.reset();
        }
    }

    // smooth throttle transition when switching from manual to automatic flight modes
    if (manual_flight_mode(old_control_mode) && !manual_flight_mode(new_control_mode) && motors.armed() && !ap.land_complete) {
        // this assumes all manual flight modes use get_pilot_desired_throttle to translate pilot input to output throttle
        set_accel_throttle_I_from_pilot_throttle(get_pilot_desired_throttle(g.rc_3.control_in));
    }
}

// returns true or false whether mode requires GPS
static bool mode_requires_GPS(uint8_t mode) {
    switch(mode) {
        case AUTO:
        case GUIDED:
        case LOITER:
        case RTL:
        case CIRCLE:
        case DRIFT:
        case POSHOLD:
            return true;
        default:
            return false;
    }

    return false;
}

// manual_flight_mode - returns true if flight mode is completely manual (i.e. roll, pitch, yaw and throttle are controlled by pilot)
static bool manual_flight_mode(uint8_t mode) {
    switch(mode) {
        case ACRO:
        case STABILIZE:
            return true;
        default:
            return false;
    }

    return false;
}

//
// print_flight_mode - prints flight mode to serial port.
//
static void
print_flight_mode(AP_HAL::BetterStream *port, uint8_t mode)
{
    switch (mode) {
    case STABILIZE:
        port->print_P(PSTR("STABILIZE"));
        break;
    case ACRO:
        port->print_P(PSTR("ACRO"));
        break;
    case ALT_HOLD:
        port->print_P(PSTR("ALT_HOLD"));
        break;
    case AUTO:
        port->print_P(PSTR("AUTO"));
        break;
    case GUIDED:
        port->print_P(PSTR("GUIDED"));
        break;
    case LOITER:
        port->print_P(PSTR("LOITER"));
        break;
    case RTL:
        port->print_P(PSTR("RTL"));
        break;
    case CIRCLE:
        port->print_P(PSTR("CIRCLE"));
        break;
    case LAND:
        port->print_P(PSTR("LAND"));
        break;
    case OF_LOITER:
        port->print_P(PSTR("OF_LOITER"));
        break;
    case DRIFT:
        port->print_P(PSTR("DRIFT"));
        break;
    case SPORT:
        port->print_P(PSTR("SPORT"));
        break;
    case FLIP:
        port->print_P(PSTR("FLIP"));
        break;
    case AUTOTUNE:
        port->print_P(PSTR("AUTOTUNE"));
        break;
    case POSHOLD:
        port->print_P(PSTR("POSHOLD"));
        break;
    default:
        port->printf_P(PSTR("Mode(%u)"), (unsigned)mode);
        break;
    }
}

