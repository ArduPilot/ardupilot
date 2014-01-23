/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <AP_Arming.h>
#include <AP_Notify.h>

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo AP_Arming::var_info[] PROGMEM = {
    // @Param: REQUIRE
    // @DisplayName: Require Arming Motors 
    // @Description: Arming disabled until some requirements are met. If 0, there are no requirements (arm immediately).  If 1, require rudder stick or GCS arming before arming motors and send THR_MIN PWM to throttle channel when disarmed.  If 2, require rudder stick or GCS arming and send 0 PWM to throttle channel when disarmed. See the ARMING_CHECK_* parameters to see what checks are done before arming. Note, if setting this parameter to 0 a reboot is required to arm the plane.  Also note, even with this parameter at 0, if ARMING_CHECK parameter is not also zero the plane may fail to arm throttle at boot due to a pre-arm check failure.
    // @Values: 0:Disabled,1:THR_MIN PWM when disarmed,2:0 PWM when disarmed
    // @User: Advanced
    AP_GROUPINFO("REQUIRE",     0,      AP_Arming,  require,                 0),

    // @Param: DIS_RUD
    // @DisplayName: Disable Rudder Arming
    // @Description: Do not allow arming via the rudder input stick.
    // @Values: 0:Disabled (Rudder Arming Allowed),1:Enabled(No Rudder Arming)
    // @User: Advanced
    AP_GROUPINFO("DIS_RUD",     1,     AP_Arming,  disable_rudder_arm,       0),
    
    // @Param: CHECK
    // @DisplayName: Arm Checks to Peform (bitmask)
    // @Description: Checks prior to arming motor. 
    // @Values: 0:None,1:All,2:Barometer,4:Compass,8:GPS,16:INS,32:Parameters,64:Manual RC Trasmitter,128:Board voltage,256:Battery Level 
    // @User: Advanced
    AP_GROUPINFO("CHECK",        2,     AP_Arming,  checks_to_perform,       0),

    AP_GROUPEND
};

//The function point is particularly hacky, hacky, tacky
//but I don't want to reimplement messaging to GCS at the moment:
AP_Arming::AP_Arming(const AP_AHRS &ahrs_ref, const AP_Baro &baro,
                     const bool &home_set, gcs_send_t_p gcs_print_func)
   : armed(false)
   , arming_method(NONE)
   , ahrs(ahrs_ref)
   , barometer(baro)
   , home_is_set(home_set)
   , gcs_send_text_P(gcs_print_func)
{
    AP_Param::setup_object_defaults(this, var_info);
}

bool AP_Arming::is_armed() 
{ 
    return require == NONE || armed; 
}

uint16_t AP_Arming::get_enabled_checks() 
{
    return checks_to_perform;
}

void AP_Arming::set_enabled_checks(uint16_t ap) {
    checks_to_perform = ap;
}

bool AP_Arming::barometer_checks(bool report) 
{
    if ((checks_to_perform & ARMING_CHECK_ALL) ||
        (checks_to_perform & ARMING_CHECK_BARO)) {
        if (! barometer.healthy) {
            if (report) {
                gcs_send_text_P(SEVERITY_HIGH,PSTR("PreArm: Baro not healthy!"));
            }
            return false;
        }
    }

    return true;
}

bool AP_Arming::compass_checks(bool report) 
{
    if ((checks_to_perform) & ARMING_CHECK_ALL ||
        (checks_to_perform) & ARMING_CHECK_COMPASS) {
        const Compass* compass = ahrs.get_compass();

        //if there is no compass and the user has specifically asked to check
        //the compass, then there is a problem
        if (compass == NULL && (checks_to_perform & ARMING_CHECK_COMPASS)) {
            if (report) {
                gcs_send_text_P(SEVERITY_HIGH,PSTR("PreArm: No compass detected."));
            }
            return false;
        } else if (compass == NULL) {
            //if the user's not asking to check and there isn't a compass
            //then skip compass checks
            return true;
        }

        if (! compass->healthy()) {
            if (report) {
                gcs_send_text_P(SEVERITY_HIGH,PSTR("PreArm: Compass not healthy!"));
            }
            return false;
        }
        // check compass learning is on or offsets have been set
        Vector3f offsets = compass->get_offsets();
        if(!compass->_learn && offsets.length() == 0) {
            if (report) {
                gcs_send_text_P(SEVERITY_HIGH,PSTR("PreArm: Compass not calibrated"));
            }
            return false;
        }
        
    }

    return true;
}

bool AP_Arming::gps_checks(bool report) 
{
    if ((checks_to_perform & ARMING_CHECK_ALL) ||
        (checks_to_perform & ARMING_CHECK_GPS)) {
        const GPS *gps = ahrs.get_gps();

        //If no GPS and the user has specifically asked to check GPS, then
        //there is a problem
        if (gps == NULL && (checks_to_perform & ARMING_CHECK_GPS)) {
            if (report) {
                gcs_send_text_P(SEVERITY_HIGH,PSTR("PreArm: No GPS detected."));
            }
            return false;
        } else if (gps == NULL) {
            //assume the user doesn't have a GPS on purpose
            return true;
        } 

        //GPS OK?
        if (!home_is_set || gps->status() != GPS::GPS_OK_FIX_3D ||              
            AP_Notify::flags.gps_glitching ||
            AP_Notify::flags.failsafe_gps) {
            if (report) {
                gcs_send_text_P(SEVERITY_HIGH,PSTR("PreArm: Bad GPS Pos"));
            }
            return false;
        }      
    }

    return true;
}

bool AP_Arming::battery_checks(bool report) 
{
    if ((checks_to_perform & ARMING_CHECK_ALL) ||
        (checks_to_perform & ARMING_CHECK_BATTERY)) {

        if (AP_Notify::flags.failsafe_battery) {
            if (report) {
                gcs_send_text_P(SEVERITY_HIGH,PSTR("PreArm: Battery failsafe on."));
            }
            return false;
        }
    }

    return true;
}

bool AP_Arming::hardware_safety_check(bool report) 
{
    // check if safety switch has been pushed
    if (hal.util->safety_switch_state() == AP_HAL::Util::SAFETY_DISARMED) {
        if (report) {
            gcs_send_text_P(SEVERITY_HIGH,PSTR("PreArm: Hardware Safety Switch"));
        }
        return false;
    }

    return true;
}

bool AP_Arming::manual_transmitter_checks(bool report) 
{
    if ((checks_to_perform & ARMING_CHECK_ALL) ||
        (checks_to_perform & ARMING_CHECK_RC)) {

        if (AP_Notify::flags.failsafe_radio) {
            if (report) {
                gcs_send_text_P(SEVERITY_HIGH,PSTR("PreArm: Radio failsafe on."));
            }
            return false;
        }

        //TODO verify radio calibration
        //Requires access to Parameters ... which are implemented a little
        //differently for Rover, Plane, and Copter.
    }

    return true;
}

bool AP_Arming::pre_arm_checks(bool report) 
{
    if (armed || require == NONE) {
        // if we are already armed or don't need any arming checks
        // then skip the checks
        return true;
    }

    if (! hardware_safety_check(report))
        return false;

    if (! barometer_checks(report))
        return false;

    if (! compass_checks(report))
        return false;

    if (! gps_checks(report))
        return false;

    if (! battery_checks(report))
        return false;

    if (! manual_transmitter_checks(report))
        return false;

    //all checks passed, allow arming!
    return true;
}

//returns true if arming occured successfully
bool AP_Arming::arm(uint8_t method) 
{
    if (armed) { //already armed
        return false;
    }

    //are arming checks disabled?
    if (checks_to_perform == ARMING_CHECK_NONE) {
        armed = true;
        arming_method = NONE;
        gcs_send_text_P(SEVERITY_HIGH,PSTR("Throttle armed!"));
        return true;
    }

    if (pre_arm_checks(true)) {
        armed = true;
        arming_method = method;

        gcs_send_text_P(SEVERITY_HIGH,PSTR("Throttle armed!"));

        //TODO: Log motor arming to the dataflash
        //Can't do this from this class until there is a unified logging library

    } else {
        armed = false;
        arming_method = NONE;
    }

    return armed;
}

//returns true if disarming occurred successfully
bool AP_Arming::disarm() 
{
    if (! armed) { // already disarmed
        return false;
    }
    armed = false;

    gcs_send_text_P(SEVERITY_HIGH,PSTR("Throttle disarmed!"));

    //TODO: Log motor disarming to the dataflash
    //Can't do this from this class until there is a unified logging library.

    return true;
}

AP_Arming::ArmingRequired AP_Arming::arming_required() 
{
    return (AP_Arming::ArmingRequired)require.get();
}

bool AP_Arming::rudder_arming_enabled() 
{
    if (disable_rudder_arm == 0)
        return true;

    return false;
}
