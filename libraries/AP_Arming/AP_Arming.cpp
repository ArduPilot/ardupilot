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
    AP_GROUPINFO("REQUIRE",     0,      AP_Arming,  require,                 1),

    // @Param: DIS_RUD
    // @DisplayName: Disable Rudder Arming
    // @Description: Do not allow arming via the rudder input stick.
    // @Values: 0:Disabled (Rudder Arming Allowed),1:Enabled(No Rudder Arming)
    // @User: Advanced
    AP_GROUPINFO("DIS_RUD",     1,     AP_Arming,  disable_rudder_arm,       0),
    
    // @Param: CHECK
    // @DisplayName: Arm Checks to Peform (bitmask)
    // @Description: Checks prior to arming motor. This is a bitmask of checks that will be performed befor allowing arming. The default is no checks, allowing arming at any time. You can select whatever checks you prefer by adding together the values of each check type to set this parameter. For example, to only allow arming when you have GPS lock and no RC failsafe you would set ARMING_CHECK to 72. For most users it is recommended that you set this to 1 to enable all checks.
    // @Values: 0:None,1:All,2:Barometer,4:Compass,8:GPS,16:INS,32:Parameters,64:RC Failsafe,128:Board voltage,256:Battery Level,512:Airspeed,1024:LoggingAvailable
    // @User: Advanced
    AP_GROUPINFO("CHECK",        2,     AP_Arming,  checks_to_perform,       ARMING_CHECK_ALL),

    AP_GROUPEND
};

//The function point is particularly hacky, hacky, tacky
//but I don't want to reimplement messaging to GCS at the moment:
AP_Arming::AP_Arming(const AP_AHRS &ahrs_ref, const AP_Baro &baro, Compass &compass,
                     const enum HomeState &home_set, gcs_send_t_p gcs_print_func)
    : armed(false)
   , logging_available(false)
   , skip_gyro_cal(false)
   , arming_method(NONE)
   , ahrs(ahrs_ref)
   , barometer(baro)
   , _compass(compass)
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
        if (! barometer.healthy()) {
            if (report) {
                gcs_send_text_P(SEVERITY_HIGH,PSTR("PreArm: Barometer not healthy!"));
            }
            return false;
        }
    }

    return true;
}

bool AP_Arming::airspeed_checks(bool report) 
{
    if ((checks_to_perform & ARMING_CHECK_ALL) ||
        (checks_to_perform & ARMING_CHECK_AIRSPEED)) {
        const AP_Airspeed *airspeed = ahrs.get_airspeed();
        if (airspeed == NULL) {
            // not an airspeed capable vehicle
            return true;
        }
        if (airspeed->enabled() && airspeed->use() && !airspeed->healthy()) {
            if (report) {
                gcs_send_text_P(SEVERITY_HIGH,PSTR("PreArm: airspeed not healthy"));
            }
            return false;
        }
    }

    return true;
}

bool AP_Arming::logging_checks(bool report) 
{
    if ((checks_to_perform & ARMING_CHECK_ALL) ||
        (checks_to_perform & ARMING_CHECK_LOGGING)) {
        if (!logging_available) {
            if (report) {
                gcs_send_text_P(SEVERITY_HIGH,PSTR("PreArm: logging not available"));
            }
            return false;
        }
    }
    return true;
}

bool AP_Arming::ins_checks(bool report) 
{
    if ((checks_to_perform & ARMING_CHECK_ALL) ||
        (checks_to_perform & ARMING_CHECK_INS)) {
        const AP_InertialSensor &ins = ahrs.get_ins();
        if (! ins.get_gyro_health_all()) {
            if (report) {
                gcs_send_text_P(SEVERITY_HIGH,PSTR("PreArm: gyros not healthy!"));
            }
            return false;
        }
        if (!skip_gyro_cal && ! ins.gyro_calibrated_ok_all()) {
            if (report) {
                gcs_send_text_P(SEVERITY_HIGH,PSTR("PreArm: gyros not calibrated!"));
            }
            return false;
        }
        if (! ins.get_accel_health_all()) {
            if (report) {
                gcs_send_text_P(SEVERITY_HIGH,PSTR("PreArm: accels not healthy!"));
            }
            return false;
        }
        if (!ahrs.healthy()) {
            if (report) {
                gcs_send_text_P(SEVERITY_HIGH,PSTR("PreArm: AHRS not healthy!"));
            }
            return false;
        }
        if (!ins.calibrated()) {
            if (report) {
                gcs_send_text_P(SEVERITY_HIGH,PSTR("PreArm: 3D accel cal needed"));
            }
            return false;
        }
#if INS_MAX_INSTANCES > 1
        // check all accelerometers point in roughly same direction
        if (ins.get_accel_count() > 1) {
            const Vector3f &prime_accel_vec = ins.get_accel();
            for(uint8_t i=0; i<ins.get_accel_count(); i++) {
                // get next accel vector
                const Vector3f &accel_vec = ins.get_accel(i);
                Vector3f vec_diff = accel_vec - prime_accel_vec;
                // allow for up to 0.3 m/s/s difference
                if (vec_diff.length() > 0.3f) {
                    if (report) {
                        gcs_send_text_P(SEVERITY_HIGH,PSTR("PreArm: inconsistent Accelerometers"));
                    }
                    return false;
                }
            }
        }

        // check all gyros are giving consistent readings
        if (ins.get_gyro_count() > 1) {
            const Vector3f &prime_gyro_vec = ins.get_gyro();
            for(uint8_t i=0; i<ins.get_gyro_count(); i++) {
                // get next gyro vector
                const Vector3f &gyro_vec = ins.get_gyro(i);
                Vector3f vec_diff = gyro_vec - prime_gyro_vec;
                // allow for up to 5 degrees/s difference
                if (vec_diff.length() > radians(5)) {
                    if (report) {
                        gcs_send_text_P(SEVERITY_HIGH,PSTR("PreArm: inconsistent gyros"));
                    }
                    return false;
                }
            }
        }
#endif
    }

    return true;
}

bool AP_Arming::compass_checks(bool report) 
{
    if ((checks_to_perform) & ARMING_CHECK_ALL ||
        (checks_to_perform) & ARMING_CHECK_COMPASS) {

        if (!_compass.use_for_yaw()) {
            // compass use is disabled
            return true;
        }

        if (!_compass.healthy()) {
            if (report) {
                gcs_send_text_P(SEVERITY_HIGH,PSTR("PreArm: Compass not healthy!"));
            }
            return false;
        }
        // check compass learning is on or offsets have been set
        if (!_compass.learn_offsets_enabled() && !_compass.configured()) {
            if (report) {
                gcs_send_text_P(SEVERITY_HIGH,PSTR("PreArm: Compass not calibrated"));
            }
            return false;
        }

        // check for unreasonable compass offsets
        Vector3f offsets = _compass.get_offsets();
        if (offsets.length() > 600) {
            if (report) {
                gcs_send_text_P(SEVERITY_HIGH,PSTR("PreArm: Compass offsets too high"));
            }
            return false;
        }

#if CONFIG_HAL_BOARD == HAL_BOARD_APM1 || CONFIG_HAL_BOARD == HAL_BOARD_APM2
# define COMPASS_MAGFIELD_EXPECTED     330        // pre arm will fail if mag field > 544 or < 115
#else // PX4, SITL
#define COMPASS_MAGFIELD_EXPECTED      530        // pre arm will fail if mag field > 874 or < 185
#endif

        // check for unreasonable mag field length
        float mag_field = _compass.get_field().length();
        if (mag_field > COMPASS_MAGFIELD_EXPECTED*1.65 || mag_field < COMPASS_MAGFIELD_EXPECTED*0.35) {
            if (report) {
                gcs_send_text_P(SEVERITY_HIGH,PSTR("PreArm: Check mag field"));
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
        const AP_GPS &gps = ahrs.get_gps();

        //GPS OK?
        if (home_is_set == HOME_UNSET || 
            gps.status() < AP_GPS::GPS_OK_FIX_3D) {
            if (report) {
                gcs_send_text_P(SEVERITY_HIGH,PSTR("PreArm: Bad GPS Position"));
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
                gcs_send_text_P(SEVERITY_HIGH,PSTR("PreArm: Radio failsafe on"));
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
    bool ret = true;
    if (armed || require == NONE) {
        // if we are already armed or don't need any arming checks
        // then skip the checks
        return true;
    }

    ret &= hardware_safety_check(report);
    ret &= barometer_checks(report);
    ret &= ins_checks(report);
    ret &= compass_checks(report);
    ret &= gps_checks(report);
    ret &= battery_checks(report);
    ret &= airspeed_checks(report);
    ret &= logging_checks(report);
    ret &= manual_transmitter_checks(report);

    return ret;
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
