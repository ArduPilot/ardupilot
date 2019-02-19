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

/*
    AP_AdvancedFailsafe.cpp

    This is an advanced failsafe module originally modelled on the
    failsafe rules of the Outback Challenge
*/
#include <AP_HAL/AP_HAL.h>
#include "AP_AdvancedFailsafe.h"
#include <RC_Channel/RC_Channel.h>
#include <SRV_Channel/SRV_Channel.h>
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL& hal;

// table of user settable parameters
const AP_Param::GroupInfo AP_AdvancedFailsafe::var_info[] = {
    // @Param: ENABLE
    // @DisplayName: Enable Advanced Failsafe
    // @Description: This enables the advanced failsafe system. If this is set to zero (disable) then all the other AFS options have no effect
    // @User: Advanced
    AP_GROUPINFO_FLAGS("ENABLE",       11, AP_AdvancedFailsafe, _enable, 0, AP_PARAM_FLAG_ENABLE),

    // @Param: MAN_PIN
    // @DisplayName: Manual Pin
    // @Description: This sets a digital output pin to set high when in manual mode
    // @User: Advanced
    AP_GROUPINFO("MAN_PIN",     0, AP_AdvancedFailsafe, _manual_pin,    -1),

    // @Param: HB_PIN
    // @DisplayName: Heartbeat Pin
    // @Description: This sets a digital output pin which is cycled at 10Hz when termination is not activated. Note that if a FS_TERM_PIN is set then the heartbeat pin will continue to cycle at 10Hz when termination is activated, to allow the termination board to distinguish between autopilot crash and termination.
    // @User: Advanced
    AP_GROUPINFO("HB_PIN",      1, AP_AdvancedFailsafe, _heartbeat_pin, -1),

    // @Param: WP_COMMS
    // @DisplayName: Comms Waypoint
    // @Description: Waypoint number to navigate to on comms loss
    // @User: Advanced
    AP_GROUPINFO("WP_COMMS",    2, AP_AdvancedFailsafe, _wp_comms_hold, 0),

    // @Param: GPS_LOSS
    // @DisplayName: GPS Loss Waypoint
    // @Description: Waypoint number to navigate to on GPS lock loss
    // @User: Advanced
    AP_GROUPINFO("WP_GPS_LOSS", 4, AP_AdvancedFailsafe, _wp_gps_loss, 0),

    // @Param: TERMINATE
    // @DisplayName: Force Terminate
    // @Description: Can be set in flight to force termination of the heartbeat signal
    // @User: Advanced
    AP_GROUPINFO("TERMINATE",   5, AP_AdvancedFailsafe, _terminate, 0),

    // @Param: TERM_ACTION
    // @DisplayName: Terminate action
    // @Description: This can be used to force an action on flight termination. Normally this is handled by an external failsafe board, but you can setup APM to handle it here. Please consult the wiki for more information on the possible values of the parameter
    // @User: Advanced
    AP_GROUPINFO("TERM_ACTION", 6, AP_AdvancedFailsafe, _terminate_action, 0),

    // @Param: TERM_PIN
    // @DisplayName: Terminate Pin
    // @Description: This sets a digital output pin to set high on flight termination
    // @User: Advanced
    AP_GROUPINFO("TERM_PIN",    7, AP_AdvancedFailsafe, _terminate_pin,    -1),

    // @Param: AMSL_LIMIT
    // @DisplayName: AMSL limit
    // @Description: This sets the AMSL (above mean sea level) altitude limit. If the pressure altitude determined by QNH exceeds this limit then flight termination will be forced. Note that this limit is in meters, whereas pressure altitude limits are often quoted in feet. A value of zero disables the pressure altitude limit.
    // @User: Advanced
    // @Units: m
    AP_GROUPINFO("AMSL_LIMIT",   8, AP_AdvancedFailsafe, _amsl_limit,    0),

    // @Param: AMSL_ERR_GPS
    // @DisplayName: Error margin for GPS based AMSL limit
    // @Description: This sets margin for error in GPS derived altitude limit. This error margin is only used if the barometer has failed. If the barometer fails then the GPS will be used to enforce the AMSL_LIMIT, but this margin will be subtracted from the AMSL_LIMIT first, to ensure that even with the given amount of GPS altitude error the pressure altitude is not breached. OBC users should set this to comply with their D2 safety case. A value of -1 will mean that barometer failure will lead to immediate termination.
    // @User: Advanced
    // @Units: m
    AP_GROUPINFO("AMSL_ERR_GPS", 9, AP_AdvancedFailsafe, _amsl_margin_gps,  -1),

    // @Param: QNH_PRESSURE
    // @DisplayName: QNH pressure
    // @Description: This sets the QNH pressure in millibars to be used for pressure altitude in the altitude limit. A value of zero disables the altitude limit.
    // @Units: mbar
    // @User: Advanced
    AP_GROUPINFO("QNH_PRESSURE", 10, AP_AdvancedFailsafe, _qnh_pressure,    0),

    // *NOTE* index 11 is "Enable" and is moved to the top to allow AP_PARAM_FLAG_ENABLE

    // *NOTE* index 12 of AP_Int16 RC_FAIL_MS was depreciated. Replaced by RC_FAIL_TIME.

    // @Param: MAX_GPS_LOSS
    // @DisplayName: Maximum number of GPS loss events
    // @Description: Maximum number of GPS loss events before the aircraft stops returning to mission on GPS recovery. Use zero to allow for any number of GPS loss events.
    // @User: Advanced
    AP_GROUPINFO("MAX_GPS_LOSS", 13, AP_AdvancedFailsafe, _max_gps_loss, 0),

    // @Param: MAX_COM_LOSS
    // @DisplayName: Maximum number of comms loss events
    // @Description: Maximum number of comms loss events before the aircraft stops returning to mission on comms recovery. Use zero to allow for any number of comms loss events.
    // @User: Advanced
    AP_GROUPINFO("MAX_COM_LOSS", 14, AP_AdvancedFailsafe, _max_comms_loss, 0),

    // @Param: GEOFENCE
    // @DisplayName: Enable geofence Advanced Failsafe
    // @Description: This enables the geofence part of the AFS. Will only be in effect if AFS_ENABLE is also 1
    // @User: Advanced
    AP_GROUPINFO("GEOFENCE",     15, AP_AdvancedFailsafe, _enable_geofence_fs, 1),

    // @Param: RC
    // @DisplayName: Enable RC Advanced Failsafe
    // @Description: This enables the RC part of the AFS. Will only be in effect if AFS_ENABLE is also 1
    // @User: Advanced
    AP_GROUPINFO("RC",           16, AP_AdvancedFailsafe, _enable_RC_fs, 1),

    // @Param: RC_MAN_ONLY
    // @DisplayName: Enable RC Termination only in manual control modes
    // @Description: If this parameter is set to 1, then an RC loss will only cause the plane to terminate in manual control modes. If it is 0, then the plane will terminate in any flight mode.
    // @User: Advanced
    AP_GROUPINFO("RC_MAN_ONLY",    17, AP_AdvancedFailsafe, _rc_term_manual_only, 1),

    // @Param: DUAL_LOSS
    // @DisplayName: Enable dual loss terminate due to failure of both GCS and GPS simultaneously
    // @Description: This enables the dual loss termination part of the AFS system. If this parameter is 1 and both GPS and the ground control station fail simultaneously, this will be considered a "dual loss" and cause termination.
    // @User: Advanced
    AP_GROUPINFO("DUAL_LOSS",      18, AP_AdvancedFailsafe, _enable_dual_loss, 1),

    // @Param: RC_FAIL_TIME
    // @DisplayName: RC failure time
    // @Description: This is the time in seconds in manual mode that failsafe termination will activate if RC input is lost. For the OBC rules this should be (1.5). Use 0 to disable.
    // @User: Advanced
    // @Units: s
    AP_GROUPINFO("RC_FAIL_TIME",   19, AP_AdvancedFailsafe, _rc_fail_time_seconds,    0),

    // @Param: TERM_DLY
    // @DisplayName: Termination Delay
    // @Description: This is the time in seconds the aircraft will stay at a failsafe waypoint before aero termination.  Set to 0 to disable.
    // @User: Advanced
    // @Units: seconds 
    AP_GROUPINFO("TERM_DLY",   20, AP_AdvancedFailsafe, _terminate_delay,   0),

    AP_GROUPEND
};

// check for Failsafe conditions. This is called at 10Hz by the main
// ArduPlane code
void
AP_AdvancedFailsafe::check(uint32_t last_heartbeat_ms, bool geofence_breached, uint32_t last_valid_rc_ms)
{    
    if (!_enable) {
        return;
    }
    // only set the termination capability, clearing it can mess up copter and sub which can always be terminated
    hal.util->set_capabilities(MAV_PROTOCOL_CAPABILITY_FLIGHT_TERMINATION);

    // we always check for fence breach
    if(_enable_geofence_fs) {
        if (geofence_breached || check_altlimit()) {
            if (!_terminate) {
                gcs().send_text(MAV_SEVERITY_CRITICAL, "Terminating due to fence breach");
                _terminate.set_and_notify(1);
            }
        }
    }

    enum control_mode mode = afs_mode();
    
    // check if RC termination is enabled
    // check for RC failure in manual mode or RC failure when AFS_RC_MANUAL is 0
    if (_state != STATE_PREFLIGHT && !_terminate && _enable_RC_fs &&
        (mode == AFS_MANUAL || mode == AFS_STABILIZED || !_rc_term_manual_only) &&
        _rc_fail_time_seconds > 0 &&
            (AP_HAL::millis() - last_valid_rc_ms) > (_rc_fail_time_seconds * 1000.0f)) {
        gcs().send_text(MAV_SEVERITY_CRITICAL, "Terminating due to RC failure");
        _terminate.set_and_notify(1);
    }
    
    // tell the failsafe board if we are in manual control
    // mode. This tells it to pass through controls from the
    // receiver
    if (_manual_pin != -1) {
        hal.gpio->pinMode(_manual_pin, HAL_GPIO_OUTPUT);
        hal.gpio->write(_manual_pin, mode==AFS_MANUAL);
    }

    uint32_t now = AP_HAL::millis();
    bool gcs_link_ok = ((now - last_heartbeat_ms) < 10000);
    bool gps_lock_ok = ((now - gps.last_fix_time_ms()) < 3000);

    bool terminate_delay_comms = ((now - last_heartbeat_ms) > ((_terminate_delay * 1000) + 10000));
    bool terminate_delay_gps   = ((now - gps.last_fix_time_ms()) > ((_terminate_delay * 1000) + 3000));

    // If the delay is set to 0 or negative values, ensure value is false.
    if (_terminate_delay <= 0) {
        terminate_delay_comms = false;
        terminate_delay_gps   = false;
    }

    switch (_state) {
    case STATE_PREFLIGHT:
        // we startup in preflight mode. This mode ends when
        // we first enter auto.
        if (mode == AFS_AUTO) {
            gcs().send_text(MAV_SEVERITY_DEBUG, "AFS State: AFS_AUTO");
            _state = STATE_AUTO;
        }
        break;

    case STATE_AUTO:
        // this is the normal mode. 
        if (!gcs_link_ok) {
            gcs().send_text(MAV_SEVERITY_DEBUG, "AFS State: DATA_LINK_LOSS");
            _state = STATE_DATA_LINK_LOSS;
            if (_wp_comms_hold) {
                _saved_wp = mission.get_current_nav_cmd().index;
                mission.set_current_cmd(_wp_comms_hold);
            }
            // if two events happen within 30s we consider it to be part of the same event
            if (now - _last_comms_loss_ms > 30*1000UL) {
                _comms_loss_count++;
                _last_comms_loss_ms = now;
            }
            break;
        }
        if (!gps_lock_ok) {
            gcs().send_text(MAV_SEVERITY_DEBUG, "AFS State: GPS_LOSS");
            _state = STATE_GPS_LOSS;
            if (_wp_gps_loss) {
                _saved_wp = mission.get_current_nav_cmd().index;
                mission.set_current_cmd(_wp_gps_loss);
            }
            // if two events happen within 30s we consider it to be part of the same event
            if (now - _last_gps_loss_ms > 30*1000UL) {
                _gps_loss_count++;
                _last_gps_loss_ms = now;
            }
            break;
        }
        break;

    case STATE_DATA_LINK_LOSS:
        if (!gps_lock_ok) {
            // losing GPS lock when data link is lost
            // leads to termination if AFS_DUAL_LOSS is 1
            if(_enable_dual_loss) {
                if (!_terminate) {
                    gcs().send_text(MAV_SEVERITY_CRITICAL, "Terminating due to dual loss");
                    _terminate.set_and_notify(1);
                }
            }
        } else if (gcs_link_ok) {
            _state = STATE_AUTO;
            gcs().send_text(MAV_SEVERITY_DEBUG, "AFS State: AFS_AUTO, GCS now OK");
            // we only return to the mission if we have not exceeded AFS_MAX_COM_LOSS
            if (_saved_wp != 0 && 
                (_max_comms_loss <= 0 || 
                 _comms_loss_count <= _max_comms_loss)) {
                mission.set_current_cmd(_saved_wp);            
                _saved_wp = 0;
            }
        } else if (terminate_delay_comms) {
            if (!_terminate) {
               gcs().send_text(MAV_SEVERITY_CRITICAL, "TERM_DLY Comms Long Failsafe: Terminating!");
               _terminate.set_and_notify(1);
            }
        }
        break;

    case STATE_GPS_LOSS:
        if (!gcs_link_ok) {
            // losing GCS link when GPS lock lost
            // leads to termination if AFS_DUAL_LOSS is 1
            if (!_terminate && _enable_dual_loss) {
                gcs().send_text(MAV_SEVERITY_CRITICAL, "Terminating due to dual loss");
                _terminate.set_and_notify(1);
            }
        } else if (gps_lock_ok) {
            gcs().send_text(MAV_SEVERITY_DEBUG, "AFS State: AFS_AUTO, GPS now OK");
            _state = STATE_AUTO;
            // we only return to the mission if we have not exceeded AFS_MAX_GPS_LOSS
            if (_saved_wp != 0 &&
                (_max_gps_loss <= 0 || _gps_loss_count <= _max_gps_loss)) {
                mission.set_current_cmd(_saved_wp);            
                _saved_wp = 0;
            }
        } else if (terminate_delay_gps) {
            if (!_terminate) {
               gcs().send_text(MAV_SEVERITY_CRITICAL, "TERM_DLY GPS Long Failsafe: Terminating!");
                _terminate.set_and_notify(1);
            }
        }
        break;
    }

    // if we are not terminating or if there is a separate terminate
    // pin configured then toggle the heartbeat pin at 10Hz
    if (_heartbeat_pin != -1 && (_terminate_pin != -1 || !_terminate)) {
        _heartbeat_pin_value = !_heartbeat_pin_value;
        hal.gpio->pinMode(_heartbeat_pin, HAL_GPIO_OUTPUT);
        hal.gpio->write(_heartbeat_pin, _heartbeat_pin_value);
    }    

    // set the terminate pin
    if (_terminate_pin != -1) {
        hal.gpio->pinMode(_terminate_pin, HAL_GPIO_OUTPUT);
        hal.gpio->write(_terminate_pin, _terminate?1:0);
    }    
}


// send heartbeat messages during sensor calibration
void
AP_AdvancedFailsafe::heartbeat(void)
{    
    if (!_enable) {
        return;
    }

    // if we are not terminating or if there is a separate terminate
    // pin configured then toggle the heartbeat pin at 10Hz
    if (_heartbeat_pin != -1 && (_terminate_pin != -1 || !_terminate)) {
        _heartbeat_pin_value = !_heartbeat_pin_value;
        hal.gpio->pinMode(_heartbeat_pin, HAL_GPIO_OUTPUT);
        hal.gpio->write(_heartbeat_pin, _heartbeat_pin_value);
    }    
}

// check for altitude limit breach
bool
AP_AdvancedFailsafe::check_altlimit(void)
{    
    if (!_enable) {
        return false;
    }
    if (_amsl_limit == 0 || _qnh_pressure <= 0) {
        // no limit set
        return false;
    }

    // see if the barometer is dead
    const AP_Baro &baro = AP::baro();
    if (AP_HAL::millis() - baro.get_last_update() > 5000) {
        // the barometer has been unresponsive for 5 seconds. See if we can switch to GPS
        if (_amsl_margin_gps != -1 &&
            gps.status() >= AP_GPS::GPS_OK_FIX_3D &&
            gps.location().alt*0.01f <= _amsl_limit - _amsl_margin_gps) {
            // GPS based altitude OK
            return false;
        }
        // no barometer - immediate termination
        return true;
    }

    float alt_amsl = baro.get_altitude_difference(_qnh_pressure*100, baro.get_pressure());
    if (alt_amsl > _amsl_limit) {
        // pressure altitude breach
        return true;
    }
    
    // all OK
    return false;
}

/*
  return true if we should crash the vehicle
 */
bool AP_AdvancedFailsafe::should_crash_vehicle(void)
{
    if (!_enable) {
        return false;
    }
    // ensure failsafe values are setup for if FMU crashes on PX4/Pixhawk
    if (!_failsafe_setup) {
        _failsafe_setup = true;
        setup_IO_failsafe();
    }

    // determine if the vehicle should be crashed
    // only possible if FS_TERM_ACTION is 42 and _terminate is non zero
    // _terminate may be set via parameters, or a mavlink command
    if (_terminate &&
        (_terminate_action == TERMINATE_ACTION_TERMINATE ||
         _terminate_action == TERMINATE_ACTION_LAND)) {
        // we are terminating
        return true;
    }

    // continue flying
    return false;
}

// update GCS based termination
// returns true if AFS is in the desired termination state
bool AP_AdvancedFailsafe::gcs_terminate(bool should_terminate, const char *reason) {
    if (!_enable) {
        gcs().send_text(MAV_SEVERITY_INFO, "AFS not enabled, can't terminate the vehicle");
        return false;
    }

    _terminate.set_and_notify(should_terminate ? 1 : 0);

    // evaluate if we will crash or not, as AFS must be enabled, and TERM_ACTION has to be correct
    bool is_terminating = should_crash_vehicle();

    if(should_terminate == is_terminating) {
        if (is_terminating) {
            gcs().send_text(MAV_SEVERITY_INFO, "Terminating due to %s", reason);
        } else {
            gcs().send_text(MAV_SEVERITY_INFO, "Aborting termination due to %s", reason);
        }
        return true;
    } else if (should_terminate && _terminate_action != TERMINATE_ACTION_TERMINATE) {
        gcs().send_text(MAV_SEVERITY_INFO, "Unable to terminate, termination is not configured");
    }
    return false;
}
