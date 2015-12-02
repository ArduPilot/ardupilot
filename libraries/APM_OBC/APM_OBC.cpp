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

/*
    APM_OBC.cpp

    Outback Challenge Failsafe module

*/
#include <AP_HAL/AP_HAL.h>
#include "APM_OBC.h"
#include <RC_Channel/RC_Channel.h>
#include <RC_Channel/RC_Channel_aux.h>
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL& hal;

// table of user settable parameters
const AP_Param::GroupInfo APM_OBC::var_info[] = {
    // @Param: MAN_PIN
    // @DisplayName: Manual Pin
    // @Description: This sets a digital output pin to set high when in manual mode
    // @User: Advanced
    AP_GROUPINFO("MAN_PIN",     0, APM_OBC, _manual_pin,    -1),

    // @Param: HB_PIN
    // @DisplayName: Heartbeat Pin
    // @Description: This sets a digital output pin which is cycled at 10Hz when termination is not activated. Note that if a FS_TERM_PIN is set then the heartbeat pin will continue to cycle at 10Hz when termination is activated, to allow the termination board to distinguish between autopilot crash and termination.
    // @User: Advanced
    AP_GROUPINFO("HB_PIN",      1, APM_OBC, _heartbeat_pin, -1),

    // @Param: WP_COMMS
    // @DisplayName: Comms Waypoint
    // @Description: Waypoint number to navigate to on comms loss
    // @User: Advanced
    AP_GROUPINFO("WP_COMMS",    2, APM_OBC, _wp_comms_hold, 0),

    // @Param: GPS_LOSS
    // @DisplayName: GPS Loss Waypoint
    // @Description: Waypoint number to navigate to on GPS lock loss
    // @User: Advanced
    AP_GROUPINFO("WP_GPS_LOSS", 4, APM_OBC, _wp_gps_loss, 0),

    // @Param: TERMINATE
    // @DisplayName: Force Terminate
    // @Description: Can be set in flight to force termination of the heartbeat signal
    // @User: Advanced
    AP_GROUPINFO("TERMINATE",   5, APM_OBC, _terminate, 0),

    // @Param: TERM_ACTION
    // @DisplayName: Terminate action
    // @Description: This can be used to force an action on flight termination. Normally this is handled by an external failsafe board, but you can setup APM to handle it here. If set to 0 (which is the default) then no extra action is taken. If set to the magic value 42 then the plane will deliberately crash itself by setting maximum throws on all surfaces, and zero throttle
    // @User: Advanced
    AP_GROUPINFO("TERM_ACTION", 6, APM_OBC, _terminate_action, 0),

    // @Param: TERM_PIN
    // @DisplayName: Terminate Pin
    // @Description: This sets a digital output pin to set high on flight termination
    // @User: Advanced
    AP_GROUPINFO("TERM_PIN",    7, APM_OBC, _terminate_pin,    -1),

    // @Param: AMSL_LIMIT
    // @DisplayName: AMSL limit
    // @Description: This sets the AMSL (above mean sea level) altitude limit. If the pressure altitude determined by QNH exceeds this limit then flight termination will be forced. Note that this limit is in meters, whereas pressure altitude limits are often quoted in feet. A value of zero disables the pressure altitude limit.
    // @User: Advanced
    // @Units: meters
    AP_GROUPINFO("AMSL_LIMIT",   8, APM_OBC, _amsl_limit,    0),

    // @Param: AMSL_ERR_GPS
    // @DisplayName: Error margin for GPS based AMSL limit
    // @Description: This sets margin for error in GPS derived altitude limit. This error margin is only used if the barometer has failed. If the barometer fails then the GPS will be used to enforce the AMSL_LIMIT, but this margin will be subtracted from the AMSL_LIMIT first, to ensure that even with the given amount of GPS altitude error the pressure altitude is not breached. OBC users should set this to comply with their D2 safety case. A value of -1 will mean that barometer failure will lead to immediate termination.
    // @User: Advanced
    // @Units: meters
    AP_GROUPINFO("AMSL_ERR_GPS", 9, APM_OBC, _amsl_margin_gps,  -1),

    // @Param: QNH_PRESSURE
    // @DisplayName: QNH pressure
    // @Description: This sets the QNH pressure in millibars to be used for pressure altitude in the altitude limit. A value of zero disables the altitude limit.
    // @Units: millibar
    // @User: Advanced
    AP_GROUPINFO("QNH_PRESSURE", 10, APM_OBC, _qnh_pressure,    0),

    // @Param: ENABLE
    // @DisplayName: Enable Advanced Failsafe
    // @Description: This enables the advanced failsafe system. If this is set to zero (disable) then all the other AFS options have no effect
    // @User: Advanced
    AP_GROUPINFO("ENABLE",       11, APM_OBC, _enable,          0),

    // @Param: RC_FAIL_MS
    // @DisplayName: RC failure time
    // @Description: This is the time in milliseconds in manual mode that failsafe termination will activate if RC input is lost. For the OBC rules this should be 1500. Use 0 to disable.
    // @User: Advanced
    AP_GROUPINFO("RC_FAIL_MS",   12, APM_OBC, _rc_fail_time,    0),

    // @Param: MAX_GPS_LOSS
    // @DisplayName: Maximum number of GPS loss events
    // @Description: Maximum number of GPS loss events before the aircraft stops returning to mission on GPS recovery. Use zero to allow for any number of GPS loss events.
    // @User: Advanced
    AP_GROUPINFO("MAX_GPS_LOSS", 13, APM_OBC, _max_gps_loss, 0),

    // @Param: MAX_COM_LOSS
    // @DisplayName: Maximum number of comms loss events
    // @Description: Maximum number of comms loss events before the aircraft stops returning to mission on comms recovery. Use zero to allow for any number of comms loss events.
    // @User: Advanced
    AP_GROUPINFO("MAX_COM_LOSS", 14, APM_OBC, _max_comms_loss, 0),

    AP_GROUPEND
};

// check for Failsafe conditions. This is called at 10Hz by the main
// ArduPlane code
void
APM_OBC::check(APM_OBC::control_mode mode, uint32_t last_heartbeat_ms, bool geofence_breached, uint32_t last_valid_rc_ms)
{    
    if (!_enable) {
        return;
    }
    // we always check for fence breach
    if (geofence_breached || check_altlimit()) {
        if (!_terminate) {
            GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_INFO, "Fence TERMINATE");
            _terminate.set(1);
        }
    }

    // check for RC failure in manual mode
    if (_state != STATE_PREFLIGHT && 
        (mode == OBC_MANUAL || mode == OBC_FBW) && 
        _rc_fail_time != 0 && 
        (AP_HAL::millis() - last_valid_rc_ms) > (unsigned)_rc_fail_time.get()) {
        if (!_terminate) {
            GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_INFO, "RC failure terminate");
            _terminate.set(1);
        }
    }
    
    // tell the failsafe board if we are in manual control
    // mode. This tells it to pass through controls from the
    // receiver
    if (_manual_pin != -1) {
        hal.gpio->pinMode(_manual_pin, HAL_GPIO_OUTPUT);
        hal.gpio->write(_manual_pin, mode==OBC_MANUAL);
    }

    uint32_t now = AP_HAL::millis();
    bool gcs_link_ok = ((now - last_heartbeat_ms) < 10000);
    bool gps_lock_ok = ((now - gps.last_fix_time_ms()) < 3000);

    switch (_state) {
    case STATE_PREFLIGHT:
        // we startup in preflight mode. This mode ends when
        // we first enter auto.
        if (mode == OBC_AUTO) {
            GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_INFO, "Starting AFS_AUTO");
            _state = STATE_AUTO;
        }
        break;

    case STATE_AUTO:
        // this is the normal mode. 
        if (!gcs_link_ok) {
            GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_ERROR, "State DATA_LINK_LOSS");
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
            GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_CRITICAL, "State GPS_LOSS");
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
            // leads to termination
            if (!_terminate) {
                GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_INFO, "Dual loss TERMINATE");
                _terminate.set(1);
            }
        } else if (gcs_link_ok) {
            _state = STATE_AUTO;
            GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_INFO, "GCS OK");
            // we only return to the mission if we have not exceeded AFS_MAX_COM_LOSS
            if (_saved_wp != 0 && 
                (_max_comms_loss <= 0 || 
                 _comms_loss_count <= _max_comms_loss)) {
                mission.set_current_cmd(_saved_wp);            
                _saved_wp = 0;
            }
        }
        break;

    case STATE_GPS_LOSS:
        if (!gcs_link_ok) {
            // losing GCS link when GPS lock lost
            // leads to termination
            if (!_terminate) {
                GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_CRITICAL, "Dual loss TERMINATE");
                _terminate.set(1);
            }
        } else if (gps_lock_ok) {
            GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_INFO, "GPS OK");
            _state = STATE_AUTO;
            // we only return to the mission if we have not exceeded AFS_MAX_GPS_LOSS
            if (_saved_wp != 0 &&
                (_max_gps_loss <= 0 || 
                 _gps_loss_count <= _max_gps_loss)) {
                mission.set_current_cmd(_saved_wp);            
                _saved_wp = 0;
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
APM_OBC::heartbeat(void)
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
APM_OBC::check_altlimit(void)
{    
    if (!_enable) {
        return false;
    }
    if (_amsl_limit == 0 || _qnh_pressure <= 0) {
        // no limit set
        return false;
    }

    // see if the barometer is dead
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
  setup the IO boards failsafe values for if the FMU firmware crashes
 */
void APM_OBC::setup_failsafe(void)
{
    if (!_enable) {
        return;
    }
    const RC_Channel *ch_roll     = RC_Channel::rc_channel(rcmap.roll()-1);
    const RC_Channel *ch_pitch    = RC_Channel::rc_channel(rcmap.pitch()-1);
    const RC_Channel *ch_yaw      = RC_Channel::rc_channel(rcmap.yaw()-1);
    const RC_Channel *ch_throttle = RC_Channel::rc_channel(rcmap.throttle()-1);

    // setup primary channel output values
    hal.rcout->set_failsafe_pwm(1U<<(rcmap.roll()-1),     ch_roll->get_limit_pwm(RC_Channel::RC_CHANNEL_LIMIT_MIN));
    hal.rcout->set_failsafe_pwm(1U<<(rcmap.pitch()-1),    ch_pitch->get_limit_pwm(RC_Channel::RC_CHANNEL_LIMIT_MAX));
    hal.rcout->set_failsafe_pwm(1U<<(rcmap.yaw()-1),      ch_yaw->get_limit_pwm(RC_Channel::RC_CHANNEL_LIMIT_MAX));
    hal.rcout->set_failsafe_pwm(1U<<(rcmap.throttle()-1), ch_throttle->get_limit_pwm(RC_Channel::RC_CHANNEL_LIMIT_MIN));

    // and all aux channels
    RC_Channel_aux::set_servo_failsafe(RC_Channel_aux::k_flap_auto, RC_Channel::RC_CHANNEL_LIMIT_MAX);
    RC_Channel_aux::set_servo_failsafe(RC_Channel_aux::k_flap, RC_Channel::RC_CHANNEL_LIMIT_MAX);
    RC_Channel_aux::set_servo_failsafe(RC_Channel_aux::k_aileron, RC_Channel::RC_CHANNEL_LIMIT_MIN);
    RC_Channel_aux::set_servo_failsafe(RC_Channel_aux::k_rudder, RC_Channel::RC_CHANNEL_LIMIT_MAX);
    RC_Channel_aux::set_servo_failsafe(RC_Channel_aux::k_elevator, RC_Channel::RC_CHANNEL_LIMIT_MAX);
    RC_Channel_aux::set_servo_failsafe(RC_Channel_aux::k_elevator_with_input, RC_Channel::RC_CHANNEL_LIMIT_MAX);
    RC_Channel_aux::set_servo_failsafe(RC_Channel_aux::k_manual, RC_Channel::RC_CHANNEL_LIMIT_TRIM);
    RC_Channel_aux::set_servo_failsafe(RC_Channel_aux::k_none, RC_Channel::RC_CHANNEL_LIMIT_TRIM);
}

/*
  setu radio_out values for all channels to termination values if we
  are terminating
 */
void APM_OBC::check_crash_plane(void)
{
    if (!_enable) {
        return;
    }
    // ensure failsafe values are setup for if FMU crashes on PX4/Pixhawk
    if (!_failsafe_setup) {
        _failsafe_setup = true;
        setup_failsafe();
    }

    // should we crash the plane? Only possible with
    // FS_TERM_ACTTION set to 42
    if (!_terminate || _terminate_action != 42) {
        // not terminating
        return;
    }

    // we are terminating. Setup primary output channels radio_out values
    RC_Channel *ch_roll     = RC_Channel::rc_channel(rcmap.roll()-1);
    RC_Channel *ch_pitch    = RC_Channel::rc_channel(rcmap.pitch()-1);
    RC_Channel *ch_yaw      = RC_Channel::rc_channel(rcmap.yaw()-1);
    RC_Channel *ch_throttle = RC_Channel::rc_channel(rcmap.throttle()-1);

    ch_roll->radio_out     = ch_roll->get_limit_pwm(RC_Channel::RC_CHANNEL_LIMIT_MIN);
    ch_pitch->radio_out    = ch_pitch->get_limit_pwm(RC_Channel::RC_CHANNEL_LIMIT_MAX);
    ch_yaw->radio_out      = ch_yaw->get_limit_pwm(RC_Channel::RC_CHANNEL_LIMIT_MAX);
    ch_throttle->radio_out = ch_throttle->get_limit_pwm(RC_Channel::RC_CHANNEL_LIMIT_MIN);

    // and all aux channels
    RC_Channel_aux::set_servo_limit(RC_Channel_aux::k_flap_auto, RC_Channel::RC_CHANNEL_LIMIT_MAX);
    RC_Channel_aux::set_servo_limit(RC_Channel_aux::k_flap, RC_Channel::RC_CHANNEL_LIMIT_MAX);
    RC_Channel_aux::set_servo_limit(RC_Channel_aux::k_aileron, RC_Channel::RC_CHANNEL_LIMIT_MIN);
    RC_Channel_aux::set_servo_limit(RC_Channel_aux::k_rudder, RC_Channel::RC_CHANNEL_LIMIT_MAX);
    RC_Channel_aux::set_servo_limit(RC_Channel_aux::k_elevator, RC_Channel::RC_CHANNEL_LIMIT_MAX);
    RC_Channel_aux::set_servo_limit(RC_Channel_aux::k_elevator_with_input, RC_Channel::RC_CHANNEL_LIMIT_MAX);
    RC_Channel_aux::set_servo_limit(RC_Channel_aux::k_manual, RC_Channel::RC_CHANNEL_LIMIT_TRIM);
    RC_Channel_aux::set_servo_limit(RC_Channel_aux::k_none, RC_Channel::RC_CHANNEL_LIMIT_TRIM);
}
