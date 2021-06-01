#include "AP_Parachute.h"

#if HAL_PARACHUTE_ENABLED

#include <AP_Relay/AP_Relay.h>
#include <AP_Math/AP_Math.h>
#include <RC_Channel/RC_Channel.h>
#include <SRV_Channel/SRV_Channel.h>
#include <AP_Notify/AP_Notify.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Logger/AP_Logger.h>
#include <AP_LandingGear/AP_LandingGear.h>
#include <AP_Arming/AP_Arming.h>
#include <AP_Vehicle/AP_Vehicle.h>

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo AP_Parachute::var_info[] = {

    // @Param: ENABLED
    // @DisplayName: Parachute release enabled or disabled
    // @Description: Parachute release enabled or disabled
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    AP_GROUPINFO_FLAGS("ENABLED", 0, AP_Parachute, _enabled, 0, AP_PARAM_FLAG_ENABLE),

    // @Param: TYPE
    // @DisplayName: Parachute release mechanism type (relay or servo)
    // @Description: Parachute release mechanism type (relay or servo)
    // @Values: 0:First Relay,1:Second Relay,2:Third Relay,3:Fourth Relay,10:Servo
    // @User: Standard
    AP_GROUPINFO("TYPE", 1, AP_Parachute, _release_type, AP_PARACHUTE_TRIGGER_TYPE_RELAY_0),

    // @Param: SERVO_ON
    // @DisplayName: Parachute Servo ON PWM value
    // @Description: Parachute Servo PWM value in microseconds when parachute is released
    // @Range: 1000 2000
    // @Units: PWM
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("SERVO_ON", 2, AP_Parachute, _servo_on_pwm, AP_PARACHUTE_SERVO_ON_PWM_DEFAULT),

    // @Param: SERVO_OFF
    // @DisplayName: Servo OFF PWM value
    // @Description: Parachute Servo PWM value in microseconds when parachute is not released
    // @Range: 1000 2000
    // @Units: PWM
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("SERVO_OFF", 3, AP_Parachute, _servo_off_pwm, AP_PARACHUTE_SERVO_OFF_PWM_DEFAULT),

    // @Param: ALT_MIN
    // @DisplayName: Parachute min altitude in meters above home
    // @Description: Parachute min altitude above home.  Parachute will not be released below this altitude.  0 to disable alt check.
    // @Range: 0 32000
    // @Units: m
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("ALT_MIN", 4, AP_Parachute, _alt_min, AP_PARACHUTE_ALT_MIN_DEFAULT),

    // @Param: DELAY_MS
    // @DisplayName: Parachute release delay
    // @Description: Delay in millseconds between motor stop and chute release
    // @Range: 0 5000
    // @Units: ms
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("DELAY_MS", 5, AP_Parachute, _delay_ms, AP_PARACHUTE_RELEASE_DELAY_MS),
    
    // @Param: CRT_SINK
    // @DisplayName: Critical sink speed rate in m/s to trigger emergency parachute
    // @Description: Release parachute when critical sink rate is reached, 0 = disabled
    // @Range: 0 15
    // @Units: m/s
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("CRT_SINK", 6, AP_Parachute, _critical_sink, AP_PARACHUTE_CRITICAL_SINK_DEFAULT),

    // @Param: MIN_ACCEL
    // @DisplayName: Critical earth frame Z acceleration to trigger emergency parachute
    // @Description: 0 measured earth frame Z acceleration would indicate the vehicle is falling at 1g, 9.81m/s/s would be a hover, 0 = disabled
    // @Range: 0 9.81
    // @Units: m/s/s
    // @User: Standard
    AP_GROUPINFO("MIN_ACCEL", 7, AP_Parachute, _min_accel, 0),

    // @Param: OPTIONS
    // @DisplayName: Options bitmask
    // @Description: 0 measured earth frame Z acceleration would indicate the vehicle is falling at 1g, 9.81m/s/s would be a hover, 0 = disabled
    // @Bitmask: 0:Don't deploy landing gear, 1:Don't disarm, 2:Notify only, parachute will not be relased! testing only!
    // @User: Standard
    AP_GROUPINFO("OPTIONS", 8, AP_Parachute, _options, 0),

    // @Param: CANCEL_MS
    // @DisplayName: Cancel time in milli seconds
    // @Description: Amount of time in milli seconds to receve a cancel message
    // @User: Standard
    AP_GROUPINFO("CANCEL_MS", 9, AP_Parachute, _cancel_delay, 0),

    // @Param: ANG_ER_MX
    // @DisplayName: Max angle error
    // @Description: Max angle error for aircraft not in standby.  Values above this will release the chute.
    // @Units: deg
    // @User: Standard
    AP_GROUPINFO("ANG_ER_MX", 10, AP_Parachute, _ang_error_max, 30.0f),

    // @Param: SB_MX_ANG
    // @DisplayName: Standby Max Angle
    // @Description: The maximum absolute roll or pitch angles for aircraft in standby.  Values above this will release the chute.
    // @Units: deg
    // @User: Standard
    AP_GROUPINFO("SB_MX_ANG", 11, AP_Parachute, _sb_rp_ang_max, 80.0f),

    AP_GROUPEND
};

/// enabled - enable or disable parachute release
void AP_Parachute::enabled(bool on_off)
{
    _enabled = on_off;

    // clear release_time
    _release_time = 0;
    _release_reasons = 0;
    _cancel_timeout_ms = 0;
    _sink_time_ms = 0;
    _fall_time_ms = 0;

    AP::logger().Write_Event(_enabled ? LogEvent::PARACHUTE_ENABLED : LogEvent::PARACHUTE_DISABLED);
}

/// release - release parachute
void AP_Parachute::release(release_reason reason)
{
    // exit immediately if not enabled
    if (_enabled <= 0) {
        return;
    }

    // only take action if there is a change in release reason
    if ((_release_reasons && (1U << reason)) != 0) {
        return;
    }

    // add the current release reason to the bitmask
    _release_reasons |= 1U << reason;

    const uint32_t now = AP_HAL::millis();
    if (_cancel_timeout_ms == 0) {
        // set the system time when the deploy will trigger
        _cancel_timeout_ms = now + _cancel_delay;
    }

    send_msg();

    const char *string = string_for_release(reason);

    const uint32_t time_till_deploy = _cancel_timeout_ms - now;
    if (_cancel_delay > 0) {
        gcs().send_text(MAV_SEVERITY_INFO,"Parachute: Releaseing in %0.1f s - %s", time_till_deploy * 0.001f, string);
    } else {
        gcs().send_text(MAV_SEVERITY_INFO,"Parachute: %s", string);
    }
}

// send user command long updating the parchute status
void AP_Parachute::send_msg()
{
    mavlink_command_long_t cmd_msg{};
    cmd_msg.command = MAV_CMD_USER_1;
    cmd_msg.param1 = _release_reasons; // bitmask of reason to release, see release_reason enum
    cmd_msg.param2 = ((_cancel_timeout_ms == 0) || released()) ? -1.0f : (_cancel_timeout_ms - AP_HAL::millis()); // ms until release
    cmd_msg.param3 = AP::vehicle()->get_standby(); // standby states
    cmd_msg.param4 = _enabled; // 0 or less for disabled, otherwise enabled
    cmd_msg.param5 = released(); // 0 for not, 1 if released
    gcs().send_to_active_channels(MAVLINK_MSG_ID_COMMAND_LONG, (char*)&cmd_msg);
}

MAV_RESULT AP_Parachute::handle_cmd(const mavlink_command_long_t &packet)
{
    if (_enabled <= 0) {
        return MAV_RESULT_UNSUPPORTED;
    }

    if (_release_reasons == 0 && _cancel_timeout_ms == 0) {
        // nothing to cancel
        return MAV_RESULT_DENIED;
    }

    if (packet.param2 > 0) {
        // reset the timer
        _cancel_timeout_ms = AP_HAL::millis() + _cancel_delay;
    }

    // Clear the release reasons given in the message
    _release_reasons = _release_reasons ^ uint8_t(packet.param1);


    if (_release_reasons == 0) {
        // release is completely canceled
        _cancel_timeout_ms = 0;
    }

    return MAV_RESULT_ACCEPTED;
}

/// update - shuts off the trigger should be called at about 10hz
void AP_Parachute::update()
{
    const uint32_t now = AP_HAL::millis();
    if (now > _last_msg_send_ms + 100) {
        // stream at hard coded 10hz, cant do anything clever with stream rates because its not a dedicated mesage, just a commnad long
        _last_msg_send_ms = now;
        send_msg();
    }

    // exit immediately if not enabled or parachute not to be released
    if (_enabled <= 0 || _cancel_timeout_ms == 0) {
        return;
    }

    // wait for a possible cancel
    if (now < _cancel_timeout_ms) {
        return;
    }

    if (!_release_initiated) {
        gcs().send_text(MAV_SEVERITY_INFO,"Parachute: Released");
        AP::logger().Write_Event(LogEvent::PARACHUTE_RELEASED);
    }

    if (_options & NOTIFY_ONLY) {
        // just send text and write to log, do not actually do anything
        // usefull for testing thresholds are not exceeded in normal flight
        _cancel_timeout_ms = 0;
        return;
    }

    // set release time to current system time
    if (_release_time == 0) {
        _release_time = now;
        _release_setup = false;
    }

    _release_initiated = true;

    // update AP_Notify
    AP_Notify::flags.parachute_release = 1;

    // calc time since release
    uint32_t time_diff = now - _release_time;
    uint32_t delay_ms = _delay_ms<=0 ? 0: (uint32_t)_delay_ms;

    // check if we should release parachute
    if ((_release_time != 0) && !_release_in_progress) {
        if (!_release_setup) {
            if ((_options & DONT_DEPLOY_LANDING_GEAR) == 0) {
                // deploy landing gear
                AP_LandingGear *lg = AP_LandingGear::get_singleton();
                if (lg != nullptr) {
                    lg->set_position(AP_LandingGear::LandingGear_Deploy);
                }
            }
            if ((_options & DONT_DISARM) == 0) {
                // force disarm
                AP::arming().disarm(AP_Arming::Method::PARACHUTE_RELEASE,false);
            }
            _release_setup = true;
        }
        if (time_diff >= delay_ms) {
            if (_release_type == AP_PARACHUTE_TRIGGER_TYPE_SERVO) {
                // move servo
                SRV_Channels::set_output_pwm(SRV_Channel::k_parachute_release, _servo_on_pwm);
            } else if (_release_type <= AP_PARACHUTE_TRIGGER_TYPE_RELAY_3) {
                // set relay
                _relay.on(_release_type);
            }
            _release_in_progress = true;
            _released = true;
        }
    } else if ((_release_time == 0) || time_diff >= delay_ms + AP_PARACHUTE_RELEASE_DURATION_MS) {
        if (_release_type == AP_PARACHUTE_TRIGGER_TYPE_SERVO) {
            // move servo back to off position
            SRV_Channels::set_output_pwm(SRV_Channel::k_parachute_release, _servo_off_pwm);
        } else if (_release_type <= AP_PARACHUTE_TRIGGER_TYPE_RELAY_3) {
            // set relay back to zero volts
            _relay.off(_release_type);
        }
        // reset released flag and release_time
        _release_in_progress = false;
        _release_time = 0;
        // update AP_Notify
        AP_Notify::flags.parachute_release = 0;
    }
}

// update - set vehicle sink rate and accel
void AP_Parachute::update(const float sink_rate, const float accel, const bool throttle_below_hover)
{
    // reset sink time if critical sink rate check is disabled or vehicle is not flying
    if (!_is_flying) {
        _sink_time_ms = 0;
        _fall_time_ms = 0;
        return;
    }

    // sink rate is posative down
    if (sink_rate <= _critical_sink || !is_positive(sink_rate) || throttle_below_hover) {
        // reset sink_time if vehicle is not sinking too fast
        _sink_time_ms = 0;
    } else if (_sink_time_ms == 0) {
        // start time when sinking too fast
        _sink_time_ms = AP_HAL::millis();
    }

    if (accel <= _min_accel || !is_positive(_min_accel)) {
        // reset fall time if vehicle is not falling too fast
        _fall_time_ms = 0;
    } else if (_fall_time_ms == 0) {
        // start time when falling too fast
        _fall_time_ms = AP_HAL::millis();
    }
}

// trigger parachute release if sink_rate or accretion checks have triggered for at least for 1sec
void AP_Parachute::check()
{
    // return immediately if parachute is being released or vehicle is not flying
    if (_release_initiated || !_is_flying) {
        _release_reasons = 0;
        return;
    }

    // if vehicle is sinking too fast for more than a second release parachute
    uint32_t now = AP_HAL::millis();
    if ((_sink_time_ms > 0) && ((now - _sink_time_ms) > 1000)) {
        release(release_reason::SINK_RATE);
    }
    if ((_fall_time_ms > 0) && ((now - _fall_time_ms) > 1000)) {
        release(release_reason::ACCEL_FALLING);
    }
}

const AP_Parachute::LookupTable AP_Parachute::lookuptable[] = {
    { release_reason::SINK_RATE,"sink rate"},
    { release_reason::ACCEL_FALLING,"acceleration"},
    { release_reason::CONTROL_LOSS,"loss of control"},
    { release_reason::MISSION_ITEM,"mission item"},
    { release_reason::MANUAL,"manual"},
};

const char *AP_Parachute::string_for_release(release_reason reason) const
{
     for (const struct LookupTable entry : lookuptable) {
        if (entry.option == reason) {
            return entry.announcement;
        }
     }
     return nullptr;
};

// singleton instance
AP_Parachute *AP_Parachute::_singleton;

namespace AP {

AP_Parachute *parachute()
{
    return AP_Parachute::get_singleton();
}

}
#endif // HAL_PARACHUTE_ENABLED
