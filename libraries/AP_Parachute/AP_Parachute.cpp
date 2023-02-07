#include "AP_Parachute.h"

#if HAL_PARACHUTE_ENABLED

#include <AP_Relay/AP_Relay.h>
#include <AP_Math/AP_Math.h>
#include <RC_Channel/RC_Channel.h>
#include <SRV_Channel/SRV_Channel.h>
#include <AP_Notify/AP_Notify.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Logger/AP_Logger.h>
#include <GCS_MAVLink/GCS.h>

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
    // @Description: Release parachute when critical sink rate is reached
    // @Range: 0 15
    // @Units: m/s
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("CRT_SINK", 6, AP_Parachute, _critical_sink, AP_PARACHUTE_CRITICAL_SINK_DEFAULT),

    // @Param: OPTIONS
    // @DisplayName: Parachute options
    // @Description: Optional behaviour for parachute
    // @Bitmask: 0:hold open forever after release
    // @User: Standard
    AP_GROUPINFO("OPTIONS", 7, AP_Parachute, _options, 0),

    AP_GROUPEND
};

/// enabled - enable or disable parachute release
void AP_Parachute::enabled(bool on_off)
{
    _enabled.set(on_off);

    // clear release_time
    _release_time = 0;

    AP::logger().Write_Event(_enabled ? LogEvent::PARACHUTE_ENABLED : LogEvent::PARACHUTE_DISABLED);
}

/// release - release parachute
void AP_Parachute::release()
{
    // exit immediately if not enabled
    if (_enabled <= 0) {
        return;
    }

    gcs().send_text(MAV_SEVERITY_INFO,"Parachute: Released");
    AP::logger().Write_Event(LogEvent::PARACHUTE_RELEASED);

    // set release time to current system time
    if (_release_time == 0) {
        _release_time = AP_HAL::millis();
    }

    _release_initiated = true;

    // update AP_Notify
    AP_Notify::flags.parachute_release = 1;
}

/// update - shuts off the trigger should be called at about 10hz
void AP_Parachute::update()
{
    // exit immediately if not enabled or parachute not to be released
    if (_enabled <= 0) {
        return;
    }

    // calc time since release
    uint32_t time_diff = AP_HAL::millis() - _release_time;
    uint32_t delay_ms = _delay_ms<=0 ? 0: (uint32_t)_delay_ms;

    bool hold_forever = (_options.get() & uint32_t(Options::HoldOpen)) != 0;

    // check if we should release parachute
    if ((_release_time != 0) && !_release_in_progress) {
        if (time_diff >= delay_ms) {
            if (_release_type == AP_PARACHUTE_TRIGGER_TYPE_SERVO) {
                // move servo
                SRV_Channels::set_output_pwm(SRV_Channel::k_parachute_release, _servo_on_pwm);
            } else if (_release_type <= AP_PARACHUTE_TRIGGER_TYPE_RELAY_3) {
                // set relay
                AP_Relay*_relay = AP::relay();
                if (_relay != nullptr) {
                    _relay->on(_release_type);
                }
            }
            _release_in_progress = true;
            _released = true;
        }
    } else if ((_release_time == 0) ||
               (!hold_forever && time_diff >= delay_ms + AP_PARACHUTE_RELEASE_DURATION_MS)) {
        if (_release_type == AP_PARACHUTE_TRIGGER_TYPE_SERVO) {
            // move servo back to off position
            SRV_Channels::set_output_pwm(SRV_Channel::k_parachute_release, _servo_off_pwm);
        } else if (_release_type <= AP_PARACHUTE_TRIGGER_TYPE_RELAY_3) {
            // set relay back to zero volts
            AP_Relay*_relay = AP::relay();
            if (_relay != nullptr) {
                _relay->off(_release_type);
            }
        }
        // reset released flag and release_time
        _release_in_progress = false;
        _release_time = 0;
        // update AP_Notify
        AP_Notify::flags.parachute_release = 0;
    }
}

// set_sink_rate - set vehicle sink rate
void AP_Parachute::set_sink_rate(float sink_rate)
{
    // reset sink time if critical sink rate check is disabled or vehicle is not flying
    if ((_critical_sink <= 0) || !_is_flying) {
        _sink_time_ms = 0;
        return;
    }

    // reset sink_time if vehicle is not sinking too fast
    if (sink_rate <= _critical_sink) {
        _sink_time_ms = 0;
        return;
    }

    // start time when sinking too fast
    if (_sink_time_ms == 0) {
        _sink_time_ms = AP_HAL::millis();
    }
}

// trigger parachute release if sink_rate is below critical_sink_rate for 1sec
void AP_Parachute::check_sink_rate()
{
    // return immediately if parachute is being released or vehicle is not flying
    if (_release_initiated || !_is_flying) {
        return;
    }

    // if vehicle is sinking too fast for more than a second release parachute
    if ((_sink_time_ms > 0) && ((AP_HAL::millis() - _sink_time_ms) > 1000)) {
        release();
    }
}

// check settings are valid
bool AP_Parachute::arming_checks(size_t buflen, char *buffer) const
{
    if (_enabled > 0) {
        if (_release_type == AP_PARACHUTE_TRIGGER_TYPE_SERVO) {
            if (!SRV_Channels::function_assigned(SRV_Channel::k_parachute_release)) {
                hal.util->snprintf(buffer, buflen, "Chute has no channel");
                return false;
            }
        } else {
            AP_Relay*_relay = AP::relay();
            if (_relay == nullptr || !_relay->enabled(_release_type)) {
                hal.util->snprintf(buffer, buflen, "Chute invalid relay %d", int(_release_type));
                return false;
            }
        }
        if (_release_initiated) {
            hal.util->snprintf(buffer, buflen, "Chute is released");
            return false;
        }
    }
    return true;
}

// singleton instance
AP_Parachute *AP_Parachute::_singleton;

namespace AP {

AP_Parachute *parachute()
{
    return AP_Parachute::get_singleton();
}

}
#endif // HAL_PARACHUTE_ENABLED
