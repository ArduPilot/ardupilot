// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include <AP_Parachute.h>
#include <AP_Relay.h>
#include <AP_Math.h>
#include <RC_Channel.h>
#include <AP_Notify.h>
#include <AP_HAL.h>

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo AP_Parachute::var_info[] PROGMEM = {

    // @Param: ENABLED
    // @DisplayName: 降落伞释放启用或禁用
    // @Description: 降落伞释放启用或禁用
    // @Values: 0:禁用,1:启用
    // @User: Standard
    AP_GROUPINFO("ENABLED", 0, AP_Parachute, _enabled, 0),

    // @Param: TYPE
    // @DisplayName: 降落伞释放机制（中继或舵机）
    // @Description: 降落伞释放机制（中继或舵机）
    // @Values: 0:d第一中继,2:第二中继,3:第四中继,10:舵机
    // @User: Standard
    AP_GROUPINFO("TYPE", 1, AP_Parachute, _release_type, AP_PARACHUTE_TRIGGER_TYPE_RELAY_0),

    // @Param: SERVO_ON
    // @DisplayName: 降落伞舵机启动PWM值
    // @Description: 当降落伞释放时的舵机PWM值
    // @Range: 1000 2000
    // @Units: pwm
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("SERVO_ON", 2, AP_Parachute, _servo_on_pwm, AP_PARACHUTE_SERVO_ON_PWM_DEFAULT),

    // @Param: SERVO_OFF
    // @DisplayName: 舵机关闭的PWM值
    // @Description: 降落伞没有释放时舵机的PWM值
    // @Range: 1000 2000
    // @Units: pwm
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("SERVO_OFF", 3, AP_Parachute, _servo_off_pwm, AP_PARACHUTE_SERVO_OFF_PWM_DEFAULT),

    // @Param: ALT_MIN
    // @DisplayName: 降落伞在家的高度之上的最小高度（Parachute min altitude in cm above home为何这里是cm我也不知道）
    // @Description: 降落伞在家之上的最小高度。在这个高度之下降落伞不会被释放。0会禁用这个检查。
    // @Range: 0 32000
    // @Units: 米
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("ALT_MIN", 4, AP_Parachute, _alt_min, AP_PARACHUTE_ALT_MIN_DEFAULT),

    AP_GROUPEND
};

/// enabled - enable or disable parachute release
void AP_Parachute::enabled(bool on_off)
{
    _enabled = on_off;

    // clear release_time
    _release_time = 0;
}

/// release - release parachute
void AP_Parachute::release()
{
    // exit immediately if not enabled
    if (_enabled <= 0) {
        return;
    }

    // set release time to current system time
    _release_time = hal.scheduler->millis();

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
    uint32_t time_diff = hal.scheduler->millis() - _release_time;

    // check if we should release parachute
    if ((_release_time != 0) && !_released) {
        if (time_diff >= AP_PARACHUTE_RELEASE_DELAY_MS) {
            if (_release_type == AP_PARACHUTE_TRIGGER_TYPE_SERVO) {
                // move servo
                RC_Channel_aux::set_radio(RC_Channel_aux::k_parachute_release, _servo_on_pwm);
            }else if (_release_type <= AP_PARACHUTE_TRIGGER_TYPE_RELAY_3) {
                // set relay
                _relay.on(_release_type);
            }
            _released = true;
        }
    }else if ((_release_time == 0) || time_diff >= AP_PARACHUTE_RELEASE_DELAY_MS + AP_PARACHUTE_RELEASE_DURATION_MS) {
        if (_release_type == AP_PARACHUTE_TRIGGER_TYPE_SERVO) {
            // move servo back to off position
            RC_Channel_aux::set_radio(RC_Channel_aux::k_parachute_release, _servo_off_pwm);
        }else if (_release_type <= AP_PARACHUTE_TRIGGER_TYPE_RELAY_3) {
            // set relay back to zero volts
            _relay.off(_release_type);
        }
        // reset released flag and release_time
        _released = false;
        _release_time = 0;
        // update AP_Notify
        AP_Notify::flags.parachute_release = 0;
    }
}
