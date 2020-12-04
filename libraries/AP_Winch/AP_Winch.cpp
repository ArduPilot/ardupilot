#include "AP_Winch.h"
#include "AP_Winch_PWM.h"
#include "AP_Winch_Daiwa.h"

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo AP_Winch::var_info[] = {
    // 0 was ENABLE

    // @Param: _TYPE
    // @DisplayName: Winch Type
    // @Description: Winch Type
    // @User: Standard
    // @Values: 0:None, 1:PWM, 2:Daiwa
    AP_GROUPINFO_FLAGS("_TYPE", 1, AP_Winch, config.type, (int8_t)WinchType::NONE, AP_PARAM_FLAG_ENABLE),

    // @Param: _RATE_MAX
    // @DisplayName: Winch deploy or retract rate maximum
    // @Description: Winch deploy or retract rate maximum.  Set to maximum rate with no load.
    // @User: Standard
    // @Range: 0 10
    // @Units: m/s
    AP_GROUPINFO("_RATE_MAX", 2, AP_Winch, config.rate_max, 1.0f),

    // @Param: _POS_P
    // @DisplayName: Winch control position error P gain
    // @Description: Winch control position error P gain
    // @Range: 0.01 10.0
    // @User: Standard
    AP_GROUPINFO("_POS_P", 3, AP_Winch, config.pos_p, 1.0f),

    // 4 was _RATE_PID

    AP_GROUPEND
};

AP_Winch::AP_Winch()
{
    if (_singleton) {
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
        AP_HAL::panic("Too many winches");
#endif
        return;
    }
    _singleton = this;

    AP_Param::setup_object_defaults(this, var_info);
}

// indicate whether this module is enabled
bool AP_Winch::enabled() const
{
   return ((config.type > 0) && (backend != nullptr));
}

// true if winch is healthy
bool AP_Winch::healthy() const
{
    if (backend != nullptr) {
        return backend->healthy();
    }
    return false;
}

void AP_Winch::init()
{
    switch ((WinchType)config.type.get()) {
    case WinchType::NONE:
        break;
    case WinchType::PWM:
        backend = new AP_Winch_PWM(config);
        break;
    case WinchType::DAIWA:
        backend = new AP_Winch_Daiwa(config);
        break;
    default:
        break;
    }
    if (backend != nullptr) {
        backend->init();
    }
}

// release specified length of cable (in meters)
void AP_Winch::release_length(float length)
{
    if (backend == nullptr) {
        return;
    }
    config.length_desired = backend->get_current_length() + length;
    config.control_mode = ControlMode::POSITION;
}

// deploy line at specified speed in m/s (+ve deploys line, -ve retracts line, 0 stops)
void AP_Winch::set_desired_rate(float rate)
{
    config.rate_desired = constrain_float(rate, -get_rate_max(), get_rate_max());
    config.control_mode = ControlMode::RATE;
}

// send status to ground station
void AP_Winch::send_status(const GCS_MAVLINK &channel)
{
    if (backend != nullptr) {
        backend->send_status(channel);
    }
}

// returns true if pre arm checks have passed
bool AP_Winch::pre_arm_check(char *failmsg, uint8_t failmsg_len) const
{
    // succeed if winch is disabled
    if ((WinchType)config.type.get() == WinchType::NONE) {
        return true;
    }

    // fail if unhealthy
    if (!healthy()) {
        hal.util->snprintf(failmsg, failmsg_len, "winch unhealthy");
        return false;
    }

    return true;
}

// update - should be called at at least 10hz
#define PASS_TO_BACKEND(function_name) \
    void AP_Winch::function_name()   \
    {                                  \
        if (!enabled()) {              \
            return;                    \
        }                              \
        if (backend != nullptr) {      \
            backend->function_name();  \
        }                              \
    }

PASS_TO_BACKEND(update)
PASS_TO_BACKEND(write_log)

#undef PASS_TO_BACKEND

/*
 * Get the AP_Winch singleton
 */
AP_Winch *AP_Winch::_singleton;
AP_Winch *AP_Winch::get_singleton()
{
    return _singleton;
}

namespace AP {

AP_Winch *winch()
{
    return AP_Winch::get_singleton();
}

};
