#include "AP_Winch.h"
#include "AP_Winch_Servo.h"

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo AP_Winch::var_info[] = {
    // @Param: ENABLE
    // @DisplayName: Winch enable/disable
    // @Description: Winch enable/disable
    // @User: Standard
    // @Values: 0:Disabled, 1:Enabled
    AP_GROUPINFO_FLAGS("_ENABLE", 0, AP_Winch, _enabled, 0, AP_PARAM_FLAG_ENABLE),

    // @Param: TYPE
    // @DisplayName: Winch Type
    // @Description: Winch Type
    // @User: Standard
    // @Values: 1:Servo with encoder
    AP_GROUPINFO("_TYPE", 1, AP_Winch, config.type, 1),

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
    AP_GROUPINFO("_POS_P", 3, AP_Winch, config.pos_p, AP_WINCH_POS_P),

    // @Param: _RATE_P
    // @DisplayName: Winch control rate P gain
    // @Description: Winch control rate P gain.  Converts rate error (in radians/sec) to pwm output (in the range -1 to +1)
    // @Range: 0.100 2.000
    // @User: Standard

    // @Param: _RATE_I
    // @DisplayName: Winch control I gain
    // @Description: Winch control I gain.  Corrects long term error between the desired rate (in rad/s) and actual
    // @Range: 0.000 2.000
    // @User: Standard

    // @Param: _RATE_IMAX
    // @DisplayName: Winch control I gain maximum
    // @Description: Winch control I gain maximum.  Constrains the output (range -1 to +1) that the I term will generate
    // @Range: 0.000 1.000
    // @User: Standard

    // @Param: _RATE_D
    // @DisplayName: Winch control D gain
    // @Description: Winch control D gain.  Compensates for short-term change in desired rate vs actual
    // @Range: 0.000 0.400
    // @User: Standard

    // @Param: _RATE_FILT
    // @DisplayName: Winch control filter frequency
    // @Description: Winch control input filter.  Lower values reduce noise but add delay.
    // @Range: 1.000 100.000
    // @Units: Hz
    // @User: Standard
    AP_SUBGROUPINFO(config.rate_pid, "_RATE_", 4, AP_Winch, AC_PID),

    AP_GROUPEND
};

AP_Winch::AP_Winch()
{
    AP_Param::setup_object_defaults(this, var_info);
}

// indicate whether this module is enabled
bool AP_Winch::enabled() const
{
    if (!_enabled) {
        return false;
    }

    return ((config.type > 0) && (backend != nullptr));
}

void AP_Winch::init(const AP_WheelEncoder *wheel_encoder)
{
    // return immediately if not enabled
    if (!_enabled.get()) {
        return;
    }

    switch(config.type.get()) {
    case 0:
        break;
    case 1:
        backend = new AP_Winch_Servo(config);
        break;
    default:
        break;
    }
    if (backend != nullptr) {
        backend->init(wheel_encoder);
    }
}

// release specified length of cable (in meters) at the specified rate
// if rate is zero, the RATE_MAX parameter value will be used
void AP_Winch::release_length(float length, float rate)
{
    config.length_desired = config.length_curr + length;
    config.state = STATE_POSITION;
    if (is_zero(rate)) {
        config.rate_desired = config.rate_max;
    } else {
        config.rate_desired = constrain_float(fabsf(rate), -get_rate_max(), get_rate_max());
    }
}

// deploy line at specified speed in m/s (+ve deploys line, -ve retracts line, 0 stops)
void AP_Winch::set_desired_rate(float rate)
{
    config.rate_desired = constrain_float(rate, -get_rate_max(), get_rate_max());
    config.state = STATE_RATE;
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

#undef PASS_TO_BACKEND
