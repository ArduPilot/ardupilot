#include "AP_Winch_PWM.h"

#if AP_WINCH_PWM_ENABLED

#include <AP_Logger/AP_Logger.h>
#include <GCS_MAVLink/GCS.h>
#include <SRV_Channel/SRV_Channel.h>

extern const AP_HAL::HAL& hal;

// true if winch is healthy
bool AP_Winch_PWM::healthy() const
{
    // return immediately if no servo is assigned to control the winch
    if (!SRV_Channels::function_assigned(SRV_Channel::k_winch)) {
        return false;
    }

    return true;
}

void AP_Winch_PWM::update()
{
    // return immediately if no servo is assigned to control the winch
    if (!SRV_Channels::function_assigned(SRV_Channel::k_winch)) {
        return;
    }

    // read pilot input
    read_pilot_desired_rate();

    // send outputs to winch
    control_winch();
}

// update pwm outputs to control winch
void AP_Winch_PWM::control_winch()
{
    const uint32_t now_ms = AP_HAL::millis();
    float dt = (now_ms - control_update_ms) * 0.001f;
    if (dt > 1.0f) {
        dt = 0.0f;
    }
    control_update_ms = now_ms;

    // if relaxed stop outputting pwm signals
    if (config.control_mode == AP_Winch::ControlMode::RELAXED) {
        // if not doing any control stop sending pwm to winch
        SRV_Channels::set_output_pwm(SRV_Channel::k_winch, 0);

        // rate used for acceleration limiting reset to zero
        set_previous_rate(0.0f);
        return;
    }

    // if doing position control, calculate position error to desired rate
    if ((config.control_mode == AP_Winch::ControlMode::POSITION) && healthy()) {
        float position_error = config.length_desired - line_length;
        config.rate_desired = constrain_float(position_error * config.pos_p, -config.rate_max, config.rate_max);
    }

    // apply acceleration limited to rate
    const float rate_limited = get_rate_limited_by_accel(config.rate_desired, dt);

    // use linear interpolation to calculate output to move winch at desired rate
    const float scaled_output = linear_interpolate(-1000, 1000, rate_limited, -config.rate_max, config.rate_max);
    SRV_Channels::set_output_scaled(SRV_Channel::k_winch, scaled_output);

    // update distance estimate assuming winch will move exactly as requested
    line_length += config.rate_desired * dt;
}

//send generator status
void AP_Winch_PWM::send_status(const GCS_MAVLINK &channel)
{
    // prepare status bitmask
    uint32_t status_bitmask = 0;
    if (healthy()) {
        status_bitmask |= MAV_WINCH_STATUS_HEALTHY;
    }

    // send status
    mavlink_msg_winch_status_send(
        channel.get_chan(),
        AP_HAL::micros64(),     // time_usec
        line_length,            // line_length
        config.rate_desired,    // speed
        std::numeric_limits<double>::quiet_NaN(),   // tension
        std::numeric_limits<double>::quiet_NaN(),   // voltage
        std::numeric_limits<double>::quiet_NaN(),   // current
        INT16_MAX,              // temperature
        status_bitmask);        // status flags
}

// write log
#if HAL_LOGGING_ENABLED
void AP_Winch_PWM::write_log()
{
    AP::logger().Write_Winch(
            healthy(),
            0,              // thread end (unsupported)
            0,              // moving (unsupported)
            0,              // clutch (unsupported)
            (uint8_t)config.control_mode,
            config.length_desired,
            get_current_length(),
            config.rate_desired,
            0,              // tension (unsupported)
            0,              // voltage (unsupported)
            0);             // temp (unsupported)
}
#endif

#endif  // AP_WINCH_PWM_ENABLED
