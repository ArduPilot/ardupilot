#include <AP_Winch/AP_Winch_Servo.h>

extern const AP_HAL::HAL& hal;

void AP_Winch_Servo::init(const AP_WheelEncoder* wheel_encoder)
{
    _wheel_encoder = wheel_encoder;

    // set servo output range
    SRV_Channels::set_angle(SRV_Channel::k_winch,  1000);
}

void AP_Winch_Servo::update()
{
    // return immediately if no servo is assigned to control the winch
    if (!SRV_Channels::function_assigned(SRV_Channel::k_winch)) {
        return;
    }

    // return immediately if no wheel encoder
    if (_wheel_encoder == nullptr) {
        return;
    }

    // if not doing any control output trim value
    if (config.state == AP_Winch::STATE_RELAXED) {
        SRV_Channels::set_output_limit(SRV_Channel::k_winch, SRV_Channel::SRV_CHANNEL_LIMIT_TRIM);
        return;
    }

    // calculate dt since last iteration
    uint32_t now = AP_HAL::millis();
    float dt = (now - last_update_ms) / 1000.0f;
    if (dt > 1.0f) {
        dt = 0.0f;
    }
    last_update_ms = now;

    // calculate latest rate
    float distance = _wheel_encoder->get_distance(0);
    float rate = 0.0f;
    if (is_positive(dt)) {
        rate = (distance - config.length_curr) / dt;
    }

    // update distance from wheel encoder
    config.length_curr = distance;

    // if doing position control, calculate position error to desired rate
    float rate_desired = 0.0f;
    if (config.state == AP_Winch::STATE_POSITION) {
        float position_error = config.length_desired - config.length_curr;
        rate_desired = constrain_float(position_error * config.pos_p, -config.rate_desired, config.rate_desired);
    }

    // if doing rate control, set desired rate
    if (config.state == AP_Winch::STATE_RATE) {
        rate_desired = config.rate_desired;
    }

    // calculate rate error and pass to pid controller
    float rate_error = rate_desired - rate;
    config.rate_pid.set_input_filter_all(rate_error);

    // get p
    float p = config.rate_pid.get_p();

    // get i unless winch hit limit on last iteration
    float i = config.rate_pid.get_integrator();
    if (((is_negative(rate_error) && !limit_low) || (is_positive(rate_error) && !limit_high))) {
        i = config.rate_pid.get_i();
    }

    // get d
    float d = config.rate_pid.get_d();

    // calculate base output
    float base = 0.0f;
    if (is_positive(config.rate_max)) {
        base = rate_desired / config.rate_max;
    }

    // constrain and set limit flags
    float output = base + p + i + d;
    limit_low = (output <= -1.0f);
    limit_high = (output >= 1.0f);
    output = constrain_float(output, -1.0f, 1.0f);

    // output to servo
    SRV_Channels::set_output_scaled(SRV_Channel::k_winch,  output * 1000);
}
