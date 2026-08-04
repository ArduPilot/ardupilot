#include "PWMForwarder.h"

#include <AP_HAL/AP_HAL.h>
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL &hal;

const AP_Param::GroupInfo PWMForwarder::var_info[] = {
    // @Param: ENABLE
    // @DisplayName: External PWM forwarding enable mask
    // @Description: Enables ExternalPWM routes configured with SERVOx_FUNCTION. Route 1 uses ExternalPWM IN1 and OUT1. Route 2 uses ExternalPWM IN2 and OUT2.
    // @Bitmask: 0:Route 1,1:Route 2
    // @User: Advanced
    AP_GROUPINFO_FLAGS("ENABLE", 1, PWMForwarder, enable, 0, AP_PARAM_FLAG_ENABLE),

    // Parameter indexes 2 to 5 were used by the old SRC/DST parameters.

    // @Param: MIN
    // @DisplayName: External PWM minimum valid pulse
    // @Description: External PWM pulses shorter than this value are rejected.
    // @Units: PWM
    // @Range: 400 2200
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("MIN", 6, PWMForwarder, pwm_min, 800),

    // @Param: MAX
    // @DisplayName: External PWM maximum valid pulse
    // @Description: External PWM pulses longer than this value are rejected.
    // @Units: PWM
    // @Range: 800 3000
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("MAX", 7, PWMForwarder, pwm_max, 2200),

    // @Param: TIMEOUT
    // @DisplayName: External PWM signal timeout
    // @Description: Time without a valid external PWM pulse before applying PWMFWD_FS_PWM.
    // @Units: ms
    // @Range: 20 10000
    // @Increment: 10
    // @User: Advanced
    AP_GROUPINFO("TIMEOUT", 8, PWMForwarder, timeout_ms, 100),

    // @Param: FS_PWM
    // @DisplayName: External PWM failsafe output
    // @Description: PWM applied after input signal loss. Zero releases the output back to its normal trim. Values from 800 to 2200 force a fixed output.
    // @Units: PWM
    // @Range: 0 2200
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("FS_PWM", 9, PWMForwarder, failsafe_pwm, 0),

    AP_GROUPEND
};

PWMForwarder::PWMForwarder()
{
    AP_Param::setup_object_defaults(this, var_info);
}

bool PWMForwarder::servo_channel_to_gpio(uint8_t servo_channel, uint8_t &gpio_pin) const
{
    for (uint16_t candidate = 0; candidate <= UINT8_MAX; candidate++) {
        uint8_t mapped_channel;
        if (hal.gpio->pin_to_servo_channel(candidate, mapped_channel) &&
            mapped_channel == servo_channel) {
            gpio_pin = candidate;
            return true;
        }
    }
    return false;
}

bool PWMForwarder::find_unique_input(SRV_Channel::Aux_servo_function_t function, uint8_t &channel) const
{
    uint8_t count = 0;
    for (uint8_t i = 0; i < NUM_SERVO_CHANNELS; i++) {
        if (SRV_Channels::channel_function(i) == function) {
            channel = i;
            count++;
        }
    }
    return count == 1;
}

bool PWMForwarder::validate_outputs(SRV_Channel::Aux_servo_function_t function, uint8_t route_index) const
{
    uint8_t count = 0;
    for (uint8_t i = 0; i < NUM_SERVO_CHANNELS; i++) {
        if (SRV_Channels::channel_function(i) != function) {
            continue;
        }
        if (SRV_Channels::is_GPIO(i)) {
            GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "PWMFWD%u: SERVO%u output is GPIO",
                          unsigned(route_index + 1), unsigned(i + 1));
            return false;
        }
        count++;
    }
    if (count == 0) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "PWMFWD%u: ExternalPWM OUT%u not assigned",
                      unsigned(route_index + 1), unsigned(route_index + 1));
        return false;
    }
    return true;
}

void PWMForwarder::write_outputs(SRV_Channel::Aux_servo_function_t function,
                                 uint16_t pwm, uint16_t timeout)
{
    for (uint8_t i = 0; i < NUM_SERVO_CHANNELS; i++) {
        if (SRV_Channels::channel_function(i) == function) {
            SRV_Channels::set_output_pwm_chan_timeout(i, pwm, timeout);
        }
    }
}

void PWMForwarder::release_outputs(SRV_Channel::Aux_servo_function_t function)
{
    for (uint8_t i = 0; i < NUM_SERVO_CHANNELS; i++) {
        if (SRV_Channels::channel_function(i) != function) {
            continue;
        }
        uint16_t current_pwm = 1500;
        SRV_Channels::get_output_pwm_chan(i, current_pwm);
        SRV_Channels::set_output_pwm_chan_timeout(i, current_pwm, 0);
    }
}

void PWMForwarder::deactivate_route(uint8_t route_index)
{
    Route &route = routes[route_index];
    route.source.set_pin(0, "PWMFWD");
    release_outputs(route.output_function);
    route.output_function = SRV_Channel::k_none;
    route.source_servo = 0;
    route.last_valid_ms = 0;
    route.active = false;
    route.failsafe = false;
}

void PWMForwarder::configure_route(uint8_t route_index, bool requested)
{
    deactivate_route(route_index);
    if (!requested) {
        return;
    }

    const auto input_function = route_index == 0 ?
        SRV_Channel::k_external_pwm_in1 : SRV_Channel::k_external_pwm_in2;
    const auto output_function = route_index == 0 ?
        SRV_Channel::k_external_pwm_out1 : SRV_Channel::k_external_pwm_out2;

    uint8_t source_channel = 0;
    if (!find_unique_input(input_function, source_channel)) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "PWMFWD%u: assign exactly one ExternalPWM IN%u",
                      unsigned(route_index + 1), unsigned(route_index + 1));
        return;
    }
    if (!validate_outputs(output_function, route_index)) {
        return;
    }

    uint8_t gpio_pin;
    if (!servo_channel_to_gpio(source_channel, gpio_pin)) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "PWMFWD%u: no GPIO for SERVO%u",
                      unsigned(route_index + 1), unsigned(source_channel + 1));
        return;
    }

    Route &route = routes[route_index];
    if (!route.source.set_pin(gpio_pin, "PWMFWD")) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "PWMFWD%u: cannot capture GPIO%u",
                      unsigned(route_index + 1), unsigned(gpio_pin));
        return;
    }

    route.output_function = output_function;
    route.source_servo = source_channel + 1;
    route.last_valid_ms = AP_HAL::millis();
    route.active = true;
    GCS_SEND_TEXT(MAV_SEVERITY_NOTICE, "PWMFWD%u: SERVO%u -> ExternalPWM OUT%u",
                  unsigned(route_index + 1), unsigned(route.source_servo),
                  unsigned(route_index + 1));
}

void PWMForwarder::configure()
{
    const uint8_t enabled_routes = constrain_int16(enable.get(), 0, 3);
    configure_route(0, (enabled_routes & 1U) != 0);
    configure_route(1, (enabled_routes & 2U) != 0);
    configured_enable = enable.get();
    configured_once = true;
}

void PWMForwarder::apply_failsafe(uint8_t route_index, uint16_t pwm)
{
    Route &route = routes[route_index];
    if (pwm >= 800 && pwm <= 2200) {
        write_outputs(route.output_function, pwm, 200);
    } else if (!route.failsafe) {
        release_outputs(route.output_function);
    }

    if (!route.failsafe) {
        route.failsafe = true;
        if (pwm >= 800 && pwm <= 2200) {
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "PWMFWD%u: signal lost, output=%u",
                          unsigned(route_index + 1), unsigned(pwm));
        } else {
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "PWMFWD%u: signal lost, output released",
                          unsigned(route_index + 1));
        }
    }
}

void PWMForwarder::update()
{
    if (!configured_once || configured_enable != enable.get()) {
        configure();
    }

    const uint16_t minimum = constrain_int16(pwm_min.get(), 400, 2200);
    const uint16_t maximum = constrain_int16(pwm_max.get(), 800, 3000);
    const uint16_t timeout = constrain_int16(timeout_ms.get(), 20, 10000);
    const uint16_t failsafe = static_cast<uint16_t>(MAX(failsafe_pwm.get(), 0));
    const uint32_t now = AP_HAL::millis();

    for (uint8_t i = 0; i < NUM_ROUTES; i++) {
        Route &route = routes[i];
        if (!route.active) {
            continue;
        }

        const uint16_t pwm = route.source.get_pwm_us();
        if (minimum < maximum && pwm >= minimum && pwm <= maximum) {
            route.last_valid_ms = now;
            const uint16_t override_timeout = MIN(timeout + 50U, UINT16_MAX);
            write_outputs(route.output_function, pwm, override_timeout);
            if (route.failsafe) {
                route.failsafe = false;
                GCS_SEND_TEXT(MAV_SEVERITY_NOTICE, "PWMFWD%u: signal restored", unsigned(i + 1));
            }
        } else if (now - route.last_valid_ms >= timeout) {
            apply_failsafe(i, failsafe);
        }
    }
}
