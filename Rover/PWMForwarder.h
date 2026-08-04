#pragma once

#include <AP_HAL/GPIO.h>
#include <AP_Param/AP_Param.h>
#include <SRV_Channel/SRV_Channel.h>

class PWMForwarder {
public:
    PWMForwarder();

    static const AP_Param::GroupInfo var_info[];

    void update();

private:
    static constexpr uint8_t NUM_ROUTES = 2;

    struct Route {
        AP_HAL::PWMSource source;
        SRV_Channel::Aux_servo_function_t output_function{SRV_Channel::k_none};
        uint8_t source_servo{};
        uint32_t last_valid_ms{};
        bool active{};
        bool failsafe{};
    } routes[NUM_ROUTES];

    AP_Int8 enable;
    AP_Int16 pwm_min;
    AP_Int16 pwm_max;
    AP_Int16 timeout_ms;
    AP_Int16 failsafe_pwm;

    int8_t configured_enable{};
    bool configured_once{};

    void configure();
    void configure_route(uint8_t route_index, bool requested);
    void deactivate_route(uint8_t route_index);
    void release_outputs(SRV_Channel::Aux_servo_function_t function);
    void write_outputs(SRV_Channel::Aux_servo_function_t function, uint16_t pwm, uint16_t timeout);
    void apply_failsafe(uint8_t route_index, uint16_t pwm);
    bool find_unique_input(SRV_Channel::Aux_servo_function_t function, uint8_t &channel) const;
    bool validate_outputs(SRV_Channel::Aux_servo_function_t function, uint8_t route_index) const;
    bool servo_channel_to_gpio(uint8_t servo_channel, uint8_t &gpio_pin) const;
};
