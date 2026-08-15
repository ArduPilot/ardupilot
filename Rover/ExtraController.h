#pragma once

#include <AP_HAL/GPIO.h>
#include <AP_Param/AP_Param.h>
#include <SRV_Channel/SRV_Channel.h>

class ExtraController {
public:
    ExtraController();

    static const AP_Param::GroupInfo var_info[];

    void update();
    void apply();
    bool ready() const;

private:
    static constexpr uint8_t NUM_INPUTS = 2;
    static constexpr uint8_t INVALID_CHANNEL = UINT8_MAX;

    struct Input {
        AP_HAL::PWMSource source;
        SRV_Channel::Aux_servo_function_t function{SRV_Channel::k_none};
        uint8_t channel{INVALID_CHANNEL};
        uint32_t last_valid_ms{};
        float value{};
        bool configured{};
    } inputs[NUM_INPUTS];

    AP_Int16 deadzone;
    AP_Int16 timeout_ms;

    uint32_t last_config_check_ms{};
    bool configured_once{};

    void configure();
    bool configuration_changed() const;
    bool find_unique_input(SRV_Channel::Aux_servo_function_t function, uint8_t &channel) const;
    bool servo_channel_to_gpio(uint8_t servo_channel, uint8_t &gpio_pin) const;
    float pwm_to_normalized(const Input &input, uint16_t pwm) const;
};
