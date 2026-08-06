#include "ExtraController.h"
#include "Rover.h"

#include <AP_HAL/AP_HAL.h>
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL &hal;

const AP_Param::GroupInfo ExtraController::var_info[] = {
    // @Param: MIN
    // @DisplayName: Extra controller minimum PWM
    // @Description: PWM value interpreted as full reverse throttle or full left steering for the extra controller.
    // @Units: PWM
    // @Range: 800 1500
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("MIN", 10, ExtraController, pwm_min, 1000),

    // @Param: TRIM
    // @DisplayName: Extra controller neutral PWM
    // @Description: PWM value interpreted as neutral throttle and centered steering for the extra controller.
    // @Units: PWM
    // @Range: 1000 2000
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("TRIM", 11, ExtraController, pwm_trim, 1500),

    // @Param: MAX
    // @DisplayName: Extra controller maximum PWM
    // @Description: PWM value interpreted as full forward throttle or full right steering for the extra controller.
    // @Units: PWM
    // @Range: 1500 2200
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("MAX", 12, ExtraController, pwm_max, 2000),

    // @Param: DZ
    // @DisplayName: Extra controller deadzone
    // @Description: Deadzone around EXTRA_TRIM applied independently to throttle and steering inputs.
    // @Units: PWM
    // @Range: 0 200
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("DZ", 13, ExtraController, deadzone, 30),

    // @Param: TIMEOUT
    // @DisplayName: Extra controller signal timeout
    // @Description: Maximum time without a valid ExtraThrottle or ExtraSteering pulse before extraArm is automatically cleared.
    // @Units: ms
    // @Range: 20 1000
    // @Increment: 10
    // @User: Advanced
    AP_GROUPINFO("TIMEOUT", 14, ExtraController, timeout_ms, 100),

    AP_GROUPEND
};

ExtraController::ExtraController()
{
    AP_Param::setup_object_defaults(this, var_info);
    inputs[0].function = SRV_Channel::k_extra_throttle;
    inputs[1].function = SRV_Channel::k_extra_steering;
}

bool ExtraController::servo_channel_to_gpio(uint8_t servo_channel, uint8_t &gpio_pin) const
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

bool ExtraController::find_unique_input(SRV_Channel::Aux_servo_function_t function,
                                        uint8_t &channel) const
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

bool ExtraController::configuration_changed() const
{
    for (const Input &input : inputs) {
        uint8_t channel = INVALID_CHANNEL;
        if (!find_unique_input(input.function, channel)) {
            channel = INVALID_CHANNEL;
        }
        if (channel != input.channel) {
            return true;
        }
    }
    return false;
}

void ExtraController::configure()
{
    for (Input &input : inputs) {
        input.source.set_pin(0, "ExtraCtrl");
        input.channel = INVALID_CHANNEL;
        input.last_valid_ms = 0;
        input.value = 0.0f;
        input.configured = false;

        uint8_t channel = INVALID_CHANNEL;
        if (!find_unique_input(input.function, channel)) {
            const char *name = input.function == SRV_Channel::k_extra_throttle ?
                "ExtraThrottle" : "ExtraSteering";
            GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "ExtraCtrl: assign exactly one %s", name);
            continue;
        }
        input.channel = channel;

        uint8_t gpio_pin;
        if (!servo_channel_to_gpio(channel, gpio_pin)) {
            GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "ExtraCtrl: no GPIO for SERVO%u",
                          unsigned(channel + 1));
            continue;
        }
        if (!input.source.set_pin(gpio_pin, "ExtraCtrl")) {
            GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "ExtraCtrl: cannot capture GPIO%u",
                          unsigned(gpio_pin));
            continue;
        }
        input.configured = true;
    }

    configured_once = true;
    if (inputs[0].configured && inputs[1].configured) {
        GCS_SEND_TEXT(MAV_SEVERITY_NOTICE,
                      "ExtraCtrl: throttle=SERVO%u steering=SERVO%u",
                      unsigned(inputs[0].channel + 1),
                      unsigned(inputs[1].channel + 1));
    }
}

float ExtraController::pwm_to_normalized(uint16_t pwm, bool reversed) const
{
    const int16_t minimum = constrain_int16(pwm_min.get(), 800, 1500);
    const int16_t trim = constrain_int16(pwm_trim.get(), minimum + 1, 2000);
    const int16_t maximum = constrain_int16(pwm_max.get(), trim + 1, 2200);
    const int16_t dz = constrain_int16(deadzone.get(), 0, 200);
    const int16_t delta = int16_t(pwm) - trim;

    float value = 0.0f;
    if (delta > dz) {
        value = float(delta - dz) / float(MAX(maximum - trim - dz, 1));
    } else if (delta < -dz) {
        value = float(delta + dz) / float(MAX(trim - minimum - dz, 1));
    }
    value = constrain_float(value, -1.0f, 1.0f);
    return reversed ? -value : value;
}

bool ExtraController::ready() const
{
    const uint32_t now = AP_HAL::millis();
    const uint16_t timeout = constrain_int16(timeout_ms.get(), 20, 1000);
    for (const Input &input : inputs) {
        if (!input.configured || input.last_valid_ms == 0 ||
            now - input.last_valid_ms > timeout) {
            return false;
        }
    }
    return true;
}

void ExtraController::update()
{
    const uint32_t now = AP_HAL::millis();
    if (!configured_once ||
        (now - last_config_check_ms >= 1000U && configuration_changed())) {
        configure();
    }
    if (now - last_config_check_ms >= 1000U) {
        last_config_check_ms = now;
    }

    const uint16_t minimum = constrain_int16(pwm_min.get(), 800, 1500);
    const uint16_t maximum = constrain_int16(pwm_max.get(), 1500, 2200);
    for (Input &input : inputs) {
        if (!input.configured) {
            continue;
        }
        const uint16_t pwm = input.source.get_pwm_us();
        if (pwm >= minimum && pwm <= maximum) {
            input.last_valid_ms = now;
            input.value = pwm_to_normalized(
                pwm,
                SRV_Channels::channel_reversed(input.channel));
        }
        else
        {
        }
        // GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "ExtraCtrl: PWM: %u, avg PWM: %u", pwm, input.source.get_pwm_avg_us());
    }

    if (!rover.extra_controller_armed) {
        return;
    }

    if (rover.arming.is_armed() || !ready()) {
        rover.arm_extra_controller(false, false);
        if (!ready()) {
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "ExtraCtrl: signal lost, disarmed");
        }
        return;
    }

    rover.g2.motors.set_steering(inputs[1].value * 4500.0f, false);
    rover.g2.motors.set_throttle(inputs[0].value * 100.0f);
}

bool Rover::arm_extra_controller(bool is_armed, bool force)
{
    if (!is_armed) {
        if (extra_controller_armed) {
            g2.motors.set_throttle(0.0f);
            g2.motors.set_steering(0.0f, false);
            extra_controller_armed = false;
            arming.update_soft_armed();
            GCS_SEND_TEXT(MAV_SEVERITY_NOTICE, "ExtraCtrl: disarmed");
        }
        return false;
    }

    if (extra_controller_armed) {
        return true;
    }
    if (!extra_controller.ready()) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "ExtraCtrl: inputs not ready");
        return false;
    }

    if (arming.is_armed()) {
        if (!force || !arming.disarm(AP_Arming::Method::SCRIPTING, false)) {
            return false;
        }
        if (arming.is_armed()) {
            return false;
        }
    }

    extra_controller_armed = true;
    arming.update_soft_armed();
    GCS_SEND_TEXT(MAV_SEVERITY_NOTICE, "ExtraCtrl: armed");
    return true;
}
