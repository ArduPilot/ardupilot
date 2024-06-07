#include "SIM_GeneratorEngine.h"

#include <AP_Math/AP_Math.h>

#include <GCS_MAVLink/GCS.h>

using namespace SITL;

void SIM_GeneratorEngine::update()
{
    if (current_current > 1 && is_zero(desired_rpm)) {
        AP_HAL::panic("Generator stalled due to high current demand (%u > 1)", (unsigned)current_current);
    } else if (current_current > max_current) {
        AP_HAL::panic("Generator stalled due to high current demand (run)");
    }

    // linear degradation in RPM up to maximum load
    if (!is_zero(desired_rpm)) {
        desired_rpm -= 1500 * (current_current/max_current);
    }

    const uint32_t now = AP_HAL::millis();

    const float max_slew_rpm = max_slew_rpm_per_second * ((now - last_rpm_update_ms) / 1000.0f);
    last_rpm_update_ms = now;
    const float rpm_delta = current_rpm - desired_rpm;
    if (rpm_delta > 0) {
        current_rpm -= MIN(max_slew_rpm, rpm_delta);
    } else {
        current_rpm += MIN(max_slew_rpm, abs(rpm_delta));
    }

    // update the temperature
    const uint32_t time_delta_ms = now - last_heat_update_ms;
    last_heat_update_ms = now;

    constexpr float heat_environment_loss_factor = 0.15f;

    const float factor = 0.0035;
    temperature += (current_rpm * time_delta_ms * (1/1000.0f) * factor);
    // cap the heat of the motor:
    temperature = MIN(temperature, 150);
    // now lose some heat to the environment
    const float heat_loss = ((temperature * heat_environment_loss_factor * (time_delta_ms * (1/1000.0f))));  // lose some % of heat per second
    // gcs().send_text(MAV_SEVERITY_INFO, "heat=%f loss=%f", temperature, heat_loss);
    temperature -= heat_loss;

}
