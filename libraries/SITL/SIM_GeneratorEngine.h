#pragma once

#include <stdint.h>

namespace SITL {

class SIM_GeneratorEngine
{
public:
    void update();

    // input variables:
    float desired_rpm;
    float current_current;
    float max_current;
    float max_slew_rpm_per_second;
    float max_rpm = 8000;

    // output variables:
    float current_rpm;
    float temperature = 20;

private:
    uint32_t last_rpm_update_ms;
    uint32_t last_heat_update_ms;
};

};  // namespace SITL
