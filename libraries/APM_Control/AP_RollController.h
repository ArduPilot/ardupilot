#pragma once

#include "AP_PitchRollController.h"

class AP_RollController : public AP_PitchRollController
{
public:
    AP_RollController(const AP_Vehicle::FixedWing &parms);

    /* Do not allow copies */
    CLASS_NO_COPY(AP_RollController);

    float get_rate_out(float desired_rate, float scaler);
    float get_servo_out(int32_t angle_err, float scaler, bool disable_integrator, bool ground_mode);

    static const struct AP_Param::GroupInfo var_info[];

    void convert_pid();

private:

    float _get_rate_out(float desired_rate, float scaler, bool disable_integrator, bool ground_mode);
};
