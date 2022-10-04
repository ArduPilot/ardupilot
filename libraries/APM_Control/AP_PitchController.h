#pragma once

#include "AP_PitchRollController.h"

class AP_PitchController : public AP_PitchRollController
{
public:
    AP_PitchController(const AP_Vehicle::FixedWing &parms);

    /* Do not allow copies */
    CLASS_NO_COPY(AP_PitchController);

    float get_rate_out(float desired_rate, float scaler);
    float get_servo_out(int32_t angle_err, float scaler, bool disable_integrator, bool ground_mode);

    static const struct AP_Param::GroupInfo var_info[];

    void convert_pid();

private:

    AP_Float _roll_ff;

    float _get_rate_out(float desired_rate, float scaler, bool disable_integrator, float aspeed, bool ground_mode);
    float _get_coordination_rate_offset(float &aspeed, bool &inverted) const;
};
