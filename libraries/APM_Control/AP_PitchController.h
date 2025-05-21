#pragma once

#include "AP_FW_Controller.h"

class AP_PitchController : public AP_FW_Controller
{
public:
    AP_PitchController(const AP_FixedWing &parms);

    /* Do not allow copies */
    CLASS_NO_COPY(AP_PitchController);

    float get_servo_out(int32_t angle_err, float scaler, bool disable_integrator, bool ground_mode) override;

    static const struct AP_Param::GroupInfo var_info[];

    void convert_pid();

private:
    AP_Float _roll_ff;

    float _get_coordination_rate_offset(const float &aspeed, bool &inverted) const;

    float get_airspeed() const override;
    bool is_underspeed(const float aspeed) const override;
    float get_measured_rate() const override;

};
