#pragma once

#include "AP_FW_Controller.h"

class AP_RollController : public AP_FW_Controller
{
public:
    AP_RollController(const AP_FixedWing &parms);

    /* Do not allow copies */
    CLASS_NO_COPY(AP_RollController);

    static const struct AP_Param::GroupInfo var_info[];

    void convert_pid();

    /*
      set the in_recovery flag, which is used during a VTOL upset recovery
      this flag only lasts one loop
    */
    void set_in_recovery(void) {
        in_recovery = true;
    }

private:
    float run_axis_rate_control(float desired_rate, float scaler, bool disable_integrator, bool ground_mode) override;

    // Return true if the airspeed should be considered as under speed
    bool is_underspeed() const override;

    // Return the measured roll angle in degrees
    float get_measured_angle() const override;

    // Return the measured roll rate in radians per second
    float get_measured_rate() const override;

    // Return true if rate limits should be applied
    bool apply_rate_limits() const override;

    // Return positive rate limit in deg per second, zero if disabled
    float get_positive_rate_limit() const override;

    // Return negative rate limit in deg per second (as a positive number) zero if disabled
    float get_negative_rate_limit() const override;

    bool in_recovery;
};
