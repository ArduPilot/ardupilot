#pragma once

#include "AP_FW_Controller.h"

class AP_RollController : public AP_FW_Controller
{
public:
    AP_RollController(const AP_FixedWing &parms);

    /* Do not allow copies */
    CLASS_NO_COPY(AP_RollController);

    float get_servo_out(int32_t angle_err, float scaler, bool disable_integrator, bool ground_mode) override;

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
    float get_airspeed() const override;
    bool is_underspeed(const float aspeed) const override;
    float get_measured_rate() const override;

    bool in_recovery;
};
