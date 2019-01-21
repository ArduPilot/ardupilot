/*
  support for autotune of quadplanes
 */

#pragma once

#include <AP_HAL/AP_HAL.h>

#define QAUTOTUNE_ENABLED !HAL_MINIMIZE_FEATURES

#if QAUTOTUNE_ENABLED

#include <AC_AutoTune/AC_AutoTune.h>

class QAutoTune : public AC_AutoTune
{
public:
    friend class QuadPlane;

    bool init() override;

protected:
    float get_pilot_desired_climb_rate_cms(void) const override;
    void get_pilot_desired_rp_yrate_cd(int32_t &roll_cd, int32_t &pitch_cd, int32_t &yaw_rate_cds) override;
    void init_z_limits() override;
    void Log_Write_Event(enum at_event id) override;
    void log_pids() override;
};

#endif // QAUTOTUNE_ENABLED
