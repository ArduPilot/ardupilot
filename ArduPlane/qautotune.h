/*
  support for autotune of quadplanes
 */

#pragma once

#include <AP_HAL/AP_HAL_Boards.h>

#include "quadplane.h"
#ifndef QAUTOTUNE_ENABLED
  #define QAUTOTUNE_ENABLED HAL_QUADPLANE_ENABLED && !HAL_MINIMIZE_FEATURES
#endif

#if QAUTOTUNE_ENABLED

#include <AC_AutoTune/AC_AutoTune_Multi.h>

class QAutoTune : public AC_AutoTune_Multi
{
public:
    friend class QuadPlane;

    bool init() override;

protected:
    float get_pilot_desired_climb_rate_cms(void) const override;
    void get_pilot_desired_rp_yrate_cd(float &roll_cd, float &pitch_cd, float &yaw_rate_cds) override;
    void init_z_limits() override;
    void log_pids() override;
};

#endif // QAUTOTUNE_ENABLED
