/*
  support for autotune of quadplanes
 */

#pragma once

#include <AP_HAL/AP_HAL_Boards.h>

#include "quadplane.h"
#ifndef QAUTOTUNE_ENABLED
#define QAUTOTUNE_ENABLED (HAL_QUADPLANE_ENABLED && CONFIG_HAL_BOARD == HAL_BOARD_SITL)
#endif

#if QAUTOTUNE_ENABLED

#include <AC_AutoTune/AC_AutoTune_Multi.h>

class QAutoTune : public AC_AutoTune_Multi
{
public:
    friend class QuadPlane;

    bool init() override;

protected:
    float get_desired_climb_rate_ms(void) const override;
    void get_pilot_desired_rp_yrate_rad(float &des_roll_rad, float &des_pitch_rad, float &des_yaw_rate_rads) override;
    void init_z_limits() override;
#if HAL_LOGGING_ENABLED
    void log_pids() override;
#endif
};

#endif // QAUTOTUNE_ENABLED
