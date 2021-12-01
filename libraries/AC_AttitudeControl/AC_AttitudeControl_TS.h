#pragma once

/// @file    AC_AttitudeControl_TVBS.h
/// @brief   ArduCopter attitude control library

#include "AC_AttitudeControl_Multi.h"

class AC_AttitudeControl_TS : public AC_AttitudeControl_Multi
{
public:
    using AC_AttitudeControl_Multi::AC_AttitudeControl_Multi;

    // empty destructor to suppress compiler warning
    virtual ~AC_AttitudeControl_TS() {}

    // Ensure attitude controllers have zero errors to relax rate controller output
    // Relax only the roll and yaw rate controllers if exclude_pitch is true
    virtual void relax_attitude_controllers(bool exclude_pitch) override;
    virtual void input_euler_rate_yaw_euler_angle_pitch_bf_roll(bool plane_controls, float body_roll_cd, float euler_pitch_cd, float euler_yaw_rate_cds) override;
};
