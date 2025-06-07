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


    // Commands a body-frame roll angle (in centidegrees), an euler pitch angle (in centidegrees), and a yaw rate (in centidegrees/s).
    // See input_euler_rate_yaw_euler_angle_pitch_bf_roll_rad() for full details.
    virtual void input_euler_rate_yaw_euler_angle_pitch_bf_roll_cd(bool plane_controls, float body_roll_cd, float euler_pitch_cd, float euler_yaw_rate_cds) override;
    
    // Commands a body-frame roll angle (in radians), an euler pitch angle (in radians), and a yaw rate (in radians/s).
    // Used by tailsitter quadplanes. Optionally swaps roll and yaw effects as pitch nears 90° if plane_controls is true.
    virtual void input_euler_rate_yaw_euler_angle_pitch_bf_roll_rad(bool plane_controls, float body_roll_rad, float euler_pitch_rad, float euler_yaw_rate_rads) override;
};
