/*
  Servo controlled mount backend class
 */
#pragma once

#include "AP_Mount_Backend.h"

#ifndef HAL_MOUNT_SERVO_ENABLED
#define HAL_MOUNT_SERVO_ENABLED HAL_MOUNT_ENABLED
#endif

#if HAL_MOUNT_SERVO_ENABLED

#include <AP_Math/AP_Math.h>
#include <AP_Common/AP_Common.h>
#include <SRV_Channel/SRV_Channel.h>

class AP_Mount_Servo : public AP_Mount_Backend
{
public:
    // Constructor
    AP_Mount_Servo(AP_Mount &frontend, AP_Mount_Params &params, bool requires_stab, uint8_t instance):
        AP_Mount_Backend(frontend, params, instance),
        requires_stabilization(requires_stab),
        _roll_idx(SRV_Channel::k_none),
        _tilt_idx(SRV_Channel::k_none),
        _pan_idx(SRV_Channel::k_none),
        _open_idx(SRV_Channel::k_none)
    {
    }

    // init - performs any required initialisation for this instance
    void init() override;

    // update mount position - should be called periodically
    void update() override;

    // returns true if this mount can control its pan (required for multicopters)
    bool has_pan_control() const override;

protected:

    // get attitude as a quaternion.  returns true on success
    bool get_attitude_quaternion(Quaternion& att_quat) override;

private:

    // update body-frame angle outputs from earth-frame targets
    void update_angle_outputs(const MountTarget& angle_rad);

    // returns closest angle to 'angle' taking into account limits.  all angles are in body-frame and degrees * 10
    int16_t closest_limit(int16_t angle, int16_t angle_min, int16_t angle_max);

    ///  moves servo with the given function id to the specified angle.  all angles are in body-frame and degrees * 10
    void move_servo(uint8_t rc, int16_t angle, int16_t angle_min, int16_t angle_max);

    /// Servo gimbals require stabilization, BrushlessPWM gimbals self-stabilize
    const bool requires_stabilization;

    // SRV_Channel - different id numbers are used depending upon the instance number
    SRV_Channel::Aux_servo_function_t    _roll_idx;  // SRV_Channel mount roll function index
    SRV_Channel::Aux_servo_function_t    _tilt_idx;  // SRV_Channel mount tilt function index
    SRV_Channel::Aux_servo_function_t    _pan_idx;   // SRV_Channel mount pan  function index
    SRV_Channel::Aux_servo_function_t    _open_idx;  // SRV_Channel mount open function index

    MountTarget _angle_rad;         // angle target
    Vector3f _angle_bf_output_deg;  // final body frame output angle in degrees
};
#endif // HAL_MOUNT_SERVO_ENABLED
