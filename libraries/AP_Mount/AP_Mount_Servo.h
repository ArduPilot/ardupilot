/*
  Servo controlled mount backend class
 */
#pragma once

#include "AP_Mount_config.h"

#if HAL_MOUNT_SERVO_ENABLED

#include "AP_Mount_Backend.h"

#include <AP_Math/AP_Math.h>
#include <AP_Common/AP_Common.h>

class AP_Mount_Servo : public AP_Mount_Backend
{
public:
    // Constructor
    AP_Mount_Servo(AP_Mount &frontend, AP_Mount_Params &params, bool requires_stab, uint8_t instance):
        AP_Mount_Backend(frontend, params, instance),
        requires_stabilization(requires_stab),
        _roll_idx(SRV_Channel::k_none),
        _tilt_idx(SRV_Channel::k_none),
        _pan_idx(SRV_Channel::k_none)
    {
    }

    // init - performs any required initialisation for this instance
    void init() override;

    // update mount position - should be called periodically
    void update() override;

    // returns true if this mount can control its roll
    bool has_roll_control() const override;

    // returns true if this mount can control its tilt
    bool has_pitch_control() const override;

    // returns true if this mount can control its pan (required for multicopters)
    bool has_pan_control() const override;

protected:

    // get attitude as a quaternion.  returns true on success
    bool get_attitude_quaternion(Quaternion& att_quat) override;

    // servo only natively supports angles:
    uint8_t natively_supported_mount_target_types() const override {
        return NATIVE_ANGLES_ONLY;
    };

private:

    // called by the backend to set the servo angles:
    void send_target_angles(const MountAngleTarget& angle_rad) override;

    // update body-frame angle outputs from earth-frame targets
    void update_angle_outputs(const MountAngleTarget& angle_rad);

    ///  moves servo with the given function id to the specified angle.  all angles are in body-frame and degrees * 10
    void move_servo(uint8_t rc, int16_t angle, int16_t angle_min, int16_t angle_max);

    /// Servo gimbals require stabilization, BrushlessPWM gimbals self-stabilize
    const bool requires_stabilization;

    // SRV_Channel - different id numbers are used depending upon the instance number
    SRV_Channel::Function    _roll_idx;  // SRV_Channel mount roll function index
    SRV_Channel::Function    _tilt_idx;  // SRV_Channel mount tilt function index
    SRV_Channel::Function    _pan_idx;   // SRV_Channel mount pan  function index

    Vector3f _angle_bf_output_rad;  // final body frame output angle in radians
};
#endif // HAL_MOUNT_SERVO_ENABLED
