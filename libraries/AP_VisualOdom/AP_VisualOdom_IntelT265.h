#pragma once

#include "AP_VisualOdom_Backend.h"

class AP_VisualOdom_IntelT265 : public AP_VisualOdom_Backend
{

public:

    using AP_VisualOdom_Backend::AP_VisualOdom_Backend;

    // ignore vision-position-delta messages from T265
    void handle_vision_position_delta_msg(const mavlink_message_t &msg) override {};

    // consume vision position estimate data and send to EKF. distances in meters
    void handle_vision_position_estimate(uint64_t remote_time_us, uint32_t time_ms, float x, float y, float z, const Quaternion &attitude) override;

    // handle request to align camera's attitude with vehicle's AHRS/EKF attitude
    void align_sensor_to_vehicle() override { _align_camera = true; }

    // arming check
    bool pre_arm_check(char *failure_msg, uint8_t failure_msg_len) const override;

protected:

    // apply rotation and correction to position
    void rotate_and_correct_position(Vector3f &position) const;

    // rotate attitude using _yaw_trim
    void rotate_attitude(Quaternion &attitude) const;

    // use sensor provided position and attitude to calculate rotation to align sensor with AHRS/EKF attitude
    bool align_sensor_to_vehicle(const Vector3f &position, const Quaternion &attitude);

    float _yaw_trim;                            // yaw angle trim (in radians) to align camera's yaw to ahrs/EKF's
    Quaternion _yaw_rotation;                   // earth-frame yaw rotation to align heading of sensor with vehicle.  use when _yaw_trim is non-zero
    Quaternion _att_rotation;                   // body-frame rotation corresponding to ORIENT parameter.  use when get_orientation != NONE
    Matrix3f _pos_rotation;                     // rotation to align position from sensor to earth frame.  use when _use_pos_rotation is true
    Vector3f _pos_correction;                   // position correction that should be added to position reported from sensor
    bool _use_att_rotation;                     // true if _att_rotation should be applied to sensor's attitude data
    bool _use_pos_rotation;                     // true if _pos_rotation should be applied to sensor's position data
    bool _align_camera = true;                  // true if camera should be aligned to AHRS/EKF
    bool _error_orientation;                    // true if the orientation is not supported
    Quaternion _attitude_last;                  // last attitude received from camera (used for arming checks)
};
