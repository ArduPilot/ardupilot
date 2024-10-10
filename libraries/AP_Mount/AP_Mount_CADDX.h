/*
  CADDX mount using serial protocol backend class
 */
#pragma once

#include "AP_Mount_Backend_Serial.h"

#if HAL_MOUNT_CADDX_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <AP_Common/AP_Common.h>

#define AP_MOUNT_CADDX_RESEND_MS   1000    // resend angle targets to gimbal once per second

class AP_Mount_CADDX : public AP_Mount_Backend_Serial
{

public:
    // Constructor
    using AP_Mount_Backend_Serial::AP_Mount_Backend_Serial;

    // update mount position - should be called periodically
    void update() override;

    // has_pan_control - returns true if this mount can control its pan (required for multicopters)
    bool has_pan_control() const override { return yaw_range_valid(); };
    
    // has_roll_control - returns true if this mount can control its roll
    bool has_roll_control() const override { return roll_range_valid(); };

    // has_pitch_control - returns true if this mount can control its tilt
    bool has_pitch_control() const override { return pitch_range_valid(); };


protected:

    // get attitude as a quaternion.  returns true on success
    bool get_attitude_quaternion(Quaternion& att_quat) override;

private:

    // send_target_angles
    void send_target_angles(const MountTarget& angle_target_rad);

    //void add_next_reply(ReplyType reply_type);
    //uint8_t get_reply_size(ReplyType reply_type);
    //bool can_send(bool with_control);

 
    struct PACKED cmd_set_angles_struct {
        uint8_t byte1;
        uint8_t byte2;
        uint8_t byte3;
        float pitch;
        float roll;
        float yaw;
        uint8_t flags;
        uint8_t type;
        uint16_t crc;
    };


    // internal variables
    uint32_t _last_send;            // system time of last do_mount_control sent to gimbal

    // keep the last _current_angle values
    Vector3l _current_angle;
};
#endif // HAL_MOUNT_CADDX_ENABLED
