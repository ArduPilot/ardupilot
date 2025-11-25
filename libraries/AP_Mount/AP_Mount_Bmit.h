/*
  Bmit mount backend class
 */
#pragma once

#include "AP_Mount_config.h"

#if HAL_MOUNT_BMIT_ENABLED

#include "AP_Mount_Backend.h"

#include <AP_Math/AP_Math.h>
#include <AP_Common/AP_Common.h>
#include <GCS_MAVLink/GCS_MAVLink.h>

class AP_Mount_Bmit : public AP_Mount_Backend
{

public:
    // Constructor
    using AP_Mount_Backend::AP_Mount_Backend;

    // update mount position
    void update() override;

    // return true if healthy
    bool healthy() const override;

    // has_pan_control
    bool has_pan_control() const override
    {
        return yaw_range_valid();
    }

    // handle GIMBAL_DEVICE_INFORMATION message
    void handle_gimbal_device_information(const mavlink_message_t &msg) override;

    // send camera information message to GCS
    void send_camera_information(mavlink_channel_t chan) const override;

    //take a picture, returns true on success
    bool take_picture() override;

    // set zoom specified as a rate or percentage
    bool set_zoom(ZoomType zoom_type, float zoom_value) override;

    // set focus specified as rate, percentage or auto
    // focus in = -1, focus hold = 0, focus out = 1
    SetFocusResult set_focus(FocusType focus_type, float focus_value) override;

    // set tracking to none, point or rectangle (see TrackingType enum)
    // if POINT only p1 is used, if RECTANGLE then p1 is top-left, p2 is bottom-right
    // p1,p2 are in range 0 to 1.  0 is left or top, 1 is right or bottom
    bool set_tracking(TrackingType tracking_type, const Vector2f& p1, const Vector2f& p2) override;

    // send camera settings message to GCS
    void send_camera_settings(mavlink_channel_t chan) const override;

    // start or stop video recording
    // set start_recording = true to start record, false to stop recording
    bool record_video(bool start_recording) override;


protected:

    // get attitude as a quaternion.  returns true on success
    bool get_attitude_quaternion(Quaternion& att_quat) override;

private:

    // search for gimbal in GCS_MAVLink routing table
    void find_gimbal();

    // request GIMBAL_DEVICE_INFORMATION from gimbal (holds vendor and model name, max lean angles)
    void request_gimbal_device_information() const;

    // start sending ATTITUDE and AUTOPILOT_STATE_FOR_GIMBAL_DEVICE to gimbal
    // returns true on success, false on failure to start sending
    bool send_attitude_to_gimbal();

    // send GIMBAL_DEVICE_SET_ATTITUDE to gimbal to control rate
    // earth_frame should be true if yaw_rads target is an earth frame rate, false if body_frame
    void send_gimbal_device_set_rate(float roll_rads, float pitch_rads, float yaw_rads, bool earth_frame) const;

    // send GIMBAL_DEVICE_SET_ATTITUDE to gimbal to control attitude
    // earth_frame should be true if yaw_rad target is an earth frame angle, false if body_frame
    void send_gimbal_device_set_attitude(float roll_rad, float pitch_rad, float yaw_rad, bool earth_frame) const;

    // send MAV_CMD_CAMERA_TRACK_POINT
    bool send_point_tracking_to_gimbal(float x, float y);

    // send MAV_CMD_CAMERA_TRACK_RECTANGLE
    bool send_rectangle_tracking_to_gimbal(float x1, float y1, float x2, float y2);

    // send MAV_CMD_CAMERA_STOP_TRACKING
    bool send_stop_tracking_to_gimbal();

    // internal variables
    bool _got_device_info;          // true once gimbal has provided device info
    bool _initialised;              // true once the gimbal has provided a GIMBAL_DEVICE_INFORMATION
    uint32_t _last_devinfo_req_ms;  // system time that GIMBAL_DEVICE_INFORMATION was last requested (used to throttle requests)
    class GCS_MAVLINK *_link;       // link we have found gimbal on; nullptr if not seen yet
    uint8_t _sysid;                 // sysid of gimbal
    uint8_t _compid;                // component id of gimbal
    mavlink_gimbal_device_attitude_status_t _gimbal_device_attitude_status;  // copy of most recently received gimbal status
    float _zoom_type;              // type of zoom continuous or step
    float _focus_type;             // type of focus auto or continuous
    bool _recording_on;            // true if video recording is on
    float _image_index = 0;
};
#endif // HAL_MOUNT_BMIT_MSIG_ENABLED
