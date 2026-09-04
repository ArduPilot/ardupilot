/*
   Simulator for an MT11 MAVLink camera and gimbal
*/

#pragma once

#include "SIM_config.h"

#if AP_SIM_MT11_ENABLED

#include "SIM_MAVLinkCamV2.h"
#include "SIM_MAVLinkGimbalv2.h"
#include <AP_Math/AP_Math.h>

namespace SITL {

class MT11_RTSPServer;

class MT11 : public MAVLinkGimbalv2, public MAVLinkCamV2 {
public:
    void set_instance(uint8_t instance) override {
        MAVLinkGimbalv2::set_instance(instance);
        set_camera_instance(instance);
    }

    void handle_message(const mavlink_message_t &msg) override {
        MAVLinkGimbalv2::handle_message(msg);
        MAVLinkCamV2::handle_message(msg);
    }

    void update(const class Aircraft &aircraft) override;

private:
    const char *get_vendor_name() const override { return "ArduPilot"; }
    const char *get_model_name() const override { return "MT11"; }
    uint32_t get_firmware_version() const override { return 1; }
    uint32_t get_cap_flags() const override {
        return GIMBAL_DEVICE_CAP_FLAGS_HAS_NEUTRAL |
               GIMBAL_DEVICE_CAP_FLAGS_HAS_PITCH_AXIS |
               GIMBAL_DEVICE_CAP_FLAGS_HAS_PITCH_FOLLOW |
               GIMBAL_DEVICE_CAP_FLAGS_HAS_YAW_AXIS |
               GIMBAL_DEVICE_CAP_FLAGS_HAS_YAW_FOLLOW |
               GIMBAL_DEVICE_CAP_FLAGS_SUPPORTS_YAW_IN_EARTH_FRAME |
               GIMBAL_DEVICE_CAP_FLAGS_CAN_POINT_LOCATION_GLOBAL;
    }
    float get_pitch_min_rad() const override { return radians(-90.0f); }
    float get_pitch_max_rad() const override { return radians(30.0f); }
    float get_yaw_min_rad() const override { return radians(-180.0f); }
    float get_yaw_max_rad() const override { return radians(180.0f); }

    const char *get_camera_vendor_name() const override { return "ArduPilot"; }
    const char *get_camera_model_name() const override { return "MT11"; }
    uint32_t get_camera_firmware_version() const override { return 1; }
    uint32_t get_camera_cap_flags() const override {
        return CAMERA_CAP_FLAGS_CAPTURE_VIDEO |
               CAMERA_CAP_FLAGS_CAPTURE_IMAGE |
               CAMERA_CAP_FLAGS_HAS_MODES |
               CAMERA_CAP_FLAGS_CAN_CAPTURE_IMAGE_IN_VIDEO_MODE |
               CAMERA_CAP_FLAGS_CAN_CAPTURE_VIDEO_IN_IMAGE_MODE |
               CAMERA_CAP_FLAGS_HAS_BASIC_ZOOM |
               CAMERA_CAP_FLAGS_HAS_BASIC_FOCUS |
               CAMERA_CAP_FLAGS_HAS_VIDEO_STREAM;
    }
    uint16_t get_camera_resolution_h() const override { return 1920; }
    uint16_t get_camera_resolution_v() const override { return 1080; }
    uint8_t get_camera_gimbal_device_id() const override { return gimbal_compid(); }

    uint8_t get_video_stream_count() const override { return 2; }
    bool get_video_stream_information(
        uint8_t stream_id,
        mavlink_video_stream_information_t &info) const override;

    MT11_RTSPServer *_rtsp_server;

    void camera_send_mavlink_message(const mavlink_message_t &msg) override {
        send_mavlink_message(msg);
    }
    uint8_t camera_vehicle_sysid() const override { return vehicle_sysid(); }
    mavlink_status_t &camera_mav_status() override { return gimbal_mav_status(); }
};

}  // namespace SITL

#endif  // AP_SIM_MT11_ENABLED
