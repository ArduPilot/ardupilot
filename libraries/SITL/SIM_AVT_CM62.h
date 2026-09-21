/*
   Simulator for an AVT CM62 MAVLink (Gimbal Protocol v2) gimbal
*/

#pragma once

#include "SIM_config.h"

#if AP_SIM_AVT_CM62_ENABLED

#include "SIM_MAVLinkGimbalv2.h"
#include "SIM_MAVLinkCamV2.h"
#include <AP_Math/AP_Math.h>

namespace SITL {

class AVT_CM62 : public MAVLinkGimbalv2, public MAVLinkCamV2 {
public:
    void set_instance(uint8_t instance) override {
        MAVLinkGimbalv2::set_instance(instance);
        set_camera_instance(instance);
    }

    void handle_message(const mavlink_message_t &msg) override {
        MAVLinkGimbalv2::handle_message(msg);
        MAVLinkCamV2::handle_message(msg);
    }

    void update(const class Aircraft &aircraft) override {
        MAVLinkGimbalv2::update(aircraft);
        MAVLinkCamV2::update(aircraft);
    }

private:
    const char *get_vendor_name()      const override { return "AVTA"; }
    const char *get_model_name()       const override { return "SIM_AVTA"; }
    // firmware_version encoding: major=1 | (minor=2)<<8 | (patch=3)<<16
    uint32_t    get_firmware_version() const override { return (3U << 16) | (2U << 8) | 1U; }
    uint16_t    get_cap_flags()        const override {
        return GIMBAL_DEVICE_CAP_FLAGS_HAS_PITCH_AXIS |
               GIMBAL_DEVICE_CAP_FLAGS_HAS_YAW_AXIS;
    }
    float get_pitch_min_rad() const override { return radians(-45.0f); }
    float get_pitch_max_rad() const override { return radians( 45.0f); }
    float get_yaw_min_rad()   const override { return radians(-180.0f); }
    float get_yaw_max_rad()   const override { return radians( 180.0f); }

    const char *get_camera_vendor_name()      const override { return "AVTA"; }
    const char *get_camera_model_name()       const override { return "SIM_AVTA_CAM"; }
    uint32_t    get_camera_firmware_version() const override { return (3U << 16) | (2U << 8) | 1U; }
    uint32_t    get_camera_cap_flags()        const override {
        return CAMERA_CAP_FLAGS_CAPTURE_IMAGE |
               CAMERA_CAP_FLAGS_HAS_BASIC_ZOOM |   // speculative: unverified on real hardware
               CAMERA_CAP_FLAGS_HAS_BASIC_FOCUS;   // speculative: unverified on real hardware
    }

    void camera_send_mavlink_message(const mavlink_message_t &msg) override {
        send_mavlink_message(msg);
    }
    uint8_t camera_vehicle_sysid() const override { return vehicle_sysid(); }
    mavlink_status_t &camera_mav_status() override { return gimbal_mav_status(); }
};

}  // namespace SITL

#endif  // AP_SIM_AVT_CM62_ENABLED
