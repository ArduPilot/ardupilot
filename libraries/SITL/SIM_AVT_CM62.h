/*
   Simulator for an AVT CM62 MAVLink (Gimbal Protocol v2) gimbal
*/

#pragma once

#include "SIM_config.h"

#if AP_SIM_AVT_CM62_ENABLED

#include "SIM_MAVLinkGimbalv2.h"
#include <AP_Math/AP_Math.h>

namespace SITL {

class AVT_CM62 : public MAVLinkGimbalv2 {
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
};

}  // namespace SITL

#endif  // AP_SIM_AVT_CM62_ENABLED
