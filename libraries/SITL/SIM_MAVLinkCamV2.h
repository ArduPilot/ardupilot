/*
   Simulator mixin for MAVLink Camera Protocol v2 peripherals.

   Mix this into a combined device class (e.g. AVT_CM62) alongside a
   MAVLinkGimbalv2 subclass.  The combined class wires the transport
   pure-virtuals to the gimbal's serial link and supplies device-specific
   identity via the identity pure-virtuals.

   Protocol reference: AP_Camera/AP_Camera_MAVLinkCamV2.cpp
*/

#pragma once

#include "SIM_config.h"

#if AP_SIM_MAVLINKCAMV2_ENABLED

#include "SIM_Camera.h"
#include <GCS_MAVLink/GCS_MAVLink.h>
#include <AP_Math/AP_Math.h>

namespace SITL {

class MAVLinkCamV2 {
public:
    void handle_message(const mavlink_message_t &msg);
    void update(const class Aircraft &aircraft);
    void set_camera_instance(uint8_t instance);

protected:
    Camera _camera;

    // identity — every combined device class must implement all of these
    virtual const char *get_camera_vendor_name()      const = 0;
    virtual const char *get_camera_model_name()       const = 0;
    virtual uint32_t    get_camera_firmware_version() const = 0;
    virtual uint32_t    get_camera_cap_flags()        const = 0;

    // transport — combined device class wires these to the shared serial link
    virtual void camera_send_mavlink_message(const mavlink_message_t &msg) = 0;
    virtual uint8_t camera_vehicle_sysid() const = 0;
    virtual mavlink_status_t &camera_mav_status() = 0;

private:
    void send_camera_heartbeat();
    void send_camera_information(uint8_t target_sysid, uint8_t target_compid);
    void send_camera_command_ack(uint8_t target_sysid, uint8_t target_compid,
                                  MAV_CMD cmd, MAV_RESULT result);

    uint8_t  _camera_compid {MAV_COMP_ID_CAMERA};
    uint32_t _last_camera_heartbeat_ms {};
};

}  // namespace SITL

#endif  // AP_SIM_MAVLINKCAMV2_ENABLED
