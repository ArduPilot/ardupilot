/*
   Simulator for MAVLink Gimbal Protocol v2 peripherals

./Tools/autotest/sim_vehicle.py --gdb --debug -v ArduCopter \
    -A --serial5=sim:avt_cm62_gimbal --speedup=1

param set MNT1_TYPE 6          # MAVLink
param set CAM1_TYPE 4          # mount
param set SERIAL5_PROTOCOL 2   # MAVLink2
reboot

long REQUEST_MESSAGE 259        # CAMERA_INFORMATION
status *CAM*
*/

#pragma once

#include "SIM_config.h"

#if AP_SIM_MAVLINKGIMBALV2_ENABLED

#include "SIM_Mount.h"
#include "SIM_Gimbal.h"
#include <GCS_MAVLink/GCS_MAVLink.h>
#include <AP_Math/AP_Math.h>
#include <AP_Common/Location.h>

namespace SITL {

/*
  Base class for Gimbal Protocol v2 simulators.  Handles all MAVLink
  framing; subclasses supply device-specific identity data via
  pure-virtual accessors.
*/
class MAVLinkGimbalv2 : public Mount {
public:
    void update(const class Aircraft &aircraft) override;
    void set_instance(uint8_t instance) override;

protected:
    // device identity — every subclass must implement all of these
    virtual const char *get_vendor_name()      const = 0;
    virtual const char *get_model_name()       const = 0;
    virtual uint32_t    get_firmware_version() const = 0;
    virtual uint16_t    get_cap_flags()        const = 0;
    virtual float get_pitch_min_rad() const = 0;
    virtual float get_pitch_max_rad() const = 0;
    virtual float get_yaw_min_rad()   const = 0;
    virtual float get_yaw_max_rad()   const = 0;

protected:
    // physics model shared by all MAVLink gimbal simulators
    Gimbal _gimbal;

private:
    void update_input();
    void update_gimbal(const class Aircraft &aircraft);
    void handle_message(const mavlink_message_t &msg);

    void send_heartbeat();
    void send_gimbal_device_information();
    void send_attitude_status();
    void send_command_ack(uint8_t target_sysid, uint8_t target_compid,
                          MAV_CMD command, MAV_RESULT result);
    void send_mavlink_message(const mavlink_message_t &msg);

    uint8_t _compid {MAV_COMP_ID_GIMBAL};

    bool     _seen_autopilot_heartbeat;
    uint8_t  _vehicle_system_id;
    uint8_t  _vehicle_component_id;

    uint32_t _last_heartbeat_ms;
    uint32_t _last_attitude_status_ms;

    // current demand from the autopilot, set by GIMBAL_DEVICE_SET_ATTITUDE
    struct {
        bool valid;               // true once a command has been received
        bool is_rate;             // true = rate command, false = angle command
        bool yaw_is_ef;           // true = rates/attitude expressed in earth frame
        Vector3f rates_rads;      // demanded body rates (is_rate == true)
        Quaternion attitude;      // desired attitude (is_rate == false)
    } _target;

    struct {
        mavlink_message_t rxmsg;
        mavlink_status_t  status;
    } mav;

    Matrix3f _vehicle_dcm;

    // ROI location tracking: set by COMMAND_INT DO_SET_ROI_LOCATION
    struct {
        bool valid;
        Location loc;
    } _roi;
};

}  // namespace SITL

#endif  // AP_SIM_MAVLINKGIMBALV2_ENABLED
