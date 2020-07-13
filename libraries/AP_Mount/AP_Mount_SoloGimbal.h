/*
  MAVLink enabled mount backend class
 */
#pragma once


#include <AP_HAL/AP_HAL.h>
#include <AP_AHRS/AP_AHRS.h>

#if AP_AHRS_NAVEKF_AVAILABLE
#include <AP_Math/AP_Math.h>
#include <AP_Common/AP_Common.h>
#include <GCS_MAVLink/GCS_MAVLink.h>
#include <RC_Channel/RC_Channel.h>
#include "AP_Mount_Backend.h"
#include "SoloGimbal.h"


class AP_Mount_SoloGimbal : public AP_Mount_Backend
{

public:
    // Constructor
    AP_Mount_SoloGimbal(AP_Mount &frontend, AP_Mount::mount_state &state, uint8_t instance);

    // init - performs any required initialisation for this instance
    void init() override;

    // update mount position - should be called periodically
    void update() override;

    // has_pan_control - returns true if this mount can control it's pan (required for multicopters)
    bool has_pan_control() const override;

    // set_mode - sets mount's mode
    void set_mode(enum MAV_MOUNT_MODE mode) override;

    // send_mount_status - called to allow mounts to send their status to GCS using the MOUNT_STATUS message
    void send_mount_status(mavlink_channel_t chan) override;

    // handle a GIMBAL_REPORT message
    void handle_gimbal_report(mavlink_channel_t chan, const mavlink_message_t &msg) override;
    void handle_gimbal_torque_report(mavlink_channel_t chan, const mavlink_message_t &msg);
    void handle_param_value(const mavlink_message_t &msg) override;

    // send a GIMBAL_REPORT message to the GCS
    void send_gimbal_report(mavlink_channel_t chan) override;

    void update_fast() override;

private:
    // internal variables
    bool _initialised;              // true once the driver has been initialised

    // Write a gimbal measurament and estimation data packet
    void Log_Write_Gimbal(SoloGimbal &gimbal);

    bool _params_saved;

    SoloGimbal _gimbal;
};

#endif // AP_AHRS_NAVEKF_AVAILABLE
