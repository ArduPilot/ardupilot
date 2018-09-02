/*
  MAVLink enabled mount backend class
 */
#pragma once


#include <AP_HAL/AP_HAL.h>
#include <AP_AHRS/AP_AHRS.h>

#if AP_AHRS_NAVEKF_AVAILABLE
#include <AP_Math/AP_Math.h>
#include <AP_Common/AP_Common.h>
#include <AP_GPS/AP_GPS.h>
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
    virtual void init(const AP_SerialManager& serial_manager);

    // update mount position - should be called periodically
    virtual void update();

    // has_pan_control - returns true if this mount can control it's pan (required for multicopters)
    virtual bool has_pan_control() const;

    // set_mode - sets mount's mode
    virtual void set_mode(enum MAV_MOUNT_MODE mode);

    // status_msg - called to allow mounts to send their status to GCS using the MOUNT_STATUS message
    virtual void status_msg(mavlink_channel_t chan);

    // handle a GIMBAL_REPORT message
    virtual void handle_gimbal_report(mavlink_channel_t chan, mavlink_message_t *msg);
    virtual void handle_gimbal_torque_report(mavlink_channel_t chan, mavlink_message_t *msg);
    virtual void handle_param_value(mavlink_message_t *msg);

    // send a GIMBAL_REPORT message to the GCS
    virtual void send_gimbal_report(mavlink_channel_t chan);

    virtual void update_fast();

private:
    // internal variables
    bool _initialised;              // true once the driver has been initialised

    // Write a gimbal measurament and estimation data packet
    void Log_Write_Gimbal(SoloGimbal &gimbal);

    bool _params_saved;

    SoloGimbal _gimbal;
};

#endif // AP_AHRS_NAVEKF_AVAILABLE
