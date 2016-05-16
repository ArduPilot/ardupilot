// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/*
  SToRM32 mount using serial protocol backend class
 */
#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_AHRS/AP_AHRS.h>

#include <AP_Math/AP_Math.h>
#include <AP_Common/AP_Common.h>
#include <AP_GPS/AP_GPS.h>
#include <GCS_MAVLink/GCS_MAVLink.h>
#include <RC_Channel/RC_Channel.h>
#include "AP_Mount_Backend.h"

#define AP_MOUNT_STORM32_SERIAL_RESEND_MS   1000    // resend angle targets to gimbal once per second

class AP_Mount_SToRM32_serial : public AP_Mount_Backend
{

public:
    // Constructor
    AP_Mount_SToRM32_serial(AP_Mount &frontend, AP_Mount::mount_state &state, uint8_t instance);

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

private:

    // send_target_angles
    void send_target_angles(float pitch_deg, float roll_deg, float yaw_deg);

    // send read data request
    void get_angles();

    // read_incoming
    void read_incoming();
    void parse_reply();

    enum ReplyType {
        ReplyType_UNKNOWN = 0,
        ReplyType_DATA,
        ReplyType_ACK
    };

    //void add_next_reply(ReplyType reply_type);
    uint8_t get_reply_size(ReplyType reply_type);
    bool can_send(bool with_control);

    struct PACKED SToRM32_reply_data_struct {
        uint16_t state;
        uint16_t status;
        uint16_t status2;

        uint16_t i2c_errors;
        uint16_t lipo_voltage;
        uint16_t systicks;
        uint16_t cycle_time;

        int16_t imu1_gx;
        int16_t imu1_gy;
        int16_t imu1_gz;

        int16_t imu1_ax;
        int16_t imu1_ay;
        int16_t imu1_az;

        int16_t ahrs_x;
        int16_t ahrs_y;
        int16_t ahrs_z;

        int16_t imu1_pitch;
        int16_t imu1_roll;
        int16_t imu1_yaw;

        int16_t cpid_pitch;
        int16_t cpid_roll;
        int16_t cpid_yaw;

        uint16_t input_pitch;
        uint16_t input_roll;
        uint16_t input_yaw;

        int16_t imu2_pitch;
        int16_t imu2_roll;
        int16_t imu2_yaw;

        int16_t mag2_yaw;
        int16_t mag2_pitch;

        int16_t ahrs_imu_confidence;

        uint16_t function_input_values;

        uint16_t crc;
        uint8_t magic;
    };

    struct PACKED SToRM32_reply_ack_struct {
        uint8_t byte1;
        uint8_t byte2;
        uint8_t byte3;
        uint8_t data;
        uint16_t crc;
    };

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
    AP_HAL::UARTDriver *_port;

    bool _initialised;              // true once the driver has been initialised
    uint32_t _last_send;            // system time of last do_mount_control sent to gimbal

    uint8_t _reply_length;
    uint8_t _reply_counter;
    ReplyType _reply_type;


    union PACKED SToRM32_reply {
        SToRM32_reply_data_struct data;
        SToRM32_reply_ack_struct ack;
        uint8_t bytes[1];
    } _buffer;

    // keep the last _current_angle values
    Vector3l _current_angle;
};
