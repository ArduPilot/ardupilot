/*
  Trllium Serial controlled mount backend class
*/
#pragma once

#include "AP_Mount.h"

#if AP_MOUNT_TRILLIUM_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>
#include <AP_AHRS/AP_AHRS.h>
#include "AP_Mount_Backend.h"

#include "Trillium_protocol/OrionPublicPacketShim.h"



// config
#define AP_MOUNT_TRILLIUM_SERIAL_MINIMUM_INTERVAL_MS       1000
#define AP_MOUNT_TRILLIUM_REQUIRE_ACKS                     (uint8_t)0      // 0 = FALSE, 1 = TRUE

//// Command IDs
#define AP_MOUNT_TRILLIUM_ID_ENABLE_GYRO_STABILISATION     0
#define AP_MOUNT_TRILLIUM_ID_SET_PAN_TILT_POSITION         22







class AP_Mount_Trillium : public AP_Mount_Backend
{
public:
    //constructor
    AP_Mount_Trillium(AP_Mount &frontend, AP_Mount::mount_state &state, uint8_t instance):
        AP_Mount_Backend(frontend, state, instance)
    {}

    // init - performs any required initialisation for this instance
    void init() override;

    // update mount position - should be called periodically
    void update() override;

    // has_pan_control - returns true if this mount can control it's pan (required for multicopters)
    bool has_pan_control() const override { return true; }

    // set_mode - sets mount's mode
    void set_mode(enum MAV_MOUNT_MODE mode) override { _state._mode = mode; }

    // send_mount_status - called to allow mounts to send their status to GCS via MAVLink
    void send_mount_status(mavlink_channel_t chan) override;

    void handle_passthrough(const mavlink_channel_t chan, const mavlink_passthrough_t &packet) override;

private:

    void send_command(const uint8_t cmd, const uint8_t* data, const uint8_t size);
    void send_command(const uint8_t cmd) { send_command(cmd, nullptr, 0); }
    void send_command(const uint8_t cmd, const uint8_t data1) { uint8_t data[1] = {data1}; send_command(cmd, data, 1); }
    void send_command(const uint8_t cmd, const uint8_t data1, const uint8_t data2) { uint8_t data[2] = {data1, data2}; send_command(cmd, data, 2); }
    void send_command(const uint8_t cmd, const uint8_t data1, const uint8_t data2, const uint8_t data3) { uint8_t data[3] = {data1,data2,data3}; send_command(cmd, data, 3); }
    void send_command(const uint8_t cmd, const uint8_t data1, const uint8_t data2, const uint8_t data3, const uint8_t data4) { uint8_t data[4] = {data1,data2,data3,data4}; send_command(cmd, data, 4); }

    void send_target_angles(float pitch_deg, float roll_deg, float yaw_deg);
    void send_target_angles(Vector3f angle, bool target_in_degrees);

    void read_incoming();

    void handle_packet(OrionPkt_t &packet);
    void handle_ack();

    void init_hw();

    AP_HAL::UARTDriver *_port;
    bool _stab_pan : 1;
    bool _stab_tilt : 1;

    uint32_t    _last_send;

    struct {
        uint32_t    timestamp_ms;
        uint32_t    duration_ms;
        uint16_t    step;
        uint8_t     rx_expected_cmd_id;
        uint8_t     rx_expected_ack_id;
        uint8_t     retries;
        bool        done : 1;
    } _booting;


    // keep the last _current_angle values
    Vector3f _current_angle_deg; // in degrees


    // Trillium SDK
    OrionPkt_t _PktIn;
    //OrionPkt_t _PktOut;
    OrionRetractStatus_t _retract_status;

    size_t OrionCommSend(const OrionPkt_t *pPkt);

};
#endif // AP_MOUNT_TRILLIUM_ENABLED

