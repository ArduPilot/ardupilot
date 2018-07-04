#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_Math/AP_Math.h>
#include <DataFlash/DataFlash.h>

class AP_Visca_Camera {
public:

    // constructor
    AP_Visca_Camera():
        _port(nullptr),
        _initialised(false),
        _last_command_confirmed(false)
    {}

    // init - performs any required initialisation for this instance
    void init(const AP_SerialManager& serial_manager);

    // update camera zoom action - should be called periodically
    void update();

    // handle a PARAM_VALUE message
    void handle_param_value(mavlink_message_t *msg) {}

    static const struct AP_Param::GroupInfo var_info[];

private:

    enum ControlMode {
        mode_direct,
        mode_progresive,
        mode_mavlink
    };

    void send_command(uint8_t* data, uint8_t size);

    void direct_control();

    AP_Visca_Camera::ControlMode get_control_mode() { return (ControlMode)_control_mode.get();}


    AP_HAL::UARTDriver *_port;
    bool _initialised : 1;

    //maybe not necesary
    bool _last_command_confirmed : 1;

    struct PACKED cmd_set_address {
        uint8_t byte_1=0x88;
        uint8_t byte_2=0x30;
        uint8_t byte_3=0x01;
        uint8_t byte_4=0xFF;
    };

    struct PACKED cmd_set_zoomup {       
        uint8_t byte_1=0x81;
        uint8_t byte_2=0x01;
        uint8_t byte_3=0x04;
        uint8_t byte_4=0x07;
        uint8_t byte_5=0x33;
        uint8_t byte_6=0xFF;

    };

    struct PACKED cmd_set_zoomdown {
        uint8_t byte_1=0x81;
        uint8_t byte_2=0x01;
        uint8_t byte_3=0x04;
        uint8_t byte_4=0x07;
        uint8_t byte_5=0x23;
        uint8_t byte_6=0xFF;
    };

    struct PACKED cmd_set_zoomstop {
        uint8_t byte_1=0x81;
        uint8_t byte_2=0x01;
        uint8_t byte_3=0x04;
        uint8_t byte_4=0x07;
        uint8_t byte_5=0x00;
        uint8_t byte_6=0xFF;
    };

    // Params
    AP_Int8 _control_mode;
    AP_Int8 _zoom_rc_in;

};
