#include "AP_Mount.h"
#include <AP_HAL/AP_HAL.h>
#include "AP_Mount_Backend.h"

class AP_Mount_Visca : public AP_Mount_Backend 
{
public:
    AP_Mount_Visca(AP_Mount &frontend, AP_Mount::mount_state &state, uint8_t instance):
        AP_Mount_Backend(frontend, state, instance)
    {}

    // init - performs any required initialisation for this instance
    virtual void init(const AP_SerialManager& serial_manager);

    // update camera control comands - should be called periodically
    virtual void update();

    // has_pan_control - returns true if this mount can control it's pan. Intended just for gimballed mounts
    virtual bool has_pan_control() const;

    // This should never be called from the front end AP_Mount. Just in case
    virtual void set_mode(enum MAV_MOUNT_MODE mode);

    // To be implemented
    virtual void status_msg(mavlink_channel_t chan);

    // To be implemented
    virtual void send_mount_status(mavlink_channel_t chan) override;


private:

    // send any command via serial
    void send_command(uint8_t* data, uint8_t size);

    // In this mode a 3 position switch or a third axis on the transmitter stick is used for stay still, +zoom, -zoom
    void direct_control();

    enum ControlMode {  
        mode_direct,
        mode_progresive,
        mode_mavlink
    };

    AP_HAL::UARTDriver *_port;

    bool _initialised : 1;

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

};


