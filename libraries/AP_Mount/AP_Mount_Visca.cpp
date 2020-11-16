#include "AP_Mount_Visca.h"
#include <AP_SerialManager/AP_SerialManager.h>

extern const AP_HAL::HAL& hal;

void AP_Mount_Visca::init()
{
    const AP_SerialManager& serial_manager = AP::serialmanager();

    // check for Visca protocol 
    if ((_port = serial_manager.find_serial(AP_SerialManager::SerialProtocol_Visca, 0))) {
        _initialised = true;

        // initialize camera address
        cmd_set_address buffer;
        send_command((uint8_t *)&buffer, sizeof(cmd_set_address));
    }
}

void AP_Mount_Visca::update()
{   
    if (_initialised != true) {
        return;
    }   

    direct_control();
}

// In this mode a 3 position switch or a third axis on the transmitter stick is used for stay still, +zoom, -zoom
void AP_Mount_Visca::direct_control()
{
    uint16_t rc_in = RC_Channels::rc_channel(_state._zoom_rc_in-1)->get_radio_in();

    if (rc_in > 1800) {
        cmd_set_zoomup cmd;
        send_command((uint8_t *)&cmd, sizeof(cmd_set_zoomup)); 

    } else if (rc_in < 1300){
        cmd_set_zoomdown cmd;
        send_command((uint8_t *)&cmd, sizeof(cmd_set_zoomdown)); 

    } else {
        cmd_set_zoomstop cmd;
        send_command((uint8_t *)&cmd, sizeof(cmd_set_zoomstop)); 
    }
}

void AP_Mount_Visca::send_command(uint8_t* data, uint8_t size)
{
    if (_port->txspace() < (size)) {
        return;
    }

    for (uint8_t i = 0; i != size; i++) {
        _port->write(data[i]);
    }
}

// has_pan_control - returns true if this mount can control it's pan. Intended just for gimballed mounts
bool AP_Mount_Visca::has_pan_control() const 
{
    return false;
}

// This should never be called from the front end AP_Mount. Just in case
void AP_Mount_Visca::set_mode(MAV_MOUNT_MODE mode)
{
    return;
}

// To be implemented, send camera status such as zoom level, aperture, day/night and some more
void AP_Mount_Visca::send_mount_status(mavlink_channel_t chan)
{
    return;
}
