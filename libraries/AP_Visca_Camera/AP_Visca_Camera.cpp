#include "AP_Visca_Camera.h"

extern const AP_HAL::HAL& hal;

//TODO's:
//-Check parameters each few seconds in case the user changes them, for allowing to change mode on fly
//-Add progresive mode, using a direct relation between a channel and the 10 position of zoom available
//-Add mavlink message for zoom at least

const AP_Param::GroupInfo AP_Visca_Camera::var_info[] = {

    // @Param: _CTRL_MODE
    // @DisplayName: Visca camera operating zoom mode
    // @Description: Zoom mode for camera with visca protocol
    // @Values: 0:direct, 1: progresive(not suported yet), 2: mavlink(not suported yet)
    // @User: Standard
    AP_GROUPINFO("_CTRL_MODE", 0, AP_Visca_Camera, _control_mode, 0),

    // @Param: _CTRL_CH
    // @DisplayName: Visca camera zoom channel input
    // @Description: Channel used for zoom control of visca protocol camera
    // @Values: 6,7,8,..
    // @User: Standard
    AP_GROUPINFO("_CTRL_CH", 1, AP_Visca_Camera, _zoom_rc_in, 0),

    AP_GROUPEND
};

void AP_Visca_Camera::init(const AP_SerialManager& serial_manager)
{
    // check for visca protocol
    if ((_port = serial_manager.find_serial(AP_SerialManager::SerialProtocol_Visca, 0))) {
        _initialised = true;

        //initialize camera address
        cmd_set_address buffer;
        send_command((uint8_t *)&buffer, sizeof(cmd_set_address));

        gcs().send_text(MAV_SEVERITY_INFO, "fcb camera initialised");
    }
}

void AP_Visca_Camera::update()
{   
    if (_initialised != true) {
        return;
    }   

    direct_control();
}

//In this mode a 3 position switch is used for stay still, +zoom, -zoom
void AP_Visca_Camera::direct_control()
{
    uint16_t rc_in = RC_Channels::rc_channel(_zoom_rc_in-1)->get_radio_in();

    if (rc_in < 1300) {
        cmd_set_zoomup cmd;
        send_command((uint8_t *)&cmd, sizeof(cmd_set_zoomup));  
    } else if (rc_in > 1800){
        cmd_set_zoomdown cmd;
        send_command((uint8_t *)&cmd, sizeof(cmd_set_zoomup)); 
    } else {
        cmd_set_zoomstop cmd;
        send_command((uint8_t *)&cmd, sizeof(cmd_set_zoomup)); 
    };
}

void AP_Visca_Camera::send_command(uint8_t* data, uint8_t size)
{
    if (_port->txspace() < (size)) {
        return;
    }

    for (uint8_t i = 0; i != size; i++) {
        _port->write(data[i]);
    }
}

