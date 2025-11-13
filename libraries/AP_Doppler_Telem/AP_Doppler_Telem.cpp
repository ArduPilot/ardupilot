

#include "AP_Doppler_config.h"



#include "AP_Doppler_Telem.h"
#include "AP_Doppler_Backend.h"
#include <AP_SerialManager/AP_SerialManager.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_Vehicle/AP_Vehicle.h>
#include <GCS_MAVLink/GCS_MAVLink.h>



extern const AP_HAL::HAL& hal;

AP_Doppler_Telem *AP_Doppler_Telem::singleton;

AP_Doppler_Telem::AP_Doppler_Telem()
{
    singleton = this;
    _doppler_parameters = &AP::vehicle()->doppler_parameters;
}


AP_Doppler_Telem::~AP_Doppler_Telem(void)
{
    singleton = nullptr;
}

/*
 * init - perform required initialisation
 */
bool AP_Doppler_Telem::init(const AP_SerialManager &serial_manager)
{
    gcs().send_text(MAV_SEVERITY_CRITICAL, "Doppler INIT");
    
    
    // check for protocol configured for a serial port - only the first serial port with one of these protocols will then run (cannot have Doppler on multiple serial ports)
    
    if ((port = serial_manager.find_serial(AP_SerialManager::SerialProtocol_Doppler, 0))) 
    {
        _backend = new AP_Doppler_Backend(port);
        gcs().send_text(MAV_SEVERITY_CRITICAL, "Doppler FIND SERIAL");
    }

    if (_backend == nullptr) {
        gcs().send_text(MAV_SEVERITY_CRITICAL, "Doppler  NO SERIAL");
        return false;
        
    }

    if (!_backend->init()) {
        delete _backend;
        _backend = nullptr;
        return false;
    }

    return true;
}



void AP_Doppler_Telem::update()
{
    
    if (_backend != nullptr) {
        _backend->loop();
    }
    else {
        return;
    }
}

void AP_Doppler_Telem::send()
{
    if (_backend != nullptr) {
        _backend->send();
    }
    else {
        return;
    }
}




namespace AP
{
AP_Doppler_Telem *Doppler_telem()
{
    return AP_Doppler_Telem::get_singleton();
}
};


