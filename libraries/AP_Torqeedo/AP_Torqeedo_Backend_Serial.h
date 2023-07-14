#pragma once

#include "AP_Torqeedo_Backend.h"

#if HAL_TORQEEDO_ENABLED

class AP_Torqeedo_Backend_Serial : public AP_Torqeedo_Backend
{
public:
    AP_Torqeedo_Backend_Serial(AP_Torqeedo &_frontend,
                               AP_Torqeedo::Torqeedo_State &_state,
                               AP_Torqeedo_Params& _params,
                               uint8_t serial_instance);

    // static detection function
    // detect if a propellers is connected by looking for a configured serial port
    // serial_instance affects which serial port is used.
    static bool detect(uint8_t serial_instance);

protected:
    virtual uint16_t rxspace() const { return 0; };

    AP_HAL::UARTDriver *_uart;              // uart for communicating with sensor

    uint8_t _serial_instance;
};

#endif // HAL_PROXIMITY_ENABLED
