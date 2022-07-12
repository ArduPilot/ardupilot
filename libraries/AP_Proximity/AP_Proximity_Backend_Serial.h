#pragma once

#include "AP_Proximity_Backend.h"

#if HAL_PROXIMITY_ENABLED

class AP_Proximity_Backend_Serial : public AP_Proximity_Backend
{
public:
    AP_Proximity_Backend_Serial(AP_Proximity &_frontend,
                                AP_Proximity::Proximity_State &_state,
                                AP_Proximity_Params& _params,
                                uint8_t serial_instance);

    // static detection function
    // detect if a proximity sensor is connected by looking for a configured serial port
    // serial_instance affects which serial port is used.  Should be 0 or 1 depending on whether this is the 1st or 2nd proximity sensor with a serial interface
    static bool detect(uint8_t serial_instance);

protected:
    virtual uint16_t rxspace() const { return 0; };

    AP_HAL::UARTDriver *_uart;              // uart for communicating with sensor
};

#endif // HAL_PROXIMITY_ENABLED
