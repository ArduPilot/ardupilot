#pragma once

#include "AP_Proximity_Backend.h"

#if HAL_PROXIMITY_ENABLED

class AP_SerialDevice;

class AP_Proximity_Backend_Serial : public AP_Proximity_Backend
{
public:
    AP_Proximity_Backend_Serial(AP_Proximity &_frontend,
                                AP_Proximity::Proximity_State &_state);
    // static detection function
    static bool detect();

protected:
    virtual uint16_t rxspace() const { return 0; };

    AP_SerialDevice *_uart;
};

#endif // HAL_PROXIMITY_ENABLED
