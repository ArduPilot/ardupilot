#pragma once

#include "AP_HAL_Linux.h"
#include "RCInput.h"
#include <AP_HAL/SPIDevice.h>

class Linux::RCInput_Raspilot : public Linux::RCInput
{
public:
    void init();
    
private:
    uint32_t _last_timer;
    
    AP_HAL::OwnPtr<AP_HAL::SPIDevice> _dev;
    
    void _poll_data(void);
};
