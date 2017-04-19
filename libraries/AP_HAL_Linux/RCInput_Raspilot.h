#pragma once

#include "AP_HAL_Linux.h"
#include "RCInput.h"
#include <AP_HAL/SPIDevice.h>


namespace Linux {

class RCInput_Raspilot : public RCInput
{
public:
    void init();

private:
    AP_HAL::OwnPtr<AP_HAL::SPIDevice> _dev;

    void _poll_data(void);
};

}
