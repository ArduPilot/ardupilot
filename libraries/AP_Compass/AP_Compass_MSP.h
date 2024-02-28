#pragma once

#include "AP_Compass_config.h"

#if AP_COMPASS_MSP_ENABLED

#include "AP_Compass.h"
#include "AP_Compass_Backend.h"
#include <AP_MSP/msp.h>

class AP_Compass_MSP : public AP_Compass_Backend
{
public:
    AP_Compass_MSP(uint8_t msp_instance);

    void read(void) override;

private:
    void handle_msp(const MSP::msp_compass_data_message_t &pkt) override;
    uint8_t msp_instance;
    uint8_t instance;
};

#endif // AP_COMPASS_MSP_ENABLED
