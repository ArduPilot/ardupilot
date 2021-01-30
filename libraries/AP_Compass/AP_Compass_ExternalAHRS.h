#pragma once

#include "AP_Compass.h"
#include "AP_Compass_Backend.h"
#include <AP_ExternalAHRS/AP_ExternalAHRS.h>

#if HAL_EXTERNAL_AHRS_ENABLED

class AP_Compass_ExternalAHRS : public AP_Compass_Backend
{
public:
    AP_Compass_ExternalAHRS(uint8_t instance);

    void read(void) override;

private:
    void handle_external(const AP_ExternalAHRS::mag_data_message_t &pkt) override;
    uint8_t instance;
};

#endif // HAL_EXTERNAL_AHRS_ENABLED

