#pragma once

#include "AP_Compass_config.h"

#if AP_COMPASS_EXTERNALAHRS_ENABLED

#include "AP_Compass.h"
#include "AP_Compass_Backend.h"
#include <AP_ExternalAHRS/AP_ExternalAHRS.h>

class AP_Compass_ExternalAHRS : public AP_Compass_Backend
{
public:
    AP_Compass_ExternalAHRS(uint8_t instance);

    void read(void) override;

private:
    void handle_external(const AP_ExternalAHRS::mag_data_message_t &pkt) override;
    uint8_t instance;
};

#endif  // AP_COMPASS_EXTERNALAHRS_ENABLED
