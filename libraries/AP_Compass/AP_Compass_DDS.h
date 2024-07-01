#pragma once

#include "AP_Compass_config.h"

#if AP_COMPASS_DDS_ENABLED

#include "AP_Compass.h"
#include "AP_Compass_Backend.h"
#include <AP_ExternalAHRS/AP_ExternalAHRS.h>

class AP_Compass_DDS : public AP_Compass_Backend
{
public:
    AP_Compass_DDS();

    void read(void) override;

private:
    void handle_external(const AP_ExternalAHRS::mag_data_message_t &pkt) override;
    uint8_t _instance;
};

#endif  // AP_COMPASS_DDS_ENABLED
