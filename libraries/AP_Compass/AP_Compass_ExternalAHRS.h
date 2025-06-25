#pragma once

#include "AP_Compass_config.h"

#if AP_COMPASS_EXTERNALAHRS_ENABLED

#include "AP_Compass.h"
#include "AP_Compass_Backend.h"
#include <AP_ExternalAHRS/AP_ExternalAHRS.h>

class AP_Compass_ExternalAHRS : public AP_Compass_Backend
{
public:
    using AP_Compass_Backend::AP_Compass_Backend;

    static AP_Compass_Backend *probe(uint8_t port);

    void read(void) override;

private:
    void handle_external(const AP_ExternalAHRS::mag_data_message_t &pkt) override;
};

#endif  // AP_COMPASS_EXTERNALAHRS_ENABLED
