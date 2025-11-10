#pragma once

#include "SIM_config.h"

#if AP_SIM_GPS_NOVA_ENABLED

#include "SIM_GPS.h"

namespace SITL {

class GPS_NOVA : public GPS_Backend {
public:
    CLASS_NO_COPY(GPS_NOVA);

    using GPS_Backend::GPS_Backend;

    void publish(const GPS_Data *d) override;

    uint32_t device_baud() const override { return 19200; }

private:

    void nova_send_message(uint8_t *header, uint8_t headerlength, uint8_t *payload, uint8_t payloadlen);
};

};

#endif  // AP_SIM_GPS_NOVA_ENABLED
