#pragma once

#include "SIM_config.h"

#if AP_SIM_GPS_SBF_ENABLED

#include "SIM_GPS.h"

namespace SITL {

class GPS_SBF : public GPS_Backend {
public:
    CLASS_NO_COPY(GPS_SBF);

    using GPS_Backend::GPS_Backend;

    void publish(const GPS_Data *d) override;

private:

    void send_sbf(uint16_t msgid, uint8_t *buf, uint16_t buf_size);
    void publish_PVTGeodetic(const GPS_Data *d);
    void publish_DOP(const GPS_Data *d);

};

};

#endif  // AP_SIM_GPS_SBF_ENABLED
