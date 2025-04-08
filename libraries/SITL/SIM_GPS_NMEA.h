#pragma once

#include "SIM_config.h"

#if AP_SIM_GPS_NMEA_ENABLED

#include "SIM_GPS.h"

namespace SITL {

class GPS_NMEA : public GPS_Backend {
public:
    CLASS_NO_COPY(GPS_NMEA);

    using GPS_Backend::GPS_Backend;

    void publish(const GPS_Data *d) override;

private:

    uint8_t nmea_checksum(const char *s);
    void nmea_printf(const char *fmt, ...);
    void update_nmea(const GPS_Data *d);

};

};

#endif  // AP_SIM_GPS_NMEA_ENABLED
