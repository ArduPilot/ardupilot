#include "SIM_config.h"

#if AP_SIM_GPS_UBLOX_ENABLED

#include "SIM_GPS.h"

namespace SITL {

class GPS_UBlox : public GPS_Backend {
public:
    CLASS_NO_COPY(GPS_UBlox);

    using GPS_Backend::GPS_Backend;

    void publish(const GPS_Data *d) override;

private:
    void send_ubx(uint8_t msgid, uint8_t *buf, uint16_t size);
};

};

#endif  // AP_SIM_GPS_UBLOX_ENABLED
