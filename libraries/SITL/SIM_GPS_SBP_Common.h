#pragma once

#include "SIM_config.h"

#if AP_SIM_GPS_ENABLED

#include "SIM_GPS.h"

namespace SITL {

class GPS_SBP_Common : public GPS_Backend {
public:
    CLASS_NO_COPY(GPS_SBP_Common);

    using GPS_Backend::GPS_Backend;

protected:

    void sbp_send_message(uint16_t msg_type, uint16_t sender_id, uint8_t len, uint8_t *payload);

};

};

#endif  // AP_SIM_GPS_ENABLED
