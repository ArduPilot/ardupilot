#pragma once

#include "SIM_config.h"

#if AP_SIM_GPS_SBP2_ENABLED

#include "SIM_GPS_SBP_Common.h"

namespace SITL {

class GPS_SBP2 : public GPS_SBP_Common {
public:
    CLASS_NO_COPY(GPS_SBP2);

    using GPS_SBP_Common::GPS_SBP_Common;

    void publish(const GPS_Data *d) override;

};

};

#endif  // AP_SIM_GPS_SBP2_ENABLED
