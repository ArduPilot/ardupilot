#pragma once

#include "SIM_config.h"

#if AP_SIM_GPS_SBP_ENABLED

#include "SIM_GPS_SBP_Common.h"

namespace SITL {

class GPS_SBP : public GPS_SBP_Common {
public:
    CLASS_NO_COPY(GPS_SBP);

    using GPS_SBP_Common::GPS_SBP_Common;

    void publish(const GPS_Data *d) override;

};

};

#endif  // AP_SIM_GPS_SBP_ENABLED
