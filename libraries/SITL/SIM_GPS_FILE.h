#include "SIM_config.h"

#if AP_SIM_GPS_FILE_ENABLED

#include "SIM_GPS.h"

namespace SITL {

class GPS_FILE : public GPS_Backend {
public:
    CLASS_NO_COPY(GPS_FILE);

    using GPS_Backend::GPS_Backend;

    void publish(const GPS_Data *d) override;
};

};

#endif  // AP_SIM_GPS_FILE_ENABLED
