#include "GCS_Plane.h"
#include "Plane.h"

bool GCS_Plane::cli_enabled() const
{
#if CLI_ENABLED == ENABLED
    return plane.g.cli_enabled;
#else
    return false;
#endif
}

AP_HAL::BetterStream* GCS_Plane::cliSerial() {
    return plane.cliSerial;
}

void GCS_Plane::send_airspeed_calibration(const Vector3f &vg)
{
    for (uint8_t i=0; i<num_gcs(); i++) {
        if (_chan[i].initialised) {
            if (HAVE_PAYLOAD_SPACE((mavlink_channel_t)i, AIRSPEED_AUTOCAL)) {
                plane.airspeed.log_mavlink_send((mavlink_channel_t)i, vg);
            }
        }
    }
}
