#include "GCS_Plane.h"
#include "Plane.h"

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
