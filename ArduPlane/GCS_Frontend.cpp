#include "GCS_Frontend.h"

#include "Plane.h"

#include "Parameters.h"
#include <AP_Airspeed/AP_Airspeed.h>

GCS_Frontend_Plane::GCS_Frontend_Plane(DataFlash_Class &DataFlash, Parameters &g, AP_Airspeed &airspeed) :
    GCS_Frontend(DataFlash, g),
    _airspeed(airspeed) { }

/*
  send airspeed calibration data
 */
void GCS_Frontend_Plane::send_airspeed_calibration(const Vector3f &vg)
{
    for (uint8_t i=0; i<num_gcs(); i++) {
        if (gcs(i).initialised) {
            if (comm_get_txspace((mavlink_channel_t)i) - MAVLINK_NUM_NON_PAYLOAD_BYTES >= 
                MAVLINK_MSG_ID_AIRSPEED_AUTOCAL_LEN) {
                _airspeed.log_mavlink_send((mavlink_channel_t)i, vg);
            }
        }
    }
}
