#include <GCS_MAVLink/GCS_config.h>

#if HAL_GCS_ENABLED

#include "AP_GenericVehicle.h"
#include "GCS_GenericVehicle.h"

uint8_t GCS_GenericVehicle::sysid_this_mav() const
{
    return genericvehicle.g.sysid_this_mav;
}

static const ap_message STREAM_PARAMS_msgs[] = {
    MSG_NEXT_PARAM
};

const struct GCS_MAVLINK::stream_entries GCS_MAVLINK::all_stream_entries[] = {
    MAV_STREAM_ENTRY(STREAM_PARAMS),
    MAV_STREAM_TERMINATOR // must have this at end of stream_entries
};

#endif  // HAL_GCS_ENABLED
