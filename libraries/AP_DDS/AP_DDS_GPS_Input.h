// Class for handling GPS fixes pushed in over DDS by a companion computer.
// Decodes obs_msgs/UbloxPvt and hands it to the AP_GPS_DDS backend
// (GPS_TYPE=27); the backend itself carries no ROS types.

#pragma once

#include "AP_DDS_config.h"
#if AP_DDS_GPS_INPUT_SUB_ENABLED

#include "obs_msgs/msg/UbloxPvt.h"
#include <AP_GPS/AP_GPS_DDS.h>

class AP_DDS_GPS_Input
{
public:

    // Handler for an incoming fix: converts it and pushes it into every
    // GPS instance configured as GPS_TYPE_DDS
    static void handle_gps_input(const obs_msgs_msg_UbloxPvt& msg);

    // Helper to convert a UbloxPvt into the backend's sample.
    // UbloxPvt carries u-blox NAV-PVT already scaled to SI: degrees, metres
    // above MSL, NED velocity in m/s. It has no hDOP/vDOP, only pDOP.
    static void convert(const obs_msgs_msg_UbloxPvt& msg, AP_GPS_DDS::NavSatFix& pkt);

};

#endif // AP_DDS_GPS_INPUT_SUB_ENABLED
