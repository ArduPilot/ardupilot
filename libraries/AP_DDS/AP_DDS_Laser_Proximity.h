#pragma once

#if AP_DDS_ENABLED
#include "sensor_msgs/msg/LaserScan.h"

class AP_DDS_Laser_Proximity {
public:
    // Handle incoming laser scan messages
    static void handle_laser_scan(const sensor_msgs_msg_LaserScan& msg);
};

#endif // AP_DDS_ENABLED
