#pragma once

#include "AP_VisualOdom_Backend.h"

class AP_VisualOdom_MAV : public AP_VisualOdom_Backend
{

public:
    // constructor
    AP_VisualOdom_MAV(AP_VisualOdom &frontend);

    // consume VISION_POSITION_DELTA MAVLink message
    void handle_msg(const mavlink_message_t &msg) override;
};
