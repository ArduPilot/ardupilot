#pragma once

#include "AP_VisualOdom_Backend.h"

class AP_VisualOdom_MAV : public AP_VisualOdom_Backend
{

public:
    // constructor
    AP_VisualOdom_MAV(AP_VisualOdom &frontend);

    // consume vision_position_delta mavlink messages
    void handle_vision_position_delta_msg(const mavlink_message_t &msg) override;
};
