#pragma once

#include <AP_Logger/LogStructure.h>

#include <AP_VisualOdom/AP_VisualOdom.h>

#if HAL_VISUALODOM_ENABLED

class AP_DAL_VisualOdom {
public:

    // return VisualOdom health
    bool healthy() const {
        return RVOH.healthy;
    }

    bool enabled() const {
        return RVOH.enabled;
    }

    uint16_t get_delay_ms() const {
        return RVOH.delay_ms;
    }

    // return a 3D vector defining the position offset of the camera in meters relative to the body frame origin
    const Vector3f &get_pos_offset() const {
        return RVOH.pos_offset;
    }

    // update position offsets to align to AHRS position
    // should only be called when this library is not being used as the position source
    void align_position_to_ahrs(bool align_xy, bool align_z);

    void start_frame();

    void handle_message(const log_RVOH &msg) {
        RVOH = msg;
    }

private:

    struct log_RVOH RVOH;
};

#endif // HAL_VISUALODOM_ENABLED
