#include "AC_PrecLand_config.h"

#if AC_PRECLAND_MAVLINK_ENABLED

#include "AC_PrecLand_MAVLink.h"
#include <AP_HAL/AP_HAL.h>
#include <GCS_MAVLink/GCS.h>

// perform any required initialisation of backend
void AC_PrecLand_MAVLink::init()
{
    // set healthy
    _state.healthy = true;
}

// retrieve updates from sensor
void AC_PrecLand_MAVLink::update()
{
    _los_meas.valid = _los_meas.valid && AP_HAL::millis() - _los_meas.time_ms <= 1000;
}

void AC_PrecLand_MAVLink::handle_msg(const mavlink_landing_target_t &packet, uint32_t timestamp_ms)
{
    // check frame is supported
    if (packet.frame != MAV_FRAME_BODY_FRD && packet.frame != MAV_FRAME_LOCAL_FRD) {
        if (!_wrong_frame_msg_sent) {
            _wrong_frame_msg_sent = true;
            GCS_SEND_TEXT(MAV_SEVERITY_INFO,"Plnd: Frame not supported");
        }
        return;
    }

    if (packet.position_valid == 1) {
        if (packet.distance > 0) {
            _los_meas.vec_unit = Vector3f{packet.x, packet.y, packet.z};
            _los_meas.vec_unit /= packet.distance;
            _los_meas.frame = (packet.frame == MAV_FRAME_BODY_FRD) ? AC_PrecLand::VectorFrame::BODY_FRD : AC_PrecLand::VectorFrame::LOCAL_FRD;
        } else {
            // distance to target must be positive
            return;
        }
    } else {
        // compute unit vector towards target
        _los_meas.vec_unit = Vector3f{-tanf(packet.angle_y), tanf(packet.angle_x), 1.0f};
        _los_meas.vec_unit /= _los_meas.vec_unit.length();
        _los_meas.frame = (packet.frame == MAV_FRAME_BODY_FRD) ? AC_PrecLand::VectorFrame::BODY_FRD : AC_PrecLand::VectorFrame::LOCAL_FRD;
    }

    _distance_to_target = MAX(0, packet.distance);
    _los_meas.time_ms = timestamp_ms;
    _los_meas.valid = true;
}

#endif // AC_PRECLAND_MAVLINK_ENABLED
