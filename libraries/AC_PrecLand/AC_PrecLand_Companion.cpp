#include "AC_PrecLand_config.h"

#if AC_PRECLAND_COMPANION_ENABLED

#include "AC_PrecLand_Companion.h"
#include <AP_HAL/AP_HAL.h>
#include <GCS_MAVLink/GCS.h>

// perform any required initialisation of backend
void AC_PrecLand_Companion::init()
{
    // set healthy
    _state.healthy = true;
}

// retrieve updates from sensor
void AC_PrecLand_Companion::update()
{
    _have_los_meas = _have_los_meas && AP_HAL::millis()-_los_meas_time_ms <= 1000;
}

void AC_PrecLand_Companion::handle_msg(const mavlink_landing_target_t &packet, uint32_t timestamp_ms)
{
    _distance_to_target = packet.distance;

    if (packet.position_valid == 1) {
        if (packet.frame == MAV_FRAME_BODY_FRD) {
            if (_distance_to_target > 0) {
                _los_meas_body = Vector3f(packet.x, packet.y, packet.z);
                _los_meas_body /= _distance_to_target;
            } else {
                // distance to target must be positive
                return;
            }
        } else {
            //we do not support this frame
            if (!_wrong_frame_msg_sent) {
                _wrong_frame_msg_sent = true;
                GCS_SEND_TEXT(MAV_SEVERITY_INFO,"Plnd: Frame not supported ");
            }
            return;
        }
    } else {
        // compute unit vector towards target
        _los_meas_body = Vector3f(-tanf(packet.angle_y), tanf(packet.angle_x), 1.0f);
        _los_meas_body /= _los_meas_body.length();
    }

    _los_meas_time_ms = timestamp_ms;
    _have_los_meas = true;
}

#endif // AC_PRECLAND_COMPANION_ENABLED
