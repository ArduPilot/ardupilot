#include <AP_BoardConfig/AP_BoardConfig.h>
#include "AP_Radar.h"

#if AP_RADAR_ENABLED

#include "AP_Radar_MSP.h"
#include <AP_Logger/AP_Logger.h>
#include <GCS_MAVLink/GCS.h>

#define RADAR_TYPE_DEFAULT 1

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo AP_Radar::var_info[] = {
    // @Param: _TYPE
    // @DisplayName: Radar sensor type
    // @Description: Radar sensor type
    // @Values: 0:None, 1:MSP
    // @User: Standard
    // @RebootRequired: True
    AP_GROUPINFO_FLAGS("_TYPE", 0,  AP_Radar,    _type,   (float)RADAR_TYPE_DEFAULT, AP_PARAM_FLAG_ENABLE),

    AP_GROUPEND
};

// default constructor
AP_Radar::AP_Radar()
{
    _singleton = this;

    AP_Param::setup_object_defaults(this, var_info);
}

void AP_Radar::init(uint32_t log_bit)
{
     _log_bit = log_bit;

    // return immediately if not enabled or backend already created
    if ((_type == Type::NONE) || (backend != nullptr)) {
        return;
    }

    switch ((Type)_type) {
    case Type::NONE:
        break;
    case Type::MSP:
        backend = AP_Radar_MSP::detect(*this);
        break;
    }

    if (backend != nullptr) {
        backend->init();
    }
}

void AP_Radar::update(void)
{
    // exit immediately if not enabled
    if (!enabled()) {
        return;
    }
    if (backend != nullptr) {
        backend->update();
    }

    // only healthy if the data is less than 3s old
    _flags.healthy = (AP_HAL::millis() - _last_update_ms < RADAR_PEER_FRESH_TIME_MS);
}

void AP_Radar::handle_msg(const mavlink_message_t &msg)
{
    // exit immediately if not enabled
    if (!enabled()) {
        return;
    }

    if (backend != nullptr) {
        backend->handle_msg(msg);
    }
}

#if HAL_MSP_RADAR_ENABLED
void AP_Radar::handle_msp(const MSP::msp_radar_pos_message_t &pkt)
{
    // exit immediately if not enabled
    if (!enabled()) {
        return;
    }

    if (backend != nullptr) {
        backend->handle_msp(pkt);
    }
}
#endif //HAL_MSP_RADAR_ENABLED

void AP_Radar::update_state(const Radar_state &state)
{
    _state = state;
    _last_update_ms = AP_HAL::millis();
}

radar_peer_t AP_Radar::get_peer(uint8_t id)
{
    return _state.peers[id];
}

bool AP_Radar::get_peer_healthy(uint8_t id)
{
    return _state.peers[id].last_update > (AP_HAL::millis() - 3000) &&
        !_state.peers[id].location.is_zero();
}

uint8_t AP_Radar::get_next_healthy_peer(uint8_t current_id)
{
        // Try all available slots
	for (uint8_t i = 1; i < RADAR_MAX_PEERS; i++) {
            uint8_t next_peer = (current_id + i) % RADAR_MAX_PEERS;
            if (get_peer_healthy(next_peer)) {
                return next_peer;
            }
        }
        return current_id;
}

// singleton instance
AP_Radar *AP_Radar::_singleton;

namespace AP {

AP_Radar *radar()
{
    return AP_Radar::get_singleton();
}

}

#endif // AP_RADAR_ENABLED
