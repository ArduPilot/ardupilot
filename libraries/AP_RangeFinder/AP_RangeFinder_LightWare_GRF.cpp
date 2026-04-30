#include "AP_RangeFinder_config.h"

#if AP_RANGEFINDER_LIGHTWARE_GRF_ENABLED

#include "AP_RangeFinder_LightWare_GRF.h"
#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/utility/sparse-endian.h>
#include <GCS_MAVLink/GCS.h>

#define GRF_STREAM_CM_DISTANCES 5

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo AP_RangeFinder_LightWareGRF::var_info[] = {
    // @Group: GRF_
    // @Path: AP_RangeFinder_LightWare_GRF_Common.cpp
    AP_SUBGROUPINFO(_common, "GRF_", 12, AP_RangeFinder_LightWareGRF, AP_RangeFinder_LightWare_GRF_Common),

    AP_GROUPEND
};

AP_RangeFinder_LightWareGRF::AP_RangeFinder_LightWareGRF(RangeFinder::RangeFinder_State &_state, AP_RangeFinder_Params &_params)
    : AP_RangeFinder_Backend_Serial(_state, _params),
    AP_LightWareSerial(AP_RangeFinder_Backend_Serial::uart)
{
    AP_Param::setup_object_defaults(this, var_info);
    state.var_info = var_info;
}

// Parses config responses and advances setup step
void AP_RangeFinder_LightWareGRF::check_config(const MessageID &resp_cmd_id, const uint8_t* response_buf, const uint16_t& response_len)
{
    bool valid = false;

    switch (config_step) {
    case ConfigStep::HANDSHAKE:
        valid = (resp_cmd_id == MessageID::PRODUCT_NAME &&
                    AP_RangeFinder_LightWare_GRF_Common::matches_product_name(response_buf, response_len));
        if (valid) {
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "LightWare %s detected", (const char*)response_buf);
        }
        break;

    case ConfigStep::UPDATE_RATE:
        if (resp_cmd_id == MessageID::UPDATE_RATE && response_len >= 4) {
            const uint8_t response_update_rate = (uint8_t)UINT32_VALUE(response_buf[3], response_buf[2], response_buf[1], response_buf[0]);
            valid = (response_update_rate == _common.update_rate);
        }
        break;

    case ConfigStep::DISTANCE_OUTPUT:
        valid = (resp_cmd_id == MessageID::DISTANCE_OUTPUT);
        break;

    case ConfigStep::STREAM:
        valid = (resp_cmd_id == MessageID::STREAM);
        break;

    case ConfigStep::DONE:
        break;
    }

    if (valid && config_step != ConfigStep::DONE) {
        config_step = static_cast<ConfigStep>(uint8_t(config_step) + 1);
        return;
    }
}

// Send configuration messages to the rangefinder
void AP_RangeFinder_LightWareGRF::configure_rangefinder()
{
    if (config_step == ConfigStep::DONE) {
        return;
    }

    const uint32_t now = AP_HAL::millis();
    if (now - last_config_message_ms < 100) {
        return;
    }

    switch (config_step) {

    case ConfigStep::HANDSHAKE:
        uart->write((uint8_t*)"UUU", 3); // Try to switch GRF sensor to serial mode
        send_message((uint8_t)MessageID::PRODUCT_NAME, false, nullptr, 0);
        break;

    case ConfigStep::UPDATE_RATE: {
        const uint8_t payload[4] = {(uint8_t)_common.update_rate, 0, 0, 0};
        send_message((uint8_t)MessageID::UPDATE_RATE, true, payload, 4);
        break;
    }

    case ConfigStep::DISTANCE_OUTPUT: {
        uint8_t payload[4];
        put_le32_ptr(payload, _common.build_distance_output_bitmask());
        send_message((uint8_t)MessageID::DISTANCE_OUTPUT, true, payload, sizeof(payload));
        break;
    }

    case ConfigStep::STREAM: {
        const uint8_t payload[4] = {GRF_STREAM_CM_DISTANCES, 0, 0, 0};
        send_message((uint8_t)MessageID::STREAM, true, payload, 4);
        break;
    }

    case ConfigStep::DONE:
        break;
    }

    last_config_message_ms = now;
}

// Processes the latest message held in the _msg structure
void AP_RangeFinder_LightWareGRF::process_message(float &sum_m, uint8_t &count)
{
    MessageID cmd_id = (MessageID)_msg.msgid;

    if (config_step != ConfigStep::DONE) {
        // sensor configuration in progress
        check_config(cmd_id, _msg.payload, _msg.payload_len);
        return;
    }

    if (cmd_id != MessageID::DISTANCE_DATA_CM) {
        // beyond the configuration steps we only expect distance data messages
        return;
    }

    float dist_m;
    if (_common.parse_distance_cm_payload(_msg.payload, _msg.payload_len, dist_m)) {
        sum_m += dist_m;
        count++;
    }
}

// Called periodically to fetch a new range reading
bool AP_RangeFinder_LightWareGRF::get_reading(float &reading_m)
{
    if (uart == nullptr) {
        return false;
    }

    if (config_step != ConfigStep::DONE) {
        configure_rangefinder();
    }

    float sum_m = 0.0f;
    uint8_t count = 0;

    // process up to 1K of characters per iteration
    uint32_t nbytes = MIN(uart->available(), 1024U);
    while (nbytes-- > 0) {
        uint8_t c;
        if (!uart->read(c)) {
            continue;
        }
        if (parse_byte(c)) {
            process_message(sum_m, count);
        }
    }
    if (count > 0) {
        reading_m = sum_m / count;
        return true;
    }
    return false;
}

#endif // AP_RANGEFINDER_LIGHTWARE_GRF_ENABLED
