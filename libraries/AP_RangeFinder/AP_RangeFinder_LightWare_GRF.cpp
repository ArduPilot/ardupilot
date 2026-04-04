#include "AP_RangeFinder_config.h"

#if AP_RANGEFINDER_LIGHTWARE_GRF_ENABLED

#include "AP_RangeFinder_LightWare_GRF.h"
#include <AP_HAL/AP_HAL.h>
#include <stdio.h>
#include <GCS_MAVLink/GCS.h>

#define GRF_UPDATE_RATE_HZ 50
#define GRF_STREAM_CM_DISTANCES 5
#define GRF_MAX_DISTANCE_CM 50000

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo AP_RangeFinder_LightWareGRF::var_info[] = {
    // @Param: GRF_RET
    // @DisplayName: LightWare GRF Distance Return Type
    // @Description: Selects which single return to use.
    // @Values: 0:FirstRaw,1:FirstFiltered,2:LastRaw,3:LastFiltered
    // @User: Standard
    // @RebootRequired: True
    AP_GROUPINFO("GRF_RET", 12, AP_RangeFinder_LightWareGRF, return_selection, (uint8_t)GRF_ReturnSelection::FIRST_RAW),

    // @Param: GRF_ST
    // @DisplayName: LightWare GRF Minimum Return Strength
    // @Description: Minimum acceptable return signal strength in dB. Returns weaker than this will be ignored. Set to 0 to disable filtering.
    // @Range: 0 255
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO("GRF_ST", 13, AP_RangeFinder_LightWareGRF, minimum_return_strength, 0),

    // @Param: GRF_RATE
    // @DisplayName: LightWare GRF Update Rate
    // @Description: The update rate of the sensor in Hz. Must match the
    // @Range: 1 50
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO("GRF_RATE", 14, AP_RangeFinder_LightWareGRF, update_rate, GRF_UPDATE_RATE_HZ),

    AP_GROUPEND
};

AP_RangeFinder_LightWareGRF::AP_RangeFinder_LightWareGRF(RangeFinder::RangeFinder_State &_state, AP_RangeFinder_Params &_params)
    : AP_RangeFinder_Backend_Serial(_state, _params),
    AP_LightWareSerial(AP_RangeFinder_Backend_Serial::uart)
{
    AP_Param::setup_object_defaults(this, var_info);
    state.var_info = var_info;
}

// Checks if PRODUCT_NAME payload matches expected GRF signature
bool AP_RangeFinder_LightWareGRF::matches_product_name(const uint8_t *buf, const uint16_t len)
{
    // Must be at least "GRFXXX\0" = 7 bytes
    if (len < 7) {
        return false;
    }

    // Compare the first 3 bytes with "GRF"
    return strncmp((const char*)buf, "GRF", 3) == 0;
}

// Parses config responses and advances setup step
void AP_RangeFinder_LightWareGRF::check_config(const MessageID &resp_cmd_id, const uint8_t* response_buf, const uint16_t& response_len)
{
    bool valid = false;

    switch (config_step) {
    case ConfigStep::HANDSHAKE:
        valid = (resp_cmd_id == MessageID::PRODUCT_NAME &&
                    matches_product_name(response_buf, response_len));
        if (valid) {
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "LightWare %s detected", (const char*)response_buf);
        }
        break;

    case ConfigStep::UPDATE_RATE:
        if (resp_cmd_id == MessageID::UPDATE_RATE && response_len >= 4) {
            const uint8_t response_update_rate = (uint8_t)UINT32_VALUE(response_buf[3], response_buf[2], response_buf[1], response_buf[0]);
            valid = (response_update_rate == update_rate);
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
        const uint8_t payload[4] = {(uint8_t)update_rate, 0, 0, 0};
        send_message((uint8_t)MessageID::UPDATE_RATE, true, payload, 4);
        break;
    }

    case ConfigStep::DISTANCE_OUTPUT: {
        uint8_t data_bit = 1, strength_bit = 2;
        switch (GRF_ReturnSelection(return_selection.get())) {
            case GRF_ReturnSelection::FIRST_RAW:
                data_bit = 0;
                break;
            case GRF_ReturnSelection::LAST_RAW:
                data_bit = 3;
                strength_bit = 5;
                break;
            case GRF_ReturnSelection::LAST_FILTERED:
                data_bit = 4;
                strength_bit = 5;
                break;
            case GRF_ReturnSelection::FIRST_FILTERED:
                break;
        }
        const uint8_t payload[4] = {
            static_cast<uint8_t>((1U << data_bit) | (1U << strength_bit)), 0, 0, 0
        };
        send_message((uint8_t)MessageID::DISTANCE_OUTPUT, true, payload, 4);
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

    if (cmd_id != MessageID::DISTANCE_DATA_CM || _msg.payload_len < 8) {
        // beyond the configuration steps we only expect distance data messages
        return;
    }

    // Extract distance and strength
    uint32_t distance_cm = UINT32_VALUE(_msg.payload[3], _msg.payload[2], _msg.payload[1], _msg.payload[0]) * 10;
    uint32_t strength_db = UINT32_VALUE(_msg.payload[7], _msg.payload[6], _msg.payload[5], _msg.payload[4]);

    if (distance_cm == 0 || distance_cm > GRF_MAX_DISTANCE_CM) {
        // out of range reading
        return;
    }

    if (minimum_return_strength == 0 || (int8_t)strength_db >= minimum_return_strength) {
        float dist_m = distance_cm * 0.01f;
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
