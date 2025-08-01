#include "AP_RangeFinder_config.h"

#if AP_RANGEFINDER_LIGHTWARE_GRF_ENABLED

#include "AP_RangeFinder_LightWare_GRF.h"
#include <AP_HAL/AP_HAL.h>
#include <stdio.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_Math/crc.h>

#define GRF_UPDATE_RATE 10 // Hz
#define GRF_STREAM_CM_DISTANCES 5
#define REINITIALIZE_GRF_TIME_MS 2000 // reinitialize if no response for this long

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo AP_RangeFinder_LightWareGRF::var_info[] = {
    // @Param: GRF_RET
    // @DisplayName: LightWare GRF Distance Return Type
    // @Description: Selects which single return to use.
    // @Values: 0:FirstRaw,1:FirstFiltered,2:LastRaw,3:LastFiltered
    // @User: Standard
    // @RebootRequired: True
    AP_GROUPINFO("GRF_RET", 12, AP_RangeFinder_LightWareGRF, return_selection, (uint8_t)GRF_ReturnSelection::FIRST_FILTERED),

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
    AP_GROUPINFO("GRF_RATE", 14, AP_RangeFinder_LightWareGRF, update_rate, GRF_UPDATE_RATE),

    AP_GROUPEND
};

AP_RangeFinder_LightWareGRF::AP_RangeFinder_LightWareGRF(RangeFinder::RangeFinder_State &_state, AP_RangeFinder_Params &_params)
    : AP_RangeFinder_Backend_Serial(_state, _params)
{
    AP_Param::setup_object_defaults(this, var_info);

    state.var_info = var_info;
    reset_state_variables();
}

// Initializes the state variables for the GRF rangefinder
void AP_RangeFinder_LightWareGRF::reset_state_variables()
{
    grf.config_step = ConfigStep::HANDSHAKE;
    grf.last_init_ms = 0;
    grf.buffer_len = 0;
}


// Builds a UART packet with CRC from components
uint16_t AP_RangeFinder_LightWareGRF::build_packet(MessageID cmd_id, bool is_write, const uint8_t *payload, uint16_t payload_len, uint8_t *out_buf)
{
    uint16_t flags = (payload_len + 1) << 6;
    if (is_write) {
        flags |= 0x01;
    }

    uint16_t idx = 0;
    out_buf[idx++] = GRF_START_BYTE;
    out_buf[idx++] = flags & 0xFF;
    out_buf[idx++] = (flags >> 8) & 0xFF;
    out_buf[idx++] = (uint8_t)cmd_id;

    for (uint16_t i = 0; i < payload_len; i++) {
        out_buf[idx++] = payload[i];
    }

    uint16_t crc = crc16_lightware(out_buf, idx);
    out_buf[idx++] = crc & 0xFF;
    out_buf[idx++] = (crc >> 8) & 0xFF;

    return idx;
}

// Attempts to parse a single streaming measurement
bool AP_RangeFinder_LightWareGRF::try_parse_stream_packet(float &reading_m)
{
    MessageID cmd_id;
    uint8_t payload[GRF_MAX_PAYLOAD];
    uint16_t payload_len = 0;

    if (!read_and_parse_response(cmd_id, payload, payload_len)) {
        return false;
    }

    if (cmd_id != MessageID::DISTANCE_DATA_CM || payload_len < 8) {
        return false;
    }

    uint32_t distance_cm = UINT32_VALUE(payload[3], payload[2], payload[1], payload[0]) * 10;
    uint32_t strength_db = UINT32_VALUE(payload[7], payload[6], payload[5], payload[4]);

    // gcs().send_text(MAV_SEVERITY_DEBUG, "GRF: dist=%dcm strength=%d", (uint16_t)distance_cm, (uint16_t)strength_db);

    if (distance_cm < GRF_DIST_MAX * 100) {
        if (minimum_return_strength == 0 || (int8_t)strength_db >= minimum_return_strength) {
            reading_m = distance_cm * 0.01f;
            return true;
        }
    }

    return false;
}

// Checks if PRODUCT_NAME payload matches expected GRF signature
bool AP_RangeFinder_LightWareGRF::matches_product_name(const uint8_t *buf, uint16_t len)
{
    // Must be at least "GRFXXX\0" = 7 bytes
    if (len < 7) {
        return false;
    }

    const char expected[] = "GRF";
    for (uint8_t i = 0; i < 3; i++) { // Only compare "GRF" for compatibility with GRF, GRF500
        if (buf[i] != expected[i]) {
            return false;
        }
    }
    return buf[6] == '\0'; // Check null after that
}

// Sends a configuration command packet
void AP_RangeFinder_LightWareGRF::send_config(MessageID cmd_id,
                                        const uint8_t* payload,
                                        uint16_t payload_len,
                                        bool is_write)
{
    // Build and send a packet directly without tracking confirm state
    uint8_t packet[32];
    uint16_t len = build_packet(cmd_id, is_write, payload, payload_len, packet);
    uart->write(packet, len);
}

// Parses config responses and advances setup step
void AP_RangeFinder_LightWareGRF::check_config()
{
    MessageID resp_cmd_id;
    uint8_t response_buf[GRF_MAX_PAYLOAD];
    uint16_t response_len = 0;

    if (!read_and_parse_response(resp_cmd_id, response_buf, response_len)) {
        return;
    }

    bool valid = false;

    switch (grf.config_step) {
    case ConfigStep::HANDSHAKE:
        valid = (resp_cmd_id == MessageID::PRODUCT_NAME && matches_product_name(response_buf, response_len));
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

    default:
        break;
    }

    if (valid) {
        grf.config_step = static_cast<ConfigStep>(static_cast<uint8_t>(grf.config_step) + 1);
    }
}

    // Configure the rangefinder
    void AP_RangeFinder_LightWareGRF::configure_rangefinder()
{
    if (grf.config_step == ConfigStep::DONE) {
        return;
    }

    check_config();

    const uint32_t now = AP_HAL::millis();
    if (now - grf.last_init_ms < 100) {
        return;
    }

    switch (grf.config_step) {
    case ConfigStep::HANDSHAKE:
        uart->write((uint8_t*)"UUU", 3); // Try to switch GRF sensor to serial mode
        send_config(MessageID::PRODUCT_NAME, nullptr, 0, false);
        break;

    case ConfigStep::UPDATE_RATE: {
        const uint8_t payload[4] = {(uint8_t)update_rate, 0, 0, 0};
        send_config(MessageID::UPDATE_RATE, payload, 4, true);
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
            default:
                break;
        }
        const uint8_t payload[4] = {
            static_cast<uint8_t>((1U << data_bit) | (1U << strength_bit)), 0, 0, 0
        };
        send_config(MessageID::DISTANCE_OUTPUT, payload, 4, true);
        break;
    }

    case ConfigStep::STREAM: {
        const uint8_t payload[4] = {GRF_STREAM_CM_DISTANCES, 0, 0, 0};
        send_config(MessageID::STREAM, payload, 4, true);
        break;
    }

    case ConfigStep::DONE:
        break;
    }

    grf.last_init_ms = now;
}

// Reads UART and tries to parse a complete response
bool AP_RangeFinder_LightWareGRF::read_and_parse_response(MessageID& cmd_id_out,
                                                    uint8_t* payload_out,
                                                    uint16_t& payload_len_out)
{
    if (uart == nullptr) {
        return false; // UART not initialized
    }

    // Step 1: Read from UART into buffer
    const uint16_t bytes_available = MIN(uart->available(), 8192U);
    const uint16_t space = GRF_BUFFER_SIZE - grf.buffer_len;
    if (bytes_available > 0 && space > 0) {
        const uint16_t to_read = MIN(bytes_available, space);
        grf.buffer_len += uart->read(&grf.parse_buffer[grf.buffer_len], to_read);
    }

    uint8_t* buf = grf.parse_buffer;
    const uint16_t len = grf.buffer_len;

    // Step 2: Try parsing a packet
    for (uint16_t i = 0; i + 6 <= len; ++i) {
        if (buf[i] != GRF_START_BYTE) {
            continue;
        }

        const uint16_t flags = buf[i + 1] | (buf[i + 2] << 8);
        const uint16_t payload_len = flags >> 6;
        // total length of the packet is 1 byte header + 2 bytes for payload length + payload + 2 bytes CRC
        const uint16_t expected_len = 3 + payload_len + 2;

        if (payload_len == 0 || payload_len > GRF_MAX_PAYLOAD) {
            continue;
        }

        if (i + expected_len > len) {
            break; // Not enough data yet
        }

        // Step 3: CRC check
        const uint16_t received_crc = buf[i + expected_len - 2] | (buf[i + expected_len - 1] << 8);
        const uint16_t calc_crc = crc16_lightware(&buf[i], expected_len - 2);

        if (received_crc != calc_crc) {
            // Skip to next possible packet
            for (uint16_t j = i + 1; j < len; ++j) {
                if (buf[j] == GRF_START_BYTE) {
                    grf.buffer_len = len - j;
                    memmove(buf, &buf[j], grf.buffer_len);
                    return false;
                }
            }
            grf.buffer_len = 0;
            return false;
        }

        // Step 4: Extract packet
        cmd_id_out = static_cast<MessageID>(buf[i + 3]);
        payload_len_out = payload_len;

        if (payload_len > 0) {
            memcpy(payload_out, &buf[i + 4], payload_len);
        }

        // Step 5: Slide buffer forward
        grf.buffer_len = len - (i + expected_len);
        memmove(buf, &buf[i + expected_len], grf.buffer_len);
        return true;
    }

    return false;
}

// Called periodically to fetch a new range reading
bool AP_RangeFinder_LightWareGRF::get_reading(float &reading_m)
{
    if (uart == nullptr) {
        return false;
    }

    if (grf.config_step != ConfigStep::DONE) {
        configure_rangefinder();
        return false;
    }

    const uint32_t now = AP_HAL::millis();
    if ((now - state.last_reading_ms > REINITIALIZE_GRF_TIME_MS) &&
        (now - grf.last_init_ms > REINITIALIZE_GRF_TIME_MS)) {
        reset_state_variables();
        return false;
    }

    return try_parse_stream_packet(reading_m);
}

#endif // AP_RANGEFINDER_LIGHTWARE_GRF_ENABLED
