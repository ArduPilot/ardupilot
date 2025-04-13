#include "AP_RangeFinder_config.h"

#if AP_RANGEFINDER_GRF250_ENABLED

#include "AP_RangeFinder_LightWare_GRF250.h"
#include <AP_HAL/AP_HAL.h>
#include <stdio.h>
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL& hal;

#define DIST_MAX 250.0f

void AP_RangeFinder_GRF250::move_preamble()
{
    uint16_t i;
    for (i = 0; i < grf.parse_ofs; i++) {
        if (grf.parse_buffer[i] == GRF250_START_BYTE) {
            break;
        }
    }
    if (i > 0 && i < grf.parse_ofs) {
        memmove(grf.parse_buffer, &grf.parse_buffer[i], grf.parse_ofs - i);
        grf.parse_ofs -= i;
    }
}

uint16_t AP_RangeFinder_GRF250::calculate_crc(const uint8_t *data, uint16_t length)
{
    uint16_t crc = 0;
    for (uint16_t i = 0; i < length; i++) {
        uint16_t code = crc >> 8;
        code ^= data[i];
        code ^= code >> 4;
        crc = (crc << 8) ^ (code << 12) ^ (code << 5) ^ code;
    }
    return crc;
}

uint16_t AP_RangeFinder_GRF250::build_packet(uint8_t cmd_id, bool is_write, const uint8_t *payload, uint16_t payload_len, uint8_t *out_buf)
{
    uint16_t flags = (payload_len + 1) << 6;
    if (is_write) {
        flags |= 0x01;
    }

    uint16_t idx = 0;
    out_buf[idx++] = GRF250_START_BYTE;
    out_buf[idx++] = flags & 0xFF;
    out_buf[idx++] = (flags >> 8) & 0xFF;
    out_buf[idx++] = cmd_id;

    for (uint16_t i = 0; i < payload_len; i++) {
        out_buf[idx++] = payload[i];
    }

    uint16_t crc = calculate_crc(out_buf, idx);
    out_buf[idx++] = crc & 0xFF;
    out_buf[idx++] = (crc >> 8) & 0xFF;

    return idx;
}

bool AP_RangeFinder_GRF250::check_grf250_response()
{
    uint8_t cmd_id;
    uint8_t payload[GRF250_MAX_PAYLOAD];
    uint16_t payload_len = 0;
    uint16_t consumed = 0;

    while (grf.parse_ofs >= 10) {  // while we have enough bytes
        if (!parse_from_buffer(grf.parse_buffer, grf.parse_ofs, cmd_id, payload, payload_len, consumed)) {
            break;
        }

        // Remove parsed bytes from buffer
        if (consumed > 0 && consumed <= grf.parse_ofs) {
            memmove(grf.parse_buffer, grf.parse_buffer + consumed, grf.parse_ofs - consumed);
            grf.parse_ofs -= consumed;
        }

        // Now check if payload contains "GRF250"
        if (payload_len >= 6 && memcmp(payload, "GRF250", 6) == 0) {
            uint8_t packet[32];
            const uint8_t stream_payload[4] = {5, 0, 0, 0};
            uint16_t len = build_packet(30, true, stream_payload, 4, packet);
            uart->write(packet, len);

            const uint8_t rate_payload[4] = {50, 0, 0, 0}; // 50Hz
            len = build_packet(74, true, rate_payload, 4, packet);
            uart->write(packet, len);

            grf.product_verified = true;
            grf.init_stage = INIT_STAGE_STREAMING;
            grf.parse_ofs = 0;
            return true;
        }
    }
    return false;
}

bool AP_RangeFinder_GRF250::parse_packet(const uint8_t *packet, uint16_t expected_len, uint8_t &cmd_id, uint8_t *payload_buf, uint16_t &payload_len)
{
    if (expected_len < 6 || packet[0] != GRF250_START_BYTE) {
        return false;
    }

    const uint16_t received_crc = packet[expected_len - 2] | (packet[expected_len - 1] << 8);
    const uint16_t calculated_crc = calculate_crc(packet, expected_len - 2);

    if (received_crc != calculated_crc) {
        return false;
    }

    cmd_id = packet[3];

    // payload_len = (full_payload_len - 1)
    payload_len = expected_len - 5; // 4 bytes header + 1 byte for cmd_id
    if (payload_len > GRF250_MAX_PAYLOAD) {
        return false;
    }

    if (payload_len > 0) {
        memcpy(payload_buf, &packet[4], payload_len);
    }

    return true;
}

bool AP_RangeFinder_GRF250::parse_from_buffer(uint8_t *buf, uint32_t len, uint8_t &cmd_id, uint8_t *payload, uint16_t &payload_len, uint16_t &consumed)
{
    for (uint16_t i = 0; i <= len - 6; i++) {
        if (buf[i] != GRF250_START_BYTE) {
            continue;
        }

        if (uint16_t(i + 3) >= len) {
            break; // not enough to read flags yet
        }

        const uint16_t flags = buf[i+1] | (buf[i+2] << 8);
        const uint16_t full_payload_len = flags >> 6;
        const uint16_t expected_len = 3 + full_payload_len + 2;

        if (i + expected_len > len) {
            break; // not enough bytes yet
        }

        if (parse_packet(&buf[i], expected_len, cmd_id, payload, payload_len)) {
            consumed = i + expected_len;
            return true;
        }
    }
    return false;
}

bool AP_RangeFinder_GRF250::try_parse_stream_packet(float &reading_m)
{
    bool parsed_any = false;

    while (grf.parse_ofs >= 10) {
        uint8_t cmd_id;
        uint8_t payload[GRF250_MAX_PAYLOAD];
        uint16_t payload_len = 0;
        uint16_t consumed = 0;

        if (!parse_from_buffer(grf.parse_buffer, grf.parse_ofs, cmd_id, payload, payload_len, consumed)) {
            break;
        }

        if (consumed > 0 && consumed <= grf.parse_ofs) {
            memmove(grf.parse_buffer, grf.parse_buffer + consumed, grf.parse_ofs - consumed);
            grf.parse_ofs -= consumed;
        }

        if (cmd_id == 44 && payload_len >= 4) {
            int32_t dist_cm = payload[0] |
                              (payload[1] << 8) |
                              (payload[2] << 16) |
                              (payload[3] << 24);
            if (dist_cm >= 0 && dist_cm < 25000) {
                reading_m = dist_cm * 0.01f;  // convert cm to meters
                parsed_any = true;
            }
        }
    }

    return parsed_any;
}

void AP_RangeFinder_GRF250::update_init()
{
    const uint32_t now = AP_HAL::millis();

    if (grf.init_stage == INIT_STAGE_STREAMING) {
        return;
    }

    if (check_grf250_response()) {
        return;
    }

    uint8_t packet[32];

    switch (grf.init_stage) {
    case INIT_STAGE_IDLE:
        {
            uint8_t dummy = 0xFF;
            uart->write(&dummy, 1);
            grf.last_init_ms = now;
            grf.init_stage = INIT_STAGE_SENT_DUMMY;
        }
        break;
    case INIT_STAGE_SENT_DUMMY:
        if (now - grf.last_init_ms >= 100) {
            uint16_t len = build_packet(0, false, nullptr, 0, packet);
            uart->write(packet, len);
            grf.last_init_ms = now;
            grf.init_stage = INIT_STAGE_SENT_HANDSHAKE1;
        }
        break;
    case INIT_STAGE_SENT_HANDSHAKE1:
        if (now - grf.last_init_ms >= 200) {
            uint16_t len = build_packet(0, false, nullptr, 0, packet);
            uart->write(packet, len);
            grf.last_init_ms = now;
            grf.init_started_ms = now;
            grf.init_stage = INIT_STAGE_WAITING_RESPONSE;
        }
        break;
    case INIT_STAGE_WAITING_RESPONSE:
        if (now - grf.init_started_ms > 5000) {
            grf.init_stage = INIT_STAGE_IDLE;
            grf.parse_ofs = 0;
        }
        break;
    default:
        break;
    }
}

bool AP_RangeFinder_GRF250::get_reading(float &reading_m)
{
    if (uart == nullptr) {
        return false;
    }

    uint16_t bytes_available = MIN(uart->available(), 8192U);
    if (bytes_available > 0) {
        const uint16_t num_bytes_to_read = MIN(bytes_available, GRF250_BUFFER_SIZE - grf.parse_ofs);
        const uint16_t num_read = uart->read(&grf.parse_buffer[grf.parse_ofs], num_bytes_to_read);
        grf.parse_ofs += num_read;
    }

    move_preamble();
    update_init();

    if (!grf.product_verified) {
        return false;
    }

    return try_parse_stream_packet(reading_m);
}

#endif // AP_RANGEFINDER_GRF250_ENABLED
