/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/
#include "SIM_config.h"

#if AP_SIM_RF_LIGHTWAR_GRF250_ENABLED

#include "SIM_RF_LightWare_GRF.h"
#include <AP_HAL/AP_HAL.h>
#include <string.h>

using namespace SITL;

#define LIGHTWARE_GRF250_PREAMBLE 0xAA

uint16_t RF_LightWareGRF::compute_crc(const uint8_t *buf, uint8_t len)
{
    uint16_t crc = 0;
    for (uint8_t i = 0; i < len; i++) {
        crc = crc_xmodem_update(crc, buf[i]);
    }
    return crc;
}

void RF_LightWareGRF::process_input()
{
    char buffer[50] {};

    ssize_t n = read_from_autopilot(buffer, sizeof(buffer));

    if (n <= 0) {
        return;
    }

    for (ssize_t i = 0; i < n; i++) {
        if (rxlen < sizeof(rxbuf)) {
            rxbuf[rxlen++] = (uint8_t)buffer[i];
        }
    }

    // Try to decode messages in rxbuf[]
    try_parse_message();
}

void RF_LightWareGRF::try_parse_message()
{
    // Need at least preamble + flags + msgid
    if (rxlen < 6) {
        return;
    }

    // find preamble in buffer
    uint8_t *p = (uint8_t*)memchr(rxbuf, LIGHTWARE_GRF250_PREAMBLE, rxlen);
    if (!p) {
        // No preamble in the entire buffer, clear buffer
        rxlen = 0;
        return;
    }

    // If preamble is not at index 0, slide data to front
    uint8_t offset = p - rxbuf;
    if (offset > 0) {
        memmove(rxbuf, p, rxlen - offset);
        rxlen -= offset;

        // Need at least 6 bytes again
        if (rxlen < 6) {
            return;
        }
    }

    // Extract flags and payload length
    uint16_t flags = (rxbuf[1] | (rxbuf[2] << 8));
    uint8_t plen = ((flags >> 6) - 1);

    if (plen > 100) {
        // invalid payload, drop preamble and retry
        memmove(rxbuf, rxbuf + 1, --rxlen);
        return;
    }

    // Full message length: preamble + flags(2) + msgid + payload + crc(2)
    const uint8_t need = 1 + 2 + 1 + plen + 2;

    if (rxlen <= need) {
        // Not enough bytes yet
        return;
    }

    const uint16_t gotcrc = uint16_t(uint8_t(rxbuf[need-2])) |
                  (uint16_t(uint8_t(rxbuf[need-1])) << 8);
    const uint16_t calcrc = compute_crc(rxbuf, need - 2);

    if (gotcrc == calcrc) {

        const MsgID msgid = (MsgID)rxbuf[3];
        switch (msgid) {

        case MsgID::PRODUCT_NAME:
            send_product_name();
            break;

        case MsgID::UPDATE_RATE: {
            uint8_t rate = rxbuf[4];
            if (rate == 0) {
                rate = 1;
            }
            update_period_ms = 1000 / rate;
            send_ack_u32(MsgID::UPDATE_RATE, (uint32_t)rate);
            break;
        }

        case MsgID::DISTANCE_OUTPUT: {
            uint32_t desired_fields;
            memcpy(&desired_fields, &rxbuf[4], 4);
            send_ack_u32(MsgID::DISTANCE_OUTPUT, desired_fields);
            break;
        }

        case MsgID::STREAM: {
            uint32_t stream;
            memcpy(&stream, &rxbuf[4], 4);
            if (stream == 5) {
                // only "5" (distance cm stream) is supported for now
                stream_enabled = true;
            } else {
                stream_enabled = false;
                break;
            }
            send_ack_u32(MsgID::STREAM, stream);
            break;
        }

        default:
            break;
        }
    }

    uint8_t remaining = rxlen - need;
    memmove(rxbuf, rxbuf + need, remaining);
    rxlen = remaining;

    if (rxlen >= 4) {
        try_parse_message();
    }
}

void RF_LightWareGRF::send_product_name()
{
    uint8_t payload[7] = {'G','R','F','2','5','0',0};
    uint8_t msg[32];
    uint8_t len = 0;

    msg[len++] = LIGHTWARE_GRF250_PREAMBLE;

    uint16_t flags = 0x1;
    flags |= (1 + sizeof(payload)) << 6;
    msg[len++] = flags & 0xFF;
    msg[len++] = (flags >> 8) & 0xFF;

    msg[len++] = (uint8_t)MsgID::PRODUCT_NAME;

    memcpy(&msg[len], payload, sizeof(payload));
    len += sizeof(payload);

    uint16_t crc = compute_crc(msg, len);
    msg[len++] = crc & 0xFF;
    msg[len++] = crc >> 8;

    write_to_autopilot((char*)msg, len);
}

void RF_LightWareGRF::send_ack_u8(MsgID id, uint8_t value)
{
    uint8_t msg[16];
    uint8_t len = 0;

    msg[len++] = LIGHTWARE_GRF250_PREAMBLE;

    // payload_len = 1 (msgid) + 1 (value) = 2
    uint16_t payload_len = 2;

    uint16_t flags = 0x1;
    flags |= (payload_len << 6);

    msg[len++] = flags & 0xFF;       // FLAGS_L
    msg[len++] = (flags >> 8) & 0xFF; // FLAGS_H

    msg[len++] = (uint8_t)id;  // MSG_ID
    msg[len++] = value;        // PAYLOAD

    uint16_t crc = compute_crc(msg, len);

    msg[len++] = crc & 0xFF;
    msg[len++] = (crc >> 8) & 0xFF;

    write_to_autopilot((char*)msg, len);
}

void RF_LightWareGRF::send_ack_u32(MsgID id, uint32_t value)
{
    uint8_t msg[24];
    uint8_t len = 0;

    msg[len++] = LIGHTWARE_GRF250_PREAMBLE;

    uint16_t flags = 0x1;
    flags |= (1 + 4) << 6;    // msgid + 4 byte payload
    msg[len++] = flags & 0xFF;
    msg[len++] = flags >> 8;

    msg[len++] = (uint8_t)id;

    memcpy(&msg[len], &value, 4);
    len += 4;

    uint16_t crc = compute_crc(msg, len);
    msg[len++] = crc & 0xFF;
    msg[len++] = crc >> 8;

    write_to_autopilot((char*)msg, len);
}

void RF_LightWareGRF::build_distance_packet(float alt_m, uint8_t *out, uint32_t &outlen)
{
    float dist_m = alt_m;
    uint32_t strength = 100; // no way to get this from the sim

    // simulate out-of-range
    if (dist_m < 0.2) {
        dist_m = 0.2;
        strength = 0;
    }
    if (dist_m > 500) {
        dist_m = 500;
        strength = 0;
    }

    /*
       For some reason the GRF expects the distance in cm, but sends it in weird units where:
         raw = UINT32(...)
         distance_cm = raw * 10

       So we must send:
         raw = (distance_cm / 10)
    */

    const uint32_t cm = uint32_t(dist_m * 100.0f);
    uint32_t raw = cm / 10;

    uint8_t len = 0;

    out[len++] = LIGHTWARE_GRF250_PREAMBLE;

    uint16_t flags = 0x1;
    flags |= (1 + 8) << 6;  // msgid + 8 byte payload
    out[len++] = flags & 0xFF;
    out[len++] = flags >> 8;

    out[len++] = (uint8_t)MsgID::DISTANCE_DATA_CM;

    // technically this could be filtered/raw and first/last distance, but we just send the raw
    memcpy(&out[len], &raw, 4);
    len += 4;
    memcpy(&out[len], &strength, 4);
    len += 4;

    uint16_t crc = compute_crc(out, len);
    out[len++] = crc & 0xFF;
    out[len++] = crc >> 8;

    outlen = len;
}

uint32_t RF_LightWareGRF::packet_for_alt(float alt_m,
                                                  uint8_t *buffer,
                                                  uint8_t buflen)
{
    // Always read incoming configuration messages
    process_input();

    // Only transmit distance if stream has been configured (this is how the real GRF works)
    if (!stream_enabled) {
        return 0;
    }

    if (buflen < 16) {
        return 0;
    }

    uint32_t outlen = 0;
    build_distance_packet(alt_m, buffer, outlen);
    return outlen;
}

#endif // AP_SIM_RF_LIGHTWAR_GRF250_ENABLED
