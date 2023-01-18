/*
  ESC Telemetry for Hobbywing XRotor v4 ESC

  This protocol only allows for one ESC per UART RX line, so using a
  CAN node per ESC works well.

  See https://github.com/dgatf/msrc#9-annex

 */
#include "AP_HobbyWing_ESC_XRotor_v4.h"

#if AP_HOBBYWING_PLATINUM_V4_ENABLED

#include <AP_HAL/utility/sparse-endian.h>
#include <AP_HAL/AP_HAL.h>
#include <GCS_MAVLink/GCS.h>

void AP_HobbyWing_XRotor_v4::update()
{
    struct PACKED {
        uint8_t header; // 0x9B
        uint8_t pkt_len; // 0x16
        uint32_t counter;
        uint16_t throttle_req;
        uint16_t throttle;
        uint16_t rpm;
        uint16_t voltage;
        int16_t current;
        int16_t phase_current;
        uint8_t mos_temperature;
        uint8_t cap_temperature;
        uint16_t status;
        uint16_t crc;

        uint16_t calc_checksum() const {
            uint16_t ret = 0;
            for (uint8_t i=0; i<sizeof(*this)-2; i++) {
                ret += ((uint8_t*)this)[i];
            }
            return ret;
        };

    } packet;

    if (!read_uart_v345((uint8_t*)&packet, sizeof(packet), 10000)) {
        return;
    }

    // check for valid frame header - and valid packet length
    if (packet.header != 0x9b || packet.pkt_len != 0x16) {
        // bad header byte after a frame gap
        // gcs().send_text(MAV_SEVERITY_INFO, "Bad header");
        return;
    }

    if (packet.calc_checksum() != packet.crc) {
        // checksum failure
        return;
    }

    // extract packet sequence number and update count of dropped frames
    check_seq(packet.counter);

    WITH_SEMAPHORE(decoded_sem);

    // decoded.counter = be32toh(pkt.counter);
    // decoded.throttle_req = be16toh(pkt.throttle_req);
    // decoded.throttle = be16toh(pkt.throttle);
    decoded.rpm = packet.rpm / motor_poles;
    decoded.voltage = be16toh(packet.voltage) * 0.1;
    decoded.phase_current = int16_t(be16toh(packet.phase_current)) * 0.01;
    decoded.current = int16_t(be16toh(packet.current)) * 0.01;
    decoded.temperature = MAX(AP_ESC_Telem_Backend::temperature_decode(packet.mos_temperature),
                              AP_ESC_Telem_Backend::temperature_decode(packet.cap_temperature));
    const uint16_t status = be16toh(packet.status);
    if (status != 0) {
        decoded.error_count++;
    }

    decoded_received_us = AP_HAL::micros();
}

#endif  // AP_HOBBYWING_PLATINUM_PRO_V4_ENABLED

