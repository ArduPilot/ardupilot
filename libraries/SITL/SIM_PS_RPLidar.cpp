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
/*
  Simulator for the RPLidar proximity sensors
*/

#include "SIM_PS_RPLidar.h"

#if AP_SIM_PS_RPLIDARA2_ENABLED || AP_SIM_PS_RPLIDARA1_ENABLED || AP_SIM_PS_RPLIDARS2_ENABLED

#include <GCS_MAVLink/GCS.h>
#include <stdio.h>
#include <errno.h>
#include <math.h>

using namespace SITL;

uint32_t PS_RPLidar::packet_for_location(const Location &location,
                                           uint8_t *data,
                                           uint8_t buflen)
{
    return 0;
}

void PS_RPLidar::move_preamble_in_buffer()
{
    uint8_t i;
    for (i=0; i<_buflen; i++) {
        if ((uint8_t)_buffer[i] == PREAMBLE) {
            break;
        }
    }
    if (i == 0) {
        return;
    }
    memmove(_buffer, &_buffer[i], _buflen-i);
    _buflen = _buflen - i;
}

void PS_RPLidar::update_input()
{
    const ssize_t n = read_from_autopilot(&_buffer[_buflen], ARRAY_SIZE(_buffer) - _buflen - 1);
    if (n < 0) {
        // TODO: do better here
        if (errno != EAGAIN && errno != EWOULDBLOCK && errno != 0) {
            AP_HAL::panic("Failed to read from autopilot");
        }
    } else {
        _buflen += n;
    }

    switch (_inputstate) {
    case InputState::WAITING_FOR_PREAMBLE:
        move_preamble_in_buffer();
        if (_buflen == 0) {
            return;
        }
        set_inputstate(InputState::GOT_PREAMBLE);
        // consume the preamble:
        memmove(_buffer, &_buffer[1], _buflen-1);
        _buflen--;
        FALLTHROUGH;
    case InputState::GOT_PREAMBLE:
        if (_buflen == 0) {
            return;
        }
        switch ((Command)_buffer[0]) {
        case Command::STOP:
            // consume the command:
            memmove(_buffer, &_buffer[1], _buflen-1);
            _buflen--;
            set_inputstate(InputState::WAITING_FOR_PREAMBLE);
            set_state(State::IDLE);
            _scan_mode = ScanMode::SCAN;
            return;
        case Command::SCAN:
            // 5-byte scan mode
            memmove(_buffer, &_buffer[1], _buflen-1);
            _buflen--;
            send_response_descriptor(0x05, SendMode::SRMR, DataType::Unknown81);
            set_inputstate(InputState::WAITING_FOR_PREAMBLE);
            set_state(State::SCANNING);
            _scan_mode = ScanMode::SCAN;
            last_scan_output_time_ms = 0;
            last_degrees_bf = 0.0f;
            return;
        case Command::EXPRESS_SCAN:
            // Dense Express mode (40 samples / packet)
            //
            // Request is: A5 82 <payload_size=0x05> <5-byte payload> <checksum>
            // Descriptor: A5 5A 54 00 00 40 85  (len=0x54=84 bytes, type=0x85)
            memmove(_buffer, &_buffer[1], _buflen-1);
            _buflen--;
            if (_buflen >= 1) {
                const uint8_t payload_size = (uint8_t)_buffer[0];
                const uint8_t total_to_discard = 1 + payload_size + 1; // size + payload + checksum
                if (_buflen >= total_to_discard) {
                    memmove(_buffer, &_buffer[total_to_discard], _buflen - total_to_discard);
                    _buflen -= total_to_discard;
                } else {
                    _buflen = 0;
                }
            } else {
                _buflen = 0;
            }
            send_response_descriptor(0x54, SendMode::SRMR, DataType::Unknown85);
            set_inputstate(InputState::WAITING_FOR_PREAMBLE);
            set_state(State::SCANNING);
            _scan_mode = ScanMode::EXPRESS_SCAN_DENSE;
            _express_w_i_deg = 0.0f;
            last_scan_output_time_ms = 0;
            return;
        case Command::GET_HEALTH: {
            // consume the command:
            memmove(_buffer, &_buffer[1], _buflen-1);
            _buflen--;
            send_response_descriptor(0x03, SendMode::SRSR, DataType::Unknown06);
            // now send the health:
            const uint8_t health[3] {}; // all zeros fine for now
            const ssize_t ret = write_to_autopilot((const char*)health, ARRAY_SIZE(health));
            if (ret != ARRAY_SIZE(health)) {
                abort();
            }
            set_inputstate(InputState::WAITING_FOR_PREAMBLE);
            return;
        }
        case Command::GET_DEVICE_INFO: {
            // consume the command:
            memmove(_buffer, &_buffer[1], _buflen-1);
            _buflen--;
            send_response_descriptor(0x14, SendMode::SRSR, DataType::Unknown04);
            // now send the device info:
            struct PACKED _device_info {
                uint8_t model;
                uint8_t firmware_minor;
                uint8_t firmware_major;
                uint8_t hardware;
                uint8_t serial[16];
            } device_info;
            device_info.model = device_info_model();
            device_info.firmware_minor = 17;
            device_info.firmware_major = 42;
            device_info.hardware = 6;
            const ssize_t ret = write_to_autopilot((const char*)&device_info, sizeof(device_info));
            if (ret != sizeof(device_info)) {
                abort();
            }
            set_inputstate(InputState::WAITING_FOR_PREAMBLE);
            return;
        }
        case Command::FORCE_SCAN:
            abort();
        case Command::RESET:
            // consume the command:
            memmove(_buffer, &_buffer[1], _buflen-1);
            _buflen--;
            set_inputstate(InputState::RESETTING_START);
            return;
        default:
            const uint8_t bad = static_cast<uint8_t>(_buffer[0]);
            ::fprintf(stderr, "SIM_RPLidar: unknown command 0x%02x, ignoring\n", bad);
            // consume this byte and resync:
            memmove(_buffer, &_buffer[1], _buflen-1);
            _buflen--;
            set_inputstate(InputState::WAITING_FOR_PREAMBLE);
            return;
        }
    case InputState::RESETTING_START:
        _firmware_info_offset = 0;
        set_inputstate(InputState::RESETTING_SEND_FIRMWARE_INFO);
        FALLTHROUGH;
    case InputState::RESETTING_SEND_FIRMWARE_INFO: {
        const ssize_t written = write_to_autopilot(&FIRMWARE_INFO[_firmware_info_offset], strlen(FIRMWARE_INFO) - _firmware_info_offset);
        if (written <= 0) {
            AP_HAL::panic("Failed to write to autopilot");
        }
        _firmware_info_offset += written;
        if (_firmware_info_offset < strlen(FIRMWARE_INFO)) {
            return;
        }
        set_inputstate(InputState::WAITING_FOR_PREAMBLE);
        return;
    }
    }
}

void PS_RPLidar::update_output_scan(const Location &location)
{
    const uint32_t now = AP_HAL::millis();
    if (last_scan_output_time_ms == 0) {
        last_scan_output_time_ms = now;
        return;
    }
    const uint32_t time_delta = (now - last_scan_output_time_ms);

    const uint32_t samples_per_second = 1000;
    const float samples_per_ms = samples_per_second / 1000.0f;
    const uint32_t sample_count = time_delta / samples_per_ms;
    const float degrees_per_ms = 3600 / 1000.0f;
    const float degrees_per_sample = degrees_per_ms / samples_per_ms;

//    ::fprintf(stderr, "Packing %u samples in for %ums interval (%f degrees/sample)\n", sample_count, time_delta, degrees_per_sample);

    last_scan_output_time_ms += sample_count/samples_per_ms;

    for (uint32_t i=0; i<sample_count; i++) {
        const float current_degrees_bf = fmodf((last_degrees_bf + degrees_per_sample), 360.0f);
        const uint8_t quality = 17; // random number
        const uint16_t angle_q6 = current_degrees_bf * 64;
        const bool is_start_packet = current_degrees_bf < last_degrees_bf;
        last_degrees_bf = current_degrees_bf;


        float distance = measure_distance_at_angle_bf(location, current_degrees_bf);
        // ::fprintf(stderr, "SIM: %f=%fm\n", current_degrees_bf, distance);
        if (distance > max_range()) {
            // sensor returns zero for out-of-range
            distance = 0.0f;
        }
        const uint16_t distance_q2 = (distance*1000 * 4); // m->mm and *4

        struct PACKED  {
            uint8_t startbit      : 1;            ///< on the first revolution 1 else 0
            uint8_t not_startbit  : 1;            ///< complementary to startbit
            uint8_t quality       : 6;            ///< Related the reflected laser pulse strength
            uint8_t checkbit      : 1;            ///< always set to 1
            uint16_t angle_q6     : 15;           ///< Actual heading = angle_q6/64.0 Degree
            uint16_t distance_q2  : 16;           ///< Actual Distance = distance_q2/4.0 mm
        } send_buffer;
        send_buffer.startbit = is_start_packet;
        send_buffer.not_startbit = !is_start_packet;
        send_buffer.quality = quality;
        send_buffer.checkbit = 1;
        send_buffer.angle_q6 = angle_q6;
        send_buffer.distance_q2 = distance_q2;

        static_assert(sizeof(send_buffer) == 5, "send_buffer correct size");

        const ssize_t ret = write_to_autopilot((const char*)&send_buffer, sizeof(send_buffer));
        if (ret != sizeof(send_buffer)) {
            abort();
        }
    }
}

void PS_RPLidar::update_output_express_dense(const Location &location)
{
    const uint32_t now = AP_HAL::millis();
    if (last_scan_output_time_ms == 0) {
        last_scan_output_time_ms = now;
        return;
    }

    // Number of dense packets per second (tunable).
    // 80 packets/s -> 3200 samples/s (40 samples/packet).
    const uint32_t packets_per_second = 80;
    const uint32_t packet_period_ms = 1000 / packets_per_second;

    const uint32_t time_delta = (now - last_scan_output_time_ms);
    const uint32_t packet_count = time_delta / packet_period_ms;

    if (packet_count == 0) {
        return;
    }

    last_scan_output_time_ms += packet_count * packet_period_ms;

    // 0.1125 degree per sample -> 4.5 degrees per packet
    const float sample_delta_deg = 0.1125f;
    const float packet_delta_deg = 40.0f * sample_delta_deg;

    for (uint32_t p = 0; p < packet_count; p++) {
        uint8_t packet[84] {};
        // Start angle in degrees for this packet
        const float w_i_deg = _express_w_i_deg;

        // start_angle_q6 is in 1/64 degrees
        const uint16_t start_angle_q6 = (uint16_t)lrintf(w_i_deg * 64.0f);

        packet[2] = start_angle_q6 & 0xFF;
        packet[3] = (start_angle_q6 >> 8) & 0xFF;

        // cabins: 40 distances (2 bytes each, little-endian)
        uint8_t* cab = &packet[4];
        for (uint32_t k = 0; k < 40; k++) {
            // angle of sample k
            float angle_deg = w_i_deg + sample_delta_deg * k;
            // wrap to [0, 360)
            if (angle_deg >= 360.0f) {
                angle_deg = fmodf(angle_deg, 360.0f);
            }

            float distance = measure_distance_at_angle_bf(location, angle_deg);
            if (distance > max_range()) {
                distance = 0.0f;
            }

            const uint16_t dist_mm = (uint16_t)lrintf(distance * 1000.0f);
            cab[0] = dist_mm & 0xFF;
            cab[1] = (dist_mm >> 8) & 0xFF;
            cab += 2;
        }

        // checksum: XOR of bytes [2..83]
        uint8_t checksum = 0;
        for (uint32_t i = 2; i < sizeof(packet); i++) {
            checksum ^= packet[i];
        }

        // sync1/sync2 and checksum nibbles (dense format)
        const uint8_t sync1 = 0x0A;
        const uint8_t sync2 = 0x05;

        packet[0] = uint8_t((sync1 << 4) | (checksum & 0x0F));
        packet[1] = uint8_t((sync2 << 4) | ((checksum >> 4) & 0x0F));

        const ssize_t ret = write_to_autopilot((const char*)packet, sizeof(packet));
        if (ret != (ssize_t)sizeof(packet)) {
            abort();
        }

        // advance start angle for next packet
        _express_w_i_deg += packet_delta_deg;
        if (_express_w_i_deg >= 360.0f) {
            _express_w_i_deg = fmodf(_express_w_i_deg, 360.0f);
        }
    }
}

void PS_RPLidar::update_output(const Location &location)
{
    switch (_state) {
    case State::IDLE:
        return;
    case State::SCANNING:
        if (_scan_mode == ScanMode::SCAN) {
            update_output_scan(location);
        } else {
            update_output_express_dense(location);
        }
        return;
    }
}

void PS_RPLidar::update(const Location &location)
{
    update_input();
    update_output(location);
}


void PS_RPLidar::send_response_descriptor(uint32_t data_response_length, SendMode sendmode, DataType datatype)
{
    const uint8_t send_buffer[] = {
        0xA5,
        0x5A,
        uint8_t((data_response_length >> 0) & 0xff),
        uint8_t((data_response_length >>  8) & 0xff),
        uint8_t((data_response_length >> 16) & 0xff),
        uint8_t(((data_response_length >> 24) & 0xff) | (uint8_t) sendmode),
        (uint8_t)datatype
    };
    static_assert(ARRAY_SIZE(send_buffer) == 7, "send_buffer correct size");

    const ssize_t ret = write_to_autopilot((const char*)send_buffer, ARRAY_SIZE(send_buffer));
    if (ret != ARRAY_SIZE(send_buffer)) {
        abort();
    }
}

#endif  // AP_SIM_PS_RPLIDARA2_ENABLED || AP_SIM_PS_RPLIDARA1_ENABLED || AP_SIM_PS_RPLIDARS2_ENABLED
