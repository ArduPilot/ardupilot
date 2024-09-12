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
  Simulator for the RPLidarA2 proximity sensor
*/

#include "SIM_PS_LightWare_SF45B.h"

#if HAL_SIM_PS_LIGHTWARE_SF45B_ENABLED

#include <GCS_MAVLink/GCS.h>
#include <stdio.h>
#include <errno.h>

using namespace SITL;

uint32_t PS_LightWare_SF45B::packet_for_location(const Location &location,
                                           uint8_t *data,
                                           uint8_t buflen)
{
    return 0;
}

void PS_LightWare_SF45B::move_preamble_in_buffer(uint8_t search_start_pos)
{
    uint8_t i;
    for (i=search_start_pos; i<_buflen; i++) {
        if ((uint8_t)_msg.buffer[i] == PREAMBLE) {
            break;
        }
    }
    if (i == 0) {
        return;
    }
    memmove(_msg.buffer, &_msg.buffer[i], _buflen-i);
    _buflen = _buflen - i;
}

// handle a valid message in _msg.buffer
void PS_LightWare_SF45B::send(const char *data, uint32_t len)
{
    const ssize_t ret = write_to_autopilot(data, len);
    if (ret < 0 || (uint32_t)ret != len) {
        // abort();
    }
}

void PS_LightWare_SF45B::handle_message()
{
    // GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Got message %u", (unsigned)_msg.packed_msgstream.msg.msgid);
    switch (MessageID(_msg.packed_msgstream.msg.msgid)) {
    case MessageID::STREAM: {
        stream = _msg.packed_msgstream.msg.stream;
        send_response.stream = true;
        return;
    }
    case MessageID::DISTANCE_OUTPUT: {
        desired_fields = _msg.packed_distance_output.msg.desired_fields;
        send_response.desired_fields = true;
        return;
    }
    case MessageID::UPDATE_RATE: {
        update_rate = _msg.packed_update_rate.msg.rate;
        send_response.update_rate = true;
        return;
    }
    case MessageID::DISTANCE_DATA_CM: {
        ::fprintf(stderr, "Got a distance data.  Weird.\n");
        return;
    }
    }
//    AP_HAL::panic("Unrecognised message (%u)", _msg.common.msgid);
    ::fprintf(stderr, "Unrecognised message (%u)\n", _msg.packed_msgstream.msg.msgid);
}

void PS_LightWare_SF45B::update_input()
{
    const ssize_t n = read_from_autopilot(&_msg.buffer[_buflen], ARRAY_SIZE(_msg.buffer) - _buflen - 1);
    if (n < 0) {
        // TODO: do better here
        if (errno != EAGAIN && errno != EWOULDBLOCK && errno != 0) {
            AP_HAL::panic("Failed to read from autopilot");
        }
    } else {
        _buflen += n;
    }

    switch (_inputstate) {
    case InputState::WANT_PREAMBLE:
        move_preamble_in_buffer();
        if (_buflen == 0) {
            return;
        }
        set_inputstate(InputState::WANT_FLAGS);
        FALLTHROUGH;
    case InputState::WANT_FLAGS:
        if (_buflen < 4) {  // 1 preamble, 2 flags, 1 msg
            return;
        }
        set_inputstate(InputState::WANT_PAYLOAD);
        FALLTHROUGH;
    case InputState::WANT_PAYLOAD: {
        // 1 preamble, 2 flags, 1 msg + payload
        const uint8_t want_len = 4 + payload_length();
        if (_buflen < want_len) {
            return;
        }
        set_inputstate(InputState::WANT_CRC);
        FALLTHROUGH;
    }
    case InputState::WANT_CRC: {
        // 1 preamble, 2 flags, 1 msg + payload + 2 crc
        const uint8_t want_len = 4 + payload_length() + 2;
        if (_buflen < want_len) {
            return;
        }
        const uint16_t got_checksum = UINT16_VALUE(uint8_t(_msg.buffer[want_len-1]), uint8_t(_msg.buffer[want_len-2]));
        const uint16_t calc_checksum = msg_checksum();
        if (calc_checksum == got_checksum) {
            // GCS_SEND_TEXT(MAV_SEVERITY_INFO, "sim-SF45B: Got one (%u)!", _msg.common.msgid);
            // see if this was a read or a write request..
            handle_message();
        } else {
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "sim-SF45B: Bad checksum");
        }
        // consume these bytes
        move_preamble_in_buffer(want_len-1);
        set_inputstate(InputState::WANT_PREAMBLE);
        return;
    }
    }
}

void PS_LightWare_SF45B::update(const Location &location)
{
    update_input();
    update_output(location);
}

void PS_LightWare_SF45B::update_output(const Location &location)
{
    switch (_state) {
    case State::SCANNING:
        update_output_responses();
        update_output_scan(location);
        return;
    }
}

void PS_LightWare_SF45B::update_output_responses()
{
    if (send_response.stream) {
        send_response.stream = false;
        PackedMessage<MsgStream> packed { MsgStream(stream), 0x1 };
        packed.update_checksum();
        send((char*)&packed, sizeof(packed));
    }
    if (send_response.update_rate) {
        send_response.update_rate = false;
        PackedMessage<UpdateRate> packed { UpdateRate(update_rate), 0x1 };
        packed.update_checksum();
        send((char*)&packed, sizeof(packed));
    }
    if (send_response.desired_fields) {
        send_response.desired_fields = false;
        PackedMessage<DistanceOutput> packed { DistanceOutput(desired_fields), 0x1 };
        packed.update_checksum();
        send((char*)&packed, sizeof(packed));
    }
}

void PS_LightWare_SF45B::update_output_scan(const Location &location)
{
    const uint32_t now_ms = AP_HAL::millis();
    const uint32_t time_delta_ms = (now_ms - last_scan_output_time_ms);
    if (time_delta_ms > 1000) {
        last_scan_output_time_ms = now_ms;
        return;
    }

    const uint32_t samples_per_second = 133;
    const float samples_per_ms = samples_per_second / 1000.0f;
    const uint32_t sample_count = samples_per_ms * time_delta_ms;
    const float degrees_per_ms = 390 / 1000.0f;
    const float degrees_per_sample = degrees_per_ms / samples_per_ms;

//    ::fprintf(stderr, "Packing %u samples in for %ums interval (%f degrees/sample)\n", sample_count, time_delta, degrees_per_sample);

    last_scan_output_time_ms += sample_count/samples_per_ms;

    for (uint32_t i=0; i<sample_count; i++) {

        const float ANGLE_MIN_DEG = -170;
        const float ANGLE_MAX_DEG = +170;
        float current_degrees_bf = last_degrees_bf + (last_dir * degrees_per_sample);
        if (current_degrees_bf < ANGLE_MIN_DEG) {
            current_degrees_bf += (ANGLE_MIN_DEG - current_degrees_bf);
            last_dir = -last_dir;
        }
        if (current_degrees_bf > ANGLE_MAX_DEG) {
            current_degrees_bf += (ANGLE_MAX_DEG - current_degrees_bf);
            last_dir = -last_dir;
        }
        last_degrees_bf = current_degrees_bf;


        const float MAX_RANGE = 53.0f;
        float distance = measure_distance_at_angle_bf(location, current_degrees_bf);
        // ::fprintf(stderr, "SIM: %f=%fm\n", current_degrees_bf, distance);
        if (distance > MAX_RANGE) {
            // sensor returns -1 for out-of-range
            distance = -1.0f;
        }

        PackedMessage<DistanceDataCM> packed_distance_data {
            DistanceDataCM(
                uint16_t(distance*100.0),
                wrap_180(current_degrees_bf) * 100
                ), 0x1 };
        packed_distance_data.update_checksum();
        send((char*)&packed_distance_data, sizeof(packed_distance_data));
    }
}

#endif  // HAL_SIM_PS_LIGHTWARE_SF45B_ENABLED
