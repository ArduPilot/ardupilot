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
  Simulator for the LD06 proximity sensors
*/

#include "SIM_config.h"

#if AP_SIM_PS_LD06_ENABLED

#include "SIM_PS_LD06.h"

#include <GCS_MAVLink/GCS.h>
#include <AP_BoardConfig/AP_BoardConfig.h>
#include <stdio.h>
#include <errno.h>
#include <AP_HAL/utility/sparse-endian.h>

using namespace SITL;

uint32_t PS_LD06::packet_for_location(const Location &location,
                                      uint8_t *data,
                                      uint8_t buflen)
{
    return 0;
}

void PS_LD06::update_input()
{
    // just discard any input
    char buffer[256];
    uint8_t buflen = 0;
    read_from_autopilot(&buffer[buflen], ARRAY_SIZE(buffer) - buflen - 1);
}

void PS_LD06::update_output(const Location &location)
{
    const uint32_t now_ms = AP_HAL::millis();
    if (last_scan_output_time_ms == 0) {
        last_scan_output_time_ms = now_ms;
        return;
    }
    const uint32_t time_delta_ms = (now_ms - last_scan_output_time_ms);

    // the driver and device only send/handle 12 readings at a time at
    // the moment
    const uint32_t sample_count = 12;

    const uint32_t samples_per_second = 4500;
    const float samples_per_ms = samples_per_second / 1000.0f;

    const uint32_t required_time_delta_ms = sample_count * samples_per_ms;

    if (time_delta_ms < required_time_delta_ms) {
        return;
    }
    // sanity check we're being called often enough:
    if (time_delta_ms / samples_per_ms > sample_count) {
        AP_HAL::panic("Not being called often enough");
    }

    const float degrees_per_s = 2152;  // see example datasheet page 8
    const float degrees_per_ms = degrees_per_s / 1000.0f;
    const float degrees_per_sample = degrees_per_ms / samples_per_ms;

    // ::fprintf(stderr, "Packing %u samples in for %ums interval (%f degrees/sample)\n", sample_count, time_delta_ms, degrees_per_sample);

    last_scan_output_time_ms += sample_count/samples_per_ms;

    const uint32_t required_send_buffer_size = 11 + 3*sample_count;
    if (required_send_buffer_size > send_buffer_size) {
        if (send_buffer != nullptr) {
            free(send_buffer);
        }
        send_buffer = (uint8_t*)malloc(required_send_buffer_size);
        if (send_buffer == nullptr) {
            AP_BoardConfig::allocation_error("LD06 send buffer");
        }
        send_buffer_size = required_send_buffer_size;
    }

    struct PACKED PacketStart {
        uint8_t preamble;
        uint8_t length : 5;
        uint8_t reserved : 3;
        uint16_t angular_rate;  // degrees/second
        uint16_t start_angle;  // centidegrees
    };
    struct PACKED Measurement {
        uint16_t distance;
        uint8_t confidence;
    };
    struct PACKED PacketEnd {
        uint16_t end_angle;
        uint16_t timestamp;
        uint8_t crc;
    };

    PacketStart &packet_start = *((PacketStart*)send_buffer);

    packet_start.preamble = 0x54;
    packet_start.length = sample_count;
    packet_start.angular_rate = htole16(degrees_per_s);
    packet_start.start_angle = htole16(last_degrees_bf * 100);

    uint16_t offset = sizeof(PacketStart);

    for (uint32_t i=0; i<sample_count; i++) {
        const float current_degrees_bf = fmod((last_degrees_bf + degrees_per_sample), 360.0f);
        last_degrees_bf = current_degrees_bf;

        float distance = measure_distance_at_angle_bf(location, current_degrees_bf);
        // ::fprintf(stderr, "SIM: %f=%fm\n", current_degrees_bf, distance);

        uint8_t confidence = 29; // random number; driver checks this
        if (distance > 12.0) {  // 12 metre max range
            // return 0 for out-of-range; is this correct?
            distance = 0.0f;
            confidence = 0;
        }

        Measurement &measurement = *((Measurement*)&send_buffer[offset]);

        measurement.distance = htole16(distance * 1000);  // m -> mm
        measurement.confidence = confidence;

        offset += sizeof(Measurement);
    }

    PacketEnd &packet_end = *((PacketEnd*)&send_buffer[offset]);

    packet_end.end_angle = htole16(last_degrees_bf * 100);  // centidegrees
    packet_end.timestamp = htole16(0);  // milliseconds
    offset += sizeof(PacketEnd);

    packet_end.crc = crc8_generic(&send_buffer[0], offset-1, 0x4D);

    const ssize_t ret = write_to_autopilot((const char*)send_buffer, offset);
    if (ret != offset) {
        // abort();  // there are startup issues with this, we fill things up before the driver is ready
    }
}

void PS_LD06::update(const Location &location)
{
    update_input();
    update_output(location);
}

#endif  // AP_SIM_PS_LD06_ENABLED
