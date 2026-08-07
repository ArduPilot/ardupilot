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
  Simulate SensAItion serial IMU/AHRS device
  Generates high-rate sensor data for ArduPilot testing

*/

#pragma once

#include "SIM_SerialDevice.h"
#include <AP_Common/AP_Common.h>

namespace SITL
{

class SensAItion : public SerialDevice
{
public:
    SensAItion(bool interleved_mode);
    void update(void);

private:
    int _tick = 0;
    int _periodMessage0 = 5;
    int _phaseMessage0 = 0;
    int _periodMessage1 = 100;
    int _phaseMessage1 = 0;
    int _periodMessage2 = 100;
    int _phaseMessage2 = 5;

    uint8_t _buffert[512];
    int _buffert_cnt = 0;


    void send_packet_0_imu(const struct sitl_fdm &fdm);
    void send_packet_1_orientation(const struct sitl_fdm &fdm);
    void send_packet_2_ins(const struct sitl_fdm &fdm);
    uint32_t calculate_itow(uint64_t now_us, uint32_t start_time_utc);


    void flush_packets();
    void write_to_autopilot_buf(const char *data, int length);

    void write_packet(uint8_t msg_id, const uint8_t* payload, uint16_t length);
    void write_legacy_packet(const uint8_t* payload, uint16_t length);
    uint16_t calculate_crc(uint8_t msg_id, const uint8_t* payload, uint16_t length, bool use_id);

    uint32_t last_update_us = 0;
    uint32_t tick_count = 0;

    bool _interleaved_mode = false; // Now mutable, Shall be driven by EAHRS_OPTIONS. TODO
};

} // namespace SITL
