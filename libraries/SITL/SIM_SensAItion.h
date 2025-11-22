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

  Supports two modes:
  - IMU mode: 1000Hz (38 bytes/packet = 304kbps)
  - AHRS mode: 500Hz (54 bytes/packet = 216kbps)

  Usage example:
     SERIAL4_PROTOCOL = 36
     SERIAL4_BAUD = 921600
     EAHRS_TYPE = 11

     sim_vehicle.py -D --console --map -A "--serial4=sim:SensAItion"
*/

#pragma once

#include "SIM_Aircraft.h"

#include <SITL/SITL.h>
#include "SIM_SerialDevice.h"

namespace SITL
{

class SensAItion : public SerialDevice
{
public:

    SensAItion();

    // update state
    void update(void);

private:
    uint32_t last_imu_pkt_us;    // Timing for IMU packets (1000Hz)
    uint32_t last_ahrs_pkt_us;   // Timing for AHRS packets (500Hz)

    void send_imu_packet();      // Generate IMU packet (9 sensors × 4 bytes = 36 bytes + header + checksum)
    void send_ahrs_packet();     // Generate AHRS packet (13 sensors × 4 bytes = 52 bytes + header + checksum)
    uint8_t calculate_xor_checksum(const uint8_t* data, uint16_t length);
};

}