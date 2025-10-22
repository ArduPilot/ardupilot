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
  Simulate SensAItion serial IMU device
  Converts SITL flight dynamics to SensAItion protocol packets
*/

#include "SIM_SensAItion.h"
#include <inttypes.h>
#include <stdio.h>
#include <sys/time.h>
#include <unistd.h>
#include <fcntl.h>
#include <AP_HAL/utility/sparse-endian.h>

using namespace SITL;

SensAItion::SensAItion() :
    SerialDevice::SerialDevice()
{
    last_imu_pkt_us = 0;
}

// SensAItion packet data structures (header and checksum sent separately)
// IMU packet: 36 bytes data (accel 12B + gyro 12B + temp 2B + mag 6B + baro 4B)
struct PACKED SensAItion_IMU_packet {
    int32_t accel_x;        // Bytes 0-3: µg units (1e-6 g)
    int32_t accel_y;        // Bytes 4-7: µg units
    int32_t accel_z;        // Bytes 8-11: µg units
    int32_t gyro_x;         // Bytes 12-15: µdeg/s units (1e-6 deg/s)
    int32_t gyro_y;         // Bytes 16-19: µdeg/s units
    int32_t gyro_z;         // Bytes 20-23: µdeg/s units
    int16_t temperature;    // Bytes 24-25: special formula (temp_c - 20) / 0.008
    int16_t mag_x;          // Bytes 26-27: mGauss units
    int16_t mag_y;          // Bytes 28-29: mGauss units
    int16_t mag_z;          // Bytes 30-31: mGauss units
    int32_t baro;           // Bytes 32-35: 0.1 Pa units
};


uint8_t SensAItion::calculate_xor_checksum(const uint8_t* data, uint16_t length)
{
    uint8_t checksum = 0;
    for (uint16_t i = 0; i < length; i++) {
        checksum ^= data[i];
    }
    return checksum;
}

// Main update loop: Generate packets at configured rates
void SensAItion::update(void)
{
    if (!init_sitl_pointer()) {
        return;
    }

    const uint32_t now_us = AP_HAL::micros();

    // Send IMU packets at 1000Hz (1ms intervals)
    // Used for IMU-only mode testing
    if (now_us - last_imu_pkt_us >= 1000) {
        send_imu_packet();
        last_imu_pkt_us = now_us;
    }
}

void SensAItion::send_imu_packet(void)
{
    const auto &fdm = _sitl->state;

    // Convert from SI units to SensAItion units
    // Acceleration: m/s² to µg (1e-6 g)
    const float gravity = 9.80665f;
    int32_t accel_x = (int32_t)(fdm.xAccel / gravity * 1e6);
    int32_t accel_y = (int32_t)(fdm.yAccel / gravity * 1e6);
    int32_t accel_z = (int32_t)(fdm.zAccel / gravity * 1e6);

    // Angular rates: deg/s to µdeg/s (1e-6 deg/s)
    int32_t gyro_x = (int32_t)(fdm.rollRate * 1e6);
    int32_t gyro_y = (int32_t)(fdm.pitchRate * 1e6);
    int32_t gyro_z = (int32_t)(fdm.yawRate * 1e6);

    // Temperature: special conversion formula
    float temp_c = 25.0f;
    int16_t temperature = (int16_t)((temp_c - 20.0f) / 0.008f);

    // Magnetometer: Gauss to mGauss
    int16_t mag_x = (int16_t)(fdm.bodyMagField.x * 1000);
    int16_t mag_y = (int16_t)(fdm.bodyMagField.y * 1000);
    int16_t mag_z = (int16_t)(fdm.bodyMagField.z * 1000);

    // Barometer: Calculate pressure from altitude (Pa to 0.1 Pa units)
    const float pressure_pa = AP_Baro::get_pressure_for_alt_amsl(fdm.altitude);
    int32_t baro = (int32_t)(pressure_pa * 10.0f);

    // Periodic status output to verify simulator operation
    static uint32_t imu_count = 0;
    if (++imu_count % 1000 == 0) {
        ::printf("SensAItion: IMU packet #%" PRIu32 " sent - accel_x=%" PRId32 " µg, gyro_x=%" PRId32 " µdeg/s\n",
                 imu_count, accel_x, gyro_x);
    }

    // Pack data in big-endian format (36 bytes data)
    uint8_t pkt[36];
    put_be32_ptr(&pkt[0], accel_x);
    put_be32_ptr(&pkt[4], accel_y);
    put_be32_ptr(&pkt[8], accel_z);
    put_be32_ptr(&pkt[12], gyro_x);
    put_be32_ptr(&pkt[16], gyro_y);
    put_be32_ptr(&pkt[20], gyro_z);
    put_be16_ptr(&pkt[24], temperature);
    put_be16_ptr(&pkt[26], mag_x);
    put_be16_ptr(&pkt[28], mag_y);
    put_be16_ptr(&pkt[30], mag_z);
    put_be32_ptr(&pkt[32], baro);

    // Send header
    const uint8_t header = 0xFA;
    write_to_autopilot((const char *)&header, 1);

    // Send packet data
    write_to_autopilot((const char *)pkt, sizeof(pkt));

    // Calculate and send checksum
    uint8_t checksum = calculate_xor_checksum(pkt, sizeof(pkt));
    write_to_autopilot((const char *)&checksum, 1);
}