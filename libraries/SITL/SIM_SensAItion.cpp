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
#include <SITL/SITL.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <AP_Baro/AP_Baro.h>
#include <AP_HAL/utility/sparse-endian.h> // Required for put_beXX_ptr
#include <sys/time.h> // Required for struct timeval

using namespace SITL;

// Constants
// Gyro: SITL provides deg/s. Driver expects uDeg/s.
// Factor = 1e6. (NOT RAD_TO_UDEG!)
const float GYRO_NOISE_DEG = 0.02f;

// Baro: SITL provides mhPa
// std 100 mhPa
const float BARO_NOISE_MHPA = 100.0f;

int sim_log_counter = 0;
static float rand_float_noise()
{
    return ((float)rand() / (float)RAND_MAX) * 2.0f - 1.0f;
}
// [CPO FIX] Time Sync Helper
// Anchors the simulation timestamp to the absolute UTC start time of the SITL run.
static void simulation_timeval(struct timeval *tv)
{
    uint64_t now = AP_HAL::micros64();
    static uint64_t first_usec;
    static struct timeval first_tv;
    if (first_usec == 0) {
        first_usec = now;
        first_tv.tv_sec = AP::sitl()->start_time_UTC;
    }
    *tv = first_tv;
    tv->tv_sec += now / 1000000ULL;
    uint64_t new_usec = tv->tv_usec + (now % 1000000ULL);
    tv->tv_sec += new_usec / 1000000ULL;
    tv->tv_usec = new_usec % 1000000ULL;
}

static uint32_t get_gps_tow_ms()
{
    struct timeval tv;
    gettimeofday(&tv, nullptr); // Get host system time (simulated wall clock)

    // Unix Epoch (1970) vs GPS Epoch (1980) offset is ~315964800 seconds
    // But simplistic SITL often just needs "seconds since Sunday".
    // Unix Epoch was a Thursday.
    // +3 days (259200 sec) aligns the modulo to Sunday.
    // 18 leap seconds (approx) for current time.

    // However, the most robust way in ArduPilot SITL is to use the
    // simulation start time which is aligned to the host clock.

    uint64_t now_us = AP_HAL::micros64();
    double sim_time_sec = (double)AP::sitl()->start_time_UTC + (now_us * 1.0e-6);

    // GPS leap seconds (18 as of 2024)
    // GPS time is ahead of UTC.
    sim_time_sec += 18;

    // Seconds in week
    uint32_t seconds_in_week = (uint32_t)sim_time_sec % 604800;
    uint32_t ms_part = (now_us / 1000) % 1000;

    return (seconds_in_week * 1000) + ms_part;
}

SensAItion::SensAItion(bool interleaved_mode) : SerialDevice::SerialDevice()
{
    _interleaved_mode = interleaved_mode;
}

void SensAItion::update(void)
{
    int tick1kHz = (AP_HAL::micros() + 500) / 1000;
    if (tick1kHz <= _tick) {
        return;
    }
    _tick = tick1kHz;

    char trash_buf[64];
    read_from_autopilot(trash_buf, sizeof(trash_buf));

    // [CPO FIX] The "Gentle Start" Delay
    // Wait 2 seconds after boot to allow ArduPilot to initialize
    // serial ports and parameters before we flood it with data.
    // This prevents race conditions during the reboot test.
    if (AP_HAL::millis() < 1000) {
        return;
    }

    if (!init_sitl_pointer()) {
        return;
    }

    const auto &fdm = _sitl->state;

    // 1. Packet 0: IMU
    if ((_tick % _periodMessage0) == _phaseMessage0) {
        send_packet_0_imu(fdm);
    }

    // 2. Interleaved-only Packets
    if (_interleaved_mode) {

        // Packet 1: Orientation
        if ((_tick % _periodMessage1) == _phaseMessage1) {
            send_packet_1_orientation(fdm);
        }

        // Packet 2: INS
        if ((_tick % _periodMessage2) == _phaseMessage2) {
            send_packet_2_ins(fdm);
        }
    }
    flush_packets();
}

void SensAItion::send_packet_0_imu(const struct sitl_fdm &fdm)
{

    // Convert from SI units to SensAItion units
    // Acceleration: m/s² to µg (1e-6 g)
    const float gravity = 9.80665f;
    int32_t accel_x = (int32_t)(fdm.xAccel / gravity * 1e6);
    int32_t accel_y = (int32_t)(fdm.yAccel / gravity * 1e6);
    int32_t accel_z = (int32_t)(fdm.zAccel / gravity * 1e6);

    // Angular rates: deg/s to µdeg/s (1e-6 deg/s)
    int32_t gyro_x = (int32_t)((fdm.rollRate + (rand_float_noise() * GYRO_NOISE_DEG)) * 1e6);
    int32_t gyro_y = (int32_t)((fdm.pitchRate + (rand_float_noise() * GYRO_NOISE_DEG)) * 1e6);
    int32_t gyro_z = (int32_t)((fdm.yawRate + (rand_float_noise() * GYRO_NOISE_DEG)) * 1e6);

    // Temperature: special conversion formula
    float temp_c = 25.0f;
    int16_t temperature = (int16_t)((temp_c - 20.0f) / 0.008f);

    // Magnetometer: Gauss to mGauss
    int16_t mag_x = (int16_t)(fdm.bodyMagField.x * 1000);
    int16_t mag_y = (int16_t)(fdm.bodyMagField.y * 1000);
    int16_t mag_z = (int16_t)(fdm.bodyMagField.z * 1000);

    // Barometer: Calculate pressure from altitude (Pa to 0.1 Pa units)
    const float pressure_pa = AP_Baro::get_pressure_for_alt_amsl(fdm.altitude);
    int32_t baro = (int32_t)(pressure_pa * 10.0f + rand_float_noise() * BARO_NOISE_MHPA);

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

    if (_interleaved_mode) {
        write_packet(0x00, pkt, sizeof(pkt));
        // --- DETAILED LOGGING (SIM SIDE) ---
        // if (sim_log_counter++ % 400 == 0) {
        //     fprintf(stderr, "[SIM-OUT] IMU Packet (Interleaved Mode: %d)\n", _interleaved_mode);
        //     fprintf(stderr, "   Acc(ug): X=%d Y=%d Z=%d\n", (int)accel_x, (int)accel_y, (int)accel_z);
        //     fprintf(stderr, "   Gyr(ud): X=%d Y=%d Z=%d\n", (int)gyro_x, (int)gyro_y, (int)gyro_z);
        //     fprintf(stderr, "   Mag(mG): X=%d Y=%d Z=%d\n", (int)mag_x, (int)mag_y, (int)mag_z);
        //     fprintf(stderr, "   Bar(0.1Pa): %d | Temp(raw): %d\n", (int)baro, (int)temperature);
        // }
    } else {
        write_legacy_packet(pkt, sizeof(pkt));
        //--- DETAILED LOGGING (SIM SIDE) ---
        //if (sim_log_counter++ % 400 == 0) {
        //    fprintf(stderr, "[SIM-OUT] IMU Packet (Legacy Mode: %d)\n", _interleaved_mode);
        //    fprintf(stderr, "   Acc(ug): X=%d Y=%d Z=%d\n", (int)accel_x, (int)accel_y, (int)accel_z);
        //    fprintf(stderr, "   Gyr(ud): X=%d Y=%d Z=%d\n", (int)gyro_x, (int)gyro_y, (int)gyro_z);
        //    fprintf(stderr, "   Mag(mG): X=%d Y=%d Z=%d\n", (int)mag_x, (int)mag_y, (int)mag_z);
        //    fprintf(stderr, "   Bar(0.1Pa): %d | Temp(raw): %d\n", (int)baro, (int)temperature);
        //}
    }

}

void SensAItion::send_packet_1_orientation(const struct sitl_fdm &fdm)
{
    // Quaternion scaled by 1e6
    int32_t q0 = (int32_t)(fdm.quaternion.q1 * 1.0e6f); // W
    int32_t q1 = (int32_t)(fdm.quaternion.q2 * 1.0e6f); // X
    int32_t q2 = (int32_t)(fdm.quaternion.q3 * 1.0e6f); // Y
    int32_t q3 = (int32_t)(fdm.quaternion.q4 * 1.0e6f); // Z

    uint8_t pkt[16];
    put_be32_ptr(&pkt[0],  (uint32_t)q0);
    put_be32_ptr(&pkt[4],  (uint32_t)q1);
    put_be32_ptr(&pkt[8],  (uint32_t)q2);
    put_be32_ptr(&pkt[12], (uint32_t)q3);

    // --- LOGGING PROBE (SIM SIDE) ---
    // if (sim_log_counter % 100 == 0) { // Match Parser 1Hz rate
    //     fprintf(stderr, "[SIM-OUT] AHRS Packet (ID 0x01)\n");
    //     fprintf(stderr, "   Quat(1e-6): W=%d X=%d Y=%d Z=%d\n", q0, q1, q2, q3);
    // }

    write_packet(0x01, pkt, sizeof(pkt));
}

// NOTE: calculate_itow removed in favor of simulation_timeval inline logic

void SensAItion::send_packet_2_ins(const struct sitl_fdm &fdm)
{
    // --- 1. PREPARE DATA ---
    uint8_t align_status = 1; // 1 = Aligned
    uint8_t gnss1_fix = 3; // 3D Fix
    uint8_t gnss2_fix = 3;

    uint32_t num_sats = 0x0C0C0C0C;

    // [CPO FIX] Calculate accurate GPS Time of Week using the SITL wall clock
    // Replaces previous calculate_itow()
    struct timeval tv;
    simulation_timeval(&tv);

    // Convert TV (Seconds+Micros) to GPS Time of Week (ms)
    // We assume SITL starts somewhat recently.
    // Calculate seconds since Sunday 00:00:00 UTC
    //uint32_t seconds_in_week = 604800;
    //uint32_t tow_ms = (tv.tv_sec % seconds_in_week) * 1000 + (tv.tv_usec / 1000);
    uint32_t itow = get_gps_tow_ms();

    // Position (deg -> 1e-7 deg)
    int32_t lat = (int32_t)(fdm.latitude * 1.0e7);
    int32_t lon = (int32_t)(fdm.longitude * 1.0e7);

    // Velocity (m/s -> mm/s)
    int32_t vel_n = (int32_t)(fdm.speedN * 1000.0f);
    int32_t vel_e = (int32_t)(fdm.speedE * 1000.0f);
    int32_t vel_d = (int32_t)(fdm.speedD * 1000.0f);

    // Altitude (m -> mm)
    int32_t alt_mm = (int32_t)(fdm.altitude * 1000.0);

    // Accuracy (mm, mm/s)
    int32_t acc_lat_mm = 100; // 0.1m
    int32_t acc_lon_mm = 100; // 0.1m
    int32_t acc_vn_mm = 20;
    int32_t acc_ve_mm = 20;
    int32_t acc_vd_mm = 20;
    int32_t acc_vd_pos_mm = 100;
    uint32_t err_flags = 0;
    uint8_t sensor_valid = 0xFF; // All valid

    // Date
    uint16_t year = 2025;
    uint16_t month = 12;
    uint8_t day = 7;

    // --- 2. PACKING (Big Endian - 69 Bytes) ---
    uint8_t pkt[69];

    // Bytes 0-3: Num Sats (4B)
    put_be32_ptr(&pkt[0], num_sats);

    // Bytes 4-7: Error Flags (4B)
    put_be32_ptr(&pkt[4], err_flags);

    // Byte 8: Sensor Valid (1B)
    pkt[8] = sensor_valid;

    // Bytes 9-12: Latitude (4B)
    put_be32_ptr(&pkt[9], (uint32_t)lat);

    // Bytes 13-16: Longitude (4B)
    put_be32_ptr(&pkt[13], (uint32_t)lon);

    // Bytes 17-28: Velocity N, E, D (12B)
    put_be32_ptr(&pkt[17], (uint32_t)vel_n);
    put_be32_ptr(&pkt[21], (uint32_t)vel_e);
    put_be32_ptr(&pkt[25], (uint32_t)vel_d);

    // Bytes 29-32: Altitude MSL (4B)
    put_be32_ptr(&pkt[29], (uint32_t)alt_mm);

    // Byte 33: Alignment Status (1B)
    pkt[33] = align_status;

    // Bytes 34-37: Time iTOW (4B)
    put_be32_ptr(&pkt[34], itow);

    // 38-39: GNSS Fix
    pkt[38] = gnss1_fix;
    pkt[39] = gnss2_fix;

    // 40-44: UTC Date/Time
    put_be16_ptr(&pkt[40], year);
    put_be16_ptr(&pkt[42], month);
    pkt[44] = day;

    // 45-68: Accuracy Metrics (mm or mm/s)
    put_be32_ptr(&pkt[45], acc_lat_mm);
    put_be32_ptr(&pkt[49], acc_lon_mm);
    put_be32_ptr(&pkt[53], acc_vn_mm);
    put_be32_ptr(&pkt[57], acc_ve_mm);
    put_be32_ptr(&pkt[61], acc_vd_mm);
    put_be32_ptr(&pkt[65], acc_vd_pos_mm);

    // --- LOGGING PROBE (SIM SIDE) ---
    // if (sim_log_counter % 100 == 0) {
    //     fprintf(stderr, "[SIM-OUT] INS Packet (iTOW: %u)\n", itow);
    //     fprintf(stderr, "   Pos: Lat=%d Lon=%d Alt=%d mm\n", lat, lon, alt_mm);
    //     fprintf(stderr, "   Vel: N=%d E=%d D=%d mm/s\n", vel_n, vel_e, vel_d);
    //     fprintf(stderr, "   Stat: Align=%d Valid=0x%02X\n", align_status, sensor_valid);
    // }


    write_packet(0x02, pkt, sizeof(pkt));
}


void SensAItion::flush_packets()
{
    if (_buffert_cnt > 0) {
        write_to_autopilot((const char *)&_buffert, _buffert_cnt);
        _buffert_cnt = 0;
    }
}

void SensAItion::write_to_autopilot_buf(const char *data, int length)
{
    memcpy(&_buffert[_buffert_cnt], data, length);
    _buffert_cnt += length;
}

void SensAItion::write_packet(uint8_t msg_id, const uint8_t* payload, uint16_t length)
{
    const uint8_t header = 0xFA;
    write_to_autopilot_buf((const char *)&header, 1);
    write_to_autopilot_buf((const char *)&msg_id, 1);
    write_to_autopilot_buf((const char *)payload, length);
    uint8_t crc = (uint8_t)calculate_crc(msg_id, payload, length, true);
    write_to_autopilot_buf((const char *)&crc, 1);
}

void SensAItion::write_legacy_packet(const uint8_t* payload, uint16_t length)
{
    const uint8_t header = 0xFA;
    write_to_autopilot_buf((const char *)&header, 1);
    write_to_autopilot_buf((const char *)payload, length);
    uint8_t crc = (uint8_t)calculate_crc(0, payload, length, false);
    write_to_autopilot_buf((const char *)&crc, 1);
}

uint16_t SensAItion::calculate_crc(uint8_t msg_id, const uint8_t* payload, uint16_t length, bool use_id)
{
    uint8_t crc = 0;
    if (use_id) {
        crc ^= msg_id;
    }
    for (uint16_t i = 0; i < length; i++) {
        crc ^= payload[i];
    }
    return crc;
}

