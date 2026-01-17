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

#include <string.h> // Required for memchr, memmove
#include "AP_ExternalAHRS_SensAItion_Parser.h"
#include <AP_GPS/GPS_Backend.h>
#include <AP_HAL/AP_HAL.h>
#include <GCS_MAVLink/GCS.h>

// Constructor
AP_ExternalAHRS_SensAItion_Parser::AP_ExternalAHRS_SensAItion_Parser(ConfigMode mode) :
    config_mode(mode)
{
    reset_parser();
}

size_t AP_ExternalAHRS_SensAItion_Parser::parse_stream(const uint8_t* data, size_t data_size, Measurement& meas)
{
    for (size_t i = 0; i < data_size; i++) {
        if (parse_single_byte(data[i])) {
            decode_packet(meas);
            reset_parser(); // Ready for more packets in same buffer
            return i + 1; // No of bytes parsed
        }
    }

    meas = Measurement(); // Uninitialized
    return data_size;
}

void AP_ExternalAHRS_SensAItion_Parser::reset_parser()
{
    parse_state = ParseState::WAITING_HEADER;
    packet_buffer_len = 0;
    target_payload_len = 0;
    current_packet_id = PacketID::UNKNOWN;
}

// Error Handler
void AP_ExternalAHRS_SensAItion_Parser::handle_invalid_packet()
{
    // Fallback if called with empty buffer, which should never happen
    if (packet_buffer_len == 0) {
        return;
    }

    // Look for a new header byte starting from index 1 to resync
    uint8_t *p = (uint8_t *)memchr(&packet_buffer[1], HEADER_BYTE, packet_buffer_len - 1);

    if (p == nullptr) {
        // No header found, reset completely
        reset_parser();
        return;
    }

    size_t bytes_to_discard = p - packet_buffer;
    size_t bytes_to_keep = packet_buffer_len - bytes_to_discard;

    memmove(&packet_buffer[0], p, bytes_to_keep);
    packet_buffer_len = bytes_to_keep;

    // Determine State based on Mode and remaining data
    if (config_mode == ConfigMode::INTERLEAVED_INS) {
        if (packet_buffer_len >= 2) {
            // We have Header + Potential ID. Process it immediately.
            uint8_t id = packet_buffer[1];
            switch (static_cast<PacketID>(id)) {
            case PacketID::IMU:
                target_payload_len = PAYLOAD_SIZE_IMU;
                parse_state = ParseState::COLLECTING_PAYLOAD;
                current_packet_id = PacketID::IMU;
                break;
            case PacketID::AHRS:
                target_payload_len = PAYLOAD_SIZE_QUAT;
                parse_state = ParseState::COLLECTING_PAYLOAD;
                current_packet_id = PacketID::AHRS;
                break;
            case PacketID::INS:
                target_payload_len = PAYLOAD_SIZE_INS;
                parse_state = ParseState::COLLECTING_PAYLOAD;
                current_packet_id = PacketID::INS;
                break;
            default:
                // The "New" ID is also bad. Drop header and retry.
                reset_parser();
                return;
            }
        } else {
            parse_state = ParseState::WAITING_ID;
        }
    } else {
        // Legacy Mode
        target_payload_len = PAYLOAD_SIZE_IMU;
        parse_state = ParseState::COLLECTING_PAYLOAD;
    }
}

// Checksum Validator & Deep Debugger
bool AP_ExternalAHRS_SensAItion_Parser::buffer_contains_valid_packet() const
{
    if (packet_buffer_len < 2 || packet_buffer[0] != HEADER_BYTE) {
        return false;
    }

    uint8_t calculated = calculate_xor_checksum(packet_buffer, packet_buffer_len - 1);
    uint8_t received = packet_buffer[packet_buffer_len - 1];

    if (calculated != received) {
        return false;
    }

    return true;
}

// The Core State Machine
bool AP_ExternalAHRS_SensAItion_Parser::parse_single_byte(uint8_t byte)
{
    // Safety: Prevent buffer overflow
    if (packet_buffer_len >= MAX_PACKET_SIZE) {
        handle_invalid_packet();
        if (packet_buffer_len >= MAX_PACKET_SIZE) {
            reset_parser();
        }
    }

    switch (parse_state) {
    case ParseState::WAITING_HEADER:
        if (byte == HEADER_BYTE) {
            packet_buffer_len = 0;
            packet_buffer[packet_buffer_len++] = byte;

            if (config_mode == ConfigMode::INTERLEAVED_INS) {
                parse_state = ParseState::WAITING_ID;
            } else {
                target_payload_len = PAYLOAD_SIZE_IMU;
                parse_state = ParseState::COLLECTING_PAYLOAD;
            }
        }
        break;

    case ParseState::WAITING_ID:
        packet_buffer[packet_buffer_len++] = byte;

        switch (static_cast<PacketID>(byte)) {
        case PacketID::IMU:
            target_payload_len = PAYLOAD_SIZE_IMU;
            current_packet_id = PacketID::IMU;
            parse_state = ParseState::COLLECTING_PAYLOAD;
            break;

        case PacketID::AHRS:
            target_payload_len = PAYLOAD_SIZE_QUAT;
            current_packet_id = PacketID::AHRS;
            parse_state = ParseState::COLLECTING_PAYLOAD;
            break;

        case PacketID::INS:
            target_payload_len = PAYLOAD_SIZE_INS;
            current_packet_id = PacketID::INS;
            parse_state = ParseState::COLLECTING_PAYLOAD;
            break;

        default:
            parse_errors++;
            handle_invalid_packet();
            break;
        }
        break;

    case ParseState::COLLECTING_PAYLOAD:
        packet_buffer[packet_buffer_len++] = byte;

        // Calculate Expected Total Length
        // Legacy: Header(1) + Payload(N) + CRC(1)
        // Interleaved: Header(1) + ID(1) + Payload(N) + CRC(1)
        size_t overhead = (config_mode == ConfigMode::INTERLEAVED_INS) ? 3 : 2;
        size_t expected_total_len = target_payload_len + overhead;

        if (packet_buffer_len >= expected_total_len) {
            if (buffer_contains_valid_packet()) {

                valid_packets++;
                return true;
            } else {
                parse_errors++;
                handle_invalid_packet();
                return false;
            }
        }
        break;
    }

    return false;
}

bool AP_ExternalAHRS_SensAItion_Parser::validate_checksum() const
{
    return buffer_contains_valid_packet();
}

uint8_t AP_ExternalAHRS_SensAItion_Parser::calculate_xor_checksum(const uint8_t* data, size_t len) const
{
    uint8_t checksum = 0;
    for (size_t i = 1; i < len; i++) {
        checksum ^= data[i];
    }
    return checksum;
}

// Router
void AP_ExternalAHRS_SensAItion_Parser::decode_packet(Measurement& measurement)
{
    const uint8_t* payload = nullptr;

    if (config_mode == ConfigMode::INTERLEAVED_INS) {
        payload = &packet_buffer[2]; // Skip Header + ID
        switch (current_packet_id) {
        case PacketID::IMU: decode_imu(payload, measurement); break;
        case PacketID::AHRS: decode_ahrs(payload, measurement); break;
        case PacketID::INS: decode_ins(payload, measurement); break;
        default: break;
        }
    } else {
        payload = &packet_buffer[1]; // Skip Header
        decode_imu(payload, measurement);
    }
}

// IMU Decoder (Packet 0)
void AP_ExternalAHRS_SensAItion_Parser::decode_imu(const uint8_t* payload, Measurement& measurement)
{
    // ... [Code omitted: No changes to decoders, use your existing implementation] ...
    // Accel (Bytes 0-11): 3 x Int32 (ug)
    int32_t accel_x_ug = big_endian_to_int32(&payload[0]);
    int32_t accel_y_ug = big_endian_to_int32(&payload[4]);
    int32_t accel_z_ug = big_endian_to_int32(&payload[8]);

    // Gyro (Bytes 12-23): 3 x Int32 (udeg/s)
    int32_t gyro_x_udegs = big_endian_to_int32(&payload[12]);
    int32_t gyro_y_udegs = big_endian_to_int32(&payload[16]);
    int32_t gyro_z_udegs = big_endian_to_int32(&payload[20]);

    // Temp (Bytes 24-25): 1 x Int16 (Scaled)
    int16_t temp_raw = big_endian_to_int16(&payload[24]);

    // Mag (Bytes 26-31): 3 x Int16 (mGauss)
    int16_t mag_x_mgauss = big_endian_to_int16(&payload[26]);
    int16_t mag_y_mgauss = big_endian_to_int16(&payload[28]);
    int16_t mag_z_mgauss = big_endian_to_int16(&payload[30]);

    // Baro (Bytes 32-35): 1 x Int32 (0.1 Pa)
    int32_t baro_raw = big_endian_to_int32(&payload[32]);

    // Accel: ug -> m/s^2 (Note: 1,000,000 ug = 9.81 m/s^2 roughly)
    const float ug_to_mss = 1.0e-6f * GRAVITY_MSS;
    measurement.acceleration_mss = Vector3f(accel_x_ug, accel_y_ug, accel_z_ug) * ug_to_mss;

    // Gyro: udeg/s -> rad/s (Note: 1,000,000 udeg/s = 1 deg/s = 0.017 rad/s)
    const float udeg_to_rad = 1.0e-6f * DEG_TO_RAD;
    measurement.angular_velocity_rads = Vector3f(gyro_x_udegs, gyro_y_udegs, gyro_z_udegs) * udeg_to_rad;

    // Temp: (Raw * 0.008) + 20
    measurement.temperature_degc = ((float)temp_raw * 0.008f) + 20.0f;

    // Mag: mGauss (Pass-through, ArduPilot expects mGauss)
    measurement.magnetic_field_mgauss = Vector3f(mag_x_mgauss, mag_y_mgauss, mag_z_mgauss);

    // Baro: 0.1 Pa -> Pascal
    measurement.air_pressure_p = (float)baro_raw * 0.1f;

    // Metadata
    measurement.type = MeasurementType::IMU;
    measurement.timestamp_us = AP_HAL::micros64();
}

void AP_ExternalAHRS_SensAItion_Parser::decode_ahrs(const uint8_t* payload, Measurement& measurement)
{
    // Payload layout: W (0-3), X (4-7), Y (8-11), Z (12-15)
    int32_t quat_w_raw = big_endian_to_int32(&payload[0]);
    int32_t quat_x_raw = big_endian_to_int32(&payload[4]);
    int32_t quat_y_raw = big_endian_to_int32(&payload[8]);
    int32_t quat_z_raw = big_endian_to_int32(&payload[12]);

    const float scale_factor = 1.0e-6f;

    measurement.orientation = Quaternion(
                                  (float)quat_w_raw * scale_factor,
                                  (float)quat_x_raw * scale_factor,
                                  (float)quat_y_raw * scale_factor,
                                  (float)quat_z_raw * scale_factor
                              );

    measurement.type = MeasurementType::AHRS;
    measurement.timestamp_us = AP_HAL::micros64();
}

void AP_ExternalAHRS_SensAItion_Parser::decode_ins(const uint8_t* payload, Measurement& measurement)
{
    // --- 1. PARSE RAW BYTES (Big-Endian) ---

    // 0-3: Num Sats (G2, G1)
    measurement.num_sats_gnss2 = payload[1];
    measurement.num_sats_gnss1 = payload[3];

    // 4-7: Error Flags
    measurement.error_flags = big_endian_to_uint32(&payload[4]);

    // 8: Sensor Valid
    measurement.sensor_valid = payload[8];

    // 9-16: Lat/Lon (1e-7 deg)
    int32_t lat_raw = big_endian_to_int32(&payload[9]);
    int32_t lon_raw = big_endian_to_int32(&payload[13]);

    // 17-28: Velocity N, E, D (mm/s)
    int32_t vel_n_mm = big_endian_to_int32(&payload[17]);
    int32_t vel_e_mm = big_endian_to_int32(&payload[21]);
    int32_t vel_d_mm = big_endian_to_int32(&payload[25]);

    // 29-32: Altitude relative to WGS 84 ellipsoid (mm)
    int32_t alt_raw_mm = big_endian_to_int32(&payload[29]);

    // 33: Alignment Status
    measurement.alignment_status = payload[33];

    // 34-37: Time of week (ms)
    measurement.time_itow_ms = big_endian_to_uint32(&payload[34]);

    // 38-39: GNSS Fix
    measurement.gnss2_fix = payload[38];
    measurement.gnss1_fix = payload[39];

    // 40-44: UTC Date/Time
    uint16_t year = (uint16_t)((payload[40]<<8) | payload[41]);
    uint16_t month = (uint16_t)((payload[42]<<8) | payload[43]);
    uint8_t day = payload[44];

    // 45-68: Accuracy Metrics (mm or mm/s)
    int32_t acc_lat_mm = big_endian_to_int32(&payload[45]);
    int32_t acc_lon_mm = big_endian_to_int32(&payload[49]);
    int32_t acc_vn_mm  = big_endian_to_int32(&payload[53]);
    int32_t acc_ve_mm  = big_endian_to_int32(&payload[57]);
    int32_t acc_vd_mm  = big_endian_to_int32(&payload[61]);
    int32_t acc_vd_pos_mm = big_endian_to_int32(&payload[65]);

    // --- 2. POPULATE & CONVERT ---
    const float mm_to_cm = 0.1f;
    measurement.location = Location(lat_raw, lon_raw, alt_raw_mm * mm_to_cm, Location::AltFrame::ABSOLUTE);

    const float mms_to_ms = 0.001f;
    const float mm_to_m = 0.001f;

    measurement.velocity_ned.x = (float)vel_n_mm * mms_to_ms;
    measurement.velocity_ned.y = (float)vel_e_mm * mms_to_ms;
    measurement.velocity_ned.z = (float)vel_d_mm * mms_to_ms;

    // Accuracy
    measurement.pos_accuracy.x = (float)acc_lat_mm * mm_to_m; // Horizontal Latitude
    measurement.pos_accuracy.y = (float)acc_lon_mm * mm_to_m; // Horizontal Longitude
    measurement.pos_accuracy.z = (float)acc_vd_pos_mm * mm_to_m; // Vertical

    measurement.vel_accuracy.x = (float)acc_vn_mm * mms_to_ms;
    measurement.vel_accuracy.y = (float)acc_ve_mm * mms_to_ms;
    measurement.vel_accuracy.z = (float)acc_vd_mm * mms_to_ms;

    // GPS Week Calculation (Gate check)
    if (measurement.gnss1_fix >= 2) {
        measurement.gps_week = calculate_gps_week(year, month, day);
    } else {
        measurement.gps_week = 0;
    }

    measurement.type = MeasurementType::INS;
    measurement.timestamp_us = AP_HAL::micros64();

}

int32_t AP_ExternalAHRS_SensAItion_Parser::big_endian_to_int32(const uint8_t* bytes) const
{
    return (int32_t)((bytes[0]<<24)|(bytes[1]<<16)|(bytes[2]<<8)|bytes[3]);
}

uint32_t AP_ExternalAHRS_SensAItion_Parser::big_endian_to_uint32(const uint8_t* bytes) const
{
    return (uint32_t)((bytes[0]<<24)|(bytes[1]<<16)|(bytes[2]<<8)|bytes[3]);
}

int16_t AP_ExternalAHRS_SensAItion_Parser::big_endian_to_int16(const uint8_t* bytes) const
{
    return (int16_t)((bytes[0]<<8)|bytes[1]);
}

uint16_t AP_ExternalAHRS_SensAItion_Parser::calculate_gps_week(uint16_t year, uint8_t month, uint8_t day)
{
    // Convert to BCD form (DDMMYY) - the GPS_Backend function assumes year is 20YY
    const uint32_t bcd_date = year % 100U + month * 100U + day * 10000U;

    const uint32_t bcd_time_ms = 0U;
    uint16_t gps_week;
    uint32_t gps_time_ms;
    AP_GPS_Backend::BCD_to_gps_time(bcd_date, bcd_time_ms, gps_week, gps_time_ms);

    return gps_week;
}
