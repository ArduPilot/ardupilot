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
  suppport for MicroStrain CX5/GX5-45 serially connected AHRS Systems
 */

#define ALLOW_DOUBLE_MATH_FUNCTIONS

#include "AP_ExternalAHRS_config.h"

#if AP_MICROSTRAIN_ENABLED

#include "MicroStrain_common.h"
#include <AP_HAL/utility/sparse-endian.h>

enum class INSPacketField {
    ACCEL = 0x04,
    GYRO = 0x05,
    QUAT = 0x0A,
    MAG = 0x06,
    PRESSURE = 0x17
};

enum class GNSSPacketField {
    LLH_POSITION = 0x03,
    NED_VELOCITY = 0x05,
    DOP_DATA = 0x07,
    GPS_TIME = 0x09,
    FIX_INFO = 0x0B
};

enum class GNSSFixType {
    FIX_3D = 0x00,
    FIX_2D = 0x01,
    TIME_ONLY = 0x02,
    NONE = 0x03,
    INVALID = 0x04
};

enum class FilterPacketField {
    FILTER_STATUS = 0x10,
    GPS_TIME = 0x11,
    LLH_POSITION = 0x01,
    NED_VELOCITY = 0x02
};

bool AP_MicroStrain::handle_byte(const uint8_t b, DescriptorSet& descriptor)
{
    switch (message_in.state) {
        case ParseState::WaitingFor_SyncOne:
            if (b == SYNC_ONE) {
                message_in.packet.header[0] = b;
                message_in.state = ParseState::WaitingFor_SyncTwo;
            }
            break;
        case ParseState::WaitingFor_SyncTwo:
            if (b == SYNC_TWO) {
                message_in.packet.header[1] = b;
                message_in.state = ParseState::WaitingFor_Descriptor;
            } else {
                message_in.state = ParseState::WaitingFor_SyncOne;
            }
            break;
        case ParseState::WaitingFor_Descriptor:
            message_in.packet.header[2] = b;
            message_in.state = ParseState::WaitingFor_PayloadLength;
            break;
        case ParseState::WaitingFor_PayloadLength:
            message_in.packet.header[3] = b;
            message_in.state = ParseState::WaitingFor_Data;
            message_in.index = 0;
            break;
        case ParseState::WaitingFor_Data:
            message_in.packet.payload[message_in.index++] = b;
            if (message_in.index >= message_in.packet.header[3]) {
                message_in.state = ParseState::WaitingFor_Checksum;
                message_in.index = 0;
            }
            break;
        case ParseState::WaitingFor_Checksum:
            message_in.packet.checksum[message_in.index++] = b;
            if (message_in.index >= 2) {
                message_in.state = ParseState::WaitingFor_SyncOne;
                message_in.index = 0;

                if (valid_packet(message_in.packet)) {
                    descriptor = handle_packet(message_in.packet);
                    return true;
                }
            }
            break;
        }
    return false;
}

bool AP_MicroStrain::valid_packet(const MicroStrain_Packet & packet)
{
    uint8_t checksum_one = 0;
    uint8_t checksum_two = 0;

    for (int i = 0; i < 4; i++) {
        checksum_one += packet.header[i];
        checksum_two += checksum_one;
    }

    for (int i = 0; i < packet.header[3]; i++) {
        checksum_one += packet.payload[i];
        checksum_two += checksum_one;
    }

    return packet.checksum[0] == checksum_one && packet.checksum[1] == checksum_two;
}

AP_MicroStrain::DescriptorSet AP_MicroStrain::handle_packet(const MicroStrain_Packet& packet)
{
    const DescriptorSet descriptor  = DescriptorSet(packet.header[2]);
    switch (descriptor) {
    case DescriptorSet::IMUData:
        handle_imu(packet);
        break;
    case DescriptorSet::GNSSData:
        handle_gnss(packet);
        break;
    case DescriptorSet::EstimationData:
        handle_filter(packet);
        break;
    case DescriptorSet::BaseCommand:
    case DescriptorSet::DMCommand:
    case DescriptorSet::SystemCommand:
        break;
    }
    return descriptor;
}


void AP_MicroStrain::handle_imu(const MicroStrain_Packet& packet)
{
    last_ins_pkt = AP_HAL::millis();

    // Iterate through fields of varying lengths in INS packet
    for (uint8_t i = 0; i < packet.header[3]; i +=  packet.payload[i]) {
        switch ((INSPacketField) packet.payload[i+1]) {
        // Scaled Ambient Pressure
        case INSPacketField::PRESSURE: {
            imu_data.pressure = be32tofloat_ptr(packet.payload, i+2) * 100; // Convert millibar to pascals
            break;
        }
        // Scaled Magnetometer Vector
        case INSPacketField::MAG: {
            imu_data.mag = populate_vector3f(packet.payload, i+2) * 1000; // Convert gauss to milligauss
            break;
        }
        // Scaled Accelerometer Vector
        case INSPacketField::ACCEL: {
            imu_data.accel = populate_vector3f(packet.payload, i+2) * GRAVITY_MSS; // Convert g's to m/s^2
            break;
        }
        // Scaled Gyro Vector
        case INSPacketField::GYRO: {
            imu_data.gyro = populate_vector3f(packet.payload, i+2);
            break;
        }
        // Quaternion
        case INSPacketField::QUAT: {
            imu_data.quat = populate_quaternion(packet.payload, i+2);
            break;
        }
        }
    }
}


void AP_MicroStrain::handle_gnss(const MicroStrain_Packet &packet)
{
    last_gps_pkt = AP_HAL::millis();

    // Iterate through fields of varying lengths in GNSS packet
    for (uint8_t i = 0; i < packet.header[3]; i += packet.payload[i]) {
        switch ((GNSSPacketField) packet.payload[i+1]) {
        // GPS Time
        case GNSSPacketField::GPS_TIME: {
            gnss_data.tow_ms = double_to_uint32(be64todouble_ptr(packet.payload, i+2) * 1000); // Convert seconds to ms
            gnss_data.week = be16toh_ptr(&packet.payload[i+10]);
            break;
        }
        // GNSS Fix Information
        case GNSSPacketField::FIX_INFO: {
            switch ((GNSSFixType) packet.payload[i+2]) {
            case (GNSSFixType::FIX_3D): {
                gnss_data.fix_type = GPS_FIX_TYPE_3D_FIX;
                break;
            }
            case (GNSSFixType::FIX_2D): {
                gnss_data.fix_type = GPS_FIX_TYPE_2D_FIX;
                break;
            }
            case (GNSSFixType::TIME_ONLY):
            case (GNSSFixType::NONE): {
                gnss_data.fix_type = GPS_FIX_TYPE_NO_FIX;
                break;
            }
            default:
            case (GNSSFixType::INVALID): {
                gnss_data.fix_type = GPS_FIX_TYPE_NO_GPS;
                break;
            }
            }

            gnss_data.satellites = packet.payload[i+3];
            break;
        }
        // LLH Position
        case GNSSPacketField::LLH_POSITION: {
            gnss_data.lat = be64todouble_ptr(packet.payload, i+2) * 1.0e7; // Decimal degrees to degrees
            gnss_data.lon = be64todouble_ptr(packet.payload, i+10) * 1.0e7;
            gnss_data.msl_altitude = be64todouble_ptr(packet.payload, i+26) * 1.0e2; // Meters to cm
            gnss_data.horizontal_position_accuracy = be32tofloat_ptr(packet.payload, i+34);
            gnss_data.vertical_position_accuracy = be32tofloat_ptr(packet.payload, i+38);
            break;
        }
        // DOP Data
        case GNSSPacketField::DOP_DATA: {
            gnss_data.hdop = be32tofloat_ptr(packet.payload, i+10);
            gnss_data.vdop = be32tofloat_ptr(packet.payload, i+14);
            break;
        }
        // NED Velocity
        case GNSSPacketField::NED_VELOCITY: {
            gnss_data.ned_velocity_north = be32tofloat_ptr(packet.payload, i+2);
            gnss_data.ned_velocity_east = be32tofloat_ptr(packet.payload, i+6);
            gnss_data.ned_velocity_down = be32tofloat_ptr(packet.payload, i+10);
            gnss_data.speed_accuracy = be32tofloat_ptr(packet.payload, i+26);
            break;
        }
        }
    }
}

void AP_MicroStrain::handle_filter(const MicroStrain_Packet &packet)
{
    last_filter_pkt = AP_HAL::millis();

    // Iterate through fields of varying lengths in filter packet
    for (uint8_t i = 0; i < packet.header[3]; i += packet.payload[i]) {
        switch ((FilterPacketField) packet.payload[i+1]) {
        // GPS Timestamp
        case FilterPacketField::GPS_TIME: {
            filter_data.tow_ms = be64todouble_ptr(packet.payload, i+2) * 1000; // Convert seconds to ms
            filter_data.week = be16toh_ptr(&packet.payload[i+10]);
            break;
        }
        // LLH Position
        case FilterPacketField::LLH_POSITION: {
            filter_data.lat = be64todouble_ptr(packet.payload, i+2) * 1.0e7; // Decimal degrees to degrees
            filter_data.lon = be64todouble_ptr(packet.payload, i+10) * 1.0e7;
            filter_data.hae_altitude = be64todouble_ptr(packet.payload, i+26) * 1.0e2; // Meters to cm
            break;
        }
        // NED Velocity
        case FilterPacketField::NED_VELOCITY: {
            filter_data.ned_velocity_north = be32tofloat_ptr(packet.payload, i+2);
            filter_data.ned_velocity_east = be32tofloat_ptr(packet.payload, i+6);
            filter_data.ned_velocity_down = be32tofloat_ptr(packet.payload, i+10);
            break;
        }
        // Filter Status
        case FilterPacketField::FILTER_STATUS: {
            filter_status.state = be16toh_ptr(&packet.payload[i+2]);
            filter_status.mode = be16toh_ptr(&packet.payload[i+4]);
            filter_status.flags = be16toh_ptr(&packet.payload[i+6]);
            break;
        }
        }
    }
}

Vector3f AP_MicroStrain::populate_vector3f(const uint8_t *data, uint8_t offset)
{
    return Vector3f {
        be32tofloat_ptr(data, offset),
        be32tofloat_ptr(data, offset+4),
        be32tofloat_ptr(data, offset+8)
    };
}

Quaternion AP_MicroStrain::populate_quaternion(const uint8_t *data, uint8_t offset)
{
    return Quaternion {
        be32tofloat_ptr(data, offset),
        be32tofloat_ptr(data, offset+4),
        be32tofloat_ptr(data, offset+8),
        be32tofloat_ptr(data, offset+12)
    };
}


#endif // AP_MICROSTRAIN_ENABLED