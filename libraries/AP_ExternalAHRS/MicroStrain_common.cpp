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
  support for MicroStrain MIP parsing
 */

#define AP_MATH_ALLOW_DOUBLE_FUNCTIONS 1

#include "AP_ExternalAHRS_config.h"
#include <AP_GPS/AP_GPS.h>

#if AP_MICROSTRAIN_ENABLED

#include "MicroStrain_common.h"
#include <AP_HAL/utility/sparse-endian.h>

// https://s3.amazonaws.com/files.microstrain.com/GQ7+User+Manual/external_content/dcp/Data/sensor_data/sensor_data_links.htm
enum class INSPacketField {
    ACCEL = 0x04,
    GYRO = 0x05,
    QUAT = 0x0A,
    MAG = 0x06,
    PRESSURE = 0x17,
};

// https://s3.amazonaws.com/files.microstrain.com/GQ7+User+Manual/external_content/dcp/Data/gnss_recv_1/gnss_recv_1_links.htm
enum class GNSSPacketField {
    LLH_POSITION = 0x03,
    NED_VELOCITY = 0x05,
    DOP_DATA = 0x07,
    FIX_INFO = 0x0B,
    // https://s3.amazonaws.com/files.microstrain.com/GQ7+User+Manual/external_content/dcp/Data/shared_data/data/mip_field_shared_gps_timestamp.htm
    GPS_TIMESTAMP = 0xD3,
};

// https://s3.amazonaws.com/files.microstrain.com/GQ7+User+Manual/external_content/dcp/Data/gnss_recv_1/data/mip_field_gnss_fix_info.htm
enum class GNSSFixType {
    FIX_3D = 0x00,
    FIX_2D = 0x01,
    TIME_ONLY = 0x02,
    NONE = 0x03,
    INVALID = 0x04,
    FIX_RTK_FLOAT = 0x05,
    FIX_RTK_FIXED = 0x06,
};

// https://s3.amazonaws.com/files.microstrain.com/GQ7+User+Manual/external_content/dcp/Data/filter_data/filter_data_links.htm
enum class FilterPacketField {
    FILTER_STATUS = 0x10,
    LLH_POSITION = 0x01,
    NED_VELOCITY = 0x02,
    // https://s3.amazonaws.com/files.microstrain.com/GQ7+User+Manual/external_content/dcp/Data/filter_data/data/mip_field_filter_attitude_quaternion.htm
    ATTITUDE_QUAT = 0x03,
    // https://s3.amazonaws.com/files.microstrain.com/GQ7+User+Manual/external_content/dcp/Data/0x82/data/0x08.htm
    LLH_POSITION_UNCERTAINTY = 0x08,
    // https://s3.amazonaws.com/files.microstrain.com/GQ7+User+Manual/external_content/dcp/Data/0x82/data/0x09.htm
    NED_VELOCITY_UNCERTAINTY = 0x09,
    // https://s3.amazonaws.com/files.microstrain.com/GQ7+User+Manual/external_content/dcp/Data/shared_data/data/mip_field_shared_gps_timestamp.htm
    GPS_TIMESTAMP = 0xD3,
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
            message_in.packet.descriptor_set(b);
            message_in.state = ParseState::WaitingFor_PayloadLength;
            break;
        case ParseState::WaitingFor_PayloadLength:
            message_in.packet.payload_length(b);
            message_in.state = ParseState::WaitingFor_Data;
            message_in.index = 0;
            break;
        case ParseState::WaitingFor_Data:
            message_in.packet.payload[message_in.index++] = b;
            if (message_in.index >= message_in.packet.payload_length()) {
                message_in.state = ParseState::WaitingFor_Checksum;
                message_in.index = 0;
            }
            break;
        case ParseState::WaitingFor_Checksum:
            message_in.packet.checksum[message_in.index++] = b;
            if (message_in.index >= 2) {
                message_in.state = ParseState::WaitingFor_SyncOne;
                message_in.index = 0;

                if (valid_packet(message_in.packet) &&
                    handle_packet(message_in.packet, descriptor)) {
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

    for (int i = 0; i < packet.payload_length(); i++) {
        checksum_one += packet.payload[i];
        checksum_two += checksum_one;
    }

    return packet.checksum[0] == checksum_one && packet.checksum[1] == checksum_two;
}

bool AP_MicroStrain::handle_packet(const MicroStrain_Packet& packet, DescriptorSet& descriptor)
{
    descriptor = packet.descriptor_set();
    switch (descriptor) {
    case DescriptorSet::IMUData:
        return handle_imu(packet);
    case DescriptorSet::FilterData:
        return handle_filter(packet);
    case DescriptorSet::BaseCommand:
    case DescriptorSet::DMCommand:
    case DescriptorSet::SystemCommand:
        return true;
    case DescriptorSet::GNSSData:
    case DescriptorSet::GNSSRecv1:
    case DescriptorSet::GNSSRecv2:
        return handle_gnss(packet);
    }
    return false;
}


bool AP_MicroStrain::handle_imu(const MicroStrain_Packet& packet)
{
    auto next_imu_data = imu_data;

    // Iterate through fields of varying lengths in INS packet
    const uint16_t payload_len = packet.payload_length();
    for (uint16_t i = 0; i < payload_len;) {
        if (payload_len - i < 2) {
            return false;
        }
        const uint8_t field_len = packet.payload[i];
        if (field_len < 2 || field_len > payload_len - i) {
            return false;
        }
        switch ((INSPacketField) packet.payload[i+1]) {
        // Scaled Ambient Pressure
        case INSPacketField::PRESSURE: {
            if (field_len < 6) {
                return false;
            }
            next_imu_data.pressure = be32tofloat_ptr(packet.payload, i+2) * 100; // Convert millibar to pascals
            break;
        }
        // Scaled Magnetometer Vector
        case INSPacketField::MAG: {
            if (field_len < 14) {
                return false;
            }
            next_imu_data.mag = populate_vector3f(packet.payload, i+2) * 1000; // Convert gauss to milligauss
            break;
        }
        // Scaled Accelerometer Vector
        case INSPacketField::ACCEL: {
            if (field_len < 14) {
                return false;
            }
            next_imu_data.accel = populate_vector3f(packet.payload, i+2) * GRAVITY_MSS; // Convert g's to m/s^2
            break;
        }
        // Scaled Gyro Vector
        case INSPacketField::GYRO: {
            if (field_len < 14) {
                return false;
            }
            next_imu_data.gyro = populate_vector3f(packet.payload, i+2);
            break;
        }
        // Quaternion
        case INSPacketField::QUAT: {
            if (field_len < 18) {
                return false;
            }
            next_imu_data.quat = populate_quaternion(packet.payload, i+2);
            break;
        }
        }
        i += field_len;
    }
    imu_data = next_imu_data;
    last_imu_pkt = AP_HAL::millis();
    return true;
}


bool AP_MicroStrain::handle_gnss(const MicroStrain_Packet &packet)
{
    uint8_t gnss_instance;
    const DescriptorSet descriptor  = DescriptorSet(packet.descriptor_set());
    if (!get_gnss_instance(descriptor, gnss_instance)) {
        return false;
    }
    auto next_gnss_data = gnss_data[gnss_instance];

    // Iterate through fields of varying lengths in GNSS packet
    const uint16_t payload_len = packet.payload_length();
    for (uint16_t i = 0; i < payload_len;) {
        if (payload_len - i < 2) {
            return false;
        }
        const uint8_t field_len = packet.payload[i];
        if (field_len < 2 || field_len > payload_len - i) {
            return false;
        }
        switch ((GNSSPacketField) packet.payload[i+1]) {
        case GNSSPacketField::GPS_TIMESTAMP: {
            if (field_len < 12) {
                return false;
            }
            next_gnss_data.tow_ms = double_to_uint32(be64todouble_ptr(packet.payload, i+2) * 1000); // Convert seconds to ms
            next_gnss_data.week = be16toh_ptr(&packet.payload[i+10]);
            break;
        }
        case GNSSPacketField::FIX_INFO: {
            if (field_len < 4) {
                return false;
            }
            switch ((GNSSFixType) packet.payload[i+2]) {
            case (GNSSFixType::FIX_RTK_FLOAT): {
                next_gnss_data.fix_type = AP_GPS_FixType::RTK_FLOAT;
                break;
            }
            case (GNSSFixType::FIX_RTK_FIXED): {
                next_gnss_data.fix_type = AP_GPS_FixType::RTK_FIXED;
                break;
            }
            case (GNSSFixType::FIX_3D): {
                next_gnss_data.fix_type = AP_GPS_FixType::FIX_3D;
                break;
            }
            case (GNSSFixType::FIX_2D): {
                next_gnss_data.fix_type = AP_GPS_FixType::FIX_2D;
                break;
            }
            case (GNSSFixType::TIME_ONLY):
            case (GNSSFixType::NONE): {
                next_gnss_data.fix_type = AP_GPS_FixType::NONE;
                break;
            }
            default:
            case (GNSSFixType::INVALID): {
                next_gnss_data.fix_type = AP_GPS_FixType::NO_GPS;
                break;
            }
            }

            next_gnss_data.satellites = packet.payload[i+3];
            break;
        }
        case GNSSPacketField::LLH_POSITION: {
            if (field_len < 42) {
                return false;
            }
            next_gnss_data.lat = be64todouble_ptr(packet.payload, i+2) * 1.0e7; // Decimal degrees to degrees
            next_gnss_data.lon = be64todouble_ptr(packet.payload, i+10) * 1.0e7;
            next_gnss_data.msl_altitude = be64todouble_ptr(packet.payload, i+26) * 1.0e2; // Meters to cm
            next_gnss_data.horizontal_position_accuracy = be32tofloat_ptr(packet.payload, i+34);
            next_gnss_data.vertical_position_accuracy = be32tofloat_ptr(packet.payload, i+38);
            break;
        }
        case GNSSPacketField::DOP_DATA: {
            if (field_len < 18) {
                return false;
            }
            next_gnss_data.hdop = be32tofloat_ptr(packet.payload, i+10);
            next_gnss_data.vdop = be32tofloat_ptr(packet.payload, i+14);
            break;
        }
        case GNSSPacketField::NED_VELOCITY: {
            if (field_len < 30) {
                return false;
            }
            next_gnss_data.ned_velocity_north = be32tofloat_ptr(packet.payload, i+2);
            next_gnss_data.ned_velocity_east = be32tofloat_ptr(packet.payload, i+6);
            next_gnss_data.ned_velocity_down = be32tofloat_ptr(packet.payload, i+10);
            next_gnss_data.speed_accuracy = be32tofloat_ptr(packet.payload, i+26);
            break;
        }
        }
        i += field_len;
    }
    gnss_data[gnss_instance] = next_gnss_data;
    last_gps_pkt = AP_HAL::millis();
    return true;
}

bool AP_MicroStrain::handle_filter(const MicroStrain_Packet &packet)
{
    auto next_filter_data = filter_data;
    auto next_filter_status = filter_status;

    // Iterate through fields of varying lengths in filter packet
    const uint16_t payload_len = packet.payload_length();
    for (uint16_t i = 0; i < payload_len;) {
        if (payload_len - i < 2) {
            return false;
        }
        const uint8_t field_len = packet.payload[i];
        if (field_len < 2 || field_len > payload_len - i) {
            return false;
        }
        switch ((FilterPacketField) packet.payload[i+1]) {
        case FilterPacketField::GPS_TIMESTAMP: {
            if (field_len < 12) {
                return false;
            }
            next_filter_data.tow_ms = be64todouble_ptr(packet.payload, i+2) * 1000; // Convert seconds to ms
            next_filter_data.week = be16toh_ptr(&packet.payload[i+10]);
            break;
        }
        case FilterPacketField::LLH_POSITION: {
            if (field_len < 34) {
                return false;
            }
            next_filter_data.lat = be64todouble_ptr(packet.payload, i+2) * 1.0e7; // Decimal degrees to degrees
            next_filter_data.lon = be64todouble_ptr(packet.payload, i+10) * 1.0e7;
            next_filter_data.hae_altitude = be64todouble_ptr(packet.payload, i+26) * 1.0e2; // Meters to cm
            break;
        }
        case FilterPacketField::NED_VELOCITY: {
            if (field_len < 14) {
                return false;
            }
            next_filter_data.ned_velocity_north = be32tofloat_ptr(packet.payload, i+2);
            next_filter_data.ned_velocity_east = be32tofloat_ptr(packet.payload, i+6);
            next_filter_data.ned_velocity_down = be32tofloat_ptr(packet.payload, i+10);
            break;
        }
        case FilterPacketField::ATTITUDE_QUAT: {
            if (field_len < 18) {
                return false;
            }
            next_filter_data.attitude_quat = populate_quaternion(packet.payload, i+2);
            next_filter_data.attitude_quat.normalize();
            break;
        }
        case FilterPacketField::LLH_POSITION_UNCERTAINTY: {
            if (field_len < 14) {
                return false;
            }
            const float north_pos_acc = be32tofloat_ptr(packet.payload, i+2);
            const float east_pos_acc = be32tofloat_ptr(packet.payload, i+6);
            const float down_pos_acc = be32tofloat_ptr(packet.payload, i+10);
            next_filter_data.ned_position_uncertainty = Vector3f(
                north_pos_acc,
                east_pos_acc,
                down_pos_acc
            );
            break;
        }
        case FilterPacketField::NED_VELOCITY_UNCERTAINTY: {
            if (field_len < 14) {
                return false;
            }
            const float north_vel_uncertainty = be32tofloat_ptr(packet.payload, i+2);
            const float east_vel_uncertainty = be32tofloat_ptr(packet.payload, i+6);
            const float down_vel_uncertainty = be32tofloat_ptr(packet.payload, i+10);
            next_filter_data.ned_velocity_uncertainty = Vector3f(
                north_vel_uncertainty,
                east_vel_uncertainty,
                down_vel_uncertainty
            );
            break;
        }
        case FilterPacketField::FILTER_STATUS: {
            if (field_len < 8) {
                return false;
            }
            next_filter_status.state = be16toh_ptr(&packet.payload[i+2]);
            next_filter_status.mode = be16toh_ptr(&packet.payload[i+4]);
            next_filter_status.flags = be16toh_ptr(&packet.payload[i+6]);
            break;
        }
        }
        i += field_len;
    }
    filter_data = next_filter_data;
    filter_status = next_filter_status;
    last_filter_pkt = AP_HAL::millis();
    return true;
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
    // https://github.com/clemense/quaternion-conventions
    // AP follows W + Xi + Yj + Zk format.
    // Microstrain follows the same
    return Quaternion {
        be32tofloat_ptr(data, offset),
        be32tofloat_ptr(data, offset+4),
        be32tofloat_ptr(data, offset+8),
        be32tofloat_ptr(data, offset+12)
    };
}

bool AP_MicroStrain::get_gnss_instance(const DescriptorSet& descriptor, uint8_t& instance){
    bool success = false;
    
    switch(descriptor) {
    case DescriptorSet::GNSSData:
    case DescriptorSet::GNSSRecv1:
        instance = 0;
        success = true;
        break;
    case DescriptorSet::GNSSRecv2:
        instance = 1;
        success = true;
        break;
    default:
        break;
    }
    return success;
}



#endif // AP_MICROSTRAIN_ENABLED
