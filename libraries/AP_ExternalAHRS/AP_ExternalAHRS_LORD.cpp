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
  suppport for LORD Microstrain CX5/GX5-45 serially connected AHRS Systems
 */

#define ALLOW_DOUBLE_MATH_FUNCTIONS

#include "AP_ExternalAHRS_LORD.h"
#if HAL_EXTERNAL_AHRS_LORD_ENABLED
#include <AP_Baro/AP_Baro.h>
#include <AP_Compass/AP_Compass.h>
#include <AP_GPS/AP_GPS.h>
#include <AP_InertialSensor/AP_InertialSensor.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_Logger/AP_Logger.h>
#include <AP_HAL/utility/sparse-endian.h>
#include <AP_BoardConfig/AP_BoardConfig.h>


enum class DescriptorSet {
    BaseCommand = 0x01,
    DMCommand = 0x0C,
    SystemCommand = 0x7F,
    IMUData = 0x80,
    GNSSData = 0x81,
    EstimationData = 0x82
};

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

extern const AP_HAL::HAL &hal;

AP_ExternalAHRS_LORD::AP_ExternalAHRS_LORD(AP_ExternalAHRS *_frontend,
        AP_ExternalAHRS::state_t &_state): AP_ExternalAHRS_backend(_frontend, _state)
{
    auto &sm = AP::serialmanager();
    uart = sm.find_serial(AP_SerialManager::SerialProtocol_AHRS, 0);

    baudrate = sm.find_baudrate(AP_SerialManager::SerialProtocol_AHRS, 0);
    port_num = sm.find_portnum(AP_SerialManager::SerialProtocol_AHRS, 0);

    if (!uart) {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "ExternalAHRS no UART");
        return;
    }

    if (!hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&AP_ExternalAHRS_LORD::update_thread, void), "AHRS", 2048, AP_HAL::Scheduler::PRIORITY_SPI, 0)) {
        AP_BoardConfig::allocation_error("Failed to allocate ExternalAHRS update thread");
    }

    hal.scheduler->delay(5000);
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "LORD ExternalAHRS initialised");
}

void AP_ExternalAHRS_LORD::update_thread(void)
{
    if (!port_open) {
        port_open = true;
        uart->begin(baudrate);
    }

    while (true) {
        build_packet();
        hal.scheduler->delay_microseconds(100);
    }
}

// Builds packets by looking at each individual byte, once a full packet has been read in it checks the checksum then handles the packet.
void AP_ExternalAHRS_LORD::build_packet()
{
    WITH_SEMAPHORE(sem);
    uint32_t nbytes = MIN(uart->available(), 2048u);
    while (nbytes--> 0) {
        const int16_t b = uart->read();

        if (b < 0) {
            break;
        }

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
                    handle_packet(message_in.packet);
                }
            }
            break;
        }
    }
}

// returns true if the fletcher checksum for the packet is valid, else false.
bool AP_ExternalAHRS_LORD::valid_packet(const LORD_Packet & packet) const
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

// Calls the correct functions based on the packet descriptor of the packet
void AP_ExternalAHRS_LORD::handle_packet(const LORD_Packet& packet)
{
    switch ((DescriptorSet) packet.header[2]) {
    case DescriptorSet::IMUData:
        handle_imu(packet);
        post_imu();
        break;
    case DescriptorSet::GNSSData:
        handle_gnss(packet);
        break;
    case DescriptorSet::EstimationData:
        handle_filter(packet);
        post_filter();
        break;
    case DescriptorSet::BaseCommand:
    case DescriptorSet::DMCommand:
    case DescriptorSet::SystemCommand:
        break;
    }
}

// Collects data from an imu packet into `imu_data`
void AP_ExternalAHRS_LORD::handle_imu(const LORD_Packet& packet)
{
    last_ins_pkt = AP_HAL::millis();

    // Iterate through fields of varying lengths in INS packet
    for (uint8_t i = 0; i < packet.header[3]; i +=  packet.payload[i]) {
        switch ((INSPacketField) packet.payload[i+1]) {
        // Scaled Ambient Pressure
        case INSPacketField::PRESSURE: {
            imu_data.pressure = extract_float(packet.payload, i+2) * 100; // Convert millibar to pascals
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

// Posts data from an imu packet to `state` and `handle_external` methods
void AP_ExternalAHRS_LORD::post_imu() const
{
    {
        WITH_SEMAPHORE(state.sem);
        state.accel = imu_data.accel;
        state.gyro = imu_data.gyro;

        state.quat = imu_data.quat;
        state.have_quaternion = true;
    }

    {
        AP_ExternalAHRS::ins_data_message_t ins {
            accel: imu_data.accel,
            gyro: imu_data.gyro,
            temperature: -300
        };
        AP::ins().handle_external(ins);
    }

    {
        AP_ExternalAHRS::mag_data_message_t mag {
            field: imu_data.mag
        };
        AP::compass().handle_external(mag);
    }

#if AP_BARO_EXTERNALAHRS_ENABLED
    {
        const AP_ExternalAHRS::baro_data_message_t baro {
            instance: 0,
            pressure_pa: imu_data.pressure,
            // setting temp to 25 effectively disables barometer temperature calibrations - these are already performed by lord
            temperature: 25,
        };        
        AP::baro().handle_external(baro);
    }
#endif
}

// Collects data from a gnss packet into `gnss_data`
void AP_ExternalAHRS_LORD::handle_gnss(const LORD_Packet &packet)
{
    last_gps_pkt = AP_HAL::millis();

    // Iterate through fields of varying lengths in GNSS packet
    for (uint8_t i = 0; i < packet.header[3]; i += packet.payload[i]) {
        switch ((GNSSPacketField) packet.payload[i+1]) {
        // GPS Time
        case GNSSPacketField::GPS_TIME: {
            gnss_data.tow_ms = extract_double(packet.payload, i+2) * 1000; // Convert seconds to ms
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
            gnss_data.lat = extract_double(packet.payload, i+2) * 1.0e7; // Decimal degrees to degrees
            gnss_data.lon = extract_double(packet.payload, i+10) * 1.0e7;
            gnss_data.wgs84_altitude = extract_double(packet.payload, i+18);
            gnss_data.msl_altitude = extract_double(packet.payload, i+26) * 1.0e2; // Meters to cm
            gnss_data.horizontal_position_accuracy = extract_float(packet.payload, i+34);
            gnss_data.vertical_position_accuracy = extract_float(packet.payload, i+38);
            break;
        }
        // DOP Data
        case GNSSPacketField::DOP_DATA: {
            gnss_data.hdop = extract_float(packet.payload, i+10);
            gnss_data.vdop = extract_float(packet.payload, i+14);
            break;
        }
        // NED Velocity
        case GNSSPacketField::NED_VELOCITY: {
            gnss_data.ned_velocity_north = extract_float(packet.payload, i+2);
            gnss_data.ned_velocity_east = extract_float(packet.payload, i+6);
            gnss_data.ned_velocity_down = extract_float(packet.payload, i+10);
            gnss_data.speed_accuracy = extract_float(packet.payload, i+26);
            break;
        }
        }
    }
}

void AP_ExternalAHRS_LORD::handle_filter(const LORD_Packet &packet)
{
    last_filter_pkt = AP_HAL::millis();

    // Iterate through fields of varying lengths in filter packet
    for (uint8_t i = 0; i < packet.header[3]; i += packet.payload[i]) {
        switch ((FilterPacketField) packet.payload[i+1]) {
        // GPS Timestamp
        case FilterPacketField::GPS_TIME: {
            filter_data.tow_ms = extract_double(packet.payload, i+2) * 1000; // Convert seconds to ms
            filter_data.week = be16toh_ptr(&packet.payload[i+10]);
            break;
        }
        // LLH Position
        case FilterPacketField::LLH_POSITION: {
            filter_data.lat = extract_double(packet.payload, i+2) * 1.0e7; // Decimal degrees to degrees
            filter_data.lon = extract_double(packet.payload, i+10) * 1.0e7;
            filter_data.hae_altitude = extract_double(packet.payload, i+26) * 1.0e2; // Meters to cm
            break;
        }
        // NED Velocity
        case FilterPacketField::NED_VELOCITY: {
            filter_data.ned_velocity_north = extract_float(packet.payload, i+2);
            filter_data.ned_velocity_east = extract_float(packet.payload, i+6);
            filter_data.ned_velocity_down = extract_float(packet.payload, i+10);
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

void AP_ExternalAHRS_LORD::post_filter() const
{
    {
        WITH_SEMAPHORE(state.sem);
        state.velocity = Vector3f{filter_data.ned_velocity_north, filter_data.ned_velocity_east, filter_data.ned_velocity_down};
        state.have_velocity = true;

        const int32_t altitude = AP::gps().get_location_altitude_frame(gnss_data.msl_altitude, int32_t(gnss_data.wgs84_altitude * 100));
        state.location = Location{filter_data.lat, filter_data.lon, altitude, Location::AltFrame::ABSOLUTE};
        state.have_location = true;
    }

    AP_ExternalAHRS::gps_data_message_t gps {
        gps_week: filter_data.week,
        ms_tow: filter_data.tow_ms,
        fix_type: (uint8_t) gnss_data.fix_type,
        satellites_in_view: gnss_data.satellites,

        horizontal_pos_accuracy: gnss_data.horizontal_position_accuracy,
        vertical_pos_accuracy: gnss_data.vertical_position_accuracy,
        horizontal_vel_accuracy: gnss_data.speed_accuracy,

        hdop: gnss_data.hdop,
        vdop: gnss_data.vdop,

        longitude: filter_data.lon,
        latitude: filter_data.lat,
        msl_altitude: gnss_data.msl_altitude,
        wgs84_altitude: gnss_data.wgs84_altitude,

        ned_vel_north: filter_data.ned_velocity_north,
        ned_vel_east: filter_data.ned_velocity_east,
        ned_vel_down: filter_data.ned_velocity_down,
    };

    if (gps.fix_type >= 3 && !state.have_origin) {
        WITH_SEMAPHORE(state.sem);

        const int32_t altitude = AP::gps().get_location_altitude_frame(gnss_data.msl_altitude, int32_t(gnss_data.wgs84_altitude * 100));
        state.origin = Location{int32_t(filter_data.lat),
                                int32_t(filter_data.lon),
                                int32_t(altitude),
                                Location::AltFrame::ABSOLUTE};
        state.have_origin = true;
    }

    AP::gps().handle_external(gps);
}

int8_t AP_ExternalAHRS_LORD::get_port(void) const
{
    if (!uart) {
        return -1;
    }
    return port_num;
};

bool AP_ExternalAHRS_LORD::healthy(void) const
{
    uint32_t now = AP_HAL::millis();
    return (now - last_ins_pkt < 40 && now - last_gps_pkt < 500 && now - last_filter_pkt < 500);
}

bool AP_ExternalAHRS_LORD::initialised(void) const
{
    return last_ins_pkt != 0 && last_gps_pkt != 0 && last_filter_pkt != 0;
}

bool AP_ExternalAHRS_LORD::pre_arm_check(char *failure_msg, uint8_t failure_msg_len) const
{
    if (!healthy()) {
        hal.util->snprintf(failure_msg, failure_msg_len, "LORD unhealthy");
        return false;
    }
    if (gnss_data.fix_type < 3) {
        hal.util->snprintf(failure_msg, failure_msg_len, "LORD no GPS lock");
        return false;
    }
    if (filter_status.state != 0x02) {
        hal.util->snprintf(failure_msg, failure_msg_len, "LORD filter not running");
        return false;
    }

    return true;
}

void AP_ExternalAHRS_LORD::get_filter_status(nav_filter_status &status) const
{
    memset(&status, 0, sizeof(status));
    if (last_ins_pkt != 0 && last_gps_pkt != 0) {
        status.flags.initalized = 1;
    }
    if (healthy() && last_ins_pkt != 0) {
        status.flags.attitude = 1;
        status.flags.vert_vel = 1;
        status.flags.vert_pos = 1;

        if (gnss_data.fix_type >= 3) {
            status.flags.horiz_vel = 1;
            status.flags.horiz_pos_rel = 1;
            status.flags.horiz_pos_abs = 1;
            status.flags.pred_horiz_pos_rel = 1;
            status.flags.pred_horiz_pos_abs = 1;
            status.flags.using_gps = 1;
        }
    }
}

void AP_ExternalAHRS_LORD::send_status_report(mavlink_channel_t chan) const
{
    // prepare flags
    uint16_t flags = 0;
    nav_filter_status filterStatus;
    get_filter_status(filterStatus);
    if (filterStatus.flags.attitude) {
        flags |= EKF_ATTITUDE;
    }
    if (filterStatus.flags.horiz_vel) {
        flags |= EKF_VELOCITY_HORIZ;
    }
    if (filterStatus.flags.vert_vel) {
        flags |= EKF_VELOCITY_VERT;
    }
    if (filterStatus.flags.horiz_pos_rel) {
        flags |= EKF_POS_HORIZ_REL;
    }
    if (filterStatus.flags.horiz_pos_abs) {
        flags |= EKF_POS_HORIZ_ABS;
    }
    if (filterStatus.flags.vert_pos) {
        flags |= EKF_POS_VERT_ABS;
    }
    if (filterStatus.flags.terrain_alt) {
        flags |= EKF_POS_VERT_AGL;
    }
    if (filterStatus.flags.const_pos_mode) {
        flags |= EKF_CONST_POS_MODE;
    }
    if (filterStatus.flags.pred_horiz_pos_rel) {
        flags |= EKF_PRED_POS_HORIZ_REL;
    }
    if (filterStatus.flags.pred_horiz_pos_abs) {
        flags |= EKF_PRED_POS_HORIZ_ABS;
    }
    if (!filterStatus.flags.initalized) {
        flags |= EKF_UNINITIALIZED;
    }

    // send message
    const float vel_gate = 4; // represents hz value data is posted at
    const float pos_gate = 4; // represents hz value data is posted at
    const float hgt_gate = 4; // represents hz value data is posted at
    const float mag_var = 0; //we may need to change this to be like the other gates, set to 0 because mag is ignored by the ins filter in vectornav
    mavlink_msg_ekf_status_report_send(chan, flags,
                                       gnss_data.speed_accuracy/vel_gate, gnss_data.horizontal_position_accuracy/pos_gate, gnss_data.vertical_position_accuracy/hgt_gate,
                                       mag_var, 0, 0);

}

Vector3f AP_ExternalAHRS_LORD::populate_vector3f(const uint8_t *data, uint8_t offset) const
{
    return Vector3f {
        extract_float(data, offset),
        extract_float(data, offset+4),
        extract_float(data, offset+8)
    };
}

Quaternion AP_ExternalAHRS_LORD::populate_quaternion(const uint8_t *data, uint8_t offset) const
{
    return Quaternion {
        extract_float(data, offset),
        extract_float(data, offset+4),
        extract_float(data, offset+8),
        extract_float(data, offset+12)
    };
}

float AP_ExternalAHRS_LORD::extract_float(const uint8_t *data, uint8_t offset) const
{
    uint32_t tmp = be32toh_ptr(&data[offset]);

    return *reinterpret_cast<float*>(&tmp);
}

double AP_ExternalAHRS_LORD::extract_double(const uint8_t *data, uint8_t offset) const
{
    uint64_t tmp = be64toh_ptr(&data[offset]);

    return *reinterpret_cast<double*>(&tmp);
}

#endif // HAL_EXTERNAL_AHRS_ENABLED

