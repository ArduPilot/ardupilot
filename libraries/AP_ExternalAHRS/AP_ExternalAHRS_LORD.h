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
  suppport for serial connected AHRS systems
 */

#pragma once

#include "AP_ExternalAHRS_backend.h"

#ifndef HAL_EXTERNAL_AHRS_LORD_ENABLED
#define HAL_EXTERNAL_AHRS_LORD_ENABLED HAL_EXTERNAL_AHRS_ENABLED
#endif

#if HAL_EXTERNAL_AHRS_LORD_ENABLED

#include <GCS_MAVLink/GCS_MAVLink.h>

class AP_ExternalAHRS_LORD: public AP_ExternalAHRS_backend
{
public:

    AP_ExternalAHRS_LORD(AP_ExternalAHRS *frontend, AP_ExternalAHRS::state_t &state);

    // get serial port number, -1 for not enabled
    int8_t get_port(void) const override;

    // accessors for AP_AHRS
    bool healthy(void) const override;
    bool initialised(void) const override;
    bool pre_arm_check(char *failure_msg, uint8_t failure_msg_len) const override;
    void get_filter_status(nav_filter_status &status) const override;
    void send_status_report(class GCS_MAVLINK &link) const override;

    // check for new data
    void update() override {
        build_packet();
    };

private:

    enum class ParseState {
        WaitingFor_SyncOne,
        WaitingFor_SyncTwo,
        WaitingFor_Descriptor,
        WaitingFor_PayloadLength,
        WaitingFor_Data,
        WaitingFor_Checksum
    };

    void update_thread();

    AP_HAL::UARTDriver *uart;
    HAL_Semaphore sem;

    uint32_t baudrate;
    int8_t port_num;
    bool port_open = false;

    const uint8_t SYNC_ONE = 0x75;
    const uint8_t SYNC_TWO = 0x65;

    uint32_t last_ins_pkt;
    uint32_t last_gps_pkt;
    uint32_t last_filter_pkt;

    // A LORD packet can be a maximum of 261 bytes
    struct LORD_Packet {
        uint8_t header[4];
        uint8_t payload[255];
        uint8_t checksum[2];
    };

    struct {
        LORD_Packet packet;
        ParseState state;
        uint8_t index;
    } message_in;

    struct {
        Vector3f accel;
        Vector3f gyro;
        Vector3f mag;
        Quaternion quat;
        float pressure;
    } imu_data;

    struct {
        uint16_t week;
        uint32_t tow_ms;
        GPS_FIX_TYPE fix_type;
        uint8_t satellites;
        float horizontal_position_accuracy;
        float vertical_position_accuracy;
        float hdop;
        float vdop;
        int32_t lon;
        int32_t lat;
        int32_t msl_altitude;
        float ned_velocity_north;
        float ned_velocity_east;
        float ned_velocity_down;
        float speed_accuracy;
    } gnss_data;

    struct {
        uint16_t state;
        uint16_t mode;
        uint16_t flags;
    } filter_status;

    struct {
        uint16_t week;
        uint32_t tow_ms;
        float horizontal_position_accuracy;
        float vertical_position_accuracy;
        int32_t lon;
        int32_t lat;
        int32_t hae_altitude;
        float ned_velocity_north;
        float ned_velocity_east;
        float ned_velocity_down;
        float speed_accuracy;
    } filter_data;

    void build_packet();
    bool valid_packet(const LORD_Packet &packet) const;
    void handle_packet(const LORD_Packet &packet);
    void handle_imu(const LORD_Packet &packet);
    void handle_gnss(const LORD_Packet &packet);
    void handle_filter(const LORD_Packet &packet);
    void post_imu() const;
    void post_gnss() const;
    void post_filter() const;

    Vector3f populate_vector3f(const uint8_t* data, uint8_t offset) const;
    Quaternion populate_quaternion(const uint8_t* data, uint8_t offset) const;
    float extract_float(const uint8_t* data, uint8_t offset) const;
    double extract_double(const uint8_t* data, uint8_t offset) const;

};

#endif // HAL_EXTERNAL_AHRS_ENABLED

