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
#pragma once

#include <AP_HAL/utility/Socket.h>
#include "SIM_Aircraft.h"

namespace SITL {

class JSON : public Aircraft {
public:
    JSON(const char *frame_str);

    /* update model by one time step */
    void update(const struct sitl_input &input) override;

    /* static object creator */
    static Aircraft *create(const char *frame_str) {
        return new JSON(frame_str);
    }

    /*  Create and set in/out socket for JSON generic simulator */
    void set_interface_ports(const char* address, const int port_in, const int port_out) override;

private:

    struct servo_packet {
        uint16_t magic = 18458; // constant magic value
        uint16_t frame_rate;
        uint32_t frame_count;
        uint16_t pwm[16];
    };

    // default connection_info_.ip_address
    const char *target_ip = "127.0.0.1";

    // default connection_info_.sitl_ip_port
    uint16_t control_port = 9002;

    SocketAPM sock;

    uint32_t frame_counter;
    double last_timestamp_s;

    void output_servos(const struct sitl_input &input);
    void recv_fdm(const struct sitl_input &input);

    bool parse_sensors(const char *json);

    // buffer for parsing pose data in JSON format
    uint8_t sensor_buffer[65000];
    uint32_t sensor_buffer_len;

    enum data_type {
        DATA_UINT64,
        DATA_FLOAT,
        DATA_DOUBLE,
        DATA_VECTOR3F,
    };

    struct {
        double timestamp_s;
        struct {
            Vector3f gyro;
            Vector3f accel_body;
        } imu;
        Vector3f position;
        Vector3f attitude;
        Vector3f velocity;
    } state;

    // table to aid parsing of JSON sensor data
    struct keytable {
        const char *section;
        const char *key;
        void *ptr;
        enum data_type type;
    } keytable[6] = {
        { "", "timestamp", &state.timestamp_s, DATA_DOUBLE },
        { "imu", "gyro",    &state.imu.gyro, DATA_VECTOR3F },
        { "imu", "accel_body", &state.imu.accel_body, DATA_VECTOR3F },
        { "", "position", &state.position, DATA_VECTOR3F },
        { "", "attitude", &state.attitude, DATA_VECTOR3F },
        { "", "velocity", &state.velocity, DATA_VECTOR3F },
    };
};

}
