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
  simulator connection for morse simulator http://morse-simulator.github.io/
*/

#pragma once

#include <AP_HAL/utility/Socket.h>
#include "SIM_Aircraft.h"

namespace SITL {

/*
  simulation interface
 */
class Morse : public Aircraft {
public:
    Morse(const char *home_str, const char *frame_str);

    /* update model by one time step */
    void update(const struct sitl_input &input);

    /* static object creator */
    static Aircraft *create(const char *home_str, const char *frame_str) {
        return new Morse(home_str, frame_str);
    }

private:
    const char *morse_ip = "127.0.0.1";

    // assume sensors are streamed on port 60000
    uint16_t morse_sensors_port = 60000;

    // assume we control vehicle on port 60001
    uint16_t morse_control_port = 60001;

    enum {
        OUTPUT_ROVER,
        OUTPUT_QUAD
    } output_type;

    bool connect_sockets(void);
    bool parse_sensors(const char *json);
    bool sensors_receive(void);
    void output_rover(const struct sitl_input &input);
    void output_quad(const struct sitl_input &input);
    void report_FPS();

    // buffer for parsing pose data in JSON format
    uint8_t sensor_buffer[50000];
    uint32_t sensor_buffer_len;

    SocketAPM *sensors_sock;
    SocketAPM *control_sock;

    uint32_t no_data_counter;
    uint32_t connect_counter;

    double initial_time_s;
    double last_time_s;
    double extrapolated_s;
    double average_frame_time_s;

    Vector3f position_offset;

    uint64_t socket_frame_counter;
    uint64_t last_socket_frame_counter;
    uint64_t frame_counter;
    double last_frame_count_s;

    enum data_type {
        DATA_FLOAT,
        DATA_DOUBLE,
        DATA_VECTOR3F,
        DATA_VECTOR3F_ARRAY,
        DATA_FLOAT_ARRAY,
    };

    struct {
        double timestamp;
        struct {
            Vector3f angular_velocity;
            Vector3f linear_acceleration;
            Vector3f magnetic_field;
        } imu;
        struct {
            float x, y, z;
        } gps;
        struct {
            float roll, pitch, yaw;
        } pose;
        struct {
            Vector3f world_linear_velocity;
        } velocity;
        struct {
            struct vector3f_array points;
            struct float_array ranges;
        } scanner;
    } state, last_state;

    // table to aid parsing of JSON sensor data
    struct keytable {
        const char *section;
        const char *key;
        void *ptr;
        enum data_type type;
    } keytable[13] = {
        { "", "timestamp", &state.timestamp, DATA_DOUBLE },
        { "vehicle.imu", "angular_velocity",    &state.imu.angular_velocity, DATA_VECTOR3F },
        { "vehicle.imu", "linear_acceleration", &state.imu.linear_acceleration, DATA_VECTOR3F },
        { "vehicle.imu", "magnetic_field",      &state.imu.magnetic_field, DATA_VECTOR3F },
        { "vehicle.gps", "x", &state.gps.x, DATA_FLOAT },
        { "vehicle.gps", "y", &state.gps.y, DATA_FLOAT },
        { "vehicle.gps", "z", &state.gps.z, DATA_FLOAT },
        { "vehicle.pose", "roll",  &state.pose.roll, DATA_FLOAT },
        { "vehicle.pose", "pitch", &state.pose.pitch, DATA_FLOAT },
        { "vehicle.pose", "yaw",   &state.pose.yaw, DATA_FLOAT },
        { "vehicle.velocity", "world_linear_velocity", &state.velocity.world_linear_velocity, DATA_VECTOR3F },
        { "vehicle.scan", "point_list", &state.scanner.points, DATA_VECTOR3F_ARRAY },
        { "vehicle.scan", "range_list", &state.scanner.ranges, DATA_FLOAT_ARRAY },
    };
};


} // namespace SITL
