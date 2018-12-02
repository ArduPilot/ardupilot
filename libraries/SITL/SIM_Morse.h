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

    // assume we control vehicle on port 4000
    uint16_t morse_motion_port = 4000;

    bool connect_sockets(void);
    bool parse_sensors(const char *json);
    bool sensors_receive(void);
    void rover_output(const struct sitl_input &input);
    void report_FPS();

    // buffer for parsing pose data in JSON format
    uint8_t sensor_buffer[2048];
    uint32_t sensor_buffer_len;

    SocketAPM sensors_sock{false};
    SocketAPM motion_sock{false};

    bool sensors_sock_connected;
    bool motion_sock_connected;
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

    float last_steering_rps;
    float last_speed_ms;

    struct {
        double timestamp;
        struct {
            double angular_velocity[3];
            double linear_acceleration[3];
            double magnetic_field[3];
        } imu;
        struct {
            double x, y, z;
        } gps;
        struct {
            double roll, pitch, yaw;
        } pose;
        struct {
            double world_linear_velocity[3];
        } velocity;
    } state, last_state;

    // table to aid parsing of JSON sensor data
    struct keytable {
        const char *section;
        const char *key;
        double *ptr;
        bool is_vector3;
    } keytable[11] = {
        { "", "timestamp", &state.timestamp },
        { "vehicle.imu", "angular_velocity",    &state.imu.angular_velocity[0], true },
        { "vehicle.imu", "linear_acceleration", &state.imu.linear_acceleration[0], true },
        { "vehicle.imu", "magnetic_field",      &state.imu.magnetic_field[0], true },
        { "vehicle.gps", "x", &state.gps.x },
        { "vehicle.gps", "y", &state.gps.y },
        { "vehicle.gps", "z", &state.gps.z },
        { "vehicle.pose", "roll",  &state.pose.roll },
        { "vehicle.pose", "pitch", &state.pose.pitch },
        { "vehicle.pose", "yaw",   &state.pose.yaw },
        { "vehicle.velocity", "world_linear_velocity", &state.velocity.world_linear_velocity[0], true },
    };
};


} // namespace SITL
