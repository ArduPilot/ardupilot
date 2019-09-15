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
  simulator connection for webots simulator https://github.com/omichel/webots
*/

#pragma once
#include <cmath>
#include <AP_HAL/utility/Socket.h>
#include "SIM_Aircraft.h"

namespace SITL {

/*
  simulation interface
 */
class Webots : public Aircraft {
public:
    Webots(const char *frame_str);

    /* update model by one time step */
    void update(const struct sitl_input &input) override;

    /* output servo to simulator */
    void output(const struct sitl_input &input);

    /* static object creator */
    static Aircraft *create(const char *frame_str) {
        return new Webots(frame_str);
    }

    

private:
    const char *webots_ip = "127.0.0.1";

    // assume sensors are streamed on port 5599
    uint16_t webots_sensors_port = 5599; 

    enum {
        OUTPUT_ROVER=1,
        OUTPUT_QUAD=2,
        OUTPUT_TRICOPTER=3,
        OUTPUT_PWM=4
    } output_type;

    bool connect_sockets(void);
    bool parse_sensors(const char *json);
    bool sensors_receive(void);
    void output_rover(const struct sitl_input &input);
    void output_quad(const struct sitl_input &input);
    void output_tricopter(const struct sitl_input &input);
    void output_pwm(const struct sitl_input &input);
    void report_FPS();

    // buffer for parsing pose data in JSON format
    uint8_t sensor_buffer[50000];
    uint32_t sensor_buffer_len;

    SocketAPM *sim_sock;

    uint32_t no_data_counter;
    uint32_t connect_counter;

    double initial_time_s;
    double extrapolated_s;
    double average_frame_time_s;

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
        { "", "ts", &state.timestamp, DATA_DOUBLE },
        { ".imu", "av",    &state.imu.angular_velocity, DATA_VECTOR3F },
        { ".imu", "la", &state.imu.linear_acceleration, DATA_VECTOR3F },
        { ".imu", "mf",      &state.imu.magnetic_field, DATA_VECTOR3F },
        { ".gps", "x", &state.gps.x, DATA_FLOAT },
        { ".gps", "y", &state.gps.y, DATA_FLOAT },
        { ".gps", "z", &state.gps.z, DATA_FLOAT },
        { ".pose", "roll",  &state.pose.roll, DATA_FLOAT },
        { ".pose", "pitch", &state.pose.pitch, DATA_FLOAT },
        { ".pose", "yaw",   &state.pose.yaw, DATA_FLOAT },
        { ".velocity", "wlv", &state.velocity.world_linear_velocity, DATA_VECTOR3F },
        { ".scan", "pl", &state.scanner.points, DATA_VECTOR3F_ARRAY },
        { ".scan", "rl", &state.scanner.ranges, DATA_FLOAT_ARRAY },
    };
};


} // namespace SITL
