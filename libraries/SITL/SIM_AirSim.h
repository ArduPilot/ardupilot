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
	Simulator connector for Airsim: https://github.com/Microsoft/AirSim
*/

#pragma once

#include <AP_HAL/utility/Socket.h>
#include "SIM_Aircraft.h"

namespace SITL {

/* 
	Airsim Simulator
*/

class AirSim : public Aircraft {
public:
	AirSim(const char *frame_str);

	/* update model by one time step */
	void update(const struct sitl_input &input) override;

	/* static object creator */
    static Aircraft *create(const char *frame_str) {
        return new AirSim(frame_str);
    }

    /*  Create and set in/out socket for Airsim simulator */
    void set_interface_ports(const char* address, const int port_in, const int port_out) override;

private:
    enum class OutputType {
        Copter = 1,
        Rover = 2
    } output_type;

    // Control packet for Rover
    struct rover_packet {
        float throttle;     // -1 to 1
        float steering;     // -1 to 1
    };

    // rotor control packet sent by Ardupilot
    static const int kArduCopterRotorControlCount = 11;

    struct servo_packet {
		uint16_t pwm[kArduCopterRotorControlCount];
	};

	// default connection_info_.ip_address
	const char *airsim_ip = "127.0.0.1";

	// connection_info_.ip_port
	uint16_t airsim_sensor_port = 9003;

	// connection_info_.sitl_ip_port
	uint16_t airsim_control_port = 9002;

	SocketAPM sock;

    double average_frame_time;
    uint64_t frame_counter;
    uint64_t last_frame_count;
    uint64_t last_timestamp;

    void output_copter(const struct sitl_input &input);
    void output_rover(const struct sitl_input &input);
    void recv_fdm();
    void report_FPS(void);

	bool parse_sensors(const char *json);

	// buffer for parsing pose data in JSON format
    uint8_t sensor_buffer[65000];
    uint32_t sensor_buffer_len;

	enum data_type {
		DATA_UINT64,
        DATA_FLOAT,
        DATA_DOUBLE,
        DATA_VECTOR3F,
        DATA_VECTOR3F_ARRAY,
        DATA_FLOAT_ARRAY,
    };

    struct {
        uint64_t timestamp;
        struct {
            Vector3f angular_velocity;
            Vector3f linear_acceleration;
        } imu;
        struct {
            double lat, lon, alt;
        } gps;
        struct {
            float roll, pitch, yaw;
        } pose;
        struct {
            Vector3f world_linear_velocity;
        } velocity;
        struct {
            struct vector3f_array points;
        } lidar;
        struct {
            struct float_array rc_channels;
        } rc;
    } state;

    // table to aid parsing of JSON sensor data
    struct keytable {
        const char *section;
        const char *key;
        void *ptr;
        enum data_type type;
    } keytable[12] = {
        { "", "timestamp", &state.timestamp, DATA_UINT64 },
        { "imu", "angular_velocity",    &state.imu.angular_velocity, DATA_VECTOR3F },
        { "imu", "linear_acceleration", &state.imu.linear_acceleration, DATA_VECTOR3F },
        { "gps", "lat", &state.gps.lat, DATA_DOUBLE },
        { "gps", "lon", &state.gps.lon, DATA_DOUBLE },
        { "gps", "alt", &state.gps.alt, DATA_DOUBLE },
        { "pose", "roll",  &state.pose.roll, DATA_FLOAT },
        { "pose", "pitch", &state.pose.pitch, DATA_FLOAT },
        { "pose", "yaw",   &state.pose.yaw, DATA_FLOAT },
        { "velocity", "world_linear_velocity", &state.velocity.world_linear_velocity, DATA_VECTOR3F },
        { "lidar", "point_cloud", &state.lidar.points, DATA_VECTOR3F_ARRAY },
        { "rc", "channels", &state.rc.rc_channels, DATA_FLOAT_ARRAY },
    };
};

}
