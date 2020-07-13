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
	Simulator Connector for AirSim
*/

#include "SIM_AirSim.h"

#include <stdio.h>
#include <arpa/inet.h>
#include <errno.h>

#include <AP_HAL/AP_HAL.h>
#include <AP_Logger/AP_Logger.h>
#include <AP_HAL/utility/replace.h>

extern const AP_HAL::HAL& hal;

using namespace SITL;

AirSim::AirSim(const char *frame_str) :
	Aircraft(frame_str),
	sock(true)
{
    if (strstr(frame_str, "-copter")) {
        output_type = OutputType::Copter;
    } else if (strstr(frame_str, "-rover")) {
        output_type = OutputType::Rover;
    } else {
        // default to copter
        output_type = OutputType::Copter;
    }

	printf("Starting SITL Airsim type %u\n", (unsigned)output_type);
}

/*
	Create & set in/out socket
*/
void AirSim::set_interface_ports(const char* address, const int port_in, const int port_out)
{
	if (!sock.bind("0.0.0.0", port_in)) {
		printf("Unable to bind Airsim sensor_in socket at port %u - Error: %s\n",
				 port_in, strerror(errno));
		return;
	}
	printf("Bind SITL sensor input at %s:%u\n", "127.0.0.1", port_in);
	sock.set_blocking(false);
	sock.reuseaddress();

	airsim_ip = address;
	airsim_control_port = port_out;
	airsim_sensor_port = port_in;

	printf("AirSim control interface set to %s:%u\n", airsim_ip, airsim_control_port);
}

/*
	Decode and send servos
*/
void AirSim::output_copter(const struct sitl_input &input)
{
    servo_packet pkt;

	for (uint8_t i=0; i<kArduCopterRotorControlCount; i++) {
		pkt.pwm[i] = input.servos[i];
	}

	ssize_t send_ret = sock.sendto(&pkt, sizeof(pkt), airsim_ip, airsim_control_port);
	if (send_ret != sizeof(pkt)) {
		if (send_ret <= 0) {
			printf("Unable to send servo output to %s:%u - Error: %s, Return value: %ld\n",
                   airsim_ip, airsim_control_port, strerror(errno), (long)send_ret);
		} else {
			printf("Sent %ld bytes instead of %lu bytes\n", (long)send_ret, (unsigned long)sizeof(pkt));
		}
	}
}

void AirSim::output_rover(const struct sitl_input &input)
{
    rover_packet pkt;

    pkt.steering = 2*((input.servos[0]-1000)/1000.0f - 0.5f);
    pkt.throttle = 2*((input.servos[2]-1000)/1000.0f - 0.5f);

    ssize_t send_ret = sock.sendto(&pkt, sizeof(pkt), airsim_ip, airsim_control_port);
    if (send_ret != sizeof(pkt)) {
        if (send_ret <= 0) {
            printf("Unable to send control output to %s:%u - Error: %s, Return value: %ld\n",
                     airsim_ip, airsim_control_port, strerror(errno), (long)send_ret);
        } else {
            printf("Sent %ld bytes instead of %lu bytes\n", (long)send_ret, (unsigned long)sizeof(pkt));
        }
    }
}

/*
  very simple JSON parser for sensor data
  called with pointer to one row of sensor data, nul terminated

  This parser does not do any syntax checking, and is not at all
  general purpose
*/
bool AirSim::parse_sensors(const char *json)
{
    // printf("%s\n", json);
    for (uint16_t i=0; i<ARRAY_SIZE(keytable); i++) {
        struct keytable &key = keytable[i];

        /* look for section header */
        const char *p = strstr(json, key.section);
        if (!p) {
            // we don't have this sensor
            continue;
        }
        p += strlen(key.section)+1;

        // find key inside section
        p = strstr(p, key.key);
        if (!p) {
            printf("Failed to find key %s/%s\n", key.section, key.key);
            return false;
        }

        p += strlen(key.key)+3;
        switch (key.type) {
            case DATA_UINT64:
                *((uint64_t *)key.ptr) = strtoul(p, nullptr, 10);
                break;

            case DATA_FLOAT:
                *((float *)key.ptr) = atof(p);
                break;

            case DATA_DOUBLE:
                *((double *)key.ptr) = atof(p);
                break;

            case DATA_VECTOR3F: {
                Vector3f *v = (Vector3f *)key.ptr;
                if (sscanf(p, "[%f, %f, %f]", &v->x, &v->y, &v->z) != 3) {
                    printf("Failed to parse Vector3f for %s/%s\n", key.section, key.key);
                    return false;
                }
                break;
            }

            case DATA_VECTOR3F_ARRAY: {
                // - array of floats that represent [x,y,z] coordinate for each point hit within the range
                //       x0, y0, z0, x1, y1, z1, ..., xn, yn, zn
                // example: [23.1,0.677024,1.4784,-8.97607135772705,-8.976069450378418,-8.642673492431641e-07,]
                if (*p++ != '[') {
                    return false;
                }
                uint16_t n = 0;
                struct vector3f_array *v = (struct vector3f_array *)key.ptr;
                while (true) {
                    if (n >= v->length) {
                        Vector3f *d = (Vector3f *)realloc(v->data, sizeof(Vector3f)*(n+1));
                        if (d == nullptr) {
                            return false;
                        }
                        v->data = d;
                        v->length = n+1;
                    }
                    if (sscanf(p, "%f,%f,%f,", &v->data[n].x, &v->data[n].y, &v->data[n].z) != 3) {
                        printf("Failed to parse Vector3f for %s/%s[%u]\n", key.section, key.key, n);
                        return false;
                    }
                    n++;
                    // Goto 3rd occurence of ,
                    p = strchr(p,',');
                    if (!p) {
                        return false;
                    }
                    p++;
                    p = strchr(p,',');
                    if (!p) {
                        return false;
                    }
                    p++;
                    p = strchr(p,',');
                    if (!p) {
                        return false;
                    }
                    p++;
                    // Reached end of point cloud
                    if (p[0] == ']') {
                        break;
                    }
                }
                v->length = n;
                break;
            }

            case DATA_FLOAT_ARRAY: {
                // example: [18.0, 12.694079399108887]
                if (*p++ != '[') {
                    return false;
                }
                uint16_t n = 0;
                struct float_array *v = (struct float_array *)key.ptr;
                while (true) {
                    if (n >= v->length) {
                        float *d = (float *)realloc(v->data, sizeof(float)*(n+1));
                        if (d == nullptr) {
                            return false;
                        }
                        v->data = d;
                        v->length = n+1;
                    }
                    v->data[n] = atof(p);
                    n++;
                    p = strchr(p,',');
                    if (!p) {
                        break;
                    }
                    p++;
                }
                v->length = n;
                break;
            }
        }
    }
    return true;
}

/*
	Receive new sensor data from simulator
	This is a blocking function
*/
void AirSim::recv_fdm()
{
    // Receive sensor packet
    ssize_t ret = sock.recv(&sensor_buffer[sensor_buffer_len], sizeof(sensor_buffer)-sensor_buffer_len, 100);
    while (ret <= 0) {
        printf("No sensor message received - %s\n", strerror(errno));
        ret = sock.recv(&sensor_buffer[sensor_buffer_len], sizeof(sensor_buffer)-sensor_buffer_len, 100);
    }

    // convert '\n' into nul
    while (uint8_t *p = (uint8_t *)memchr(&sensor_buffer[sensor_buffer_len], '\n', ret)) {
        *p = 0;
    }
    sensor_buffer_len += ret;

    const uint8_t *p2 = (const uint8_t *)memrchr(sensor_buffer, 0, sensor_buffer_len);
    if (p2 == nullptr || p2 == sensor_buffer) {
        return;
    }
    const uint8_t *p1 = (const uint8_t *)memrchr(sensor_buffer, 0, p2 - sensor_buffer);
    if (p1 == nullptr) {
        return;
    }

    parse_sensors((const char *)(p1+1));

    memmove(sensor_buffer, p2, sensor_buffer_len - (p2 - sensor_buffer));
    sensor_buffer_len = sensor_buffer_len - (p2 - sensor_buffer);

    accel_body = Vector3f(state.imu.linear_acceleration[0],
                          state.imu.linear_acceleration[1],
                          state.imu.linear_acceleration[2]);

    gyro = Vector3f(state.imu.angular_velocity[0],
                    state.imu.angular_velocity[1],
                    state.imu.angular_velocity[2]);

    velocity_ef = Vector3f(state.velocity.world_linear_velocity[0],
                           state.velocity.world_linear_velocity[1],
                           state.velocity.world_linear_velocity[2]);

    location.lat = state.gps.lat * 1.0e7;
    location.lng = state.gps.lon * 1.0e7;
    location.alt = state.gps.alt * 100.0f;

    dcm.from_euler(state.pose.roll, state.pose.pitch, state.pose.yaw);

    if (last_timestamp) {
        int deltat = state.timestamp - last_timestamp;
        time_now_us += deltat;

        if (deltat > 0 && deltat < 100000) {
            if (average_frame_time < 1) {
                average_frame_time = deltat;
            }
            average_frame_time = average_frame_time * 0.98 + deltat * 0.02;
        }
    }

    scanner.points = state.lidar.points;

    rcin_chan_count = state.rc.rc_channels.length < 8 ? state.rc.rc_channels.length : 8;
    for (uint8_t i=0; i < rcin_chan_count; i++) {
        rcin[i] = state.rc.rc_channels.data[i];
    }

#if 0
// @LoggerMessage: ASM1
// @Description: AirSim simulation data
// @Field: TimeUS: Time since system startup
// @Field: TUS: Simulation's timestamp
// @Field: R: Simulation's roll
// @Field: P: Simulation's pitch
// @Field: Y: Simulation's yaw
// @Field: GX: Simulated gyroscope, X-axis
// @Field: GY: Simulated gyroscope, Y-axis
// @Field: GZ: Simulated gyroscope, Z-axis
    AP::logger().Write("ASM1", "TimeUS,TUS,R,P,Y,GX,GY,GZ",
                       "QQffffff",
                       AP_HAL::micros64(),
                       state.timestamp,
                       degrees(state.pose.roll),
                       degrees(state.pose.pitch),
                       degrees(state.pose.yaw),
                       degrees(gyro.x),
                       degrees(gyro.y),
                       degrees(gyro.z));

    Vector3f velocity_bf = dcm.transposed() * velocity_ef;
    position = home.get_distance_NED(location);

// @LoggerMessage: ASM2
// @Description: More AirSim simulation data
// @Field: TimeUS: Time since system startup
// @Field: AX: simulation's acceleration, X-axis
// @Field: AY: simulation's acceleration, Y-axis
// @Field: AZ: simulation's acceleration, Z-axis
// @Field: VX: simulation's velocity, X-axis
// @Field: VY: simulation's velocity, Y-axis
// @Field: VZ: simulation's velocity, Z-axis
// @Field: PX: simulation's position, X-axis
// @Field: PY: simulation's position, Y-axis
// @Field: PZ: simulation's position, Z-axis
// @Field: Alt: simulation's gps altitude
// @Field: SD: simulation's earth-frame speed-down
    AP::logger().Write("ASM2", "TimeUS,AX,AY,AZ,VX,VY,VZ,PX,PY,PZ,Alt,SD",
                       "Qfffffffffff",
                       AP_HAL::micros64(),
                       accel_body.x,
                       accel_body.y,
                       accel_body.z,
                       velocity_bf.x,
                       velocity_bf.y,
                       velocity_bf.z,
                       position.x,
                       position.y,
                       position.z,
                       state.gps.alt,
                       velocity_ef.z);
#endif

    last_timestamp = state.timestamp;
}

/*
  update the AirSim simulation by one time step
*/
void AirSim::update(const struct sitl_input &input)
{
    switch (output_type) {
        case OutputType::Copter:
            output_copter(input);
            break;

        case OutputType::Rover:
            output_rover(input);
            break;
    }

    recv_fdm();

    // update magnetic field
    update_mag_field_bf();

    report_FPS();
}

/*
  report frame rates
 */
void AirSim::report_FPS(void)
{
    if (frame_counter++ % 1000 == 0) {
        if (last_frame_count != 0) {
            printf("FPS avg=%.2f\n", 1.0e6/average_frame_time);
        }
        last_frame_count = state.timestamp;
    }
}
