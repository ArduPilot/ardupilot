/* 
	Simulator Connector for AirSim
*/

#include "SIM_AirSim.h"

#include <stdio.h>
#include <arpa/inet.h>
#include <errno.h>

#include <AP_HAL/AP_HAL.h>
#include <AP_Logger/AP_Logger.h>

extern const AP_HAL::HAL& hal;

using namespace SITL;

AirSim::AirSim(const char *home_str, const char *frame_str) :
	Aircraft(home_str, frame_str),
	sock(true)
{
	printf("Starting SITL Airsim\n");
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
void AirSim::send_servos(const struct sitl_input &input)
{
	servo_packet pkt{0};

	for (uint8_t i=0; i<kArduCopterRotorControlCount; i++) {
		pkt.pwm[i] = input.servos[i];
	}

	ssize_t send_ret = sock.sendto(&pkt, sizeof(pkt), airsim_ip, airsim_control_port);
	if (send_ret != sizeof(pkt)) {
		if (send_ret <= 0) {
			printf("Unable to send servo output to %s:%u - Error: %s, Return value: %ld\n",
					 airsim_ip, airsim_control_port, strerror(errno), send_ret);
		} else {
			printf("Sent %ld bytes instead of %ld bytes\n", send_ret, sizeof(pkt));
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

    bool parse_ok = parse_sensors((const char *)(p1+1));

    memmove(sensor_buffer, p2, sensor_buffer_len - (p2 - sensor_buffer));
    sensor_buffer_len = sensor_buffer_len - (p2 - sensor_buffer);

    if (!parse_ok) {
        return;
    }

    accel_body = Vector3f(state.imu.linear_acceleration[0],
                          state.imu.linear_acceleration[1],
                          state.imu.linear_acceleration[2]);

    gyro = Vector3f(state.imu.angular_velocity[0],
                    state.imu.angular_velocity[1],
                    state.imu.angular_velocity[2]);

    velocity_ef = Vector3f(state.velocity.world_linear_velocity[0],
                           state.velocity.world_linear_velocity[1],
                           state.velocity.world_linear_velocity[0]);

    location.lat = state.gps.lat * 1.0e7;
    location.lng = state.gps.lon * 1.0e7;
    location.alt = state.gps.alt * 100.0f;

    dcm.from_euler(state.pose.roll, state.pose.pitch, state.pose.yaw);

    if (last_state.timestamp) {
        double deltat = state.timestamp - last_state.timestamp;
        time_now_us += deltat;

        if (deltat > 0 && deltat < 100000) {
            if (average_frame_time < 1) {
                average_frame_time = deltat;
            }
            average_frame_time = average_frame_time * 0.98 + deltat * 0.02;
        }
    }

#if 0
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

    last_state = state;
}

/*
  update the AirSim simulation by one time step
*/
void AirSim::update(const struct sitl_input &input)
{
	send_servos(input);
    recv_fdm();
    // Airsim takes approximately 3ms between each message (or 333 Hz)
    adjust_frame_time(1.0e6/3000);
    time_advance();

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
