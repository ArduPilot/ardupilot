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
    Simulator Connector for JSON based interfaces
*/

#include "SIM_JSON.h"

#include <stdio.h>
#include <arpa/inet.h>
#include <errno.h>

#include <AP_HAL/AP_HAL.h>
#include <AP_Logger/AP_Logger.h>
#include <AP_HAL/utility/replace.h>

#define UDP_TIMEOUT_MS 100

extern const AP_HAL::HAL& hal;

using namespace SITL;

static const struct {
    const char *name;
    float value;
    bool save;
} sim_defaults[] = {
    { "BRD_OPTIONS", 0},
    { "INS_GYR_CAL", 0 },
    { "INS_ACC2OFFS_X",    0.001 },
    { "INS_ACC2OFFS_Y",    0.001 },
    { "INS_ACC2OFFS_Z",    0.001 },
    { "INS_ACC2SCAL_X",    1.001 },
    { "INS_ACC2SCAL_Y",    1.001 },
    { "INS_ACC2SCAL_Z",    1.001 },
    { "INS_ACCOFFS_X",     0.001 },
    { "INS_ACCOFFS_Y",     0.001 },
    { "INS_ACCOFFS_Z",     0.001 },
    { "INS_ACCSCAL_X",     1.001 },
    { "INS_ACCSCAL_Y",     1.001 },
    { "INS_ACCSCAL_Z",     1.001 },
};


JSON::JSON(const char *frame_str) :
    Aircraft(frame_str),
    sock(true)
{
    printf("Starting SITL: JSON\n");

    const char *colon = strchr(frame_str, ':');
    if (colon) {
        target_ip = colon+1;
    }

    for (uint8_t i=0; i<ARRAY_SIZE(sim_defaults); i++) {
    AP_Param::set_default_by_name(sim_defaults[i].name, sim_defaults[i].value);
        if (sim_defaults[i].save) {
            enum ap_var_type ptype;
            AP_Param *p = AP_Param::find(sim_defaults[i].name, &ptype);
            if (!p->configured()) {
                p->save();
            }
        }
    }
}

/*
    Create & set in/out socket
*/
void JSON::set_interface_ports(const char* address, const int port_in, const int port_out)
{
    sock.set_blocking(false);
    sock.reuseaddress();

    if (strcmp("127.0.0.1",address) != 0) {
        target_ip = address;
    }
    control_port = port_out;

    printf("JSON control interface set to %s:%u\n", target_ip, control_port);
}

/*
    Decode and send servos
*/
void JSON::output_servos(const struct sitl_input &input)
{
    servo_packet pkt;
    pkt.frame_rate = rate_hz;
    pkt.frame_count = frame_counter;
    for (uint8_t i=0; i<16; i++) {
        pkt.pwm[i] = input.servos[i];
    }

    size_t send_ret = sock.sendto(&pkt, sizeof(pkt), target_ip, control_port);
    if (send_ret != sizeof(pkt)) {
        if (send_ret <= 0) {
            printf("Unable to send servo output to %s:%u - Error: %s, Return value: %ld\n",
                   target_ip, control_port, strerror(errno), (long)send_ret);
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
bool JSON::parse_sensors(const char *json)
{
    //printf("%s\n", json);
    for (uint16_t i=0; i<ARRAY_SIZE(keytable); i++) {
        struct keytable &key = keytable[i];

        /* look for section header */
        const char *p = strstr(json, key.section);
        if (!p) {
            // we don't have this sensor
            printf("Failed to find %s\n", key.section);
            continue;
        }
        p += strlen(key.section)+1;

        // find key inside section
        p = strstr(p, key.key);
        if (!p) {
            printf("Failed to find key %s/%s\n", key.section, key.key);
            return false;
        }

        p += strlen(key.key)+2;
        switch (key.type) {
            case DATA_UINT64:
                *((uint64_t *)key.ptr) = strtoull(p, nullptr, 10);
                //printf("%s/%s = %lu\n", key.section, key.key, *((uint64_t *)key.ptr));
                break;

            case DATA_FLOAT:
                *((float *)key.ptr) = atof(p);
                //printf("%s/%s = %f\n", key.section, key.key, *((float *)key.ptr));
                break;

            case DATA_DOUBLE:
                *((double *)key.ptr) = atof(p);
                //printf("%s/%s = %f\n", key.section, key.key, *((double *)key.ptr));
                break;

            case DATA_VECTOR3F: {
                Vector3f *v = (Vector3f *)key.ptr;
                if (sscanf(p, "[%f, %f, %f]", &v->x, &v->y, &v->z) != 3) {
                    printf("Failed to parse Vector3f for %s/%s\n", key.section, key.key);
                    return false;
                }
                //printf("%s/%s = %f, %f, %f\n", key.section, key.key, v->x, v->y, v->z);
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
void JSON::recv_fdm(const struct sitl_input &input)
{
    // Receive sensor packet
    ssize_t ret = sock.recv(&sensor_buffer[sensor_buffer_len], sizeof(sensor_buffer)-sensor_buffer_len, UDP_TIMEOUT_MS);
    uint32_t wait_ms = UDP_TIMEOUT_MS;
    while (ret <= 0) {
        //printf("No JSON sensor message received - %s\n", strerror(errno));
        ret = sock.recv(&sensor_buffer[sensor_buffer_len], sizeof(sensor_buffer)-sensor_buffer_len, UDP_TIMEOUT_MS);
        wait_ms += UDP_TIMEOUT_MS;
        // if no sensor message is received after 10 second resend servos, this help cope with SITL and the physics getting out of sync
        if (wait_ms > 1000) {
            wait_ms = 0;
            printf("No JSON sensor message received, resending servos\n");
            output_servos(input);
        }
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

    accel_body = Vector3f(state.imu.accel_body[0],
                          state.imu.accel_body[1],
                          state.imu.accel_body[2]);

    gyro = Vector3f(state.imu.gyro[0],
                    state.imu.gyro[1],
                    state.imu.gyro[2]);

    velocity_ef = Vector3f(state.velocity[0],
                           state.velocity[1],
                           state.velocity[2]);

    position = Vector3f(state.position[0],
                            state.position[1],
                            state.position[2]);

    dcm.from_euler(state.attitude[0], state.attitude[1], state.attitude[2]);

    // Convert from a meters from origin physics to a lat long alt
    update_position();

    double deltat;
    if (state.timestamp_s < last_timestamp_s) {
        // Physics time has gone backwards, don't reset AP, assume an average size timestep
        printf("Detected physics reset\n");
        deltat = 0;
    } else {
        deltat = state.timestamp_s - last_timestamp_s;
    }
    time_now_us += deltat * 1.0e6;

    if (deltat > 0 && deltat < 0.1) {
        // time in us to hz
        adjust_frame_time(1.0 / deltat);

        // match actual frame rate with desired speedup
        time_advance();
    }
    last_timestamp_s = state.timestamp_s;
    frame_counter++;

#if 0
// @LoggerMessage: JSN1
// @Description: Log data received from JSON simulator
// @Field: TimeUS: Time since system startup
// @Field: TUS: Simulation's timestamp
// @Field: R: Simulation's roll
// @Field: P: Simulation's pitch
// @Field: Y: Simulation's yaw
// @Field: GX: Simulated gyroscope, X-axis
// @Field: GY: Simulated gyroscope, Y-axis
// @Field: GZ: Simulated gyroscope, Z-axis
    AP::logger().Write("JSN1", "TimeUS,TUS,R,P,Y,GX,GY,GZ",
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

// @LoggerMessage: JSN2
// @Description: Log data received from JSON simulator
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
    AP::logger().Write("JSN2", "TimeUS,AX,AY,AZ,VX,VY,VZ,PX,PY,PZ,Alt,SD",
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

}

/*
   update the JSON simulation by one time step
*/
void JSON::update(const struct sitl_input &input)
{
    // send to JSON model
    output_servos(input);

    // receive from JSON model
    recv_fdm(input);

    // update magnetic field
    // as the model does not provide mag feild we calculate it from position and attitude
    update_mag_field_bf();

    // allow for changes in physics step
    adjust_frame_time(constrain_float(sitl->loop_rate_hz, rate_hz-1, rate_hz+1));

#if 0
    // report frame rate
    if (frame_counter % 1000 == 0) {
        printf("FPS %.2f\n", achieved_rate_hz); // this is instantaneous rather than any clever average
    }
#endif
}
